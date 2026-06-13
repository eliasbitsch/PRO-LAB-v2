#pragma once
// Extended Kalman Filter (pure, no ROS).
//
// Notation follows Thrun, Burgard, Fox - Probabilistic Robotics §3.3:
//   x_t = g(u_t, x_{t-1}) + ε_t       (non-linear motion model)
//   z_t = h(x_t)         + δ_t       (non-linear measurement model)
//   G_t = ∂g/∂x  evaluated at (u_t, μ_{t-1})    (motion Jacobian)
//   H_t = ∂h/∂x  evaluated at predicted μ̄_t      (measurement Jacobian)
//   R_t = process noise covariance               (≠ aerospace R!)
//   Q_t = measurement noise covariance           (≠ aerospace Q!)
//
// Motion: unicycle (3D state x, y, yaw, control [v, ω]).
// Measurements: IMU yaw, range-bearing landmarks, full pose,
// optional likelihood-field scan beams (for the EKF-LF advanced variant).
//
// Parameter naming note: ROS parameters keep "r_*" for measurement noise
// and "q_*" for process noise (aerospace convention) so old scenario
// YAMLs stay valid. Internally everything follows Thrun (G/H, R = process,
// Q = measurement).
#include <Eigen/Dense>
#include <cmath>

#include "pro_lab_filters/LikelihoodField.h"

namespace pro_lab_filters {

inline double ekf_wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class EKF {
public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  EKF() = default;

  void init(const Vec3& x0, const Mat3& Sigma0, const Mat3& R_proc) {
    x_ = x0; P_ = Sigma0; R_ = R_proc;
  }

  // Unicycle predict with motion Jacobian G.
  //   μ̄_t = g(u_t, μ_{t-1})
  //   Σ̄_t = G_t · Σ_{t-1} · G_tᵀ + R_t
  void predict(double v, double w, double dt) {
    const double theta = x_(2);
    x_(0) += v * std::cos(theta) * dt;
    x_(1) += v * std::sin(theta) * dt;
    x_(2) = ekf_wrap_pi(theta + w * dt);
    Mat3 G;
    G << 1.0, 0.0, -v * std::sin(theta) * dt,
         0.0, 1.0,  v * std::cos(theta) * dt,
         0.0, 0.0,  1.0;
    P_ = G * P_ * G.transpose() + R_ * dt;
  }

  // K_t = Σ̄_t · H_tᵀ · (H_t · Σ̄_t · H_tᵀ + Q_t)⁻¹
  void updateImuYaw(double yaw_meas, double q_yaw) {
    Eigen::Matrix<double, 1, 3> H;
    H << 0.0, 0.0, 1.0;
    const double innov = ekf_wrap_pi(yaw_meas - x_(2));
    const double S = (H * P_ * H.transpose())(0, 0) + q_yaw;
    Vec3 K = P_ * H.transpose() / S;
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  // Range-bearing landmark update (Probabilistic Robotics §7.5).
  //   z = (range, bearing) measured from robot to a known landmark at (lx, ly).
  // Linearises h(x) around the current mean to build the Jacobian H, then
  // does the standard EKF correction. Bearing residual is wrapped to (-π, π].
  void updateLandmark(double lx, double ly,
                      double meas_range, double meas_bearing,
                      double q_range, double q_bearing) {
    const double dx = lx - x_(0);
    const double dy = ly - x_(1);
    const double q  = dx * dx + dy * dy;   // local variable, not Thrun's Q
    if (q < 1e-9) {
      return;
    }
    const double sq = std::sqrt(q);
    const double pred_range   = sq;
    const double pred_bearing = ekf_wrap_pi(std::atan2(dy, dx) - x_(2));

    Eigen::Matrix<double, 2, 3> H;
    H << -dx / sq,        -dy / sq,         0.0,
          dy / q,         -dx / q,         -1.0;

    Eigen::Matrix<double, 2, 2> Q_meas;
    Q_meas << q_range, 0.0, 0.0, q_bearing;

    Eigen::Vector2d innov(meas_range - pred_range,
                          ekf_wrap_pi(meas_bearing - pred_bearing));

    Eigen::Matrix<double, 2, 2> S = H * P_ * H.transpose() + Q_meas;
    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  void updatePose(double mx, double my, double myaw, const Vec3& q_diag) {
    Mat3 H = Mat3::Identity();
    Mat3 Q_meas = q_diag.asDiagonal();
    Vec3 z(mx, my, myaw);
    Vec3 innov = z - x_;
    innov(2) = ekf_wrap_pi(innov(2));
    Mat3 S = H * P_ * H.transpose() + Q_meas;
    Mat3 K = P_ * H.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = ekf_wrap_pi(x_(2));
    P_ = (Mat3::Identity() - K * H) * P_;
  }

  // Likelihood-field scan update (Probabilistic Robotics §7.4 / §6.4).
  // Each kept beam contributes a scalar pseudo-measurement: the predicted
  // endpoint should land on a wall, i.e. distance-to-nearest-obstacle = 0.
  // Innovation is the actual miss distance; H is the gradient of the
  // distance field times the endpoint's Jacobian w.r.t. the pose.
  //
  // Iterates per beam and applies the sequential Joseph-style scalar
  // update - that's numerically the same as stacking all beams into a
  // single measurement, but lets us (a) apply Mahalanobis gating per
  // beam and (b) drop ones that fall outside the map.
  //
  // Args:
  //   ranges          beam ranges (m), in scanner frame
  //   angle_min, angle_inc  ranges[i] is at angle = angle_min + i·angle_inc
  //                          (relative to the robot's heading)
  //   range_min/max   beams outside this band are skipped (invalid)
  //   stride          subsample 1-in-N beams (10..30 is sane)
  //   sigma_hit       std-dev of the per-beam likelihood (m)
  //   d_max           gating: skip beams whose miss distance exceeds this
  //                          (these are outliers - open doors, dynamic
  //                           obstacles, or simply wrong init)
  void updateScanLikelihood(const LikelihoodField& lf,
                            const std::vector<float>& ranges,
                            double angle_min, double angle_inc,
                            double range_min, double range_max,
                            double lidar_x = 0.0,
                            double lidar_y = 0.0,
                            double lidar_yaw = 0.0,
                            int stride = 20,
                            double sigma_hit = 0.10,
                            double d_max = 0.50) {
    if (!lf.valid() || ranges.empty()) return;
    const double sx = x_(0), sy = x_(1), syaw = x_(2);
    const double Q_meas = sigma_hit * sigma_hit;
    // 2-cell finite-difference step for the gradient. Cell size isn't
    // exposed by LikelihoodField, so we fix a small metric step that's
    // bigger than typical map resolution (5 cm) but smaller than wall
    // thickness - 0.10 m is a fine compromise.
    const double h_step = 0.10;
    const double cos_yaw = std::cos(syaw);
    const double sin_yaw = std::sin(syaw);
    // Scanner pose in world frame.
    const double scan_x = sx + lidar_x * cos_yaw - lidar_y * sin_yaw;
    const double scan_y = sy + lidar_x * sin_yaw + lidar_y * cos_yaw;
    const int n = static_cast<int>(ranges.size());
    for (int i = 0; i < n; i += std::max(1, stride)) {
      const float r = ranges[i];
      if (!std::isfinite(r) || r < range_min || r > range_max) continue;
      const double a = angle_min + static_cast<double>(i) * angle_inc;
      // Beam direction in world frame (robot yaw + lidar yaw + beam angle).
      const double beam_yaw = syaw + lidar_yaw + a;
      const double cab = std::cos(beam_yaw);
      const double sab = std::sin(beam_yaw);
      const double ex = scan_x + r * cab;
      const double ey = scan_y + r * sab;

      // Miss distance at the predicted endpoint.
      const double d = lf.distanceMeters(ex, ey);
      if (!(d < d_max)) continue;   // gate (also drops NaN/inf)

      // ∇d at endpoint via central differences on the field.
      const double dx_pos = lf.distanceMeters(ex + h_step, ey);
      const double dx_neg = lf.distanceMeters(ex - h_step, ey);
      const double dy_pos = lf.distanceMeters(ex, ey + h_step);
      const double dy_neg = lf.distanceMeters(ex, ey - h_step);
      const double gx = (dx_pos - dx_neg) / (2.0 * h_step);
      const double gy = (dy_pos - dy_neg) / (2.0 * h_step);

      // ∂e/∂(x,y,yaw) accounting for the lidar offset:
      //   ex = x + lidar_x·cos(yaw) - lidar_y·sin(yaw) + r·cos(yaw + lidar_yaw + a)
      //   ey = y + lidar_x·sin(yaw) + lidar_y·cos(yaw) + r·sin(yaw + lidar_yaw + a)
      // ∂ex/∂yaw = -lidar_x·sin(yaw) - lidar_y·cos(yaw) - r·sin(beam_yaw)
      // ∂ey/∂yaw =  lidar_x·cos(yaw) - lidar_y·sin(yaw) + r·cos(beam_yaw)
      const double dex_dyaw = -lidar_x * sin_yaw - lidar_y * cos_yaw - r * sab;
      const double dey_dyaw =  lidar_x * cos_yaw - lidar_y * sin_yaw + r * cab;
      Eigen::Matrix<double, 1, 3> H;
      H << gx, gy, gx * dex_dyaw + gy * dey_dyaw;
      const double innov = -d;        // we want d=0, observe d
      const double S = (H * P_ * H.transpose())(0, 0) + Q_meas;
      if (!(S > 0.0)) continue;
      // Cheap Mahalanobis gate: drop the rare beam where innov² / S
      // exceeds chi²(1, 0.99) ≈ 6.63. Stops a single bad beam (eg open
      // door) from yanking the estimate when init is already wrong.
      if (innov * innov / S > 6.63) continue;
      Vec3 K = P_ * H.transpose() / S;
      x_ += K * innov;
      x_(2) = ekf_wrap_pi(x_(2));
      P_ = (Mat3::Identity() - K * H) * P_;
    }
  }

  const Vec3& state() const { return x_; }
  const Mat3& covariance() const { return P_; }

private:
  Vec3 x_ = Vec3::Zero();
  Mat3 P_ = Mat3::Identity();           // Σ_t (Thrun)
  Mat3 R_ = Mat3::Identity() * 0.05;    // process noise R_t (Thrun)
};

}  // namespace pro_lab_filters
