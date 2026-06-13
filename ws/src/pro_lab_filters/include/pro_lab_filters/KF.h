#pragma once
// Linear Kalman Filter (pure, no ROS).
//
// Notation follows Thrun, Burgard, Fox - Probabilistic Robotics §3.2:
//   x_t = A_t · x_{t-1} + B_t · u_t + ε_t       (motion model)
//   z_t = C_t · x_t + δ_t                        (measurement model)
//   R_t = process noise covariance               (≠ R from aerospace!)
//   Q_t = measurement noise covariance           (≠ Q from aerospace!)
//
// State: [x, y, yaw, vx, vy, omega]   (6D, world-frame velocities)
//
// Why 6D and not the 3D unicycle that EKF/PF use?
// A linear KF requires both the predict and every measurement model to be of
// the form A·x + B·u + w (no products of state variables, no trig). The
// 3D unicycle predict x' = x + v·cos(yaw)·dt is non-linear in yaw, which
// would force a Jacobian linearisation - at which point you're not a KF
// anymore, you're an EKF.
//
// By promoting the velocities into the state (constant-velocity model in
// world frame), the predict becomes a pure linear transformation:
//
//        ⎡I  Δt·I⎤
//   A  = ⎢       ⎥          (no B_t - no /cmd_vel control input)
//        ⎣0   I  ⎦
//
// No /cmd_vel control input is consumed - body-frame velocity → world-frame
// velocity would itself be non-linear in yaw. The filter learns vx/vy/omega
// passively from sequential position/yaw measurements; the process noise on
// the velocity block absorbs the unmodelled accelerations.
//
// Measurements:
//   IMU yaw            scalar     C = [0 0 1 0 0 0]
//   IMU omega (gyro_z) scalar     C = [0 0 0 0 0 1]
//   Position (x, y)    2-vector   C = [I  0]   (2×6)
//
// Parameter naming note: ROS parameters keep "r_*" for measurement noise
// and "q_*" for process noise (aerospace convention) so old scenario YAMLs
// stay valid. Internally everything follows Thrun (A/C, R = process,
// Q = measurement).
#include <Eigen/Dense>
#include <cmath>

namespace pro_lab_filters {

inline double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class KF {
public:
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  KF() = default;

  // Σ_0 in Thrun = our P_; R_t = process noise (constant per tick scaled
  // by dt in predict).
  void init(const Vec6& x0, const Mat6& Sigma0, const Mat6& R_proc) {
    x_ = x0; P_ = Sigma0; R_ = R_proc;
  }

  // Constant-velocity predict - fully linear.
  // x_t = A_t · x_{t-1}      Σ_t = A_t · Σ_{t-1} · A_tᵀ + R_t
  void predict(double dt) {
    Mat6 A = Mat6::Identity();
    A(0, 3) = dt;   // x  += vx·dt
    A(1, 4) = dt;   // y  += vy·dt
    A(2, 5) = dt;   // yaw += omega·dt
    x_ = A * x_;
    x_(2) = wrap_pi(x_(2));
    P_ = A * P_ * A.transpose() + R_ * dt;
  }

  // IMU yaw measurement (scalar).
  // K_t = Σ_t · C_tᵀ · (C_t · Σ_t · C_tᵀ + Q_t)⁻¹
  void updateImuYaw(double yaw_meas, double q_yaw) {
    Eigen::Matrix<double, 1, 6> C = Eigen::Matrix<double, 1, 6>::Zero();
    C(0, 2) = 1.0;
    const double innov = wrap_pi(yaw_meas - x_(2));
    const double S = (C * P_ * C.transpose())(0, 0) + q_yaw;
    Vec6 K = P_ * C.transpose() / S;
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat6::Identity() - K * C) * P_;
  }

  // IMU gyro_z (angular velocity) measurement.
  void updateImuOmega(double omega_meas, double q_omega) {
    Eigen::Matrix<double, 1, 6> C = Eigen::Matrix<double, 1, 6>::Zero();
    C(0, 5) = 1.0;
    const double innov = omega_meas - x_(5);
    const double S = (C * P_ * C.transpose())(0, 0) + q_omega;
    Vec6 K = P_ * C.transpose() / S;
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat6::Identity() - K * C) * P_;
  }

  // 3D velocity measurement [vx, vy, omega] from wheel odometry. The encoder
  // forward speed v and yaw rate ω are a real sensor; the body→world rotation
  // (vx=v·cosψ, vy=v·sinψ) is done in the node with the current yaw estimate -
  // the same linearise-outside-the-filter trick the landmark update uses - so
  // C stays constant and the filter stays linear. Without this anchor the CV
  // model infers velocity only from noisy position differences and coasts away
  // (a stopped robot would keep "drifting" at a bogus estimated velocity).
  void updateVelocity(double vx, double vy, double omega, double q_v, double q_w) {
    Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
    C(0, 3) = 1.0; C(1, 4) = 1.0; C(2, 5) = 1.0;
    Eigen::Matrix3d Q_meas = Eigen::Matrix3d::Zero();
    Q_meas(0, 0) = q_v; Q_meas(1, 1) = q_v; Q_meas(2, 2) = q_w;
    Eigen::Vector3d z(vx, vy, omega);
    Eigen::Vector3d innov = z - Eigen::Vector3d(x_(3), x_(4), x_(5));
    Eigen::Matrix3d S = C * P_ * C.transpose() + Q_meas;
    Eigen::Matrix<double, 6, 3> K = P_ * C.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat6::Identity() - K * C) * P_;
  }

  // 2D position measurement [x, y]. Used by the kf_node landmark
  // triangulation: visible landmarks + current yaw → averaged (x, y).
  void updatePosition(double mx, double my, double q_xy) {
    Eigen::Matrix<double, 2, 6> C = Eigen::Matrix<double, 2, 6>::Zero();
    C(0, 0) = 1.0;
    C(1, 1) = 1.0;
    Eigen::Matrix2d Q_meas = Eigen::Matrix2d::Identity() * q_xy;
    Eigen::Vector2d z(mx, my);
    Eigen::Vector2d innov = z - Eigen::Vector2d(x_(0), x_(1));
    Eigen::Matrix2d S = C * P_ * C.transpose() + Q_meas;
    Eigen::Matrix<double, 6, 2> K = P_ * C.transpose() * S.inverse();
    x_ += K * innov;
    x_(2) = wrap_pi(x_(2));
    P_ = (Mat6::Identity() - K * C) * P_;
  }

  const Vec6& state() const { return x_; }
  const Mat6& covariance() const { return P_; }

private:
  Vec6 x_ = Vec6::Zero();
  Mat6 P_ = Mat6::Identity();           // Σ_t (Thrun)
  Mat6 R_ = Mat6::Identity() * 0.05;    // process noise R_t (Thrun)
};

}  // namespace pro_lab_filters
