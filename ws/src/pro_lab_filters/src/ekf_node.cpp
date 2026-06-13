// ROS2 wrapper for EKF. Inputs: /odom, /imu, /landmarks/observations.
// Outputs: /ekf/pose, /ekf/runtime_us.
//
// Velocity control input comes from /odom (measured wheel-encoder velocity),
// NOT /cmd_vel (commanded velocity). Two reasons:
//   1. /cmd_vel reflects what we SAY the robot should do; /odom what it
//      actually does. The latter survives wheel slip, dropped commands,
//      external impacts.
//   2. During the 14 s warmup before trajectory_player publishes, nav2's
//      controller_server briefly emitted non-zero /cmd_vel during its own
//      bring-up. The EKF integrated that ~1.3 m before the watchdog zeroed
//      it out, and the running RMSE never recovered. /odom has wheels=0
//      during warmup, so no spurious motion is integrated.
//
// State: [x, y, yaw]   (3D — v, omega enter as control inputs, not state)
//
// Why 3D — paired with the 6D KF on purpose:
//   The EKF DOES linearise. At every predict step it forms the Jacobian
//       F = ∂f/∂x = | 1  0  -v·sin(yaw)·dt |
//                   | 0  1   v·cos(yaw)·dt |
//                   | 0  0          1      |
//   evaluated at the current estimate, which is a 1st-order Taylor expansion
//   of the natural (nonlinear) Unicycle model around x̂_k. Because the model
//   is allowed to be nonlinear, v and omega stay as control inputs — they do
//   NOT have to live in the state. Hence n=3 vs. the KF's n=6 (which has to
//   carry vx/vy/omega in state to keep the model linear). Per-tick matrix
//   work scales with n^3, so the EKF's predict (3^3=27 flops) is empirically
//   cheaper than the KF's (6^3=216) even though it does extra Jacobian work.
//
// Landmarks (range/bearing) come in as Float32MultiArray with stride 3:
//   [id_0, range_0, bearing_0, id_1, range_1, bearing_1, ...]
// where id matches the landmark_*s arrays declared on this node. Every
// observation triggers one EKF correction step (see EKF::updateLandmark),
// which also linearises the nonlinear h(x) = [range, bearing] measurement
// model with its own Jacobian H.
#include <chrono>
#include <random>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pro_lab_filters/EKF.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class EKFNode : public rclcpp::Node {
public:
  EKFNode() : Node("ekf_node") {
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov", 0.1);
    declare_parameter("init_spread_xy",  0.0);
    declare_parameter("init_spread_yaw", 0.0);
    declare_parameter("rng_seed", 0);
    declare_parameter("q_scale", 0.05);
    declare_parameter("r_yaw_imu", 0.02);
    declare_parameter("frame_id", std::string("odom"));
    // Dead-reckoning twin support: with both flags false this node becomes a
    // pure odometry integrator (predict-only). Used as `ekf_dr` in the launch
    // to produce the lecture-style "odometry drift + growing uncertainty"
    // baseline for the explainer plots — real integration, not a mock-up.
    declare_parameter("use_imu",       true);
    declare_parameter("use_landmarks", true);
    declare_parameter("topic_prefix",  std::string("/ekf"));
    // Velocity source for the predict step:
    //   "odom"            — measured wheel-encoder velocity (Thrun §5.4,
    //                       odometry motion model). Default; all comparison
    //                       filters use this.
    //   any other string  — treated as a Twist topic carrying COMMANDED
    //                       velocity (Thrun §5.3, velocity motion model).
    //                       Used by the ekf_dr twin with "/cmd_vel_in":
    //                       integrating the command instead of the encoder
    //                       exposes the real cmd-vs-actual mismatch
    //                       (acceleration ramps, wheel slip during pivots)
    //                       as honest, visible dead-reckoning drift.
    declare_parameter("velocity_source", std::string("odom"));
    // Landmark fusion: same xs/ys/ids the landmark_detector_node uses.
    // r_landmark_range / r_landmark_bearing are measurement variances (= σ²).
    declare_parameter<std::vector<int64_t>>("landmark_ids",
        {1, 2, 3, 4, 5, 6, 7, 8});
    declare_parameter<std::vector<double>>("landmark_xs", {
        -7.45,  7.49, -7.42, 7.46, -7.51, 7.43, -7.45, 7.43});
    declare_parameter<std::vector<double>>("landmark_ys", {
       -15.02,-15.02, -7.55,-7.52, -0.02,-0.02,  7.54, 7.48});
    declare_parameter("r_landmark_range",   0.01);   // σ=0.10 m
    declare_parameter("r_landmark_bearing", 0.0025); // σ≈0.05 rad

    Eigen::Vector3d mean(
      get_parameter("init_x").as_double(),
      get_parameter("init_y").as_double(),
      get_parameter("init_yaw").as_double());
    const double spread_xy  = get_parameter("init_spread_xy").as_double();
    const double spread_yaw = get_parameter("init_spread_yaw").as_double();
    const int    seed       = get_parameter("rng_seed").as_int();
    Eigen::Vector3d x0 = mean;
    if (seed > 0 && (spread_xy > 0.0 || spread_yaw > 0.0)) {
      // EKF + KF use the same seed so paired filters start from the same
      // sampled x0 in a given run; that keeps the comparison apples-to-apples.
      std::mt19937_64 rng(static_cast<std::uint64_t>(seed) ^ 0xE2u);
      std::normal_distribution<double> nx(0.0, spread_xy);
      std::normal_distribution<double> ny(0.0, spread_xy);
      std::normal_distribution<double> nt(0.0, spread_yaw);
      x0 += Eigen::Vector3d(nx(rng), ny(rng), nt(rng));
    }
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * get_parameter("init_cov").as_double();
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * get_parameter("q_scale").as_double();
    ekf_.init(x0, P0, Q);
    RCLCPP_INFO(get_logger(),
                "ekf init: mean=(%.2f,%.2f,%.2f) seed=%d spread_xy=%.2f -> x0=(%.2f,%.2f,%.2f)",
                mean(0), mean(1), mean(2), seed, spread_xy, x0(0), x0(1), x0(2));

    r_yaw_imu_ = get_parameter("r_yaw_imu").as_double();
    frame_id_  = get_parameter("frame_id").as_string();

    const auto vel_src = get_parameter("velocity_source").as_string();
    if (vel_src == "odom") {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 20,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          v_ = m->twist.twist.linear.x;
          w_ = m->twist.twist.angular.z;
        });
    } else {
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        vel_src, 20,
        [this](geometry_msgs::msg::Twist::SharedPtr m) {
          v_ = m->linear.x;
          w_ = m->angular.z;
        });
    }
    if (get_parameter("use_imu").as_bool()) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20,
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
          ekf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
        });
    }

    // Build the {id → (lx, ly)} lookup for landmark updates.
    {
      const auto ids = get_parameter("landmark_ids").as_integer_array();
      const auto xs  = get_parameter("landmark_xs").as_double_array();
      const auto ys  = get_parameter("landmark_ys").as_double_array();
      const std::size_t n = std::min({ids.size(), xs.size(), ys.size()});
      for (std::size_t i = 0; i < n; ++i) {
        landmarks_[static_cast<int>(ids[i])] = {xs[i], ys[i]};
      }
    }
    r_lm_range_   = get_parameter("r_landmark_range").as_double();
    r_lm_bearing_ = get_parameter("r_landmark_bearing").as_double();

    if (get_parameter("use_landmarks").as_bool()) {
      landmarks_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/landmarks/observations", 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
          // Stride-3 (id, range, bearing) flat array.
          for (std::size_t i = 0; i + 2 < m->data.size(); i += 3) {
            const int    id      = static_cast<int>(m->data[i]);
            const double range   = m->data[i + 1];
            const double bearing = m->data[i + 2];
            auto it = landmarks_.find(id);
            if (it == landmarks_.end()) continue;
            ekf_.updateLandmark(it->second.first, it->second.second,
                                range, bearing,
                                r_lm_range_, r_lm_bearing_);
          }
        });
    }

    // Output prefix parameterised so a second instance (ekf_dr, the
    // dead-reckoning twin) can publish /ekf_dr/pose alongside /ekf/pose.
    const std::string prefix = get_parameter("topic_prefix").as_string();
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(prefix + "/pose", 10);
    runtime_pub_ = create_publisher<std_msgs::msg::Float64>(prefix + "/runtime_us", 10);
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  void tick() {
    auto now = this->now();
    double t = now.seconds();
    if (!have_last_) { last_t_ = t; have_last_ = true; return; }
    double dt = t - last_t_;
    last_t_ = t;
    const auto t_start = std::chrono::steady_clock::now();
    if (dt > 0) ekf_.predict(v_, w_, dt);
    const auto& x = ekf_.state();
    const auto& P = ekf_.covariance();
    pub_->publish(pro_lab_filters::make_pose_xy(
        now, frame_id_, x(0), x(1), x(2),
        P(0, 0), P(1, 1), P(0, 1), P(2, 2)));
    const auto t_end = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::EKF ekf_;
  double v_ = 0.0, w_ = 0.0;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.02;
  std::string frame_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<int, std::pair<double, double>> landmarks_;
  double r_lm_range_   {0.01};
  double r_lm_bearing_ {0.0025};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}
