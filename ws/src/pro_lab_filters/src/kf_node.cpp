// ROS2 wrapper for the linear KF.
//
// State: [x, y, yaw, vx, vy, omega]   (6D, world-frame velocities)
//
// Why 6D - paired with the 3D EKF on purpose:
//   The KF cannot linearise anything; it requires its motion model to be
//   linear in the state. The natural Unicycle model x_{k+1} = x_k + v·cos(yaw)·dt
//   is NOT linear in yaw, so we cannot use it directly. The workaround is to
//   reformulate the problem with a Constant-Velocity model in world frame:
//       x_{k+1}  = x_k  + vx · dt          (linear)
//       y_{k+1}  = y_k  + vy · dt          (linear)
//       yaw_{k+1}= yaw_k+ omega · dt       (linear)
//   For these equations to hold, the velocity components have to live IN the
//   state - hence n=6 (vs. n=3 for the EKF, which keeps v/w as control inputs
//   and instead linearises the Unicycle model with a Jacobian at every tick).
//   Trade-off: KF pays linearity with bigger matrices (predict ~ n^3 → 6^3=216
//   flops); EKF pays nonlinearity with per-tick Jacobian construction at n=3.
//
// Inputs:
//   /imu                              Imu                  - yaw + gyro_z
//   /odom                             Odometry             - encoder v + yaw rate
//   /landmarks/observations           Float32MultiArray    - stride-3 [id, range, bearing]
//
// Output: /kf/pose, /kf/runtime_us
//
// Notable: /cmd_vel (a control input) is intentionally NOT consumed - using it
// would re-introduce a non-linear body→world rotation in the predict. The
// velocities live in the state (constant-velocity model, world frame). The
// wheel ODOMETRY (a measurement, real encoder) anchors them: its body speed is
// rotated to world with the current yaw estimate (same linearise-outside trick
// as the landmark update) and fed as a linear [vx,vy,omega] measurement. IMU
// supplies yaw/gyro; landmarks supply position. Without the odom anchor the CV
// model infers velocity from noisy position diffs and coasts away when the
// robot stops.
//
// Landmark fusion: triangulation. For each visible landmark we recover an
// implied robot (x, y) from the known landmark position and the current yaw
// estimate, average across visible landmarks, and feed one linear (x, y)
// measurement into KF::updatePosition.
#include <chrono>
#include <random>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pro_lab_filters/KF.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class KFNode : public rclcpp::Node {
public:
  KFNode() : Node("kf_node") {
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov", 0.1);
    declare_parameter("init_cov_vel", 0.5);   // velocity components - wider since we have no prior
    declare_parameter("init_spread_xy",  0.0);
    declare_parameter("init_spread_yaw", 0.0);
    declare_parameter("rng_seed", 0);
    // Process noise: small on position/yaw (kinematics propagate them),
    // bigger on velocities (they absorb the un-modelled accelerations).
    declare_parameter("q_pos",  0.01);
    declare_parameter("q_yaw",  0.01);
    declare_parameter("q_vel",  0.50);
    declare_parameter("q_omega", 0.20);
    declare_parameter("r_yaw_imu",   0.02);
    declare_parameter("r_omega_imu", 0.05);
    declare_parameter("r_landmark_xy", 0.05);
    declare_parameter("r_odom_v",     0.04);   // encoder linear-velocity noise
    declare_parameter("r_odom_omega", 0.05);   // encoder yaw-rate noise
    declare_parameter("frame_id", std::string("odom"));
    declare_parameter<std::vector<int64_t>>("landmark_ids",
        {1, 2, 3, 4, 5, 6, 7, 8});
    declare_parameter<std::vector<double>>("landmark_xs", {
        -7.45,  7.49, -7.42, 7.46, -7.51, 7.43, -7.45, 7.43});
    declare_parameter<std::vector<double>>("landmark_ys", {
       -15.02,-15.02, -7.55,-7.52, -0.02,-0.02,  7.54, 7.48});

    // Mean init for [x, y, yaw, vx=0, vy=0, omega=0]. The wrong-init study
    // perturbs the pose components; velocities always start at 0.
    using Vec6 = pro_lab_filters::KF::Vec6;
    using Mat6 = pro_lab_filters::KF::Mat6;
    Vec6 mean = Vec6::Zero();
    mean(0) = get_parameter("init_x").as_double();
    mean(1) = get_parameter("init_y").as_double();
    mean(2) = get_parameter("init_yaw").as_double();

    const double spread_xy  = get_parameter("init_spread_xy").as_double();
    const double spread_yaw = get_parameter("init_spread_yaw").as_double();
    const int    seed       = get_parameter("rng_seed").as_int();
    Vec6 x0 = mean;
    if (seed > 0 && (spread_xy > 0.0 || spread_yaw > 0.0)) {
      std::mt19937_64 rng(static_cast<std::uint64_t>(seed));
      std::normal_distribution<double> nx(0.0, spread_xy);
      std::normal_distribution<double> ny(0.0, spread_xy);
      std::normal_distribution<double> nt(0.0, spread_yaw);
      x0(0) += nx(rng);
      x0(1) += ny(rng);
      x0(2) += nt(rng);
    }

    Mat6 P0 = Mat6::Zero();
    const double cov_pose = get_parameter("init_cov").as_double();
    const double cov_vel  = get_parameter("init_cov_vel").as_double();
    P0(0, 0) = cov_pose; P0(1, 1) = cov_pose; P0(2, 2) = cov_pose;
    P0(3, 3) = cov_vel;  P0(4, 4) = cov_vel;  P0(5, 5) = cov_vel;

    Mat6 Q = Mat6::Zero();
    const double q_pos = get_parameter("q_pos").as_double();
    const double q_yaw = get_parameter("q_yaw").as_double();
    const double q_vel = get_parameter("q_vel").as_double();
    const double q_w   = get_parameter("q_omega").as_double();
    Q(0, 0) = q_pos; Q(1, 1) = q_pos; Q(2, 2) = q_yaw;
    Q(3, 3) = q_vel; Q(4, 4) = q_vel; Q(5, 5) = q_w;

    kf_.init(x0, P0, Q);
    RCLCPP_INFO(get_logger(),
                "kf init (6D CV): mean=(%.2f,%.2f,%.2f, 0,0,0) seed=%d "
                "spread_xy=%.2f -> x0=(%.2f,%.2f,%.2f)",
                mean(0), mean(1), mean(2), seed, spread_xy, x0(0), x0(1), x0(2));

    r_yaw_imu_     = get_parameter("r_yaw_imu").as_double();
    r_omega_imu_   = get_parameter("r_omega_imu").as_double();
    r_landmark_xy_ = get_parameter("r_landmark_xy").as_double();
    r_odom_v_      = get_parameter("r_odom_v").as_double();
    r_odom_omega_  = get_parameter("r_odom_omega").as_double();
    frame_id_      = get_parameter("frame_id").as_string();

    {
      const auto ids = get_parameter("landmark_ids").as_integer_array();
      const auto xs  = get_parameter("landmark_xs").as_double_array();
      const auto ys  = get_parameter("landmark_ys").as_double_array();
      const std::size_t n = std::min({ids.size(), xs.size(), ys.size()});
      for (std::size_t i = 0; i < n; ++i) {
        landmarks_[static_cast<int>(ids[i])] = {xs[i], ys[i]};
      }
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 20,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        kf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
        kf_.updateImuOmega(m->angular_velocity.z, r_omega_imu_);
      });
    // Wheel-odometry velocity anchor: rotate the encoder body speed into world
    // frame using the current yaw estimate, then feed a linear [vx,vy,omega]
    // measurement. Pins the CV velocity to reality (≈0 when stopped) so the
    // filter cannot coast away during the stationary tail of the run.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const double yaw = kf_.state()(2);
        const double v   = m->twist.twist.linear.x;
        const double vx  = v * std::cos(yaw);
        const double vy  = v * std::sin(yaw);
        kf_.updateVelocity(vx, vy, m->twist.twist.angular.z, r_odom_v_, r_odom_omega_);
      });
    landmarks_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/landmarks/observations", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
        const double yaw = kf_.state()(2);
        double sum_x = 0.0, sum_y = 0.0;
        int n = 0;
        for (std::size_t i = 0; i + 2 < m->data.size(); i += 3) {
          const int id = static_cast<int>(m->data[i]);
          auto it = landmarks_.find(id);
          if (it == landmarks_.end()) continue;
          const double range   = m->data[i + 1];
          const double bearing = m->data[i + 2];
          const double world_angle = yaw + bearing;
          const double mx = it->second.first  - range * std::cos(world_angle);
          const double my = it->second.second - range * std::sin(world_angle);
          sum_x += mx;
          sum_y += my;
          n += 1;
        }
        if (n > 0) {
          kf_.updatePosition(sum_x / n, sum_y / n, r_landmark_xy_ / n);
        }
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf/pose", 10);
    runtime_pub_ = create_publisher<std_msgs::msg::Float64>("/kf/runtime_us", 10);
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
    if (dt > 0) kf_.predict(dt);
    const auto& x = kf_.state();
    const auto& P = kf_.covariance();
    pub_->publish(pro_lab_filters::make_pose_xy(
        now, frame_id_, x(0), x(1), x(2),
        P(0, 0), P(1, 1), P(0, 1), P(2, 2)));
    const auto t_end = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::KF kf_;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.02, r_omega_imu_ = 0.05;
  double r_landmark_xy_ = 0.05;
  double r_odom_v_ = 0.04, r_odom_omega_ = 0.05;
  std::string frame_id_;
  std::unordered_map<int, std::pair<double, double>> landmarks_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KFNode>());
  rclcpp::shutdown();
  return 0;
}
