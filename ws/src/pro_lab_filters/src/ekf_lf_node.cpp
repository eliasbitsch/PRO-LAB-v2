// EKF-LF: Extended Kalman Filter with **likelihood-field scan update**.
// Drops the AMCL `/pose` channel that ekf_node uses and instead consumes
// `/scan` + `/map` directly via a precomputed Euclidean distance transform
// (LikelihoodField). Each subsampled beam contributes a scalar Kalman
// update - see Probabilistic Robotics §7.4.
//
// This is the *advanced* EKF variant. The story for the wrong-init study:
// linearising the scan model around an init pose that is far from truth
// gives a useless Jacobian, so EKF-LF is expected to diverge under big
// init errors - exactly the lesson we want in the paper.
//
// Inputs: /cmd_vel, /imu, /map (transient_local), /scan
// Outputs: /ekf_lf/pose, /ekf_lf/runtime_us
#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "pro_lab_filters/EKF.h"
#include "pro_lab_filters/LikelihoodField.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class EKFLFNode : public rclcpp::Node {
public:
  EKFLFNode() : Node("ekf_lf_node") {
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov", 0.1);
    declare_parameter("init_spread_xy",  0.0);
    declare_parameter("init_spread_yaw", 0.0);
    declare_parameter("rng_seed", 0);
    declare_parameter("q_scale", 0.05);
    declare_parameter("r_yaw_imu", 0.02);
    declare_parameter("frame_id", std::string("map"));
    declare_parameter("base_frame", std::string("base_footprint"));
    // Scan-likelihood tuning. stride=20 with a TB4 scan (~640 beams) keeps
    // ~32 beams per update - enough info, light enough for 20 Hz, and
    // sparse enough that the per-beam updates stay quasi-independent.
    declare_parameter("scan_stride",  20);
    declare_parameter("sigma_hit",     0.10);
    declare_parameter("d_max",         0.50);
    declare_parameter("scan_min_range", 0.20);
    declare_parameter("scan_max_range", 12.0);

    Eigen::Vector3d mean(
      get_parameter("init_x").as_double(),
      get_parameter("init_y").as_double(),
      get_parameter("init_yaw").as_double());
    const double spread_xy  = get_parameter("init_spread_xy").as_double();
    const double spread_yaw = get_parameter("init_spread_yaw").as_double();
    const int    seed       = get_parameter("rng_seed").as_int();
    Eigen::Vector3d x0 = mean;
    if (seed > 0 && (spread_xy > 0.0 || spread_yaw > 0.0)) {
      // Use a different seed XOR than KF/EKF so the LF variant lands at a
      // distinct sampled init within the same Gaussian - otherwise EKF and
      // EKF-LF would always start at identical x0 and the comparison
      // degenerates.
      std::mt19937_64 rng(static_cast<std::uint64_t>(seed) ^ 0x1Fu);
      std::normal_distribution<double> nx(0.0, spread_xy);
      std::normal_distribution<double> ny(0.0, spread_xy);
      std::normal_distribution<double> nt(0.0, spread_yaw);
      x0 += Eigen::Vector3d(nx(rng), ny(rng), nt(rng));
    }
    Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * get_parameter("init_cov").as_double();
    Eigen::Matrix3d Q  = Eigen::Matrix3d::Identity() * get_parameter("q_scale").as_double();
    ekf_.init(x0, P0, Q);

    r_yaw_imu_      = get_parameter("r_yaw_imu").as_double();
    frame_id_       = get_parameter("frame_id").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    scan_stride_    = get_parameter("scan_stride").as_int();
    sigma_hit_      = get_parameter("sigma_hit").as_double();
    d_max_          = get_parameter("d_max").as_double();
    scan_min_range_ = get_parameter("scan_min_range").as_double();
    scan_max_range_ = get_parameter("scan_max_range").as_double();

    RCLCPP_INFO(get_logger(),
                "ekf_lf init: mean=(%.2f,%.2f,%.2f) seed=%d spread_xy=%.2f -> x0=(%.2f,%.2f,%.2f)",
                mean(0), mean(1), mean(2), seed, spread_xy, x0(0), x0(1), x0(2));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        v_ = m->linear.x; w_ = m->angular.z;
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 20,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        ekf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
      });

    // Map: transient_local QoS to receive the latched /map from the
    // standalone map_server in the launch.
    rclcpp::QoS map_qos(10);
    map_qos.reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        lf_.build(m->info.width, m->info.height, m->info.resolution,
                  m->info.origin.position.x, m->info.origin.position.y,
                  m->data.data());
        RCLCPP_INFO(get_logger(),
            "ekf_lf likelihood-field built: %dx%d @ %.3fm",
            m->info.width, m->info.height, m->info.resolution);
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });

    pub_         = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_lf/pose", 10);
    runtime_pub_ = create_publisher<std_msgs::msg::Float64>("/ekf_lf/runtime_us", 10);
    timer_       = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  void cacheLidarTf(const std::string& sensor_frame) {
    if (have_lidar_tf_) return;
    try {
      auto tf = tf_buffer_->lookupTransform(
          base_frame_, sensor_frame, tf2::TimePointZero);
      lidar_x_ = tf.transform.translation.x;
      lidar_y_ = tf.transform.translation.y;
      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
      double r, p, y;
      tf2::Matrix3x3(q).getRPY(r, p, y);
      lidar_yaw_     = y;
      have_lidar_tf_ = true;
      RCLCPP_INFO(get_logger(),
          "ekf_lf lidar TF cached: %s -> %s = (%.3f, %.3f, yaw=%.3f rad)",
          base_frame_.c_str(), sensor_frame.c_str(),
          lidar_x_, lidar_y_, lidar_yaw_);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "ekf_lf: waiting for TF %s -> %s: %s",
          base_frame_.c_str(), sensor_frame.c_str(), ex.what());
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan& m) {
    if (!lf_.valid()) return;
    cacheLidarTf(m.header.frame_id);
    if (!have_lidar_tf_) return;

    const auto t_start = std::chrono::steady_clock::now();
    ekf_.updateScanLikelihood(
        lf_, m.ranges,
        m.angle_min, m.angle_increment,
        std::max<double>(scan_min_range_, m.range_min),
        std::min<double>(scan_max_range_, m.range_max),
        lidar_x_, lidar_y_, lidar_yaw_,
        scan_stride_, sigma_hit_, d_max_);
    const auto t_end = std::chrono::steady_clock::now();
    last_scan_us_ = std::chrono::duration<double, std::micro>(t_end - t_start).count();
  }

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
    // Runtime budget = predict tick + most recent scan update. Reflects
    // the total cost the filter pays per cycle; honest comparison vs
    // KF/EKF whose predict-only tick has no scan workload to amortise.
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count()
              + last_scan_us_;
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::EKF ekf_;
  pro_lab_filters::LikelihoodField lf_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double v_ = 0.0, w_ = 0.0;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.02;
  std::string frame_id_, base_frame_;
  int    scan_stride_   = 20;
  double sigma_hit_     = 0.10;
  double d_max_         = 0.50;
  double scan_min_range_ = 0.20, scan_max_range_ = 12.0;
  double lidar_x_ = 0.0, lidar_y_ = 0.0, lidar_yaw_ = 0.0;
  bool   have_lidar_tf_ = false;
  double last_scan_us_  = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFLFNode>());
  rclcpp::shutdown();
  return 0;
}
