// ROS2 wrapper for PF. Inputs: /cmd_vel, /imu, /map, /scan.
// Outputs: /pf/pose, /pf/particles (PoseArray), /pf/ess (Float64).
//
// Wrong-init experiment knobs:
//   init_distribution: "gaussian" | "uniform"
//   init_x, init_y, init_yaw      — Gaussian center / Uniform center
//   init_spread_xy, init_spread_yaw — Gaussian std-dev (per axis)
//   init_extent_xy, init_extent_yaw — Uniform half-extent (per axis)
//
// Set init_x/init_y/init_yaw far from truth + a tiny init_spread_* to study
// "wrong + over-confident" failure modes; use init_distribution=uniform with
// large extents for global localisation ("kidnapped robot"). With the lidar
// likelihood-field update wired in (see /map and /scan subs below) the PF
// can actually *recover* from a kidnap, instead of just dead-reckoning.
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pro_lab_filters/PF.h"
#include "pro_lab_filters/LikelihoodField.h"
#include "pro_lab_filters/common.hpp"

using namespace std::chrono_literals;

class PFNode : public rclcpp::Node {
public:
  PFNode() : Node("pf_node") {
    // Init pose / spread
    declare_parameter("init_distribution", std::string("gaussian"));
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_spread_xy", 0.3);
    declare_parameter("init_spread_yaw", 0.2);
    declare_parameter("init_extent_xy", 5.0);
    declare_parameter("init_extent_yaw", 3.14159265);
    // Filter knobs
    declare_parameter("num_particles", 500);
    declare_parameter("motion_noise_v", 0.05);
    declare_parameter("motion_noise_w", 0.05);
    declare_parameter("r_yaw_imu", 0.05);
    declare_parameter("frame_id", std::string("odom"));
    declare_parameter("publish_particles", true);
    declare_parameter("rng_seed", 42);
    // Lidar likelihood-field knobs
    declare_parameter("scan_subsample",  6);     // every Nth beam (~60 beams ≈ AMCL default)
    declare_parameter("scan_sigma",      0.20);  // AMCL: sigma_hit=0.2
    declare_parameter("scan_w_rand",     0.50);  // AMCL: z_rand=0.5 — outlier robustness
    declare_parameter("scan_max_range",  10.0);
    declare_parameter("scan_min_range",  0.10);
    // Motion-gating thresholds (AMCL: update_min_d=0.25, update_min_a=0.2):
    // skip scan update unless robot has actually moved this much. This is
    // the *main* anti-jitter trick — without it, repeated reweighting on
    // identical stationary scans makes the estimate dance.
    declare_parameter("update_min_d",    0.25);
    declare_parameter("update_min_a",    0.20);
    declare_parameter("base_frame",      std::string("base_footprint"));
    // Velocity motion model alpha1..6 (Thrun §5.3 / AMCL alpha1..5).
    // 0.0 → fall back to simple Gaussian on (v, w).
    declare_parameter("alpha1", 0.2);
    declare_parameter("alpha2", 0.2);
    declare_parameter("alpha3", 0.2);
    declare_parameter("alpha4", 0.2);
    declare_parameter("alpha5", 0.2);
    declare_parameter("alpha6", 0.2);
    // Augmented-MCL random-particle injection. OFF by default — only useful
    // for global/kidnap recovery; during normal driving it scatters particles
    // across the map and the estimate diverges. Enable in kidnapped.yaml.
    declare_parameter("use_augmented_mcl", false);
    // w_slow / w_fast decay rates (Thrun §8.3.3). Thrun's canonical values
    // (0.001 / 0.1) assume a ~10 Hz laser over thousands of updates; our PF
    // only sees ~1.3 scan updates per second (~40 per run), so the averages
    // must adapt ~50x faster or w_slow stays frozen at its (terrible)
    // uniform-init value and the injection probability never goes positive.
    declare_parameter("aug_alpha_slow", 0.05);
    declare_parameter("aug_alpha_fast", 0.50);
    // Beam skip
    declare_parameter("do_beamskip",                false);
    declare_parameter("beam_skip_distance",         0.5);
    declare_parameter("beam_skip_threshold",        0.3);
    declare_parameter("beam_skip_error_threshold",  0.9);
    // KLD-sampling adaptive particle count
    declare_parameter("use_kld",          false);
    declare_parameter("kld_n_min",        100);
    declare_parameter("kld_n_max",        5000);
    declare_parameter("kld_epsilon",      0.05);
    declare_parameter("kld_z",            2.33);
    declare_parameter("kld_bin_xy",       0.5);
    declare_parameter("kld_bin_yaw",      0.17);

    Eigen::Vector3d mean(
      get_parameter("init_x").as_double(),
      get_parameter("init_y").as_double(),
      get_parameter("init_yaw").as_double());
    auto N = static_cast<std::size_t>(get_parameter("num_particles").as_int());
    auto seed = static_cast<std::uint64_t>(get_parameter("rng_seed").as_int());
    double mnv = get_parameter("motion_noise_v").as_double();
    double mnw = get_parameter("motion_noise_w").as_double();

    auto dist = get_parameter("init_distribution").as_string();
    if (dist == "uniform") {
      Eigen::Vector3d half(
        get_parameter("init_extent_xy").as_double(),
        get_parameter("init_extent_xy").as_double(),
        get_parameter("init_extent_yaw").as_double());
      pf_.initUniform(mean, half, N, mnv, mnw, seed);
      RCLCPP_INFO(get_logger(),
        "PF init: UNIFORM, center=(%.2f, %.2f, %.2f rad), half_extent=(%.2f, %.2f, %.2f), N=%zu",
        mean(0), mean(1), mean(2), half(0), half(1), half(2), N);
    } else {
      Eigen::Vector3d spread(
        get_parameter("init_spread_xy").as_double(),
        get_parameter("init_spread_xy").as_double(),
        get_parameter("init_spread_yaw").as_double());
      pf_.init(mean, spread, N, mnv, mnw, seed);
      RCLCPP_INFO(get_logger(),
        "PF init: GAUSSIAN, mean=(%.2f, %.2f, %.2f rad), spread=(%.2f, %.2f, %.2f), N=%zu",
        mean(0), mean(1), mean(2), spread(0), spread(1), spread(2), N);
    }

    // ── AMCL-style refinements ────────────────────────────────────────
    pf_.setMotionAlphas(
        get_parameter("alpha1").as_double(), get_parameter("alpha2").as_double(),
        get_parameter("alpha3").as_double(), get_parameter("alpha4").as_double(),
        get_parameter("alpha5").as_double(), get_parameter("alpha6").as_double());
    pf_.enableAugmentedMcl(get_parameter("use_augmented_mcl").as_bool());
    pf_.setAugmentedMcl(get_parameter("aug_alpha_slow").as_double(),
                        get_parameter("aug_alpha_fast").as_double());
    if (get_parameter("do_beamskip").as_bool()) {
      pf_.setBeamSkip(true,
        get_parameter("beam_skip_distance").as_double(),
        get_parameter("beam_skip_threshold").as_double(),
        get_parameter("beam_skip_error_threshold").as_double());
    }
    if (get_parameter("use_kld").as_bool()) {
      pf_.setKld(
        static_cast<std::size_t>(get_parameter("kld_n_min").as_int()),
        static_cast<std::size_t>(get_parameter("kld_n_max").as_int()),
        get_parameter("kld_epsilon").as_double(),
        get_parameter("kld_z").as_double(),
        get_parameter("kld_bin_xy").as_double(),
        get_parameter("kld_bin_yaw").as_double());
      RCLCPP_INFO(get_logger(), "PF: KLD-sampling enabled, N in [%ld, %ld]",
        get_parameter("kld_n_min").as_int(),
        get_parameter("kld_n_max").as_int());
    }

    r_yaw_imu_ = get_parameter("r_yaw_imu").as_double();
    frame_id_  = get_parameter("frame_id").as_string();
    publish_particles_ = get_parameter("publish_particles").as_bool();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        v_ = m->linear.x; w_ = m->angular.z;
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 20,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        pf_.updateImuYaw(pro_lab_filters::quat_to_yaw(m->orientation), r_yaw_imu_);
      });
    // AMCL-style global reinit. RViz's built-in "2D Pose Estimate" tool
    // publishes here. This ONLY re-seeds the PF — it does not move the GZ
    // model. The robot_teleporter listens on /kidnap_pose instead, so the
    // two experiments stay independent.
    // Spread is taken from the message covariance (sigma = sqrt(cov[0])).
    rclcpp::QoS init_qos(10);
    init_qos.reliable().transient_local();
    initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", init_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        Eigen::Vector3d mean(m->pose.pose.position.x,
                             m->pose.pose.position.y,
                             pro_lab_filters::quat_to_yaw(m->pose.pose.orientation));
        double sxy  = std::sqrt(std::max(m->pose.covariance[0],  0.05));
        double syaw = std::sqrt(std::max(m->pose.covariance[35], 0.05));
        Eigen::Vector3d spread(sxy, sxy, syaw);
        auto N = static_cast<std::size_t>(get_parameter("num_particles").as_int());
        double mnv = get_parameter("motion_noise_v").as_double();
        double mnw = get_parameter("motion_noise_w").as_double();
        auto seed = static_cast<std::uint64_t>(this->now().nanoseconds());
        pf_.init(mean, spread, N, mnv, mnw, seed);
        RCLCPP_INFO(get_logger(),
          "PF reinit from /initialpose: mean=(%.2f, %.2f, %.2f rad), spread=(%.2f, %.2f, %.2f)",
          mean(0), mean(1), mean(2), spread(0), spread(1), spread(2));
      });

    // ── Likelihood-field scan update ───────────────────────────────────
    scan_subsample_ = static_cast<int>(get_parameter("scan_subsample").as_int());
    scan_sigma_     = get_parameter("scan_sigma").as_double();
    scan_w_rand_    = get_parameter("scan_w_rand").as_double();
    scan_max_range_ = get_parameter("scan_max_range").as_double();
    scan_min_range_ = get_parameter("scan_min_range").as_double();
    update_min_d_   = get_parameter("update_min_d").as_double();
    update_min_a_   = get_parameter("update_min_a").as_double();
    base_frame_     = get_parameter("base_frame").as_string();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        lf_.build(m->info.width, m->info.height, m->info.resolution,
                  m->info.origin.position.x, m->info.origin.position.y,
                  m->data.data());
        // Hand the same bounds to the PF so augmented-MCL knows where to
        // inject random particles for kidnap recovery.
        const double xmin = m->info.origin.position.x;
        const double ymin = m->info.origin.position.y;
        const double xmax = xmin + m->info.width  * m->info.resolution;
        const double ymax = ymin + m->info.height * m->info.resolution;
        pf_.setMapBounds(xmin, ymin, xmax, ymax);
        RCLCPP_INFO(get_logger(),
          "PF likelihood-field built: %dx%d @ %.3fm world=(%.1f..%.1f, %.1f..%.1f)",
          m->info.width, m->info.height, m->info.resolution,
          xmin, xmax, ymin, ymax);
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });

    runtime_pub_  = create_publisher<std_msgs::msg::Float64>("/pf/runtime_us", 10);
    pose_pub_     = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf/pose", 10);
    if (publish_particles_) {
      particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/pf/particles", 10);
    }
    ess_pub_      = create_publisher<std_msgs::msg::Float64>("/pf/ess", 10);
    timer_        = create_wall_timer(50ms, [this]() { tick(); });
  }

private:
  // Convert a LaserScan into beam endpoints (x, y) in the robot base frame.
  // Looks up the static TF base_frame ← scan->header.frame_id once and
  // caches it.
  void onScan(const sensor_msgs::msg::LaserScan & m) {
    if (!lf_.valid()) {
      return;
    }
    if (!have_lidar_tf_) {
      try {
        auto tf = tf_buffer_->lookupTransform(
            base_frame_, m.header.frame_id, tf2::TimePointZero);
        lidar_x_   = tf.transform.translation.x;
        lidar_y_   = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        lidar_yaw_     = y;
        have_lidar_tf_ = true;
        RCLCPP_INFO(get_logger(),
          "PF lidar TF cached: %s -> %s = (%.3f, %.3f, yaw=%.3f rad)",
          base_frame_.c_str(), m.header.frame_id.c_str(),
          lidar_x_, lidar_y_, lidar_yaw_);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "PF: waiting for TF %s -> %s: %s",
          base_frame_.c_str(), m.header.frame_id.c_str(), ex.what());
        return;
      }
    }

    // Build subsampled (x, y) endpoints in base frame.
    std::vector<Eigen::Vector2d> beams;
    beams.reserve(m.ranges.size() / std::max(1, scan_subsample_) + 1);
    const double cs_l = std::cos(lidar_yaw_);
    const double sn_l = std::sin(lidar_yaw_);
    for (std::size_t i = 0; i < m.ranges.size(); i += scan_subsample_) {
      const double r = m.ranges[i];
      if (!std::isfinite(r) || r < scan_min_range_ || r > scan_max_range_) {
        continue;
      }
      const double a = m.angle_min + i * m.angle_increment;
      // Endpoint in lidar frame
      const double lx = r * std::cos(a);
      const double ly = r * std::sin(a);
      // Rotate + translate into base frame
      const double bx = lidar_x_ + cs_l * lx - sn_l * ly;
      const double by = lidar_y_ + sn_l * lx + cs_l * ly;
      beams.emplace_back(bx, by);
    }
    if (beams.size() < 8) {
      return;  // degenerate scan
    }

    // Motion gating: only re-weight particles if the *estimated* robot
    // pose has shifted by update_min_d_ / update_min_a_ since last scan
    // update. AMCL does this against the odometry pose; we use the PF
    // estimate which is equivalent once the filter is converged. While
    // the robot is stationary we still let augmented-MCL run though, so
    // a kidnap can be detected even before any motion happens.
    auto est = pf_.estimate();
    bool moved_enough = !have_last_update_pose_;
    if (have_last_update_pose_) {
      const double dx = est(0) - last_update_x_;
      const double dy = est(1) - last_update_y_;
      double dyaw = est(2) - last_update_yaw_;
      while (dyaw >  M_PI) dyaw -= 2 * M_PI;
      while (dyaw < -M_PI) dyaw += 2 * M_PI;
      moved_enough = (std::hypot(dx, dy) >= update_min_d_) ||
                     (std::abs(dyaw)     >= update_min_a_);
    }
    // Also force occasional updates so scan-likelihood / augmented-MCL
    // can react to a kidnap that happens while we're standing still.
    auto now_t = this->now();
    bool periodic_due = !have_last_update_pose_ ||
                        (now_t - last_update_t_).seconds() > 0.5;

    if (!moved_enough && !periodic_due) {
      return;
    }

    pf_.updateScan(beams, lf_, scan_sigma_, scan_w_rand_);
    last_update_x_   = est(0);
    last_update_y_   = est(1);
    last_update_yaw_ = est(2);
    last_update_t_   = now_t;
    have_last_update_pose_ = true;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "PF scan: beams=%zu  ess=%.0f  inject_prob=%.2f",
      beams.size(), pf_.ess(), pf_.injectProbability());
  }

  void tick() {
    auto now = this->now();
    double t = now.seconds();
    if (!have_last_) { last_t_ = t; have_last_ = true; return; }
    double dt = t - last_t_;
    last_t_ = t;
    const auto t_start = std::chrono::steady_clock::now();
    if (dt > 0) pf_.predict(v_, w_, dt);

    auto x    = pf_.estimate();
    auto C    = pf_.covarianceXY();
    auto vyaw = pf_.variance()(2);
    pose_pub_->publish(pro_lab_filters::make_pose_xy(now, frame_id_,
        x(0), x(1), x(2), C(0, 0), C(1, 1), C(0, 1), vyaw));

    if (particles_pub_) {
      geometry_msgs::msg::PoseArray pa;
      pa.header.stamp = now;
      pa.header.frame_id = frame_id_;
      const auto& ps = pf_.particles();
      pa.poses.reserve(ps.size());
      for (const auto& p : ps) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p(0);
        pose.position.y = p(1);
        pose.position.z = 0.0;
        pose.orientation = pro_lab_filters::yaw_to_quat(p(2));
        pa.poses.push_back(pose);
      }
      particles_pub_->publish(pa);
    }

    std_msgs::msg::Float64 ess_msg;
    ess_msg.data = pf_.ess();
    ess_pub_->publish(ess_msg);

    const auto t_end = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 rt;
    rt.data = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    runtime_pub_->publish(rt);
  }

  pro_lab_filters::PF pf_;
  double v_ = 0.0, w_ = 0.0;
  double last_t_ = 0.0;
  bool have_last_ = false;
  double r_yaw_imu_ = 0.05;
  std::string frame_id_;
  bool publish_particles_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ess_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr runtime_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Lidar / map likelihood
  pro_lab_filters::LikelihoodField                 lf_;
  std::shared_ptr<tf2_ros::Buffer>                 tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>      tf_listener_;
  bool          have_lidar_tf_ {false};
  double        lidar_x_ {0.0}, lidar_y_ {0.0}, lidar_yaw_ {0.0};
  std::string   base_frame_;
  int           scan_subsample_ {10};
  double        scan_sigma_     {0.20};
  double        scan_w_rand_    {0.50};
  double        scan_max_range_ {10.0};
  double        scan_min_range_ {0.10};
  double        update_min_d_   {0.25};
  double        update_min_a_   {0.20};
  bool          have_last_update_pose_ {false};
  double        last_update_x_   {0.0};
  double        last_update_y_   {0.0};
  double        last_update_yaw_ {0.0};
  rclcpp::Time  last_update_t_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFNode>());
  rclcpp::shutdown();
  return 0;
}
