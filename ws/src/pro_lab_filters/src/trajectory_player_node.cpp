// Scripted /cmd_vel sequence player. Drives a deterministic path so every
// filter run sees the same trajectory — a smooth S-curve (two opposite arcs)
// followed by a tight sharp curve. Publishes to /cmd_vel_in (the watchdog
// forwards it to /cmd_vel and zeroes after we stop). When the sequence
// completes, /trajectory/done (Bool=true) is latched once so the orchestrator
// (launch auto-shutdown / external runner) can react.
//
// Parameters:
//   start_delay_s   double  default 12.0  — wait before first segment so
//                                            Gazebo + filters are warm
//   loop            bool    default false — replay forever (debug)
//   speed_scale     double  default 1.0   — multiply all v/w
//   publish_hz      double  default 20.0
//   topic_in        string  default "/cmd_vel_in"
//   topic_done      string  default "/trajectory/done"
//
// Sequence (each row = duration_s, v, w), repeated 4×:
//   straight leg   5.00   0.30   0.00   (1.5 m)
//   turn 90° left  3.14   0.00   0.50   (≈ π/2)
// then a 1 s final stop. Total ≈ 33 s; wrap in 60 s duration_s for settling.

#include <chrono>
#include <cmath>      // M_PI for exact π/6 pivot rate
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

struct Segment {
  double duration_s;
  double v;
  double w;
  const char* label;
};

// In-place pivots (v == 0, w != 0) are executed CLOSED-LOOP on the IMU yaw
// rather than open-loop on time. Reason: the gz DiffDrive plugin ramps the
// angular rate (max_angular_acceleration = 3 rad/s²) and the wheels slip a
// few percent during pure rotation, so "3 s × π/6 rad/s" lands at ~85°, not
// 90° — and the north leg then visibly leans in every trajectory plot. The
// IMU is a real onboard sensor (the filters consume it too), so feedback on
// it stays faithful to the no-ground-truth rule. Timed segments are
// unaffected; a pivot ends when |yaw - yaw_at_pivot_start| reaches
// |w| · duration_s (the intended angle), with a duration × 1.8 timeout as a
// safety net.
static bool is_pivot(const Segment& s) { return s.v == 0.0 && s.w != 0.0; }

// S-curve followed by a sharp curve. The two opposite smooth arcs (the "S")
// test how each filter tracks gentle continuous turning; the final tight,
// high-curvature turn stresses the nonlinearity — where the linear CV KF lags
// most and the EKF/PF should win. Same path every run (deterministic).
// Kept compact (~1.5 m extent around spawn) so it stays in the open depot
// area and clear of shelves/landmark posts.
// 3-point-turn path from world origin facing east (+x). Designed to expose
// each filter's classical weaknesses in distinct phases for the paper:
//   1. S-curve (smooth nonlinear motion)    — KF (CV) lags; EKF/PF good
//   2. ~90° pivot left  (near in-place)     — pure-rotation stresses ALL
//                                              filters that infer ω from
//                                              landmark deltas
//   3. Reverse leg in the NEW heading (south, not retracing the forward
//      curve)                               — sign flip on v, exposes
//                                              filters that integrate
//                                              speed naively
//   4. ~90° pivot right (in-place)          — second rotation; recovery
//   5. Forward in new direction             — final settle phase
// End pose is laterally offset from start → forward and reverse legs are
// visually separated in the trajectory plot.
// Pillars passed within ~1.5–3 m: (7.5, 0) throughout, plus (-7.5, 0)
// and (7.5, -7.5) within the 10 m lidar range.
static const std::vector<Segment> kPath = {
    {2.0,  0.50,  0.00, "approach_east"},    // 1.0 m east
    {4.0,  0.50,  0.25, "s_curve_a"},        // S — left arc (~57° tilt, drifts N)
    {4.0,  0.50, -0.25, "s_curve_b"},        // S — right arc, back to east
    {3.0,  0.50,  0.00, "drive_east"},       // 1.5 m east — closes on pillar (7.5,0)
    {3.0,  0.00,  M_PI / 6.0, "pivot_left"},  // exactly 90° in-place (3 × π/6 = π/2)
    {6.0,  0.50,  0.00,        "north_leg"},  // 3.0 m north → near pillar (7.5,+7.5)
    {4.0, -0.50,  0.00,        "reverse_south"}, // REVERSE — 2 m south
    {3.0,  0.00, -M_PI / 6.0, "pivot_right"}, // exactly 90° in-place right
    {2.0,  0.50,  0.00, "forward_east"},     // 1.0 m east, settle
    {1.0,  0.00,  0.00, "final_stop"},
};

class TrajectoryPlayer : public rclcpp::Node {
public:
  TrajectoryPlayer() : Node("trajectory_player") {
    declare_parameter("start_delay_s", 12.0);
    declare_parameter("loop", false);
    declare_parameter("speed_scale", 1.0);
    declare_parameter("publish_hz", 20.0);
    declare_parameter("topic_in",   std::string("/cmd_vel_in"));
    declare_parameter("topic_done", std::string("/trajectory/done"));
    declare_parameter("start_x",    -8.0);   // robot spawn — used to integrate
    declare_parameter("start_y",    -0.5);   //   kPath into a world-frame /planned_path
    declare_parameter("start_yaw",   0.0);   //   that RViz can display.
    declare_parameter("planned_frame", std::string("map"));

    start_delay_ = get_parameter("start_delay_s").as_double();
    loop_        = get_parameter("loop").as_bool();
    scale_       = get_parameter("speed_scale").as_double();
    const double hz = get_parameter("publish_hz").as_double();
    const auto topic_in   = get_parameter("topic_in").as_string();
    const auto topic_done = get_parameter("topic_done").as_string();

    cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>(topic_in, 10);
    // IMU yaw for closed-loop pivots (see is_pivot above).
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20,
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
          const auto& q = m->orientation;
          const double siny = 2.0 * (q.w * q.z + q.x * q.y);
          const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
          imu_yaw_ = std::atan2(siny, cosy);
          have_imu_ = true;
        });
    // Latched-style done topic (transient_local QoS) so late subscribers see
    // the final state. Useful when an outer script polls after launch.
    rclcpp::QoS done_qos(1);
    done_qos.transient_local();
    done_pub_ = create_publisher<std_msgs::msg::Bool>(topic_done, done_qos);

    // Pre-integrate kPath from the spawn pose and publish a latched
    // nav_msgs/Path so RViz can show the full planned trajectory (thick blue
    // future-line) alongside the truth/filter paths the robot accumulates.
    publishPlannedPath(
        get_parameter("start_x").as_double(),
        get_parameter("start_y").as_double(),
        get_parameter("start_yaw").as_double(),
        get_parameter("planned_frame").as_string());

    // t_start_ is captured on the first tick that sees a valid sim clock —
    // NOT here. With use_sim_time the constructor often runs before /clock
    // arrives (this->now() == 0); if a gz server from a previous run is still
    // publishing a high sim time, now-t_start_ jumps to hundreds of seconds and
    // the whole path is skipped ("trajectory complete (656s)" → robot never
    // moves). Deferring the start to the first real tick makes it robust.
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "trajectory_player: %zu segments, total %.1fs (after %.1fs delay)",
                kPath.size(), totalDuration(), start_delay_);
  }

private:
  static double totalDuration() {
    double t = 0.0;
    for (const auto& s : kPath) t += s.duration_s;
    return t;
  }

  // Wrap angle difference to (-π, π].
  static double angDiff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  void tick() {
    // Capture the start on the first tick with a valid (non-zero) sim clock.
    if (!started_) {
      const auto t = this->now();
      if (t.nanoseconds() == 0) { publishStop(); return; }  // /clock not up yet
      t_start_ = t;
      started_ = true;
    }
    const double now = (this->now() - t_start_).seconds();
    if (now < start_delay_) {
      publishStop();
      return;
    }

    if (done_) {
      publishStop();
      return;
    }

    // Stateful segment machine: seg_idx_ advances when the current segment
    // finishes. Timed segments finish after duration_s; pivot segments
    // (v==0, w!=0) finish when the IMU yaw delta reaches the intended angle
    // |w|·duration_s — robust against DiffDrive accel ramps + wheel slip
    // that make open-loop pivots undershoot by ~5°.
    if (seg_idx_ >= kPath.size()) {
      finishOrLoop(now);
      return;
    }
    const auto& s = kPath[seg_idx_];

    // First tick inside this segment: capture its start time + start yaw.
    if (!seg_entered_) {
      seg_entered_   = true;
      seg_start_t_   = now;
      seg_start_yaw_ = imu_yaw_;
      seg_yaw_acc_   = 0.0;
      seg_last_yaw_  = imu_yaw_;
      RCLCPP_INFO(get_logger(), "[%6.1fs] segment: %s (v=%.2f, w=%.2f)%s",
                  now - start_delay_, s.label, s.v * scale_, s.w * scale_,
                  is_pivot(s) && have_imu_ ? " [IMU-closed-loop]" : "");
    }
    const double seg_t = now - seg_start_t_;

    bool seg_done;
    if (is_pivot(s) && have_imu_) {
      // Accumulate yaw progress incrementally (handles ±π wrap mid-pivot).
      seg_yaw_acc_ += std::abs(angDiff(imu_yaw_, seg_last_yaw_));
      seg_last_yaw_ = imu_yaw_;
      const double target = std::abs(s.w) * s.duration_s * scale_;
      // Stop slightly early: the DiffDrive decelerates ω at 3 rad/s², which
      // adds ω²/(2·3) rad of overshoot after we command w=0.
      const double brake = (s.w * scale_) * (s.w * scale_) / (2.0 * 3.0);
      seg_done = (seg_yaw_acc_ >= target - brake)
                 || (seg_t > s.duration_s * 1.8);   // safety timeout
    } else {
      seg_done = seg_t >= s.duration_s;
    }

    if (seg_done) {
      if (is_pivot(s) && have_imu_) {
        RCLCPP_INFO(get_logger(), "pivot '%s' done: |Δyaw|=%.1f° in %.2fs",
                    s.label, seg_yaw_acc_ * 180.0 / M_PI, seg_t);
      }
      seg_idx_ += 1;
      seg_entered_ = false;
      publishStop();   // one stop tick between segments settles the ramp
      return;
    }

    publish(s.v * scale_, s.w * scale_);
  }

  void finishOrLoop(double now) {
    if (loop_) {
      t_start_ = this->now() - rclcpp::Duration::from_seconds(start_delay_);
      seg_idx_ = 0;
      seg_entered_ = false;
      RCLCPP_INFO(get_logger(), "looping back to segment 0");
      return;
    }
    publishStop();
    if (!done_) {
      done_ = true;
      std_msgs::msg::Bool m; m.data = true;
      done_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "trajectory complete (%.1fs)", now - start_delay_);
    }
  }

  void publishPlannedPath(double x, double y, double yaw, const std::string& frame) {
    rclcpp::QoS qos(1);
    qos.transient_local();           // latched: late subscribers (RViz) still get it
    planned_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", qos);
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame;
    msg.header.stamp = this->now();
    const double dt = 0.05;          // 50 ms integration step
    auto push = [&](double px, double py, double pyaw) {
      geometry_msgs::msg::PoseStamped p;
      p.header = msg.header;
      p.pose.position.x = px; p.pose.position.y = py;
      p.pose.orientation.z = std::sin(pyaw * 0.5);
      p.pose.orientation.w = std::cos(pyaw * 0.5);
      msg.poses.push_back(p);
    };
    push(x, y, yaw);
    for (const auto& s : kPath) {
      const int n = std::max(1, static_cast<int>(std::ceil(s.duration_s / dt)));
      const double step = s.duration_s / n;
      for (int i = 0; i < n; ++i) {
        x   += s.v * std::cos(yaw) * step;
        y   += s.v * std::sin(yaw) * step;
        yaw += s.w * step;
        push(x, y, yaw);
      }
    }
    planned_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "trajectory_player: published /planned_path with %zu poses",
                msg.poses.size());
  }

  void publish(double v, double w) {
    geometry_msgs::msg::Twist m;
    m.linear.x  = v;
    m.angular.z = w;
    cmd_pub_->publish(m);
  }
  void publishStop() { publish(0.0, 0.0); }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       done_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       planned_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time t_start_;
  double start_delay_ = 12.0;
  double scale_       = 1.0;
  bool   loop_        = false;
  bool   done_        = false;
  bool   started_     = false;

  // segment state machine
  std::size_t seg_idx_     = 0;
  bool   seg_entered_      = false;
  double seg_start_t_      = 0.0;
  double seg_start_yaw_    = 0.0;
  double seg_yaw_acc_      = 0.0;   // |Δyaw| integrated over the pivot
  double seg_last_yaw_     = 0.0;

  // IMU feedback
  double imu_yaw_  = 0.0;
  bool   have_imu_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlayer>());
  rclcpp::shutdown();
  return 0;
}
