// Reproducible "kidnapped robot" publisher: emits /kidnap_pose at scheduled
// sim-time instants so headless batches produce identical kidnap events across
// seeds. Replaces the RViz Kidnap tool (manual click) for paper experiments.
//
// Schedule (param `kidnap_schedule`, flat doubles, stride 4):
//   [t1, x1, y1, yaw1, t2, x2, y2, yaw2, ...]
//   ti = sim-seconds since this node started (after start_delay_s)
// Empty / single-element schedule = no-op (node is launched unconditionally;
// only `kidnapped` sets a non-empty schedule).
//
// C++ port of auto_kidnapper.py.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;

struct KidnapEvent { double t, x, y, yaw; };

class AutoKidnapper : public rclcpp::Node
{
public:
  AutoKidnapper() : rclcpp::Node("auto_kidnapper")
  {
    // Default single-element placeholder fixes the param type to DOUBLE_ARRAY
    // (matches the YAML); a 1-element value is treated as "no schedule".
    auto flat = declare_parameter<std::vector<double>>("kidnap_schedule", {0.0});
    start_delay_s_ = declare_parameter<double>("start_delay_s", 0.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    if (flat.size() == 1) flat.clear();
    if (flat.size() % 4 != 0) {
      RCLCPP_WARN(get_logger(),
        "kidnap_schedule length %zu is not a multiple of 4 (t,x,y,yaw); "
        "dropping the tail", flat.size());
      flat.resize(flat.size() - (flat.size() % 4));
    }
    for (size_t i = 0; i + 3 < flat.size(); i += 4)
      schedule_.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
    std::sort(schedule_.begin(), schedule_.end(),
              [](const KidnapEvent & a, const KidnapEvent & b) { return a.t < b.t; });

    if (schedule_.empty())
      RCLCPP_INFO(get_logger(), "auto_kidnapper: empty schedule — no-op");
    else
      RCLCPP_INFO(get_logger(), "auto_kidnapper: %zu kidnap event(s) scheduled",
                  schedule_.size());

    // robot_teleporter subscribes /kidnap_pose reliable + transient_local.
    rclcpp::QoS qos(1);
    qos.reliable().transient_local().keep_last(1);
    pub_ = create_publisher<PoseWithCovarianceStamped>("/kidnap_pose", qos);

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this]() { tick(); });  // 10 Hz vs sim clock
  }

private:
  void tick()
  {
    if (next_idx_ >= schedule_.size()) return;
    const double now_s = get_clock()->now().seconds();
    if (t0_ < 0.0) { t0_ = now_s; return; }
    const double elapsed = now_s - t0_ - start_delay_s_;
    while (next_idx_ < schedule_.size() && schedule_[next_idx_].t <= elapsed) {
      const auto & e = schedule_[next_idx_];
      publish(e.x, e.y, e.yaw);
      RCLCPP_INFO(get_logger(),
        "kidnap #%zu fired @ t=%.1fs (scheduled %.1fs): -> (%.2f, %.2f, %.2f rad)",
        next_idx_ + 1, elapsed, e.t, e.x, e.y, e.yaw);
      ++next_idx_;
    }
  }

  void publish(double x, double y, double yaw)
  {
    PoseWithCovarianceStamped msg;
    msg.header.stamp.sec = 0;       // "latest available" for teleporter TF
    msg.header.stamp.nanosec = 0;
    msg.header.frame_id = frame_id_;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    pub_->publish(msg);
  }

  std::vector<KidnapEvent> schedule_;
  double start_delay_s_{0.0}, t0_{-1.0};
  size_t next_idx_{0};
  std::string frame_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoKidnapper>());
  rclcpp::shutdown();
  return 0;
}
