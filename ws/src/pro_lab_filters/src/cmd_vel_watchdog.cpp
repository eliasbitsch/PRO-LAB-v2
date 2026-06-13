// cmd_vel watchdog: forwards /cmd_vel_in to /cmd_vel and emits a single
// zero-Twist when the input goes silent for `timeout` seconds.
//
// Key-hold publishers (RViz teleop panel, teleop_twist_keyboard) only emit
// while a key is held. On release they stop publishing - but Gazebo's
// diff-drive plugin keeps applying the last command forever, so the robot
// rolls on. This relay closes that gap.
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class CmdVelWatchdog : public rclcpp::Node {
public:
  CmdVelWatchdog() : Node("cmd_vel_watchdog") {
    timeout_s_   = declare_parameter<double>("timeout", 0.4);
    auto in_top  = declare_parameter<std::string>("input_topic",  "/cmd_vel_in");
    auto out_top = declare_parameter<std::string>("output_topic", "/cmd_vel");

    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_top, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        in_top, 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { on_input(*msg); });

    last_input_ = now();
    timer_ = create_wall_timer(50ms, [this] { tick(); });

    RCLCPP_INFO(get_logger(),
                "watchdog: %s -> %s, stop_after=%.2fs",
                in_top.c_str(), out_top.c_str(), timeout_s_);
  }

private:
  void on_input(const geometry_msgs::msg::Twist & msg) {
    last_input_ = now();
    zero_sent_  = false;
    pub_->publish(msg);
  }

  void tick() {
    if (!zero_sent_ &&
        (now() - last_input_).seconds() >= timeout_s_) {
      pub_->publish(geometry_msgs::msg::Twist{});
      zero_sent_ = true;
    }
  }

  double                                                            timeout_s_;
  bool                                                              zero_sent_{false};
  rclcpp::Time                                                      last_input_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
