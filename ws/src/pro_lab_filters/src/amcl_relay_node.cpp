// Relay /amcl_pose -> /amcl/pose so AMCL fits the /<filter>/pose convention
// every other node (metrics_node, csv_logger) already speaks. Replaces the
// topic_tools/relay node, which isn't packaged for ros-jazzy.
//
// C++ port of amcl_relay.py (task: everything except launch files and plotting
// scripts in C++).
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;

class AmclRelay : public rclcpp::Node
{
public:
  AmclRelay() : rclcpp::Node("amcl_relay")
  {
    pub_ = create_publisher<PoseWithCovarianceStamped>("/amcl/pose", 10);
    sub_ = create_subscription<PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](const PoseWithCovarianceStamped & m) { pub_->publish(m); });
  }

private:
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclRelay>());
  rclcpp::shutdown();
  return 0;
}
