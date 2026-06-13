// Ground-truth relay using gz transport DIRECTLY (not via ros_gz_bridge).
//
// Why: the bridge converts gz.msgs.Pose_V to tf2_msgs/TFMessage and DROPS
// the entity name in the process, so we can't filter for the robot. The
// dynamic_pose/info topic lists every moving entity (chairs, robot, …),
// and the robot is not reliably first in the list - picking by index gave
// us a chair pose as "truth", which made the kidnapped scenario unusable.
//
// This node subscribes to /world/<world>/dynamic_pose/info via gz transport,
// finds the pose whose `name` field matches `model_name`, and republishes it
// as geometry_msgs/PoseStamped on /ground_truth/pose. The pose is stamped
// with the current ROS sim time so downstream nodes (metrics, csv_logger)
// can compute deltas.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class GzTruthRelay : public rclcpp::Node {
public:
  GzTruthRelay() : Node("gz_truth_relay") {
    declare_parameter("world",        std::string("warehouse"));
    declare_parameter("model_name",   std::string("turtlebot4"));
    declare_parameter("frame_id",     std::string("map"));
    declare_parameter("topic",        std::string("/ground_truth/pose"));
    declare_parameter("path_topic",   std::string("/ground_truth/path"));

    world_      = get_parameter("world").as_string();
    model_name_ = get_parameter("model_name").as_string();
    frame_id_   = get_parameter("frame_id").as_string();
    auto topic      = get_parameter("topic").as_string();
    auto path_topic = get_parameter("path_topic").as_string();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    path_.header.frame_id = frame_id_;

    const std::string gz_topic =
        "/world/" + world_ + "/dynamic_pose/info";
    if (!gz_node_.Subscribe(gz_topic, &GzTruthRelay::on_pose, this)) {
      RCLCPP_ERROR(get_logger(),
          "failed to subscribe to %s via gz transport", gz_topic.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
          "gz_truth_relay: subscribed to %s, model='%s' -> %s",
          gz_topic.c_str(), model_name_.c_str(), topic.c_str());
    }
  }

private:
  void on_pose(const gz::msgs::Pose_V& msg) {
    // dynamic_pose carries every moving entity. Linear scan for our model
    // name. Hit rate is once per gz tick (~50 Hz), so this is cheap.
    const gz::msgs::Pose* hit = nullptr;
    for (int i = 0; i < msg.pose_size(); ++i) {
      if (msg.pose(i).name() == model_name_) {
        hit = &msg.pose(i);
        break;
      }
    }
    if (hit == nullptr) return;

    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now();
    p.header.frame_id = frame_id_;
    p.pose.position.x = hit->position().x();
    p.pose.position.y = hit->position().y();
    p.pose.position.z = hit->position().z();
    p.pose.orientation.x = hit->orientation().x();
    p.pose.orientation.y = hit->orientation().y();
    p.pose.orientation.z = hit->orientation().z();
    p.pose.orientation.w = hit->orientation().w();
    pose_pub_->publish(p);

    if (path_.poses.empty() ||
        std::hypot(p.pose.position.x - path_.poses.back().pose.position.x,
                   p.pose.position.y - path_.poses.back().pose.position.y) > 0.02) {
      path_.poses.push_back(p);
    }
    path_.header.stamp = p.header.stamp;
    path_pub_->publish(path_);
  }

  std::string world_, model_name_, frame_id_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzTruthRelay>());
  rclcpp::shutdown();
  return 0;
}
