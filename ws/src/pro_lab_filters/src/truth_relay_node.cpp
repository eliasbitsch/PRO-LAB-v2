// Resolves the robot's true world pose and republishes it as PoseStamped on
// /ground_truth/pose @ publish_hz. Used by metrics_node as the reference.
//
// Two source modes:
//
// 1) `gz_pose_topic` is set (preferred for the wrong-init study)
//      Subscribes to a TFMessage of /world/<world>/pose/info bridged from
//      Gazebo, picks the Transform whose child_frame_id matches `model_name`
//      (default "turtlebot4"), and publishes that pose directly.
//      This is the SIMULATOR'S OWN ground truth, completely independent of
//      AMCL - the kidnapped scenario needs this because the previous
//      target=map mode reads the AMCL-published map->odom transform, which
//      flails during global localisation and made the truth jump wildly.
//
// 2) Otherwise: legacy TF lookup `target_frame -> source_frame` (defaults
//      "odom" -> "base_footprint", works for any tb4 spawn).
//
// Parameters:
//   gz_pose_topic   default ""  - empty disables mode 1 and falls back to TF
//   model_name      default "turtlebot4"
//   target_frame    default "odom"   (mode 2 only)
//   source_frame    default "base_footprint"  (mode 2 only)
//   publish_hz      default 20.0  (mode 2 only; mode 1 republishes on receive)
//   topic           default "/ground_truth/pose"
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class TruthRelay : public rclcpp::Node {
public:
  TruthRelay() : Node("truth_relay") {
    declare_parameter("target_frame", std::string("odom"));
    declare_parameter("source_frame", std::string("base_footprint"));
    declare_parameter("publish_hz", 20.0);
    declare_parameter("topic", std::string("/ground_truth/pose"));
    declare_parameter("gz_pose_topic", std::string(""));
    declare_parameter("model_name",    std::string("turtlebot4"));
    declare_parameter("publish_frame", std::string("map"));

    target_ = get_parameter("target_frame").as_string();
    source_ = get_parameter("source_frame").as_string();
    double hz = get_parameter("publish_hz").as_double();
    auto topic         = get_parameter("topic").as_string();
    auto gz_pose_topic = get_parameter("gz_pose_topic").as_string();
    model_name_        = get_parameter("model_name").as_string();
    publish_frame_     = get_parameter("publish_frame").as_string();

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    // Accumulated driven path as a line for RViz (Path display on this topic).
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/ground_truth/path", 10);
    path_.header.frame_id = publish_frame_;

    if (!gz_pose_topic.empty()) {
      // Mode 1: subscribe to bridged Gazebo pose/info and pull the entity
      // matching `model_name` out of every Pose_V tick.
      gz_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
          gz_pose_topic, 50,
          [this](tf2_msgs::msg::TFMessage::SharedPtr m) { on_gz_pose(*m); });
      RCLCPP_INFO(get_logger(),
          "truth_relay (gz mode): %s, model='%s' -> %s in '%s'",
          gz_pose_topic.c_str(), model_name_.c_str(),
          topic.c_str(), publish_frame_.c_str());
    } else {
      // Mode 2: TF lookup (legacy behaviour).
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz)));
      timer_ = create_wall_timer(period, [this]() { tick(); });
      RCLCPP_INFO(get_logger(),
          "truth_relay (tf mode): %s <- %s @ %.1f Hz -> %s",
          target_.c_str(), source_.c_str(), hz, topic.c_str());
    }
  }

private:
  void tick() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_, source_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      // TF not yet available (sim still spinning up): silent throttle.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "no TF %s -> %s yet: %s", target_.c_str(), source_.c_str(), ex.what());
      return;
    }
    geometry_msgs::msg::PoseStamped p;
    p.header = tf.header;
    p.pose.position.x = tf.transform.translation.x;
    p.pose.position.y = tf.transform.translation.y;
    p.pose.position.z = tf.transform.translation.z;
    p.pose.orientation = tf.transform.rotation;
    pub_->publish(p);

    // Append to the running path (only when the robot has actually moved, to
    // keep the line compact) and republish for the RViz Path display.
    if (path_.poses.empty() ||
        std::hypot(p.pose.position.x - path_.poses.back().pose.position.x,
                   p.pose.position.y - path_.poses.back().pose.position.y) > 0.02) {
      path_.poses.push_back(p);
    }
    path_.header.stamp = p.header.stamp;
    path_pub_->publish(path_);
  }

  void on_gz_pose(const tf2_msgs::msg::TFMessage& m) {
    // The ros_gz_bridge drops the entity name when converting Pose_V to
    // TFMessage (child_frame_id arrives as ""). So we cannot match by name
    // - we either use a dynamic_pose topic (only moving entities, robot
    // is reliably the first) or we accept ANY transform if model_name
    // is empty. If model_name is set AND we find a non-empty
    // child_frame_id match we still prefer that.
    const geometry_msgs::msg::TransformStamped* match = nullptr;
    if (!model_name_.empty()) {
      for (const auto& tf : m.transforms) {
        if (!tf.child_frame_id.empty()
            && tf.child_frame_id.find(model_name_) != std::string::npos) {
          match = &tf;
          break;
        }
      }
    }
    if (match == nullptr && !m.transforms.empty()) {
      match = &m.transforms.front();
    }
    if (match == nullptr) return;

    geometry_msgs::msg::PoseStamped p;
    // dynamic_pose stamps arrive as 0/0 from the bridge - stamp with
    // current sim time instead so downstream consumers (metrics, csv_logger)
    // can compute deltas.
    p.header.stamp = now();
    p.header.frame_id = publish_frame_;
    p.pose.position.x = match->transform.translation.x;
    p.pose.position.y = match->transform.translation.y;
    p.pose.position.z = match->transform.translation.z;
    p.pose.orientation = match->transform.rotation;
    pub_->publish(p);

    if (path_.poses.empty() ||
        std::hypot(p.pose.position.x - path_.poses.back().pose.position.x,
                   p.pose.position.y - path_.poses.back().pose.position.y) > 0.02) {
      path_.poses.push_back(p);
    }
    path_.header.stamp = p.header.stamp;
    path_pub_->publish(path_);
  }

  std::string target_, source_, model_name_, publish_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gz_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TruthRelay>());
  rclcpp::shutdown();
  return 0;
}
