// AMCL-style "map -> odom" TF publisher driven by a localiser pose.
//
// AMCL doesn't actually publish "map -> base_link" because that would
// fight robot_state_publisher's "odom -> base_link" chain. Instead it
// computes the correction needed so that:
//
//     map_T_base  ==  map_T_odom * odom_T_base
//
// where `map_T_base` is the localiser estimate and `odom_T_base` comes
// from wheel odometry. Re-arranging:
//
//     map_T_odom  =  map_T_base * inv(odom_T_base)
//
// We subscribe to a PoseWithCovarianceStamped in the "map" frame (the
// chosen filter - KF / EKF / PF - selectable via param) and to the
// current TF tree, and broadcast the resulting "map -> odom" transform.
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class MapOdomTfPublisher : public rclcpp::Node {
public:
  MapOdomTfPublisher() : Node("map_odom_tf_publisher"), tf_buffer_(get_clock()) {
    pose_topic_  = declare_parameter<std::string>("pose_topic", "/pf/pose");
    map_frame_   = declare_parameter<std::string>("map_frame",  "map");
    odom_frame_  = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");
    // When true the source topic is a plain PoseStamped (e.g. /ground_truth/pose
    // for the demo, so the robot model sits on the true pose) instead of a
    // PoseWithCovarianceStamped filter estimate.
    pose_is_stamped_ = declare_parameter<bool>("pose_is_stamped", false);

    listener_    = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    if (pose_is_stamped_) {
      sub_ps_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic_, 10,
          [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
            on_pose(m->pose, m->header.stamp);
          });
    } else {
      sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_topic_, 10,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
            on_pose(m->pose.pose, m->header.stamp);
          });
    }

    RCLCPP_INFO(get_logger(),
                "AMCL-style TF: %s drives %s -> %s (via %s)",
                pose_topic_.c_str(), map_frame_.c_str(),
                odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::Pose & pose,
               const builtin_interfaces::msg::Time & stamp) {
    geometry_msgs::msg::TransformStamped odom_T_base_msg;
    try {
      odom_T_base_msg = tf_buffer_.lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "TF lookup %s -> %s failed: %s",
                           odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    // map_T_base from the localiser (or ground truth)
    tf2::Transform map_T_base;
    map_T_base.setOrigin({pose.position.x, pose.position.y, pose.position.z});
    map_T_base.setRotation({pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w});

    // odom_T_base from TF
    tf2::Transform odom_T_base;
    odom_T_base.setOrigin({odom_T_base_msg.transform.translation.x,
                           odom_T_base_msg.transform.translation.y,
                           odom_T_base_msg.transform.translation.z});
    odom_T_base.setRotation({odom_T_base_msg.transform.rotation.x,
                             odom_T_base_msg.transform.rotation.y,
                             odom_T_base_msg.transform.rotation.z,
                             odom_T_base_msg.transform.rotation.w});

    // map_T_odom = map_T_base * inv(odom_T_base)
    tf2::Transform map_T_odom = map_T_base * odom_T_base.inverse();

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp    = stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id  = odom_frame_;
    out.transform.translation.x = map_T_odom.getOrigin().x();
    out.transform.translation.y = map_T_odom.getOrigin().y();
    out.transform.translation.z = map_T_odom.getOrigin().z();
    auto q = map_T_odom.getRotation();
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
    broadcaster_->sendTransform(out);
  }

  std::string                                                                       pose_topic_;
  std::string                                                                       map_frame_;
  std::string                                                                       odom_frame_;
  std::string                                                                       base_frame_;
  bool                                                                              pose_is_stamped_{false};
  tf2_ros::Buffer                                                                   tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                                       listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                                    broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr    sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                   sub_ps_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
