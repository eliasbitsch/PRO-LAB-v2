// "Kidnap Robot" RViz2 tool - clone of the built-in 2D Pose Estimate that
// publishes on /kidnap_pose instead of /initialpose, so a click teleports
// the GZ model (via robot_teleporter) without re-seeding the PF.
//
// Click-and-drag: click position is the new (x, y), drag direction sets
// the new yaw. Same UX as the standard pose tool.
#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace pro_lab_rviz {

class KidnapTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT
public:
  KidnapTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Node::SharedPtr                                                       node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr   pub_;
};

}  // namespace pro_lab_rviz
