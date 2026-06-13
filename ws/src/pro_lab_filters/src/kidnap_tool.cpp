#include "pro_lab_filters/kidnap_tool.hpp"

#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QPolygonF>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/arrow.hpp>

namespace {

// Draw a red-orange arrow icon at runtime so we don't have to ship a PNG.
// Mirrors the visual language of "2D Pose Estimate" (a coloured arrow) but
// in a different colour so the two tools are obviously distinct.
QIcon makeKidnapIcon() {
  QPixmap pix(24, 24);
  pix.fill(Qt::transparent);
  QPainter p(&pix);
  p.setRenderHint(QPainter::Antialiasing);
  QColor c("#e64545");          // same red as the panel's stop disc
  QPen pen(c, 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  p.setPen(pen);
  p.drawLine(QPointF(3.5, 17.5), QPointF(16.0, 5.5));   // shaft
  p.setBrush(c);
  QPolygonF head;
  head << QPointF(20.5, 2.5) << QPointF(11.5, 4.5) << QPointF(13.5, 13.5);
  p.drawPolygon(head);
  return QIcon(pix);
}

}  // namespace

namespace pro_lab_rviz {

KidnapTool::KidnapTool() : PoseTool() {
  // Keyboard shortcut "k" - quick to reach next to "p" (Pose Estimate).
  shortcut_key_ = 'k';
}

void KidnapTool::onInitialize() {
  PoseTool::onInitialize();
  setName("Kidnap Robot");
  setIcon(makeKidnapIcon());
  // PoseTool's drag preview arrow is green by default; recolour it red so
  // the user immediately sees this is the kidnap action, not pose-estimate.
  if (arrow_) {
    arrow_->setColor(0.9f, 0.27f, 0.27f, 1.0f);
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node_ = std::make_shared<rclcpp::Node>("rviz_kidnap_tool");

  // Match robot_teleporter's QoS so a click survives any subscriber restart.
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/kidnap_pose", qos);
}

void KidnapTool::onPoseSet(double x, double y, double theta) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp    = node_->now();
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Covariance is unused by robot_teleporter - leave zero.
  pub_->publish(msg);
}

}  // namespace pro_lab_rviz

PLUGINLIB_EXPORT_CLASS(pro_lab_rviz::KidnapTool, rviz_common::Tool)
