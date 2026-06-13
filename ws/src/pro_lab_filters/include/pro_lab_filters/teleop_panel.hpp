// Directional teleop panel for RViz2 (dpad).
//
// Pie-slice "dpad" with a circular Stop button in the centre. Press-and-hold
// to drive, release (or move outside) to stop. Publishes geometry_msgs/Twist
// on /cmd_vel_in - cmd_vel_watchdog forwards it to /cmd_vel and zeroes after
// a short silence, so the release-to-stop semantic is enforced even if the
// last message gets dropped.
//
// Global keyboard control: arrow keys drive the robot anywhere in RViz
// (qApp event filter), regardless of which RViz panel has focus. Space/S
// stops. The filter ignores keys when a text input widget has focus, so
// typing into RViz config fields still works.
#pragma once

#include <QPaintEvent>
#include <QMouseEvent>
#include <QTimer>
#include <QWidget>

#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace pro_lab_rviz {

// Custom-painted dpad widget. 4 pie slices (forward / back / left / right)
// + a central "stop" disc. Reports the currently-held sector via a
// std::function so the parent panel can drive a Twist publisher.
class TeleopPad : public QWidget {
  Q_OBJECT
public:
  enum Sector { NONE, UP, DOWN, LEFT, RIGHT, STOP };

  explicit TeleopPad(QWidget * parent = nullptr);
  std::function<void(Sector)> onSectorChanged;

protected:
  void paintEvent(QPaintEvent * e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseReleaseEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;

private:
  Sector hitTest(const QPoint & p) const;
  void   setSector(Sector s);

  Sector sector_ {NONE};
};

class TeleopPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit TeleopPanel(QWidget * parent = nullptr);
  ~TeleopPanel() override;
  void onInitialize() override;

protected:
  bool eventFilter(QObject * obj, QEvent * ev) override;

private Q_SLOTS:
  void publishTick();

private:
  void onSector(TeleopPad::Sector s);

  TeleopPad *                                              pad_  {nullptr};
  rclcpp::Node::SharedPtr                                  node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_;
  QTimer *                                                 timer_ {nullptr};
  double v_     {0.0};
  double w_     {0.0};
  double v_max_ {0.25};
  double w_max_ {0.6};
  // Currently-pressed arrow key (0 = none). Tracked so a release event for
  // a different key doesn't accidentally zero a still-held one.
  int    held_key_ {0};
};

}  // namespace pro_lab_rviz
