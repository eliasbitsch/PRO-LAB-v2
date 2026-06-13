// Publish a one-shot /initialpose for AMCL so its lifecycle can leave the
// "Please set the initial pose..." state and start particle filtering.
//
// Reads the same scenario init params (init_x/y/yaw/cov) the other filters
// consume, so AMCL starts from the SAME (possibly wrong) pose as KF/EKF/PF -
// the whole point of the wrong-init study.
//
// C++ port of amcl_init_pose.py. Scheduling is done by polling the (sim) clock
// at 10 Hz instead of nested timers, so it stays correct under use_sim_time.
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using geometry_msgs::msg::PoseWithCovarianceStamped;

class AmclInitPose : public rclcpp::Node
{
public:
  AmclInitPose() : rclcpp::Node("amcl_init_pose")
  {
    init_x_   = declare_parameter<double>("init_x", 0.0);
    init_y_   = declare_parameter<double>("init_y", 0.0);
    init_yaw_ = declare_parameter<double>("init_yaw", 0.0);
    init_cov_ = declare_parameter<double>("init_cov", 0.25);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    // Delay so AMCL is up (lifecycle activates it ~3-5 s after launch); 6 s safe.
    delay_s_  = declare_parameter<double>("delay_s", 6.0);
    // AMCL can drop the pose if its TF cache lags the stamp; resend a few times.
    period_s_ = declare_parameter<double>("period_s", 2.0);
    attempts_ = static_cast<int>(declare_parameter<int>("attempts", 6));

    // AMCL's /initialpose subscription is TRANSIENT_LOCAL - default VOLATILE
    // triggers "incompatible QoS, no messages will be sent".
    rclcpp::QoS qos(1);
    qos.reliable().transient_local().keep_last(1);
    pub_ = create_publisher<PoseWithCovarianceStamped>("/initialpose", qos);

    // 10 Hz poll against the (sim) clock: first publish at delay_s, then every
    // period_s, capped at `attempts`.
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this]() { tick(); });
  }

private:
  void tick()
  {
    const double now_s = get_clock()->now().seconds();
    if (t0_ < 0.0) { t0_ = now_s; return; }
    const double elapsed = now_s - t0_;
    if (elapsed < delay_s_) return;
    if (sent_ >= attempts_) return;
    if (sent_ == 0 || (elapsed - last_sent_) >= period_s_) {
      publishOnce();
      last_sent_ = elapsed;
    }
  }

  void publishOnce()
  {
    PoseWithCovarianceStamped msg;
    // stamp=0 → TF uses "latest available" instead of extrapolating to now.
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    msg.header.frame_id = frame_id_;
    msg.pose.pose.position.x = init_x_;
    msg.pose.pose.position.y = init_y_;
    msg.pose.pose.orientation.z = std::sin(init_yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(init_yaw_ / 2.0);
    msg.pose.covariance[0]  = init_cov_;   // xx
    msg.pose.covariance[7]  = init_cov_;   // yy
    msg.pose.covariance[35] = init_cov_;   // yaw-yaw
    pub_->publish(msg);
    ++sent_;
    RCLCPP_INFO(get_logger(),
      "sent /initialpose [%d/%d]: x=%.2f y=%.2f yaw=%.2f cov=%.2f",
      sent_, attempts_, init_x_, init_y_, init_yaw_, init_cov_);
  }

  double init_x_, init_y_, init_yaw_, init_cov_, delay_s_, period_s_;
  std::string frame_id_;
  int attempts_{6}, sent_{0};
  double t0_{-1.0}, last_sent_{0.0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclInitPose>());
  rclcpp::shutdown();
  return 0;
}
