// Subscribes to ground truth + each filter's pose, publishes per-filter
// instantaneous error and running RMSE, plus a convergence flag.
//
// Inputs (subscribed):
//   /ground_truth/pose     PoseStamped (truth, from truth_relay)
//   /kf/pose, /ekf/pose, /pf/pose  PoseWithCovarianceStamped (estimates)
//
// Outputs (published, per filter "<f>"):
//   /metrics/<f>/error_xy        Float64 (m)
//   /metrics/<f>/error_yaw       Float64 (rad)
//   /metrics/<f>/rmse_xy         Float64 (m, cumulative)
//   /metrics/<f>/rmse_yaw        Float64 (rad, cumulative)
//   /metrics/<f>/converged       Bool   (true once error_xy stays below
//                                        `convergence_threshold_xy` for
//                                        `convergence_window_s` seconds)
//   /metrics/<f>/time_to_converge  Float64 (s, set once on first convergence)
//   /metrics/<f>/nees           Float64  Normalized Estimation Error Squared
//                                         e^T · P^-1 · e for the published
//                                         covariance. χ²-consistent filter
//                                         tracks the state-dim mean (≈3 here).
//                                         A persistent NEES >> 3 means the
//                                         filter is overconfident - the
//                                         classic wrong-init failure mode.
//
// Parameters:
//   convergence_threshold_xy  default 0.20 m
//   convergence_window_s      default 2.0 s
//   filters                   default ["kf","ekf","pf"]
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "pro_lab_filters/common.hpp"

namespace {

inline double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct FilterState {
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_xy_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_yaw_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rmse_xy_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rmse_yaw_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr conv_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ttc_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nees_pub;

  // running-RMSE accumulators
  std::size_t n = 0;
  double sse_xy = 0.0;
  double sse_yaw = 0.0;

  // convergence detection: rolling history of (stamp_s, err_xy)
  std::deque<std::pair<double, double>> history;
  bool converged = false;
  double time_to_converge = std::numeric_limits<double>::quiet_NaN();
  double experiment_start = 0.0;
};

}  // namespace

class MetricsNode : public rclcpp::Node {
public:
  MetricsNode() : Node("metrics_node") {
    declare_parameter("convergence_threshold_xy", 0.20);
    declare_parameter("convergence_window_s", 2.0);
    declare_parameter("filters", std::vector<std::string>{"kf", "ekf", "pf"});

    conv_thresh_ = get_parameter("convergence_threshold_xy").as_double();
    conv_window_ = get_parameter("convergence_window_s").as_double();
    auto filters = get_parameter("filters").as_string_array();

    truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ground_truth/pose", 20,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          truth_x_   = m->pose.position.x;
          truth_y_   = m->pose.position.y;
          truth_yaw_ = pro_lab_filters::quat_to_yaw(m->pose.orientation);
          have_truth_ = true;
        });

    for (const auto& f : filters) {
      auto& fs = states_[f];
      std::string in  = "/" + f + "/pose";
      std::string out = "/metrics/" + f + "/";
      fs.err_xy_pub   = create_publisher<std_msgs::msg::Float64>(out + "error_xy", 10);
      fs.err_yaw_pub  = create_publisher<std_msgs::msg::Float64>(out + "error_yaw", 10);
      fs.rmse_xy_pub  = create_publisher<std_msgs::msg::Float64>(out + "rmse_xy", 10);
      fs.rmse_yaw_pub = create_publisher<std_msgs::msg::Float64>(out + "rmse_yaw", 10);
      fs.conv_pub     = create_publisher<std_msgs::msg::Bool>(out + "converged", 10);
      fs.ttc_pub      = create_publisher<std_msgs::msg::Float64>(out + "time_to_converge", 10);
      fs.nees_pub     = create_publisher<std_msgs::msg::Float64>(out + "nees", 10);
      fs.sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          in, 10,
          [this, f](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
            handleEstimate(f, *m);
          });
      RCLCPP_INFO(get_logger(),
          "metrics: tracking '%s' (in=%s out=%s*)", f.c_str(), in.c_str(), out.c_str());
    }
  }

private:
  void handleEstimate(const std::string& key,
                      const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    if (!have_truth_) return;
    auto it = states_.find(key);
    if (it == states_.end()) return;
    auto& fs = it->second;

    double t = rclcpp::Time(msg.header.stamp).seconds();
    if (fs.experiment_start == 0.0) fs.experiment_start = t;

    double ex = msg.pose.pose.position.x - truth_x_;
    double ey = msg.pose.pose.position.y - truth_y_;
    double err_xy  = std::sqrt(ex * ex + ey * ey);
    double err_yaw = wrap_pi(
        pro_lab_filters::quat_to_yaw(msg.pose.pose.orientation) - truth_yaw_);

    // Reject NaN/Inf samples before they poison the accumulator. A filter
    // sometimes publishes a NaN pose during init/transient (PF before first
    // resample, AMCL before /initialpose); accumulating NaN into sse_xy
    // makes every subsequent rmse_xy publish NaN for the rest of the run.
    if (!std::isfinite(err_xy) || !std::isfinite(err_yaw)) {
      return;
    }

    // running RMSE
    fs.n += 1;
    fs.sse_xy  += err_xy * err_xy;
    fs.sse_yaw += err_yaw * err_yaw;
    double rmse_xy  = std::sqrt(fs.sse_xy  / static_cast<double>(fs.n));
    double rmse_yaw = std::sqrt(fs.sse_yaw / static_cast<double>(fs.n));

    // publish instantaneous + running
    auto pub_d = [](auto& pub, double v) {
      std_msgs::msg::Float64 m; m.data = v; pub->publish(m);
    };
    pub_d(fs.err_xy_pub, err_xy);
    pub_d(fs.err_yaw_pub, err_yaw);
    pub_d(fs.rmse_xy_pub, rmse_xy);
    pub_d(fs.rmse_yaw_pub, rmse_yaw);

    // NEES = e^T · P^-1 · e over (x, y, yaw). The PoseWithCovariance cov is a
    // 6×6 row-major buffer for [x, y, z, roll, pitch, yaw]; we lift the
    // (0,1,5) sub-block. If P is singular or zero we publish NaN - that's
    // also diagnostic ("filter forgot to publish covariance").
    Eigen::Matrix3d P;
    auto Cij = [&](int i, int j) { return msg.pose.covariance[i * 6 + j]; };
    P << Cij(0,0), Cij(0,1), Cij(0,5),
         Cij(1,0), Cij(1,1), Cij(1,5),
         Cij(5,0), Cij(5,1), Cij(5,5);
    Eigen::Vector3d e(ex, ey, err_yaw);
    double nees = std::numeric_limits<double>::quiet_NaN();
    Eigen::FullPivLU<Eigen::Matrix3d> lu(P);
    if (lu.isInvertible() && P.diagonal().minCoeff() > 1e-12) {
      nees = e.transpose() * lu.inverse() * e;
    }
    pub_d(fs.nees_pub, nees);

    // convergence detection - error_xy under threshold for window_s seconds
    if (!fs.converged) {
      fs.history.emplace_back(t, err_xy);
      while (!fs.history.empty() && t - fs.history.front().first > conv_window_) {
        fs.history.pop_front();
      }
      bool all_below =
          !fs.history.empty()
          && (t - fs.history.front().first) >= conv_window_ - 1e-3;
      if (all_below) {
        for (const auto& [_, e] : fs.history) {
          if (e > conv_thresh_) { all_below = false; break; }
        }
      }
      if (all_below) {
        fs.converged = true;
        fs.time_to_converge = t - fs.experiment_start - conv_window_;
        std_msgs::msg::Bool b; b.data = true; fs.conv_pub->publish(b);
        std_msgs::msg::Float64 ttc; ttc.data = fs.time_to_converge;
        fs.ttc_pub->publish(ttc);
        RCLCPP_INFO(get_logger(),
            "[%s] CONVERGED after %.2fs (err_xy < %.2f m for %.1fs)",
            key.c_str(), fs.time_to_converge, conv_thresh_, conv_window_);
      } else {
        std_msgs::msg::Bool b; b.data = false; fs.conv_pub->publish(b);
      }
    } else {
      std_msgs::msg::Bool b; b.data = true; fs.conv_pub->publish(b);
      std_msgs::msg::Float64 ttc; ttc.data = fs.time_to_converge;
      fs.ttc_pub->publish(ttc);
    }
  }

  // Truth state
  bool have_truth_ = false;
  double truth_x_ = 0.0, truth_y_ = 0.0, truth_yaw_ = 0.0;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr truth_sub_;

  // Per-filter
  std::map<std::string, FilterState> states_;
  double conv_thresh_ = 0.20;
  double conv_window_ = 2.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MetricsNode>());
  rclcpp::shutdown();
  return 0;
}
