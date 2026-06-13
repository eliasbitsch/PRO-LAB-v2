// Log metrics + filter outputs + ground truth to two CSV files for offline
// analysis (RMSE, plots, paper table). C++ port of csv_logger.py.
//
// Outputs (<out_dir>/<scenario>[_seedNN]_{timeseries,summary}.csv):
//   timeseries: time, truth_{x,y,yaw}, per-filter {x,y,yaw,err_xy,err_yaw,
//               rmse_xy,rmse_yaw,nees,cov_xx,cov_yy,cov_xy,runtime_us},
//               pf_ess, n_landmarks_detected
//   summary (one row on shutdown): scenario, seed, duration_s, per-filter
//               {final_rmse_xy,final_rmse_yaw,converged,time_to_converge,
//                runtime_mean_us,runtime_max_us}
//
// Single source of truth for rmse: whatever metrics_node publishes is latched
// into both the timeseries column and the summary.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using std_msgs::msg::Bool;
using std_msgs::msg::Float64;
using std_msgs::msg::Float32MultiArray;

static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

static double quatToYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

struct Summary {
  double rmse_xy{NaN}, rmse_yaw{NaN}, ttc{NaN};
  bool converged{false};
  double rt_mean{0.0}, rt_max{0.0};
  long rt_n{0};
};

class CsvLogger : public rclcpp::Node
{
public:
  CsvLogger() : rclcpp::Node("csv_logger")
  {
    scenario_ = declare_parameter<std::string>("scenario", "unnamed");
    out_dir_  = declare_parameter<std::string>("out_dir", "/tmp/pro_lab_results");
    flush_every_ = declare_parameter<int>("flush_every", 10);
    seed_ = declare_parameter<int>("seed", 0);
    filters_ = declare_parameter<std::vector<std::string>>(
      "filters", std::vector<std::string>{"kf", "ekf", "pf"});

    std::filesystem::create_directories(out_dir_);
    for (const auto & f : filters_) summary_[f] = Summary{};

    seed_suffix_ = seed_ > 0 ? "_seed" + two(seed_) : "";
    const std::string ts_path =
      out_dir_ + "/" + scenario_ + seed_suffix_ + "_timeseries.csv";
    RCLCPP_INFO(get_logger(), "CSV logger writing to %s", ts_path.c_str());
    ts_.open(ts_path);
    ts_ << std::setprecision(12);

    // header
    ts_ << "time,truth_x,truth_y,truth_yaw";
    for (const auto & f : filters_)
      ts_ << "," << f << "_x," << f << "_y," << f << "_yaw,"
          << f << "_err_xy," << f << "_err_yaw,"
          << f << "_rmse_xy," << f << "_rmse_yaw," << f << "_nees,"
          << f << "_cov_xx," << f << "_cov_yy," << f << "_cov_xy,"
          << f << "_runtime_us";
    ts_ << ",pf_ess,n_landmarks_detected\n";

    auto qos = rclcpp::QoS(20).best_effort().keep_last(20);
    subs_.push_back(create_subscription<PoseStamped>(
      "/ground_truth/pose", qos,
      [this](const PoseStamped & m) { onTruth(m); }));

    for (const auto & f : filters_) {
      subs_.push_back(create_subscription<PoseWithCovarianceStamped>(
        "/" + f + "/pose", qos,
        [this, f](const PoseWithCovarianceStamped & m) { onPose(f, m); }));
      subF64(qos, "/metrics/" + f + "/error_xy", f + "_err_xy");
      subF64(qos, "/metrics/" + f + "/error_yaw", f + "_err_yaw");
      // rmse: latch into both timeseries column and summary
      subs_.push_back(create_subscription<Float64>(
        "/metrics/" + f + "/rmse_xy", qos,
        [this, f](const Float64 & m) { setLatest(f + "_rmse_xy", m.data);
                                       setSummaryRmseXy(f, m.data); }));
      subs_.push_back(create_subscription<Float64>(
        "/metrics/" + f + "/rmse_yaw", qos,
        [this, f](const Float64 & m) { setLatest(f + "_rmse_yaw", m.data);
                                       setSummaryRmseYaw(f, m.data); }));
      subs_.push_back(create_subscription<Bool>(
        "/metrics/" + f + "/converged", qos,
        [this, f](const Bool & m) { std::lock_guard<std::mutex> lk(mtx_);
                                    summary_[f].converged = m.data; }));
      subs_.push_back(create_subscription<Float64>(
        "/metrics/" + f + "/time_to_converge", qos,
        [this, f](const Float64 & m) { std::lock_guard<std::mutex> lk(mtx_);
                                       summary_[f].ttc = m.data; }));
      subF64(qos, "/metrics/" + f + "/nees", f + "_nees");
      subs_.push_back(create_subscription<Float64>(
        "/" + f + "/runtime_us", qos,
        [this, f](const Float64 & m) { onRuntime(f, m.data); }));
    }
    subF64(qos, "/pf/ess", "pf_ess");
    subs_.push_back(create_subscription<Float32MultiArray>(
      "/landmarks/observations", qos,
      [this](const Float32MultiArray & m) {
        setLatest("n_landmarks_detected", static_cast<double>(m.data.size() / 3)); }));

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this]() { maybeWriteRow(); });
  }

  void writeSummary()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const std::string path =
      out_dir_ + "/" + scenario_ + seed_suffix_ + "_summary.csv";
    std::ofstream f(path);
    f << std::setprecision(12);
    f << "scenario,seed,duration_s";
    for (const auto & fn : filters_)
      f << "," << fn << "_final_rmse_xy," << fn << "_final_rmse_yaw,"
        << fn << "_converged," << fn << "_time_to_converge,"
        << fn << "_runtime_mean_us," << fn << "_runtime_max_us";
    f << "\n";
    const double duration = getLatest("time", NaN);
    f << scenario_ << "," << seed_ << "," << duration;
    for (const auto & fn : filters_) {
      const Summary & s = summary_[fn];
      f << "," << s.rmse_xy << "," << s.rmse_yaw << ","
        << (s.converged ? "True" : "False") << "," << s.ttc << ","
        << s.rt_mean << "," << s.rt_max;
    }
    f << "\n";
    RCLCPP_INFO(get_logger(), "wrote summary %s", path.c_str());
  }

  void closeFiles() { if (ts_.is_open()) { ts_.flush(); ts_.close(); } }

private:
  static std::string two(int v) { return (v < 10 ? "0" : "") + std::to_string(v); }

  void subF64(const rclcpp::QoS & qos, const std::string & topic,
              const std::string & key)
  {
    subs_.push_back(create_subscription<Float64>(
      topic, qos, [this, key](const Float64 & m) { setLatest(key, m.data); }));
  }

  void onTruth(const PoseStamped & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const double t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
    if (!have_t0_) { t0_ = t; have_t0_ = true; }
    latest_["time"] = t - t0_;
    latest_["truth_x"] = m.pose.position.x;
    latest_["truth_y"] = m.pose.position.y;
    latest_["truth_yaw"] = quatToYaw(m.pose.orientation.x, m.pose.orientation.y,
                                     m.pose.orientation.z, m.pose.orientation.w);
  }

  void onPose(const std::string & key, const PoseWithCovarianceStamped & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_[key + "_x"] = m.pose.pose.position.x;
    latest_[key + "_y"] = m.pose.pose.position.y;
    latest_[key + "_yaw"] = quatToYaw(m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                      m.pose.pose.orientation.z, m.pose.pose.orientation.w);
    latest_[key + "_cov_xx"] = m.pose.covariance[0];
    latest_[key + "_cov_yy"] = m.pose.covariance[7];
    latest_[key + "_cov_xy"] = m.pose.covariance[1];
  }

  void setLatest(const std::string & key, double v)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_[key] = v;
  }
  void setSummaryRmseXy(const std::string & f, double v)
  {
    std::lock_guard<std::mutex> lk(mtx_); summary_[f].rmse_xy = v;
  }
  void setSummaryRmseYaw(const std::string & f, double v)
  {
    std::lock_guard<std::mutex> lk(mtx_); summary_[f].rmse_yaw = v;
  }
  void onRuntime(const std::string & f, double v)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_[f + "_runtime_us"] = v;
    Summary & s = summary_[f];
    s.rt_n += 1;
    s.rt_mean += (v - s.rt_mean) / s.rt_n;   // Welford running mean
    s.rt_max = std::max(s.rt_max, v);
  }

  double getLatest(const std::string & key, double dflt)
  {
    auto it = latest_.find(key);
    return it == latest_.end() ? dflt : it->second;
  }

  void maybeWriteRow()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (latest_.find("time") == latest_.end()) return;
    auto g = [this](const std::string & k, double d) {
      auto it = latest_.find(k); return it == latest_.end() ? d : it->second; };
    ts_ << g("time", NaN) << "," << g("truth_x", NaN) << ","
        << g("truth_y", NaN) << "," << g("truth_yaw", NaN);
    for (const auto & f : filters_) {
      for (const char * k : {"_x", "_y", "_yaw", "_err_xy", "_err_yaw",
                             "_rmse_xy", "_rmse_yaw", "_nees",
                             "_cov_xx", "_cov_yy", "_cov_xy", "_runtime_us"})
        ts_ << "," << g(f + std::string(k), NaN);
    }
    ts_ << "," << g("pf_ess", NaN) << "," << g("n_landmarks_detected", 0.0) << "\n";
    if (++row_count_ % flush_every_ == 0) ts_.flush();
  }

  std::string scenario_, out_dir_, seed_suffix_;
  int flush_every_{10}, seed_{0};
  std::vector<std::string> filters_;

  std::mutex mtx_;
  bool have_t0_{false};
  double t0_{0.0};
  std::map<std::string, double> latest_;
  std::map<std::string, Summary> summary_;

  std::ofstream ts_;
  long row_count_{0};

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CsvLogger>();
  rclcpp::spin(node);
  // On SIGINT spin() returns - write the summary + flush before shutdown.
  node->writeSummary();
  node->closeFiles();
  rclcpp::shutdown();
  return 0;
}
