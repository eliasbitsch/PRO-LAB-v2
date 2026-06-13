// AprilTag landmark detector (C++) — the REAL, honest data-association channel.
//
// The robot reads marker IDs straight off the warehouse pillars with its OAK-D
// camera (no ground-truth leak). Per image:
//   1. libapriltag (AprilTag-3, family tag36h11) detects every visible tag and,
//      with the camera intrinsics + physical tag size, estimates its pose in the
//      camera optical frame (estimate_tag_pose).
//   2. The tag translation is transformed into the robot base frame (static
//      camera TF) and reduced to (range, bearing) — the measurement the
//      landmark-based KF/EKF consume.
//   3. Published as Float32MultiArray [id, range, bearing, ...] on obs_topic,
//      the SAME format the old detector used → filters need no change.
//
// Ground truth is used ONLY to publish a validation error (err_topic) — never to
// label or associate a detection. landmark_* is the a-priori known marker map,
// used only to look up the true range/bearing for that error.
//
// C++ port of the validated apriltag_landmark_detector.py (kept the <1 cm /
// <0.25 deg accuracy). Required because the task wants everything except launch
// files and plotting scripts in C++.
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}
// NOTE: tag pose comes from OpenCV solvePnP on the detected corners, NOT
// libapriltag's estimate_tag_pose() — Debian's libapriltag does not export the
// matd_* helpers needed to read its apriltag_pose_t, so we use the corners
// (det->p, plain struct data) + the known tag size with cv::solvePnP instead.

using std::placeholders::_1;

static double yawFromQuat(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

class AprilTagLandmarkDetector : public rclcpp::Node
{
public:
  AprilTagLandmarkDetector()
  : rclcpp::Node("apriltag_landmark_detector")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/rgbd_camera/image");
    info_topic_  = declare_parameter<std::string>("camera_info_topic", "/rgbd_camera/camera_info");
    obs_topic_   = declare_parameter<std::string>("obs_topic", "/landmarks/observations");
    err_topic_   = declare_parameter<std::string>("err_topic", "/landmarks/detector_error");
    base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");
    optical_frame_ = declare_parameter<std::string>("optical_frame", "oakd_rgb_camera_optical_frame");
    // physical side of the black tag square [m]. Plate is 0.8 m incl. quiet-zone
    // border (border_frac 0.18 -> tag fraction 1/1.36) -> 0.8/1.36 = 0.588 m.
    tag_size_  = declare_parameter<double>("tag_size", 0.588);
    max_range_ = declare_parameter<double>("max_range", 9.0);
    frame_id_  = declare_parameter<std::string>("frame_id", "map");
    // dir holding marker_NN.dae (+ .png) for the RViz tag-image markers.
    mesh_dir_  = declare_parameter<std::string>("marker_mesh_dir",
      "/home/ros/ws/install/pro_lab_filters/share/pro_lab_filters/config/aruco");

    auto ids = declare_parameter<std::vector<int64_t>>("landmark_ids", {0});
    auto xs  = declare_parameter<std::vector<double>>("landmark_xs", {0.0});
    auto ys  = declare_parameter<std::vector<double>>("landmark_ys", {0.0});
    for (size_t i = 0; i < ids.size() && i < xs.size() && i < ys.size(); ++i)
      lm_map_[static_cast<int>(ids[i])] = {xs[i], ys[i]};

    // libapriltag detector (tag36h11). quad_decimate=1.0 keeps full resolution
    // so small/distant tags survive; refine_edges sharpens the corners.
    tf_family_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_family_);
    td_->quad_decimate = 1.0f;
    td_->refine_edges = 1;
    td_->nthreads = 4;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    obs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(obs_topic_, 10);
    err_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(err_topic_, 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/landmark/detected_tags", 10);

    auto sensor_qos = rclcpp::SensorDataQoS();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, sensor_qos, std::bind(&AprilTagLandmarkDetector::onInfo, this, _1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos, std::bind(&AprilTagLandmarkDetector::onImage, this, _1));
    gt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose", 10, std::bind(&AprilTagLandmarkDetector::onGt, this, _1));

    RCLCPP_INFO(get_logger(),
      "apriltag_landmark_detector up: tag_size=%.3f m, %zu known markers (validation map)",
      tag_size_, lm_map_.size());
  }

  ~AprilTagLandmarkDetector() override
  {
    if (td_) apriltag_detector_destroy(td_);
    if (tf_family_) tag36h11_destroy(tf_family_);
  }

private:
  void onInfo(const sensor_msgs::msg::CameraInfo & m)
  {
    if (have_cam_) return;
    fx_ = m.k[0]; fy_ = m.k[4]; cx_ = m.k[2]; cy_ = m.k[5];
    K_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    dist_ = cv::Mat::zeros(4, 1, CV_64F);   // gz camera is rectified / no distortion
    // tag corners in the tag's own frame (z=0), ordered to match det->p
    // (apriltag: 0=bottom-left, 1=bottom-right, 2=top-right, 3=top-left).
    const double h = tag_size_ / 2.0;
    objpts_ = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
    have_cam_ = true;
    RCLCPP_INFO(get_logger(), "camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                fx_, fy_, cx_, cy_);
  }

  void onGt(const geometry_msgs::msg::PoseStamped & m)
  {
    const auto & q = m.pose.orientation;
    gt_x_ = m.pose.position.x;
    gt_y_ = m.pose.position.y;
    gt_yaw_ = yawFromQuat(q.x, q.y, q.z, q.w);
    have_gt_ = true;
  }

  bool ensureTf()
  {
    if (have_tf_) return true;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, optical_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "waiting for TF %s<-%s: %s", base_frame_.c_str(), optical_frame_.c_str(), ex.what());
      return false;
    }
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    tf2::Matrix3x3 R(tf2::Quaternion(q.x, q.y, q.z, q.w));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R_bo_[r][c] = R[r][c];
    t_bo_[0] = t.x; t_bo_[1] = t.y; t_bo_[2] = t.z;
    have_tf_ = true;
    RCLCPP_INFO(get_logger(), "camera->base TF cached");
    return true;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!have_cam_ || !ensureTf()) return;

    cv_bridge::CvImageConstPtr gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    image_u8_t im{gray->image.cols, gray->image.rows,
                  static_cast<int32_t>(gray->image.step), gray->image.data};
    zarray_t * dets = apriltag_detector_detect(td_, &im);

    // keep the strongest detection per id (avoid double EKF updates)
    std::map<int, apriltag_detection_t *> best;
    for (int i = 0; i < zarray_size(dets); ++i) {
      apriltag_detection_t * det;
      zarray_get(dets, i, &det);
      auto it = best.find(det->id);
      if (it == best.end() || det->decision_margin > it->second->decision_margin)
        best[det->id] = det;
    }

    std::vector<float> obs, err;
    visualization_msgs::msg::MarkerArray viz;
    std::string read_log;
    for (auto & [tid, det] : best) {
      // pose from solvePnP on the four detected corners (det->p) + tag size
      std::vector<cv::Point2d> imgpts = {
        {det->p[0][0], det->p[0][1]}, {det->p[1][0], det->p[1][1]},
        {det->p[2][0], det->p[2][1]}, {det->p[3][0], det->p[3][1]}};
      cv::Vec3d rvec, tvec;
      if (!cv::solvePnP(objpts_, imgpts, K_, dist_, rvec, tvec,
                        false, cv::SOLVEPNP_IPPE_SQUARE))
        continue;
      const double px = tvec[0], py = tvec[1], pz = tvec[2];

      // optical -> base: p_base = R_bo * p_opt + t_bo
      const double bx = R_bo_[0][0] * px + R_bo_[0][1] * py + R_bo_[0][2] * pz + t_bo_[0];
      const double by = R_bo_[1][0] * px + R_bo_[1][1] * py + R_bo_[1][2] * pz + t_bo_[1];
      const double bz = R_bo_[2][0] * px + R_bo_[2][1] * py + R_bo_[2][2] * pz + t_bo_[2];

      const double rng = std::hypot(bx, by);
      if (rng > max_range_) continue;
      const double bearing = std::atan2(by, bx);

      obs.push_back(static_cast<float>(tid));
      obs.push_back(static_cast<float>(rng));
      obs.push_back(static_cast<float>(bearing));

      if (have_gt_ && lm_map_.count(tid)) {
        const auto & [lx, ly] = lm_map_[tid];
        const double tr = std::hypot(lx - gt_x_, ly - gt_y_);
        double tb = std::atan2(ly - gt_y_, lx - gt_x_) - gt_yaw_;
        tb = std::atan2(std::sin(tb), std::cos(tb));
        err.push_back(static_cast<float>(tid));
        err.push_back(static_cast<float>(rng - tr));
        err.push_back(static_cast<float>(bearing - tb));
      }

      viz.markers.push_back(textMarker(tid, bx, by, bz));
      viz.markers.push_back(meshMarker(tid, bx, by, bz, bearing));
      viz.markers.push_back(rayMarker(tid, bx, by, bz));
      read_log += " id" + std::to_string(tid) + "@" +
                  std::to_string(rng).substr(0, 4) + "m";
    }
    apriltag_detections_destroy(dets);

    if (!obs.empty()) {
      std_msgs::msg::Float32MultiArray m; m.data = obs; obs_pub_->publish(m);
    }
    if (!err.empty()) {
      std_msgs::msg::Float32MultiArray m; m.data = err; err_pub_->publish(m);
    }
    if (!viz.markers.empty()) viz_pub_->publish(viz);
    if (!obs.empty())
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "%zu tag(s) read:%s", obs.size() / 3, read_log.c_str());
  }

  visualization_msgs::msg::Marker baseMarker(int tid, const std::string & ns)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = ns;
    m.id = tid;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    return m;
  }

  // floating "TAG N" label above the tag
  visualization_msgs::msg::Marker textMarker(int tid, double bx, double by, double bz)
  {
    auto m = baseMarker(tid, "detected_tags");
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.pose.position.x = bx; m.pose.position.y = by; m.pose.position.z = bz + 0.6;
    m.scale.z = 0.5;
    m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.1f; m.color.a = 1.0f;
    m.text = "TAG " + std::to_string(tid);
    return m;
  }

  // the ACTUAL tag image at the detected position, as a textured COLLADA quad
  // (marker_NN.dae carries marker_NN.png). The quad lies in its local Y-Z plane
  // with normal +x; yaw = bearing+pi turns that face back toward the robot.
  visualization_msgs::msg::Marker meshMarker(int tid, double bx, double by, double bz,
                                             double bearing)
  {
    auto m = baseMarker(tid, "detected_tag_meshes");
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.mesh_resource = "file://" + mesh_dir_ + "/marker_" + two(tid) + ".dae";
    m.mesh_use_embedded_materials = true;
    m.pose.position.x = bx; m.pose.position.y = by; m.pose.position.z = bz;
    const double yaw = bearing + M_PI;   // tag face looks back at the robot
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
    m.scale.x = m.scale.y = m.scale.z = 1.0;   // .dae already sized in metres
    m.color.r = m.color.g = m.color.b = m.color.a = 0.0f;  // use texture
    return m;
  }

  static std::string two(int v) { return (v < 10 ? "0" : "") + std::to_string(v); }

  // measurement ray from the robot to the detected tag
  visualization_msgs::msg::Marker rayMarker(int tid, double bx, double by, double bz)
  {
    auto m = baseMarker(tid, "detected_tag_rays");
    m.type = visualization_msgs::msg::Marker::ARROW;
    geometry_msgs::msg::Point a, b;
    a.x = 0.0; a.y = 0.0; a.z = 0.2;          // robot base (camera height-ish)
    b.x = bx; b.y = by; b.z = bz;
    m.points = {a, b};
    m.scale.x = 0.03; m.scale.y = 0.07; m.scale.z = 0.0;  // shaft / head width
    m.color.r = 1.0f; m.color.g = 0.8f; m.color.b = 0.0f; m.color.a = 0.7f;
    return m;
  }

  // params
  std::string image_topic_, info_topic_, obs_topic_, err_topic_;
  std::string base_frame_, optical_frame_, frame_id_, mesh_dir_;
  double tag_size_{0.588}, max_range_{9.0};
  std::map<int, std::pair<double, double>> lm_map_;

  // camera intrinsics
  bool have_cam_{false};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  cv::Mat K_, dist_;
  std::vector<cv::Point3d> objpts_;

  // ground truth (validation only)
  bool have_gt_{false};
  double gt_x_{0}, gt_y_{0}, gt_yaw_{0};

  // static camera->base transform
  bool have_tf_{false};
  double R_bo_[3][3]{}, t_bo_[3]{};

  // apriltag
  apriltag_family_t * tf_family_{nullptr};
  apriltag_detector_t * td_{nullptr};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr err_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagLandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
