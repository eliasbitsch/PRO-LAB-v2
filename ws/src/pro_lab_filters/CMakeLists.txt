cmake_minimum_required(VERSION 3.14)
project(pro_lab_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
# gz_transport_vendor / gz_msgs_vendor for the direct (non-bridged) ground
# truth subscriber, used by gz_truth_relay.
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
# AprilTag landmark detector: OpenCV (image -> mono8), cv_bridge, libapriltag.
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_library(APRILTAG_LIBRARY NAMES apriltag REQUIRED)
find_path(APRILTAG_INCLUDE_DIR NAMES apriltag.h
    PATHS /usr/include/apriltag /usr/local/include/apriltag)
set(CMAKE_AUTOMOC ON)

set(COMMON_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# ── Filter nodes ─────────────────────────────────────────────────────────
add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node ${COMMON_DEPS})

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node ${COMMON_DEPS})

add_executable(ekf_lf_node src/ekf_lf_node.cpp)
ament_target_dependencies(ekf_lf_node ${COMMON_DEPS})

add_executable(pf_node src/pf_node.cpp)
ament_target_dependencies(pf_node ${COMMON_DEPS})

# ── Experiment infrastructure (wrong-init study) ────────────────────────
add_executable(truth_relay_node src/truth_relay_node.cpp)
ament_target_dependencies(truth_relay_node ${COMMON_DEPS})

add_executable(metrics_node src/metrics_node.cpp)
ament_target_dependencies(metrics_node ${COMMON_DEPS})
target_include_directories(metrics_node PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(cmd_vel_watchdog src/cmd_vel_watchdog.cpp)
ament_target_dependencies(cmd_vel_watchdog rclcpp geometry_msgs)

add_executable(robot_teleporter src/robot_teleporter.cpp)
ament_target_dependencies(robot_teleporter rclcpp geometry_msgs)

add_executable(map_odom_tf_publisher src/map_odom_tf_publisher.cpp)
ament_target_dependencies(map_odom_tf_publisher
    rclcpp geometry_msgs tf2 tf2_ros)

# Real camera-based AprilTag landmark detector (libapriltag + cv_bridge).
add_executable(apriltag_landmark_detector_node src/apriltag_landmark_detector_node.cpp)
ament_target_dependencies(apriltag_landmark_detector_node
    rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros cv_bridge)
target_include_directories(apriltag_landmark_detector_node PRIVATE
    ${APRILTAG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltag_landmark_detector_node
    ${APRILTAG_LIBRARY} ${OpenCV_LIBS})

add_executable(trajectory_player_node src/trajectory_player_node.cpp)
ament_target_dependencies(trajectory_player_node
    rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

add_executable(gz_truth_relay src/gz_truth_relay.cpp)
ament_target_dependencies(gz_truth_relay rclcpp geometry_msgs nav_msgs)
target_link_libraries(gz_truth_relay gz-transport13::core gz-msgs10::core)

# ── Experiment IO nodes (ported from Python; task wants C++) ─────────────
add_executable(amcl_relay_node src/amcl_relay_node.cpp)
ament_target_dependencies(amcl_relay_node rclcpp geometry_msgs)

add_executable(amcl_init_pose_node src/amcl_init_pose_node.cpp)
ament_target_dependencies(amcl_init_pose_node rclcpp geometry_msgs)

add_executable(auto_kidnapper_node src/auto_kidnapper_node.cpp)
ament_target_dependencies(auto_kidnapper_node rclcpp geometry_msgs)

add_executable(csv_logger_node src/csv_logger_node.cpp)
ament_target_dependencies(csv_logger_node rclcpp std_msgs geometry_msgs)

install(TARGETS
    kf_node ekf_node ekf_lf_node pf_node
    truth_relay_node metrics_node gz_truth_relay
    cmd_vel_watchdog robot_teleporter map_odom_tf_publisher
    trajectory_player_node
    apriltag_landmark_detector_node
    amcl_relay_node amcl_init_pose_node auto_kidnapper_node csv_logger_node
  DESTINATION lib/${PROJECT_NAME})

# ── RViz2 panel plugin (teleop dpad) ────────────────────────────────────
add_library(pro_lab_rviz_plugins SHARED
    src/teleop_panel.cpp
    src/kidnap_tool.cpp
    include/pro_lab_filters/teleop_panel.hpp
    include/pro_lab_filters/kidnap_tool.hpp)
target_include_directories(pro_lab_rviz_plugins PUBLIC include)
ament_target_dependencies(pro_lab_rviz_plugins
    rclcpp geometry_msgs rviz_common rviz_default_plugins rviz_rendering pluginlib tf2)
target_link_libraries(pro_lab_rviz_plugins Qt5::Core Qt5::Widgets)

install(TARGETS pro_lab_rviz_plugins
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(FILES rviz_plugins.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

# Python scripts (CSV logger + AMCL relay)
# All runtime nodes are now C++ (task: everything except launch files and
# plotting scripts in C++). The remaining scripts/ entries are dev/CI helpers
# run by path (gen_*markers, provision_container.sh) or the retired Python
# originals — none are installed as ROS executables.

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
