#!/usr/bin/env bash
# Provision the prolab_jazzy container for the AprilTag landmark pipeline.
# These changes live OUTSIDE the mounted workspace (system ROS share + pip
# site-packages), so they are lost on an image rebuild — re-run this script
# after recreating the container. Idempotent: safe to run repeatedly.
#
# Run as ROOT inside the container:
#   docker exec -u 0 prolab_jazzy bash /home/ros/ws/src/pro_lab_filters/scripts/provision_container.sh
#
# What it does:
#   1. Re-enables the OAK-D rgb camera sensor (the minimal TB4 model ships it
#      commented out: "<!-- rgbd_camera disabled -->") at 1280x720 so AprilTag
#      36h11 markers stay readable across the ~7.5 m warehouse aisles.
#   2. Installs libapriltag-dev (the dedicated AprilTag-3 detector C library,
#      linked by apriltag_landmark_detector_node — a BUILD dependency, must be
#      present before colcon build).
set -u

OAKD=/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/sensors/oakd.urdf.xacro

echo "[provision] 1/2 enabling OAK-D rgb camera sensor in $OAKD"
python3 - "$OAKD" <<'PY'
import sys, shutil, os
f = sys.argv[1]
s = open(f).read()
if 'type="rgbd_camera"' in s:
    print("[provision]   camera already enabled — skipping")
    sys.exit(0)
old = "    <!-- rgbd_camera disabled -->\n"
new = """    <!-- rgbd_camera RE-ENABLED for ArUco/AprilTag landmark detection.
         1280x720 so 36h11 tags stay readable across the ~7.5 m aisles. -->
    <sensor name="rgbd_camera" type="rgbd_camera">
      <topic>rgbd_camera</topic>
      <gz_frame_id>${name}_rgb_camera_optical_frame</gz_frame_id>
      <camera>
        <horizontal_fov>1.25</horizontal_fov>
        <image><width>1280</width><height>720</height></image>
        <clip><near>0.1</near><far>100</far></clip>
        <optical_frame_id>${name}_rgb_camera_optical_frame</optical_frame_id>
      </camera>
      <always_on>1</always_on>
      <update_rate>15</update_rate>
      <visualize>true</visualize>
    </sensor>
"""
if old not in s:
    print("[provision]   ERROR: disabled-camera marker not found; model changed upstream", file=sys.stderr)
    sys.exit(1)
if not os.path.exists(f + ".orig"):
    shutil.copy(f, f + ".orig")
open(f, "w").write(s.replace(old, new))
print("[provision]   camera enabled (1280x720)")
PY

echo "[provision] 2/2 installing libapriltag-dev (AprilTag-3 C library)"
if [ -f /usr/include/apriltag/apriltag.h ]; then
    echo "[provision]   libapriltag-dev already installed — skipping"
else
    apt-get install -y libapriltag-dev
fi

echo "[provision] done."
