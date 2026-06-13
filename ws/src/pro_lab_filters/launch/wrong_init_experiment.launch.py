"""Wrong-initialization experiment launcher.

Runs the simulation + all 3 filters + truth relay + metrics + CSV logger
under a chosen scenario YAML. Optionally auto-shuts-down after `duration_s`.

Visualization: RViz only.

Usage:
    ros2 launch pro_lab_filters wrong_init_experiment.launch.py \\
        scenario:=offset_5m duration_s:=60 out_dir:=/tmp/results

Available scenarios (config/scenarios/<name>.yaml):
    correct_init, offset_1m, offset_5m, wrong_yaw_pi2,
    overconfident_wrong, underconfident, kidnapped
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction, RegisterEventHandler,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression,
                                  TextSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pro_lab_filters')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')

    scenario   = LaunchConfiguration('scenario')
    duration_s = LaunchConfiguration('duration_s')
    out_dir    = LaunchConfiguration('out_dir')
    world      = LaunchConfiguration('world')
    x_pose     = LaunchConfiguration('x_pose')
    y_pose     = LaunchConfiguration('y_pose')
    yaw        = LaunchConfiguration('yaw')
    map_yaml   = LaunchConfiguration('map')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_nav2   = LaunchConfiguration('use_nav2')
    use_amcl   = LaunchConfiguration('use_amcl')
    use_traj   = LaunchConfiguration('use_trajectory')
    start_gz   = LaunchConfiguration('start_gz')
    filter_sel = LaunchConfiguration('filter')
    seed       = LaunchConfiguration('seed')
    truth_tf   = LaunchConfiguration('truth_tf')

    args = [
        DeclareLaunchArgument('scenario',   default_value='correct_init'),
        DeclareLaunchArgument('duration_s', default_value='0',
                              description='Auto-shutdown after N seconds. 0 = run forever.'),
        DeclareLaunchArgument('out_dir',    default_value='/tmp/pro_lab_results'),
        DeclareLaunchArgument('world',      default_value='warehouse'),
        # Spawn pose in world coords. map->odom is offset by this (see below)
        # so `map` coincides with the world frame and /scan aligns with /map.
        # Scenario YAMLs (init_x/init_y/init_yaw) start the filters here too.
        # The warehouse occupancy map matches the warehouse world (unlike the
        # available depot map, which is incomplete) → clean scan-vs-map.
        DeclareLaunchArgument('x_pose',     default_value='0.00'),
        DeclareLaunchArgument('y_pose',     default_value='0.00'),
        DeclareLaunchArgument('yaw',        default_value='0.0'),      # face +x (east) - IMU-aligned, avoids filter init issues
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav2_bringup, 'maps', 'warehouse.yaml')),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('use_nav2',     default_value='true'),
        DeclareLaunchArgument('use_amcl',     default_value='true',
                              description='Run AMCL as 4th comparison baseline '
                                          '(relays /amcl_pose -> /amcl/pose). '
                                          'Requires use_nav2=true.'),
        DeclareLaunchArgument('use_trajectory', default_value='true',
                              description='Run scripted trajectory_player so every '
                                          'run drives the same path. Disable for '
                                          'manual teleop debugging.'),
        DeclareLaunchArgument('filter',       default_value='all',
            description='Which estimator(s) to run: kf | ekf | ekf_lf | pf | all'),
        DeclareLaunchArgument('seed',         default_value='0',
            description='Random seed for filter init sampling and PF RNG. '
                        '0 = deterministic (single-run baseline). '
                        '>0 enables Gaussian sampling around init_x/y/yaw '
                        'using init_spread_xy/yaw - used for 10-run sweeps.'),
        DeclareLaunchArgument('gz_gui',       default_value='true',
                              description='Show the Gazebo GUI window. Set false for batch '
                                          'runs (run_experiments.sh / wrong_init_batch.sh).'),
        DeclareLaunchArgument('start_gz',     default_value='true',
                              description='Start the headless gz server inside the container. '
                                          'Set false only if a gz server is already running '
                                          'externally on the same GZ_PARTITION.'),
        DeclareLaunchArgument('truth_tf',     default_value='false',
                              description='DEMO ONLY: drive map->odom from /ground_truth/pose so '
                                          'the RViz RobotModel sits on the TRUE robot pose (filters '
                                          'visibly converge onto it). Off for scored runs, which '
                                          'use the fixed spawn-derived static map->odom.'),
    ]

    scenario_file = PathJoinSubstitution(
        [pkg_share, 'config', 'scenarios', [scenario, TextSubstitution(text='.yaml')]])

    # ── Simulation stack (gz + RSP + spawn) ──────────────────────────────
    # Where the robot's GPU sensors (gpu_lidar/camera) render depends on the
    # render API gz picks, which we split by run mode:
    #
    #   Batch / server-only (gz_gui:=false): we pass --headless-rendering so gz
    #   renders the sensors offscreen via *EGL*. The EGL vendor is pinned to the
    #   NVIDIA dGPU in docker-compose (__EGL_VENDOR_LIBRARY_FILENAMES), so the
    #   gpu_lidar raycasting runs on the RTX (verified: gz appears as a graphics
    #   process on the NVIDIA in nvidia-smi). Without the flag gz falls back to
    #   GLX on DISPLAY=:1 and renders on the Mesa/iGPU path instead - slower and
    #   not what we want for the sweeps.
    #
    #   Interactive / with GUI (gz_gui:=true): the GUI window uses *GLX* on the
    #   host X server (DISPLAY), routed via XWayland. We must NOT force the
    #   NVIDIA GLX vendor here (yields GLXBadFBConfig under XWayland and gz
    #   crashes), so the GUI path stays on the default GLX backend. Needs
    #   `xhost +local:` on the host once.
    #
    # The world file is chosen at runtime: for `depot` we use our own
    # config/depot.sdf (the upstream one gates SceneBroadcaster behind xacro,
    # which gz can't expand → empty GUI); other worlds load from the sim share.
    def make_gz_server(context, *a, **k):
        if context.launch_configurations.get('start_gz', 'true') != 'true':
            return []
        w = context.launch_configurations.get('world', 'depot')
        if w == 'depot':
            world_file = os.path.join(pkg_share, 'config', 'depot.sdf')
        else:
            world_file = os.path.join(sim_dir, 'worlds', w + '.sdf')
        # `-s` = server-only (headless), no `-s` = also start the Gazebo GUI.
        # Default on for interactive runs; batch scripts pass gz_gui:=false.
        # Server-only also gets --headless-rendering so sensor rendering goes
        # through EGL (pinned to the NVIDIA dGPU) rather than GLX on the iGPU.
        gz_gui = context.launch_configurations.get('gz_gui', 'true') == 'true'
        if gz_gui:
            cmd = ['gz', 'sim', '-r', '-v', '3', world_file]
        else:
            cmd = ['gz', 'sim', '-s', '-r', '--headless-rendering',
                   '-v', '3', world_file]
        return [ExecuteProcess(cmd=cmd, output='screen')]
    gz_server = OpaqueFunction(function=make_gz_server)

    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare('nav2_minimal_tb4_description'),
         'urdf', 'standard', 'turtlebot4.urdf.xacro'])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_xacro]),
        }],
    )

    spawn = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw}.items(),
        )
    ])

    # Physical landmark posts (cylinders) the lidar can actually see, so the
    # scan-cluster landmark_detector has real returns to cluster. Coordinates
    # match config/landmarks.yaml. Spawned after the world + robot are up.
    post_sdf = os.path.join(pkg_share, 'config', 'landmark_post.sdf')
    # Compact triangle bracketing the driven path (spawn -8,-0.5 → stop -6,1):
    # one NE, one S, one N. Close together (~2.5 m spread) so the trajectory +
    # covariance plot is tightly framed, and close to the path so every pose -
    # including the final stop - sees a post and the landmark-based KF/EKF keep
    # getting corrections instead of dead-reckoning away.
    # ALL real warehouse pillars auto-discovered from the occupancy map.
    # Spawning our thick I-beams here overlays them on the map's existing
    # structure → scan-based filters see no mismatch, landmark filters get
    # many anchors → richer triangulation, more robust to occlusion.
    # All shifted -0.10 m in y to align with the real orange pillars
    # Physical I-beam pillar coords (what the lidar/scan-likelihood filters see).
    pillar_coords = [
        (-7.45, -15.02), ( 7.49, -15.02),
        (-7.42,  -7.55), ( 7.46,  -7.52),
        (-7.51,  -0.02), ( 7.43,  -0.02),
        (-7.45,   7.54), ( 7.43,   7.48),
    ]
    # The AprilTag plates sit MARKER_OFFSET in front of each pillar toward the
    # aisle (so the camera has line of sight). The localisation landmark is now
    # the TAG, not the I-beam centre, so the KF/EKF map AND the detector's
    # validation map use the MARKER coords - otherwise the camera (measuring to
    # the tag) is biased ~0.3 m against a pillar-centre map. Sign: left column
    # (x<0) offsets +x toward centre, right column -x.
    MARKER_OFFSET = 0.6
    marker_coords = [(x + (MARKER_OFFSET if x < 0 else -MARKER_OFFSET), y)
                     for (x, y) in pillar_coords]

    # Single source of truth for the landmark MAP (= marker positions). The
    # KF/EKF look up each observed id here to triangulate; the detector uses it
    # only to validate range/bearing against ground truth. id = index + 1.
    landmark_params = {
        'landmark_ids': [i + 1 for i in range(len(marker_coords))],
        'landmark_xs':  [c[0] for c in marker_coords],
        'landmark_ys':  [c[1] for c in marker_coords],
    }
    # Spawn the I-beam posts at the pillar coords, and an AprilTag plate (id =
    # landmark id) at each marker coord. The plate's -x face (non-mirrored - see
    # gen_marker_models.py) is turned into the aisle: yaw=pi for the left column
    # (robot is at +x), yaw=0 for the right column. The detector READS these IDs
    # off the camera - no ground-truth leak in the measurement.
    aruco_dir = os.path.join(pkg_share, 'config', 'aruco')
    spawn_landmarks = TimerAction(period=7.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', world, '-file', post_sdf,
                        '-name', f'landmark_{i + 1}',
                        '-x', f'{x}', '-y', f'{y}', '-z', '0',
                        '-Y', '1.5708'],   # rotate 90° to align with real pillars
             output='screen')
        for i, (x, y) in enumerate(pillar_coords)
    ] + [
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', world,
                        '-file', os.path.join(aruco_dir, f'marker_{i + 1:02d}.sdf'),
                        '-name', f'marker_{i + 1}',
                        '-x', f'{mx}', '-y', f'{my}', '-z', '0',
                        '-Y', f'{3.14159 if px < 0 else 0.0}'],
             output='screen')
        for i, ((mx, my), (px, _py)) in enumerate(zip(marker_coords, pillar_coords))
    ])

    # ros_gz_bridge: maps gz topics (/cmd_vel, /imu, /odom, /scan, /tf, /clock)
    # to ROS so the filter nodes see them. Required whether gz runs inside the
    # container or natively in WSL.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    # ── Nav2 (provides AMCL as external comparison track) ───────────────
    # Filters do NOT consume /pose any more - KF/EKF/PF each have their own
    # measurement channel (landmarks for KF/EKF, scan-likelihood for PF).
    # AMCL runs solely so its output can be logged alongside the filters
    # for the "our filters vs Nav2 standard" comparison plot.
    # params_file: stock nav2 params with amcl.tf_broadcast=false. Our
    # map_odom_tf_publisher owns map->odom (static, spawn-derived ground
    # truth). If AMCL also broadcast it, the two would fight and - whenever
    # AMCL is mislocalized, which is the POINT of the wrong-init scenarios -
    # the landmark detector's association pose jumps metres off, the gate
    # rejects every cluster, and KF/EKF silently lose their measurements.
    nav2 = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map': map_yaml,
                'autostart': 'true',
                'params_file': os.path.join(
                    pkg_share, 'config', 'nav2_params_no_amcl_tf.yaml'),
            }.items(),
            condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_nav2),
        )
    ])

    # ── Standalone map_server so /map is always published ──────────────
    # Even when Nav2 is disabled the filters (PF/EKF-LF) and RViz need /map.
    # Uses the same `map` arg as Nav2 so it follows the selected world.
    #
    # IMPORTANT: when use_nav2:=true, nav2_bringup ALREADY starts its own
    # map_server + lifecycle_manager_localization. Running this standalone
    # pair in parallel causes the two lifecycle managers to race the same
    # node, leaving map_server stuck in 'active' while bringup's manager
    # tries to re-configure it → "Transition is not registered" → whole
    # bringup aborts and AMCL never comes up (RMSE = NaN in the plots).
    # So this standalone pair only fires when use_nav2:=false.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
        condition=UnlessCondition(use_nav2),
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server']}],
        condition=UnlessCondition(use_nav2),
    )

    # ── cmd_vel watchdog ────────────────────────────────────────────────
    # The RViz teleop panel (and other key-hold publishers) only publish
    # while a key is held. On release they stop publishing - but Gazebo
    # keeps applying the last twist forever. This relay forwards
    # /cmd_vel_in to /cmd_vel and zeroes /cmd_vel after a short silence,
    # so releasing the button actually stops the robot.
    cmd_vel_watchdog = Node(
        package='pro_lab_filters',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # 2.0 s (was 0.4): the scripted trajectory_player publishes at 20 Hz,
            # but under RViz/GUI load its wall-timer can be starved >0.4 s, which
            # made the watchdog zero /cmd_vel and the robot froze after ~0.1 m.
            # Teleop still stops promptly enough at 2 s.
            'timeout': 2.0,
            'input_topic': '/cmd_vel_in',
            'output_topic': '/cmd_vel',
        }],
    )

    # Live "kidnap" the robot: the RViz Kidnap tool publishes /initialpose,
    # which robot_teleporter forwards to gz set_pose so the robot jumps there.
    robot_teleporter = Node(
        package='pro_lab_filters',
        executable='robot_teleporter',
        name='robot_teleporter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_name': 'turtlebot4',
            'world_name': world,
            'spawn_z': 0.05,
            'gz_partition': 'prolab',
        }],
    )

    # Landmark detector - REAL camera-based AprilTag (36h11) detection. The OAK-D
    # reads each pillar's marker ID and estimates its range/bearing from the
    # image (no ground-truth leak in the measurement). Publishes the same
    # /landmarks/observations [id, range, bearing] the KF/EKF already consume.
    # landmark_params (the a-priori pillar map) is used ONLY to publish a
    # validation error against ground truth - never to label/associate.
    landmark_detector = Node(
        package='pro_lab_filters',
        executable='apriltag_landmark_detector_node',
        name='apriltag_landmark_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            **landmark_params,
            'image_topic':       '/rgbd_camera/image',
            'camera_info_topic': '/rgbd_camera/camera_info',
            'optical_frame':     'oakd_rgb_camera_optical_frame',
            'base_frame':        'base_footprint',
            'tag_size':           0.588,
            'max_range':          9.0,
            'frame_id':          'map',
            # dir with marker_NN.dae (+ .png) for the RViz tag-image markers
            'marker_mesh_dir':   os.path.join(pkg_share, 'config', 'aruco'),
        }],
    )

    # Filter selector - defined here so the conditional map_odom_* nodes
    # below can branch on which filter is active. Used again at the
    # filter-node block for the actual KF/EKF/PF instantiation.
    run_kf     = PythonExpression(["'", filter_sel, "' in ('kf',     'all')"])
    run_ekf    = PythonExpression(["'", filter_sel, "' in ('ekf',    'all')"])
    run_ekf_lf = PythonExpression(["'", filter_sel, "' in ('ekf_lf', 'all')"])
    run_pf     = PythonExpression(["'", filter_sel, "' in ('pf',     'all')"])

    # ── Ground-truth map -> odom transform ──────────────────────────────
    # gz anchors the odom frame at the robot's SPAWN pose (odom->base == 0 at
    # spawn), NOT at the world origin. To make `map` coincide with the world
    # frame we offset map->odom by the spawn pose (x_pose, y_pose). Then
    # map->base_footprint == the robot's true WORLD pose - which is what
    # truth_relay reads and what every filter is initialised in (init_x/y from
    # the scenario YAML are world coords). This is a FIXED ground-truth
    # transform (not the PF estimate), so the truth reference is independent of
    # any filter - essential for a fair wrong-init comparison.
    # Scored runs: FIXED spawn-derived static map->odom (truth reference is
    # independent of any filter - essential for a fair wrong-init comparison).
    # Disabled in demo mode (truth_tf:=true), where the truth-driven node below
    # owns map->odom instead.
    map_odom_static = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_odom_static',
        arguments=[x_pose, y_pose, '0', yaw, '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=UnlessCondition(truth_tf),
    )
    # DEMO ONLY (truth_tf:=true): drive map->odom from /ground_truth/pose so the
    # RViz RobotModel sits on the TRUE robot pose. Then odometry drift and kidnap
    # jumps no longer displace the model, and the filter pose arrows visibly
    # converge onto the real robot. NOT used for scored runs.
    map_odom_tf = Node(
        package='pro_lab_filters',
        executable='map_odom_tf_publisher',
        name='map_odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic':  '/ground_truth/pose',
            'pose_is_stamped': True,
            'map_frame':   'map',
            'odom_frame':  'odom',
            'base_frame':  'base_footprint',
        }],
        condition=IfCondition(truth_tf),
    )

    # ── Visualizer ───────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'wrong_init.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_rviz),
    )

    # ── Filters: each loads the same scenario YAML so init_x/spread/etc.
    #    are consistent across KF, EKF, PF. The `filter` launch arg gates
    #    which one(s) actually run, so single-filter testing is one CLI
    #    flag away (kf_only.launch.py / ekf_only.launch.py / pf_only.launch.py
    #    are thin wrappers around this).
    kf  = Node(package='pro_lab_filters', executable='kf_node',  name='kf_node',
               parameters=[scenario_file, {'frame_id': 'map',
                                           'use_sim_time': True,
                                           'rng_seed': seed,
                                           **landmark_params}],
               output='screen', condition=IfCondition(run_kf))
    ekf = Node(package='pro_lab_filters', executable='ekf_node', name='ekf_node',
               parameters=[scenario_file, {'frame_id': 'map',
                                           'use_sim_time': True,
                                           'rng_seed': seed,
                                           **landmark_params}],
               output='screen', condition=IfCondition(run_ekf))
    # Dead-reckoning twin: same EKF binary, but predict-only (no IMU, no
    # landmarks). Provides the honest "dead reckoning drifts, uncertainty
    # grows unbounded" baseline for the lecture-style explainer plots
    # (panels 1+2), against which panel 3 shows what landmarks buy.
    #
    # velocity_source /cmd_vel_in: the twin integrates COMMANDED velocity
    # (Thrun §5.3 velocity motion model) instead of the encoder. The real
    # cmd-vs-actual mismatch (accel ramps, wheel slip during pivots) then
    # shows up as genuine, clearly visible drift - no synthetic noise.
    # /cmd_vel_in (pre-watchdog) carries only the trajectory_player script,
    # so nav2's bring-up /cmd_vel spam cannot contaminate the twin.
    ekf_dr = Node(package='pro_lab_filters', executable='ekf_node', name='ekf_dr_node',
                  parameters=[scenario_file, {'frame_id': 'map',
                                              'use_sim_time': True,
                                              'rng_seed': seed,
                                              'use_imu': False,
                                              'use_landmarks': False,
                                              'velocity_source': '/cmd_vel_in',
                                              'topic_prefix': '/ekf_dr'}],
                  output='screen', condition=IfCondition(run_ekf))
    # EKF-LF: advanced variant with direct scan-likelihood update. Frame is
    # 'map' because corrections come from the warehouse map directly, not
    # AMCL-on-odom.
    ekf_lf = Node(package='pro_lab_filters', executable='ekf_lf_node',
                  name='ekf_lf_node',
                  parameters=[scenario_file, {'frame_id': 'map',
                                              'use_sim_time': True,
                                              'rng_seed': seed}],
                  output='screen', condition=IfCondition(run_ekf_lf))
    pf  = Node(package='pro_lab_filters', executable='pf_node',  name='pf_node',
               parameters=[scenario_file, {'frame_id': 'map', 'use_sim_time': True,
                                           'publish_particles': True,
                                           'rng_seed': seed}],
               output='screen', condition=IfCondition(run_pf))

    # AMCL is the gold-standard baseline. nav2_amcl publishes /amcl_pose;
    # we relay it onto /amcl/pose so metrics_node and csv_logger can treat
    # it like any other filter (subscribed under /<name>/pose convention).
    # topic_tools/relay isn't packaged for ros-jazzy on this container,
    # so we ship a small C++ equivalent (src/amcl_relay_node.cpp).
    amcl_relay = Node(
        package='pro_lab_filters', executable='amcl_relay_node', name='amcl_relay',
        condition=IfCondition(use_amcl),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    # AMCL doesn't localise until it receives an /initialpose. We publish one
    # shot from the scenario YAML so AMCL starts from the same (possibly
    # wrong) pose as the in-house filters - that's the whole comparison.
    amcl_init_pose = Node(
        package='pro_lab_filters', executable='amcl_init_pose_node',
        name='amcl_init_pose',
        parameters=[scenario_file, {'use_sim_time': True, 'frame_id': 'map'}],
        condition=IfCondition(use_amcl),
        output='screen',
    )
    # Reproducible kidnap events for headless batches. Reads kidnap_schedule
    # from the scenario YAML; empty schedule is a no-op, so we can launch it
    # unconditionally and only the `kidnapped` scenario actually triggers
    # warps. start_delay_s matches trajectory_player so kidnap times are
    # "seconds into scripted motion".
    auto_kidnapper = Node(
        package='pro_lab_filters', executable='auto_kidnapper_node',
        name='auto_kidnapper',
        parameters=[scenario_file, {
            'use_sim_time': True,
            'frame_id': 'map',
            'start_delay_s': 14.0,
        }],
        output='screen',
    )

    # Scripted trajectory: deterministic varied path so every run is fair.
    # start_delay_s gives Gazebo + filters time to come up before motion.
    trajectory_player = Node(
        package='pro_lab_filters', executable='trajectory_player_node',
        name='trajectory_player',
        parameters=[{
            'use_sim_time': True,
            'start_delay_s': 14.0,
            'speed_scale':   1.0,
            'publish_hz':    20.0,
            'topic_in':      '/cmd_vel_in',
            'topic_done':    '/trajectory/done',
            'start_x':        0.00,    # warehouse spawn at origin - matches x_pose
            'start_y':        0.00,    # matches y_pose default
            'start_yaw':      0.00,    # face +x (east), matches yaw default
            'planned_frame': 'map',
        }],
        condition=IfCondition(use_traj),
        output='screen',
    )

    # ── Truth + metrics + CSV logger ────────────────────────────────────
    # Truth via gz transport DIRECTLY (no ros_gz_bridge) because the bridge
    # drops entity names when converting Pose_V -> TFMessage, leaving no way
    # to filter the robot out of the dynamic_pose list (which also contains
    # chairs and other moving entities). Critical for the kidnapped scenario
    # where AMCL's map->odom flails and the old TF-based truth jumped wildly.
    truth = Node(package='pro_lab_filters', executable='gz_truth_relay',
                 name='gz_truth_relay',
                 parameters=[{'use_sim_time': True,
                              'world':       'warehouse',
                              'model_name':  'turtlebot4',
                              'frame_id':    'map',
                              'topic':       '/ground_truth/pose',
                              'path_topic':  '/ground_truth/path'}],
                 output='screen')
    metrics = Node(package='pro_lab_filters', executable='metrics_node',
                   name='metrics_node',
                   parameters=[{'use_sim_time': True,
                                'convergence_threshold_xy': 0.20,
                                'convergence_window_s': 2.0,
                                'filters': ['kf', 'ekf', 'ekf_lf', 'pf', 'amcl', 'ekf_dr']}],
                   output='screen')
    csv_logger = Node(package='pro_lab_filters', executable='csv_logger_node',
                      name='csv_logger',
                      parameters=[{'use_sim_time': True,
                                   'scenario': scenario,
                                   'out_dir': out_dir,
                                   'seed': seed,
                                   'filters': ['kf', 'ekf', 'ekf_lf', 'pf', 'amcl', 'ekf_dr']}],
                      output='screen')

    filters_group = TimerAction(period=10.0, actions=[
        kf, ekf, ekf_dr, ekf_lf, pf, amcl_relay, amcl_init_pose, auto_kidnapper, trajectory_player,
        truth, metrics, csv_logger,
    ])

    # ── Optional auto-shutdown after duration_s seconds ─────────────────
    def maybe_shutdown(context, *args, **kwargs):
        try:
            d = float(context.launch_configurations.get('duration_s', '0'))
        except ValueError:
            d = 0.0
        if d <= 0:
            return []
        return [TimerAction(period=d, actions=[Shutdown(reason='duration elapsed')])]

    return LaunchDescription(
        args + [gz_server, robot_state_pub, spawn, spawn_landmarks, bridge, nav2,
                map_server, map_lifecycle, map_odom_static, map_odom_tf,
                cmd_vel_watchdog, robot_teleporter,
                landmark_detector,
                rviz, filters_group,
                OpaqueFunction(function=maybe_shutdown)])
