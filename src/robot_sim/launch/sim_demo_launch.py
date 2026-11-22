"""
sim_demo_launch.py -- full AMR simulation for demonstration (no robot hardware needed)

  Gazebo Fortress (headless, GPU sensor rendering)
    demo_room.sdf + simulated AMR (diff drive, D455-like RGB-D camera, IMU)
  ros_gz_bridge          Gazebo <-> ROS 2 topics (/cmd_vel, /odom, /tf, camera, imu, clock)
  robot_state_publisher  URDF -> TF for the robot body / camera optical frame
  nvblox                 simulated depth -> 3D TSDF -> 2D ESDF slice -> Nav2 costmaps
  Nav2                   SAME nav2_params.yaml as the real robot (NavFn A* planner, DWB)
  RViz2                  robot, costmaps, A* path, nvblox map, camera
  Foxglove bridge        ws://<jetson-ip>:8765 (view from a laptop browser)
  waypoint_demo          (optional) autonomous A* waypoint tour with live metrics

Odometry/localization in sim: Gazebo's wheel odometry provides odom->base_link and
map->odom is identity. On the real robot cuVSLAM provides both TFs instead
(see robot_bringup/bringup_launch.py) -- everything downstream is identical.

Usage:
  ros2 launch robot_sim sim_demo_launch.py                 # RViz + autonomous tour
  ros2 launch robot_sim sim_demo_launch.py tour:=false     # click goals in RViz ("2D Goal Pose")
  ros2 launch robot_sim sim_demo_launch.py gz_gui:=true    # also open the Gazebo 3D window
  ros2 launch robot_sim sim_demo_launch.py rviz:=false     # headless (use Foxglove)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# Robot spawn pose in the world (map/odom origin = this pose)
SPAWN_X, SPAWN_Y, SPAWN_Z = '-4.0', '-3.0', '0.06'


def generate_launch_description():
    sim_share = get_package_share_directory('robot_sim')
    bringup_share = get_package_share_directory('robot_bringup')

    world = os.path.join(sim_share, 'worlds', 'demo_room.sdf')
    xacro_file = os.path.join(sim_share, 'urdf', 'amr_sim.urdf.xacro')
    bridge_cfg = os.path.join(sim_share, 'config', 'gz_bridge.yaml')
    rviz_cfg = os.path.join(sim_share, 'rviz', 'sim_demo.rviz')

    use_rviz = LaunchConfiguration('rviz')
    use_tour = LaunchConfiguration('tour')
    gz_gui = LaunchConfiguration('gz_gui')

    args = [
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('tour', default_value='true',
                              description='Run the autonomous waypoint tour'),
        DeclareLaunchArgument('log_level', default_value='error',
                              description='Nav2 log level (error keeps the terminal clean)'),
        DeclareLaunchArgument('gz_gui', default_value='false',
                              description='Also open the Gazebo GUI (heavy on Orin Nano)'),
    ]

    # Reuse the real robot's Nav2 config; only the odometry source differs in sim.
    nav2_params = RewrittenYaml(
        source_file=os.path.join(bringup_share, 'config', 'nav2_params.yaml'),
        param_rewrites={'use_sim_time': 'true', 'odom_topic': '/odom'},
        convert_types=True,
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # --- Gazebo server (headless rendering on the GPU via EGL) ---
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '--headless-rendering', world],
        output='log',
    )
    gz_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=IfCondition(gz_gui),
    )

    spawn_robot = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='log',
        arguments=['-topic', 'robot_description', '-name', 'amr',
                   '-x', SPAWN_X, '-y', SPAWN_Y, '-z', SPAWN_Z],
    )])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='log',
    )

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='log',
    )

    # Sim localization: map == odom (on the real robot cuVSLAM publishes this)
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    nvblox_node = Node(
        package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
        parameters=[{
            'use_sim_time': True,
            'global_frame': 'odom',
            'use_tf_transforms': True,
            'num_cameras': 1,
            'use_depth': True,
            'use_color': False,               # colour not needed for the map; saves CPU/GPU
            'use_lidar': False,
            'voxel_size': 0.05,
            'esdf_mode': '2d',
            'mapping_type': 'static_tsdf',
            'input_qos': 'SENSOR_DATA',
            'map_clearing_frame_id': 'base_link',
            'esdf_slice_bounds_visualization_attachment_frame_id': 'base_link',
            # Keep the WHOLE map for the demo (default clears >5 m from the robot
            # and slowly decays unobserved areas, which erases rooms already visited)
            'map_clearing_radius_m': -1.0,
            'static_mapper.tsdf_decay_factor': 0.9999,   # must be < 1.0
            'decay_tsdf_rate_hz': 1.0,                    # ~94% kept after 10 min
            # Visualization output is CPU-heavy; 2 Hz is enough
            'update_mesh_rate_hz': 2.0,
            'publish_layer_rate_hz': 2.0,
            # odom z=0 is axle height (5 cm above floor): count 5 cm..65 cm above floor
            'static_mapper.esdf_slice_height': 0.0,
            'static_mapper.esdf_slice_min_height': 0.0,
            'static_mapper.esdf_slice_max_height': 0.6,
        }],
        remappings=[
            ('camera_0/depth/image', '/camera/depth_image'),
            ('camera_0/depth/camera_info', '/camera/camera_info'),
        ],
        output='log',
    )

    nav2 = TimerAction(period=8.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )])

    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        # No GPU OpenGL over remote desktop -> software rendering; cap its CPU use
        additional_env={'LP_NUM_THREADS': '2'},
        condition=IfCondition(use_rviz), output='log',
    )

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{'port': 8765, 'use_sim_time': True}],
    )

    tour = TimerAction(period=15.0, actions=[Node(
        package='robot_sim', executable='waypoint_demo', name='waypoint_demo',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_tour), output='screen',
    )])

    return LaunchDescription(args + [
        gz_server, gz_client, bridge, robot_state_pub, spawn_robot, map_to_odom,
        nvblox_node, nav2, rviz, foxglove, tour,
    ])
