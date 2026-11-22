"""
bringup_launch.py -- full autonomous navigation on the real robot.

  perception_container (ONE process, GPU; see robot_bringup/perception.py):
      RealSense D455 driver + cuVSLAM (map->odom->base_link TF) + nvblox (2D ESDF slice)
  Nav2      NavFn A* planner + DWB controller, obstacles from the nvblox costmap layer
  Foxglove  ws://<jetson-ip>:8765

TF tree:
  map --(cuVSLAM)--> odom --(cuVSLAM)--> base_link --(static, below)--> camera_link
      --(realsense driver)--> camera_*_optical_frame

Run with:
  ros2 launch robot_bringup bringup_launch.py
  ros2 launch robot_bringup bringup_launch.py cam_x:=0.18 cam_z:=0.12   # real mount offset
  ros2 launch robot_bringup bringup_launch.py imu:=true                 # visual-inertial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_bringup.perception import emitter_set, perception_container


def generate_launch_description():

    nav2_params_file = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'nav2_params.yaml')

    # Camera mount relative to base_link (centre of the robot, at axle level).
    # Defaults come from ugv_description/urdf/ugv.urdf.xacro (realsense_mount).
    # MEASURE YOUR ROBOT and override these.
    args = [
        DeclareLaunchArgument('cam_x', default_value='0.20'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.15'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('imu', default_value='false',
                              description='IMU fusion in cuVSLAM'),
    ]

    # base_link is at axle height (~5 cm above the floor): obstacles 5 cm .. 0.5 m up
    perception = perception_container(
        imu_fusion=LaunchConfiguration('imu'),
        slice_min=0.05, slice_max=0.5, slice_height=0.0)

    static_tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='log',
    )

    # Nav2 (planner_server uses NavFn with use_astar: true)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{'port': 8765}], output='log',
    )

    return LaunchDescription(args + [
        foxglove_bridge,
        static_tf_node,
        perception,
        emitter_set('0'),
        nav2_bringup_launch,
    ])
