"""
mapping_launch.py -- build a map of a real room with the RealSense D455 + Jetson (no motors).

  perception_container (ONE process, zero-copy, GPU):  D455 driver + cuVSLAM + nvblox
     -> TF map->odom->base_link, 2D occupancy grid /nvblox_node/static_occupancy_grid, 3D mesh
  RViz2      live map, 3D mesh, camera trajectory
  Foxglove   ws://<jetson-ip>:8765

HANDHELD MAPPING: base_link is placed on the FLOOR directly below the camera, so the
map's obstacle height band is measured from the floor. Hold the camera level at about
cam_height, keep it still ~3 s at start, then walk slowly.

  ros2 launch robot_bringup mapping_launch.py                   # handheld at ~1.0 m
  ros2 launch robot_bringup mapping_launch.py cam_height:=0.20  # camera on the robot
  ros2 launch robot_bringup mapping_launch.py emitter:=true     # denser depth, weaker tracking
  ros2 launch robot_bringup mapping_launch.py imu:=true         # visual-inertial tracking
Save the map with ~/save_map.sh <name> (host) while this is running.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from robot_bringup.perception import emitter_set, perception_container


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    cam_height = LaunchConfiguration('cam_height')
    emitter = LaunchConfiguration('emitter')

    args = [
        DeclareLaunchArgument('cam_height', default_value='1.0',
                              description='Camera height above the floor (m)'),
        DeclareLaunchArgument('emitter', default_value='false',
                              description='IR emitter on: denser depth, but hurts cuVSLAM'),
        DeclareLaunchArgument('imu', default_value='false',
                              description='IMU fusion in cuVSLAM'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    # base_link on the floor directly below the camera
    base_to_camera = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_camera_tf',
        arguments=['--z', cam_height, '--frame-id', 'base_link',
                   '--child-frame-id', 'camera_link'],
        output='log',
    )

    # obstacles = anything 10 cm .. 1.2 m above the floor
    perception = perception_container(
        imu_fusion=LaunchConfiguration('imu'),
        slice_min=0.10, slice_max=1.2, slice_height=0.3)

    emitter_fix = emitter_set(PythonExpression(["'1' if '", emitter, "' == 'true' else '0'"]))

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'mapping.rviz')],
        additional_env={'LP_NUM_THREADS': '2'},
        condition=IfCondition(LaunchConfiguration('rviz')), output='log',
    )

    foxglove = Node(package='foxglove_bridge', executable='foxglove_bridge',
                    name='foxglove_bridge', parameters=[{'port': 8765}], output='log')

    return LaunchDescription(args + [
        base_to_camera, perception, emitter_fix, rviz, foxglove,
    ])
