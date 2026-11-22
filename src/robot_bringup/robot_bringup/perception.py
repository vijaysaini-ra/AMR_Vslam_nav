"""
perception.py -- shared GPU perception pipeline for the AMR (used by mapping_launch.py and
bringup_launch.py so both always run the identical, tested configuration).

ONE PROCESS, GPU:
  RealSense driver, cuVSLAM and nvblox are loaded as components into a single
  component_container_mt (no inter-process transport). cuVSLAM and nvblox take the
  frames as NITROS images and upload them to the GPU once.
  All heavy processing then runs on the GPU:
    - stereo rectification: on the D455 ASIC
    - feature tracking / SLAM: cuVSLAM (CUDA)
    - TSDF integration, ESDF, mesh, occupancy grid: nvblox (CUDA)

CRASH PROTECTION:
  - nvblox workspace bounds: blocks are only allocated in a fixed height band and a
    bounded area, so a bad pose can never grow the map until the GPU runs out of memory
    (previous crash: cudaMallocAsync out of memory -> SIGSEGV).
  - nvblox integrates depth at a capped rate (frees GPU/CPU time for cuVSLAM).
  - cuVSLAM IMU fusion is optional (it diverged when frames were dropped under load).
  - IR emitter is forced off after start-up (D455 firmware re-enables it).
"""

from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# NOTE: use_intra_process_comms cannot be enabled: on Humble, intra-process only
# allows VOLATILE publishers, and cuVSLAM / nvblox create transient-local ones
# ("intraprocess communication allowed only with volatile durability").
# Same-process loading still avoids inter-process transport; images are uploaded to
# the GPU once by the NITROS subscribers in cuVSLAM and nvblox.
INTRA = []


def realsense_component():
    return ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera', namespace='camera',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': True,
            'enable_color': False,
            'pointcloud.enable': False,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,              # linear interpolation -> /camera/imu
            'enable_sync': True,
            'depth_module.infra_profile': '848x480x30',
            'depth_module.profile': '848x480x30',
            'gyro_fps': 200,
            'accel_fps': 200,
            'depth_module.emitter_enabled': 0,
            'publish_tf': True,
            'tf_publish_rate': 0.0,             # static TFs only
        }],
        extra_arguments=INTRA,
    )


def cuvslam_component(imu_fusion):
    """imu_fusion: LaunchConfiguration / substitution evaluating to 'true'/'false'."""
    return ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': ParameterValue(imu_fusion, value_type=bool),
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 34.00,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'publish_map_to_odom_tf': True,
            'publish_odom_to_base_tf': True,
            'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': ['camera_infra1_optical_frame',
                                      'camera_infra2_optical_frame'],
            'path_max_size': 5000,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/imu'),
        ],
        extra_arguments=INTRA,
    )


def nvblox_component(slice_min, slice_max, slice_height, area_half_size_m=15.0):
    """Heights are in the odom frame. odom z=0 is the floor when base_link is on the floor."""
    return ComposableNode(
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        name='nvblox_node',
        parameters=[{
            'global_frame': 'odom',
            'use_tf_transforms': True,
            'num_cameras': 1,
            'use_depth': True,
            'use_color': False,
            'use_lidar': False,
            'voxel_size': 0.05,
            'esdf_mode': '2d',
            'mapping_type': 'static_tsdf',
            'input_qos': 'SENSOR_DATA',
            'map_clearing_frame_id': 'base_link',
            'esdf_slice_bounds_visualization_attachment_frame_id': 'base_link',
            # keep the whole map (default clears > 5 m from the robot)
            'map_clearing_radius_m': -1.0,
            'static_mapper.tsdf_decay_factor': 0.9999,   # must be < 1.0
            'decay_tsdf_rate_hz': 1.0,
            # passive-stereo depth is unreliable far away
            'static_mapper.projective_integrator_max_integration_distance_m': 4.0,
            'static_mapper.esdf_slice_height': slice_height,
            'static_mapper.esdf_slice_min_height': slice_min,
            'static_mapper.esdf_slice_max_height': slice_max,
            # GPU memory protection: never allocate outside this box.
            # NOTE: in nvblox 3.2 these MUST carry the "static_mapper." prefix; without it
            # they are silently ignored (bounds stay kUnbounded) -> CUDA out of memory.
            # Check: ros2 param get /nvblox_node static_mapper.workspace_bounds_type
            'static_mapper.workspace_bounds_type': 'bounding_box',
            'static_mapper.workspace_bounds_min_height_m': -0.3,
            'static_mapper.workspace_bounds_max_height_m': 2.2,
            'static_mapper.workspace_bounds_min_corner_x_m': -area_half_size_m,
            'static_mapper.workspace_bounds_min_corner_y_m': -area_half_size_m,
            'static_mapper.workspace_bounds_max_corner_x_m': area_half_size_m,
            'static_mapper.workspace_bounds_max_corner_y_m': area_half_size_m,
            # rates: depth 30 Hz in, integrate 15 Hz is plenty
            'integrate_depth_rate_hz': 15.0,
            'update_esdf_rate_hz': 5.0,
            'update_mesh_rate_hz': 2.0,
            'publish_layer_rate_hz': 2.0,
        }],
        remappings=[
            ('camera_0/depth/image', '/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
        ],
        extra_arguments=INTRA,
    )


def perception_container(imu_fusion, slice_min, slice_max, slice_height):
    return ComposableNodeContainer(
        name='perception_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            realsense_component(),
            cuvslam_component(imu_fusion),
            nvblox_component(slice_min, slice_max, slice_height),
        ],
        # Higher CPU priority than the desktop / screen recorder: if camera frames are
        # starved, cuVSLAM tracking degrades (seen: 4.5 Hz odometry under load).
        prefix='nice -n -10',
        output='screen',
    )


def emitter_set(value='0', delay=10.0):
    """D455 firmware re-enables the IR emitter when streaming starts, ignoring the start-up
    parameter; set the wanted state again once the camera is running.
    value: '0'/'1' or a substitution producing it."""
    return TimerAction(period=delay, actions=[ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/camera/camera', 'depth_module.emitter_enabled', value],
        output='log')])
