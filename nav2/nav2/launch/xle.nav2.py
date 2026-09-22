import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Pointing strictly to the new opus_params.yaml (MPPI, holonomic, goal-yaw enforced)
    map_file    = '/home/rpd/semantic_nav/src/semantic_nav/nav2/nav2/map/xle_room_map.yaml'
    filter_yaml = '/home/rpd/semantic_nav/src/semantic_nav/nav2/nav2/params/laser_filter.yaml'
    nav2_params = '/home/rpd/semantic_nav/src/semantic_nav/nav2/nav2/params/my_nav2_params.yaml'

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/xle_lidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
        remappings=[('scan', '/scan_raw')]
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[filter_yaml],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )

    # Base to Laser TF (confirmed correct: x=0.01, z=0.6, yaw=1.57)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.01', '0.0', '0.6', '1.57', '0.0', '0.0', 'base_link', 'laser']
    )

    # Footprint to Link TF (identity)
    static_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_link_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        rplidar_node,
        laser_filter_node,
        static_tf_laser,
        static_tf_footprint,
        nav2_launch
    ])
