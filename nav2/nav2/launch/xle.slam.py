import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    rplidar_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch_file = os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
    
    # Path to our new filter config
    filter_params = '/home/rpd/semantic_nav/src/semantic_nav/nav2/nav2/params/laser_filter.yaml'

    return LaunchDescription([

        # 1. RPLiDAR (Wrapped to publish raw data to /scan_raw instead of /scan)
        GroupAction([
            SetRemap(src='/scan', dst='/scan_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rplidar_launch_file),
                launch_arguments={'serial_port': '/dev/ttyUSB1'}.items()
            )
        ]),

        # 2. Laser Filter (Reads the YAML file, removes rods, outputs to /scan)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_params],
            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan')
            ]
        ),

        # 3. Static TF Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.01', '0.0', '0.6', '1.57', '0.0', '0.0', 'base_link', 'laser']
        ),

        # 4. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'transform_timeout': '0.5'
            }.items()
        ),
    ])
