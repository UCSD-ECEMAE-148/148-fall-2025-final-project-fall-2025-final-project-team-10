import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    vesc_config_path = '/home/projects/ros2_ws/src/vesc_config/vesc2.yaml'
    
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    slam_pkg = get_package_share_directory('slam_toolbox')
    rosbridge_pkg = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'my_lidar.launch.py'))
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[vesc_config_path] # Loads port/baudrate from your yaml
        ),

        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'speed_to_erpm_gain': 4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': 1.0,
                'steering_angle_to_servo_offset': 0.0,
                'wheelbase': 0.25
            }],
            remappings=[
                ('sensors/core', '/sensors/core'),
                ('sensors/servo_position_command', '/sensors/servo_position_command')
            ]
        ),

        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[{
                'speed_to_erpm_gain': 4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': 1.0,
                'steering_angle_to_servo_offset': 0.0
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_laser']
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'scan_topic': '/scan',
                        'odom_topic': '/odom',
                        'base_frame': 'base_link', # Force SLAM to use base_link
                        'use_sim_time': 'False'
                    }.items()
                )
            ]
        )
    ])
