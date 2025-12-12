import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- PATHS ---
    vesc_config = '/home/projects/ros2_ws/src/vesc_config/vesc2.yaml'

    return LaunchDescription([
        
        # 1. VESC Driver Node
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[vesc_config]
        ),

        # 2. VESC to Odometry Node
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'speed_to_erpm_gain': -4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -1.0,
                'steering_angle_to_servo_offset': 0.5,
                'wheelbase': 0.32,
                'publish_tf': True
            }],
            remappings=[
                ('sensors/core', '/sensors/core'),
                ('sensors/servo_position_command', '/sensors/servo_position_command')
            ]
        ),

        # 3. Ackermann to VESC Node
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[{
                'speed_to_erpm_gain': -4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -1.0,
                'steering_angle_to_servo_offset': 0.5
            }]
        ),

        # 4. Static Transform: base_link -> base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        )
    ])
