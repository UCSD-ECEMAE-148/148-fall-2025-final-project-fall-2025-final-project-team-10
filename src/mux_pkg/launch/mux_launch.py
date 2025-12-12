from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Twist Mux node
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=['/home/projects/ros2_ws/src/mux_pkg/mux_pkg/twist_mux_params.yaml']
        ),

        # Stop publisher
        Node(
            package='mux_pkg',
            executable='stop_publisher',
            name='stop_publisher',
            output='screen'
        ),

        # Controller node
        Node(
            package='mux_pkg',
            executable='mux_controller',
            name='mux_controller',
            output='screen'
        )
    ])
