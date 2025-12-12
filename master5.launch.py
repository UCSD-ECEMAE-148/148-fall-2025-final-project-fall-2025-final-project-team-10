import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- PATHS ---
    vesc_config = '/home/projects/ros2_ws/src/vesc_config/vesc2.yaml'
    nav2_params = '/home/projects/ros2_ws/src/my_nav2_params.yaml'
    
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    slam_pkg = get_package_share_directory('slam_toolbox')
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        
        # 1. HARDWARE: Lidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'my_lidar.launch.py'))
        ),

        # 2. HARDWARE: VESC Driver & Odom
        Node(
            package='vesc_driver', executable='vesc_driver_node',
            name='vesc_driver_node', parameters=[vesc_config]
        ),
        Node(
            package='vesc_ackermann', executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[{
                'odom_frame': 'odom', 'base_frame': 'base_link',
                'speed_to_erpm_gain': -4614.0, 'steering_angle_to_servo_gain': -1.0,
                'wheelbase': 0.32, 'publish_tf': True,
                'speed_to_erpm_offset': 0.0, 'steering_angle_to_servo_offset': 0.5
            }],
            remappings=[('sensors/core', '/sensors/core'),
                        ('sensors/servo_position_command', '/sensors/servo_position_command')]
        ),
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

        # --- New Nodes Added ---

        # camera_pkg 5fps node
#        Node(
#            package='camera_pkg',
#            executable='5fps',
#            name='camera_5fps_node'
#        ),

        # speaker_pkg speaker_node
        Node(
            package='speaker_pkg',
            executable='speaker_node',
            name='speaker_node'
        ),

        # servo_controller servo_node
        Node(
            package='servo_controller',
            executable='servo_node',
            name='servo_node'
        ),

        # locate_pkg locate_node
#        Node(
#            package='locate_pkg',
#            executable='locate_node',
#            name='locate_node'
#        ),

        # 3. STATIC TRANSFORMS (The "Skeleton")
        Node( # base_link -> base_laser
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_laser']
        ),
        Node( # base_link -> base_footprint (RESTORED THIS ONE)
            package='tf2_ros', executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # 4. THE BRIDGE (The Heartbeat)
        ExecuteProcess(
            cmd=['python3', '/home/projects/ros2_ws/cmd_vel_to_ackermann.py'],
            output='screen'
        ),

        # 5. ROSBRIDGE (For Foxglove)
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
            )
        ),

        # 6. SLAM & NAV STACK (Delayed to let hardware settle)
        TimerAction(
            period=5.0,
            actions=[
                # SLAM
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
                    launch_arguments={'use_sim_time': 'False'}.items()
                ),
                
                # NAV2 (With Custom Params)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': 'False',
                        'map_subscribe_transient_local': 'true',
                        'params_file': nav2_params
                    }.items()
                )
            ]
        )
    ])
