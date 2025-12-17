from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Pointcloud to Laserscan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': 'camera_link', 
                'transform_tolerance': 0.5,     # Reduced from 1.0 for tighter sync
                'min_height': -0.1,             # Slightly below ground to catch slopes
                'max_height': 0.8,              # Reduced from 1.0 (focus on obstacles)
                'angle_min': -1.5708,           # -M_PI/2
                'angle_max': 1.5708,            # M_PI/2
                'angle_increment': 0.01745,     # π/180 = exactly 181 readings (fixes off-by-one warning)
                'scan_time': 0.1,               # Faster updates (was 0.333)
                'range_min': 0.3,               # Reduced from 0.4 to catch closer obstacles
                'range_max': 6.0,               # Increased from 4.0 for better planning
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/camera/depth/points'),
                ('scan', '/scan')
            ]
        ),

        # Scan Relay (BestEffort -> Reliable)
        Node(
            package='robot_bringup',
            executable='scan_relay.py',
            name='scan_relay',
            output='screen',
            remappings=[
                ('scan', '/scan'),
                ('scan_reliable', '/scan_reliable')
            ]
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan_reliable',
                'mode': 'mapping', # defaults to mapping
                # Match laser range to actual Orbbec camera/depth sensor capabilities
                'min_laser_range': 0.25,  # Slightly below sensor's 0.3m to avoid warning
                'max_laser_range': 6.0,  # Match range_max from pointcloud_to_laserscan
            }]
        )
    ])
