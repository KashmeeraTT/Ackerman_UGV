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
                'transform_tolerance': 1.0,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,   # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.4,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/camera/depth/points'),
                ('scan', '/scan')
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
                'scan_topic': '/scan',
                'mode': 'mapping', # defaults to mapping
                # Match laser range to actual Orbbec camera/depth sensor capabilities
                'min_laser_range': 0.4,  # Match sensor minimum range
                'max_laser_range': 4.0,       # Match sensor maximum range
            }]
        )
    ])
