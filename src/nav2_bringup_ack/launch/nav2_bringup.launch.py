import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Path to our Nav2 params
    bringup_dir = get_package_share_directory('nav2_bringup_ack')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # Path to the default Nav2 behavior tree XML
    bt_dir = get_package_share_directory('nav2_bt_navigator')
    bt_xml = os.path.join(
        bt_dir,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    return LaunchDescription([
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),


        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

        # Lifecycle manager - using params_file for all configuration
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file],
        ),
    ])
