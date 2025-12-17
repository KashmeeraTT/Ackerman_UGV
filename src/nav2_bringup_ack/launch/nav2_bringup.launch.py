import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    # Path to our Nav2 params
    bringup_dir = get_package_share_directory('nav2_bringup_ack')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # Nav2 nodes (start immediately)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    # Lifecycle manager - DELAYED to allow TF tree to be established
    # This prevents the "odom frame does not exist" errors during startup
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[params_file],
    )

    # Delay lifecycle manager to give VSLAM and TF time to initialize
    delayed_lifecycle_manager = TimerAction(
        period=3.0,  # 3 second delay
        actions=[lifecycle_manager]
    )

    return LaunchDescription([
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        delayed_lifecycle_manager,
    ])

