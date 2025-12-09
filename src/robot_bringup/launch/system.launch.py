
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_perception_vslam = get_package_share_directory('perception_vslam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup_ack')

    # Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    rviz_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'rviz', 'robot.rviz']
    )

    # Robot State Publisher (URDF)
    urdf_file = os.path.join(pkg_robot_bringup, 'description', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Motor Driver Placeholder
    # We run this from the source or installed script.
    # Since it's a script in an ament_cmake package, we assume it's installed to lib/robot_bringup
    # or just run as a process if we install it as a program.
    # micro-ROS Agent (Low-level Driver)
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port, '-b', '115200'
        ],
        name='micro_ros_agent',
        output='screen'
    )

    # VSLAM Bringup
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception_vslam, 'launch', 'vslam_bringup.launch.py')
        )
    )

    # Nav2 Bringup
    # We delay Nav2 slightly or just launch it parallel.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'nav2_bringup.launch.py')
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_rviz,
        rsp_node,
        vslam_launch,
        nav2_launch,
        micro_ros_agent,
        rviz_node
    ])
