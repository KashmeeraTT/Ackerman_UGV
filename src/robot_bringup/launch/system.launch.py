
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
    use_micro_ros = LaunchConfiguration('use_micro_ros')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_use_micro_ros = DeclareLaunchArgument(
        'use_micro_ros',
        default_value='false',
        description='Whether to start micro-ROS agent (set to true if motors are connected)'
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for micro-ROS agent'
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

    # Joint State Publisher (Smart Publisher for Visualization)
    jsp_node = Node(
        package='robot_bringup',
        executable='smart_joint_publisher.py',
        name='joint_state_publisher',
        output='screen'
    )

    # Motor Driver Placeholder
    # We run this from the source or installed script.
    # Since it's a script in an ament_cmake package, we assume it's installed to lib/robot_bringup
    # or just run as a process if we install it as a program.
    # micro-ROS Agent (Low-level Driver)
    serial_port = LaunchConfiguration('serial_port')

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', serial_port, '-b', '115200'
        ],
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(use_micro_ros)
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

    # Ackermann Bridge
    pkg_ackermann_bridge = get_package_share_directory('ackermann_bridge_demo')
    ackermann_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ackermann_bridge, 'launch', 'bridge.launch.py')
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
        declare_use_micro_ros,
        declare_serial_port,
        rsp_node,
        jsp_node,
        vslam_launch,
        nav2_launch,
        ackermann_bridge_launch,
        micro_ros_agent,
        rviz_node
    ])
