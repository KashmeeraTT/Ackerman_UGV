
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    fastdds_config = os.path.join(pkg_robot_bringup, 'config', 'fastdds_no_shm.xml')
    pkg_perception_vslam = get_package_share_directory('perception_vslam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup_ack')

    # Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_micro_ros = LaunchConfiguration('use_micro_ros')
    
    set_fastdds_env = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_config)
    
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

    # Health Monitor (Production monitoring)
    health_monitor_node = Node(
        package='robot_bringup',
        executable='health_monitor.py',
        name='health_monitor',
        output='screen',
        parameters=[{
            'odom_timeout': 2.0,
            'safe_stop_enabled': True,
            'diagnostics_rate': 1.0
        }]
    )

    # EKF Sensor Fusion (OPTIONAL ENHANCEMENT)
    # Always runs, provides /odom_filtered for enhanced localization when IMUs available
    # Visual SLAM (slam_odom_bridge) is ALWAYS the primary TF source
    # EKF gracefully handles missing IMU data
    ekf_config = os.path.join(pkg_robot_bringup, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odom_filtered')
        ]
    )

    # Delay EKF startup by 8 seconds to let VSLAM initialize
    from launch.actions import TimerAction
    delayed_ekf = TimerAction(
        period=8.0,  # Wait 8 seconds for VSLAM to start publishing /odom
        actions=[ekf_node]
    )

    # Adaptive Sensor Fusion (health monitoring)
    # Monitors IMU reliability against expected motion state
    # Uses lazy subscriptions - camera topics subscribed after 15s internal delay
    adaptive_fusion_node = Node(
        package='sensor_fusion',
        executable='adaptive_fusion.py',
        name='adaptive_fusion',
        output='screen',
        parameters=[{
            'stationary_threshold': 0.02,
            'imu_variance_threshold': 0.5,
            'sensor_timeout': 0.5,
            'startup_delay': 15.0  # Internal delay before camera subscription
        }]
    )

    # Delay adaptive fusion startup by 10 seconds (plus 15s internal delay = 25s total)
    delayed_adaptive_fusion = TimerAction(
        period=10.0,  # Wait 10 seconds for camera topics to be available
        actions=[adaptive_fusion_node]
    )

    # Command Velocity Priority Mux
    # Routes navigation commands through mux, with safety commands having priority
    # Safety: health_monitor -> /cmd_vel_safety -> mux
    # Nav: twist_to_ackermann -> /cmd_vel_nav_mux -> mux -> /cmd_vel
    cmd_vel_mux_node = Node(
        package='robot_bringup',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'safety_timeout': 1.0,
            'nav_timeout': 0.5,
            'output_rate': 20.0
        }]
    )

    # Sensor Fusion Status GUI
    status_gui_node = Node(
        package='sensor_fusion',
        executable='status_gui.py',
        name='sensor_fusion_gui',
        output='screen'
    )

    return LaunchDescription([
        set_fastdds_env,  # Disable FastDDS shared memory (prevents some issues)
        declare_use_rviz,
        declare_use_micro_ros,
        declare_serial_port,
        rsp_node,
        jsp_node,
        vslam_launch,
        delayed_ekf,              # EKF sensor fusion (8s delay)
        delayed_adaptive_fusion,  # Sensor health monitoring (10s launch + 15s internal delay)
        cmd_vel_mux_node,         # Priority mux for cmd_vel (safety > nav)
        # status_gui_node,        # GUI - run manually: ros2 run sensor_fusion status_gui.py
        nav2_launch,
        ackermann_bridge_launch,
        micro_ros_agent,
        rviz_node,
        health_monitor_node
    ])

