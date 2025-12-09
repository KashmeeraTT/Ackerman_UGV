from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    """
    Launch file for UGV teleop control system
    
    Launches:
    1. micro-ROS agent (serial connection to ESP32)
    2. Teleop keyboard node
    """
    
    return LaunchDescription([
        # micro-ROS agent for ESP32 communication
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', '/dev/ttyUSB0', '-b', '115200'
            ],
            name='micro_ros_agent',
            output='screen',
        ),
        
        # Teleop keyboard node
        Node(
            package='ugv_teleop',
            executable='teleop_keyboard',
            name='ugv_teleop_keyboard',
            output='screen',
            parameters=[{
                'speed': 0.5,  # Default linear speed (m/s)
                'turn': 1.0,   # Default angular speed (rad/s)
            }]
        ),
    ])
