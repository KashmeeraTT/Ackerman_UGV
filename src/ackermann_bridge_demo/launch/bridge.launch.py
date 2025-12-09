import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ackermann_bridge_demo'),
        'ackermann_bridge_demo',
        'params.yaml'
    )
    return LaunchDescription([
        Node(package='ackermann_bridge_demo', executable='twist_to_ackermann',
             name='twist_to_ack', output='screen', parameters=[params])
    ])
