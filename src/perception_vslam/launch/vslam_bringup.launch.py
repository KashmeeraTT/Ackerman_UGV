from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    orbbec_share = get_package_share_directory('orbbec_camera')
    gem2l_launch = os.path.join(orbbec_share, 'launch', 'gemini2L.launch.py')

    orbslam_share = get_package_share_directory('orbslam3')
    voc_path = os.path.join(orbslam_share, 'vocabulary', 'ORBvoc.txt')
    tum3_yaml = os.path.join(orbslam_share, 'config', 'rgb-d', 'Orbbec_Gemini2.yaml')

    vslam_share = get_package_share_directory('perception_vslam')
    bridge_params = os.path.join(vslam_share, 'config', 'slam_odom_bridge.yaml')

    return LaunchDescription([
        # Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gem2l_launch),
            launch_arguments={'enable_align_depth': 'true'}.items()
        ),

        # Static TF base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_cam_tf',
            # Apply optical rotation: yaw=-90, pitch=0, roll=-90
            # x=0.45 (front), y=0.0, z=1.5 (height)
            arguments=['0.45', '0.0', '1.5', '-1.5707', '0.0', '-1.5707',
                       'base_link', 'camera_link']
        ),

        # Static TF map -> odom (Identity)
        # Required because slam_odom_bridge generally bridges odom->base_link
        # and we need to link map->odom for Nav2 global costmap in map frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom_tf',
            arguments=['0','0','0','0','0','0','map','odom']
        ),

        # ORB-SLAM3
        Node(
            package='orbslam3',
            executable='rgbd',
            name='orbslam3',
            output='screen',
            arguments=[voc_path, tum3_yaml],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
            ]
        ),

        # NEW: SLAM ➜ /odom bridge
        Node(
            package='slam_odom_bridge',
            executable='slam_odom_bridge',
            name='slam_odom_bridge',
            output='screen',
            parameters=[bridge_params],
        ),
    ])
