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
        # Camera with IMU enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gem2l_launch),
            launch_arguments={
                'enable_align_depth': 'true',
                'depth_fps': '10',
                'color_fps': '10',
                # Enable built-in 6-axis IMU for sensor fusion
                'enable_sync_output_accel_gyro': 'true',
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'accel_rate': '200hz',
                'gyro_rate': '200hz',
                # Disable camera TF publishing - we use URDF for camera_link position
                'publish_tf': 'false',
            }.items()
        ),

        # Static TF base_link -> camera_link
        # Static TF base_link -> camera_link REMOVED (Handled by URDF)

        # Static TF map -> odom (Identity fallback)
        # This provides a valid map->odom TF until SLAM Toolbox starts providing it.
        # Ensures Nav2 can activate without TF errors.
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_map_odom_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # ),

        # ORB-SLAM3
        Node(
            package='orbslam3',
            executable='rgbd',
            name='orbslam3',
            output='screen',
            arguments=[voc_path, tum3_yaml],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('depth/image',     '/camera/depth/image_raw'),
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

        # NEW: Mapping Pipeline (Pointcloud->Laser + SLAM Toolbox)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vslam_share, 'launch', 'mapping.launch.py')
            )
        ),
    ])
