from setuptools import setup, find_packages

package_name = 'perception_vslam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/vslam_bringup.launch.py', 'launch/mapping.launch.py']),
        ('share/' + package_name + '/config/camera',
         ['config/camera/gemini_2l.yaml']),
        ('share/' + package_name + '/config/slam',
         ['config/slam/orbslam_rgbd.yaml']),
        ('share/' + package_name + '/config',
         ['config/slam_odom_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavierai',
    maintainer_email='you@example.com',
    description='Orbbec + ORB-SLAM3 bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slam_odom_bridge = perception_vslam.slam_odom_bridge:main',
        ],
    },
)
