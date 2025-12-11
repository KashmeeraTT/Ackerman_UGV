from setuptools import setup, find_packages

package_name = 'nav2_bringup_ack'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/nav2_bringup.launch.py']),
        ('share/' + package_name + '/config',
         ['config/nav2_params.yaml']),
        ('share/' + package_name + '/scripts',
         ['scripts/activate_nav2.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavierai',
    maintainer_email='you@example.com',
    description='Nav2 bringup for ackermann robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            # no python nodes here, just launch + params
        ],
    },
)
