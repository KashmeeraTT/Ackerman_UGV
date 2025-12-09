from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_bridge_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, package_name), glob(package_name + '/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xavierai',
    maintainer_email='xaviereconsulate22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = ackermann_bridge_demo.twist_to_ackermann:main'
        ],
    },
)
