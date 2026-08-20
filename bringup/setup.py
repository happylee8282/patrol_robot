from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'tools'),
            glob('tools/*.cpp')),
        (os.path.join('share', package_name, 'map', '3d'),
            glob('map/3d/*.pcd')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unicon',
    maintainer_email='ursonice@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uart=bringup.uart_ros2_humble:main',
            'wheel_cmd=bringup.wheel_cmd_ros2_humble:main',
            'mode_keyboard=bringup.mode_keyboard_ros2_humble:main',
            'waypoint_sender=bringup.waypoint_sender:main',
            'nav2_tf_2d=bringup.nav2_tf_2d:main',
        ],
    },
)
