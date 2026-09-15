from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    scripts=['tools/pose_coordinate_recorder.py'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'ps4'),
            glob('config/ps4/*.yaml')),
        (os.path.join('share', package_name, 'tools'),
            glob('tools/*.cpp') + glob('tools/*.py') + glob('tools/*.mp3')),
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
            'nav2_tf_2d=bringup.nav2_tf_2d:main',
            'json_navigation=bringup.json_navigation:main',
            'robot_connection=bringup.robot_connection:main',
            'robot_speaker=bringup.robot_speaker:main',
            'light_node=bringup.light_node:main',
            'ps4_control=bringup.ps4:main',
        ],
    },
)
