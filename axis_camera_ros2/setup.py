import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'axis_camera_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='ROS 2 driver for AXIS M5075 MJPEG streaming and PTZ control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_camera_node = axis_camera_ros2.axis_camera_node:main',
            'axis_ptz_node = axis_camera_ros2.axis_ptz_node:main',
            'rgb_detector = axis_camera_ros2.rgb_detector:main',
        ],
    },
)
