from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'gateway_pk'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools', 'websockets>=12,<16'],
    zip_safe=True,
    maintainer='unicon',
    maintainer_email='unicon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'axis_mjpeg_bridge=gateway_pk.axis_mjpeg_bridge:main',
            'meta_gateway=gateway_pk.meta_gateway:main',
        ],
    },
)
