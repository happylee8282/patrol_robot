from setuptools import setup

package_name = 'axis_image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unicon',
    maintainer_email='unicon@todo.todo',
    description='AXIS RGB image processor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_detector = axis_image_processor.rgb_detector:main',
        ],
    },
)
