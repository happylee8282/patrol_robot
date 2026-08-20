from setuptools import find_packages, setup


package_name = "axis_web_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/axis_web_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CoastalPatrol Team",
    maintainer_email="coastalpatrol@example.com",
    description="ROS 2 AXIS camera topics to MJPEG and WebSocket bridge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "axis_web_bridge = axis_web_bridge.bridge_node:main",
        ],
    },
)
