"""
FAST_LIO_LOCALIZATION ROS2包的setuptools安装配置

该setup.py文件为fast_lio_localization ROS2包配置Python包的安装。
使用setuptools定义包元数据、依赖项、入口点(可执行脚本)
和数据文件的安装。

该文件被colcon build / ROS2用于：
    - 将Python模块安装到ROS2环境
    - 创建可执行命令(entry_points)
    - 将配置、启动和资源文件复制到适当位置
    - 声明包依赖关系

标准ROS2 Python包结构：
    fast_lio_localization/
        setup.py              (本文件)
        package.xml           (ROS包清单文件)
        fast_lio_localization/ (Python模块目录)
            __init__.py
            fastlio_mapping.py
            global_localization.py
            transform_fusion.py
            invert_livox_scan.py
        launch/               (ROS2启动文件目录)
            livox.launch.py
            localization.launch.py
        config/               (YAML参数配置文件目录)
            mid360.yaml
        rviz/                 (RViz配置文件目录)
            fastlio_localization.rviz

安装方法：
    colcon build --packages-select fast_lio_localization
    source install/setup.bash

验证安装：
    ros2 pkg list | grep fast_lio_localization
"""

from setuptools import setup
import os
import glob


# ========== 包标识 ==========

package_name = "fast_lio_localization"
"""
ROS2包名称。必须与以下三者匹配：
    1. src/目录中的文件夹名称
    2. package.xml中的<name>标签
    3. Python模块名称(如不同，需使用'packages'参数)
"""

# ========== setup配置 ==========

setup(
    # 包元数据
    name=package_name,
    version="0.0.0",
    description="Fast LIO Localization ROS2 package",
    license="TODO",  # TODO：替换为实际许可证(如"BSD-3-Clause")
    
    # 要安装的Python包
    # packages=[package_name]确保Python模块目录被安装
    # 必须与实际Python模块名称匹配
    packages=[package_name],
    
    # 安装依赖
    # setuptools总是必需的；在此添加其他Python依赖(如numpy, open3d)
    install_requires=["setuptools"],
    
    # zip_safe：包是否可作为zip压缩包安装
    # 对于可能需要文件系统访问的ROS包，False更安全
    zip_safe=True,
    
    # 测试依赖(用于pytest)
    tests_require=["pytest"],

    # ========== 入口点(控制台脚本) ==========
    # 这些创建安装后可用的ROS2命令行可执行文件。
    # 每个入口将命令名映射到Python函数(模块:函数)。
    # 
    # 安装后的使用方法：
    #   ros2 run fast_lio_localization global_localization
    #   (调用global_localization.py中的main()函数)
    #
    # 入口点格式：
    #   "命令名 = python模块:函数名"
    #
    # 注意：Python文件必须是包含main()函数的可执行脚本
    entry_points={
        "console_scripts": [
            # global_localization：主定位节点
            # 运行全局定位算法以估计初始位姿
            "global_localization = fast_lio_localization.global_localization:main",
            
            # publish_initial_pose：位姿发布工具
            # 用于手动发布初始位姿估计
            "publish_initial_pose = fast_lio_localization.publish_initial_pose:main",
            
            # transform_fusion：坐标变换融合节点
            # 将多个位姿源融合为单一鲁棒估计
            "transform_fusion = fast_lio_localization.transform_fusion:main",
            
            # invert_livox_scan：点云倒置校正工具
            # 当LiDAR上下颠倒安装时校正点云
            "invert_livox_scan = fast_lio_localization.invert_livox_scan:main",
        ],
    },

    # ========== 数据文件 ==========
    # 随包安装的其他非Python文件。
    # 格式：(安装目录, [文件列表])
    # 
    # 安装位置(相对于ROS2 install/share/<包名>/):
    #   share/<包名>/launch/     - 启动文件
    #   share/<包名>/config/     - 配置文件YAML
    #   share/<包名>/rviz/       - RViz配置文件
    #   share/<包名>/            - package.xml (清单文件)
    #
    # glob()模式匹配：
    #   "launch/*.py"     - launch/目录下所有Python启动文件
    #   "config/*.yaml"   - config/目录下所有YAML配置文件
    #   "rviz/*.rviz"     - rviz/目录下所有RViz配置文件
    data_files=[
        # 安装package.xml到 share/<包名>/
        (os.path.join("share", package_name), ["package.xml"]),
        
        # 安装所有启动文件到 share/<包名>/launch/
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        
        # 安装所有配置文件YAML到 share/<包名>/config/
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        
        # 安装所有RViz配置文件到 share/<包名>/rviz/
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
)
