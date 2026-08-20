"""
ROS 占据栅格地图生成器 - 从点云数据转换

该脚本将3D点云文件(PCD格式)转换为ROS兼容的占据栅格地图。
占据栅格地图是一种2D表示，每个栅格根据点云高程(Z轴)值
表示占据、空闲或未知状态。

转换流程：
1. 使用Open3D从PCD文件加载点云
2. 基于Z轴高度范围(z_range)过滤点
3. 将点投影到指定分辨率的2D栅格
4. 将栅格分类为占据(1)、空闲(0)或未知(0.5)
5. 保存为灰度PNG图像和符合ROS地图格式的YAML元数据

输出文件：
- {map_name}.png：灰度占据栅格图(0=黑=空闲, 255=白=占据)
- {map_name}.yaml：ROS地图元数据，包含分辨率、原点、阈值等

使用方法：
    python pointcloud_to_ros_map.py

注意：这是FAST-LIO定位系统运行前的预处理工具，用于生成地图。
"""

import numpy as np
import open3d as o3d
from matplotlib.pyplot import imsave
import yaml
import os


if __name__ == "__main__":
    # ========== 配置参数 ==========

    # 输入点云文件路径(PCD格式)
    # 该文件应包含来自LiDAR或其他传感器的3D点数据
    #happy - pcd 2 map
    pcd_path = (
        "/home/unicon/nav_ws/src/bringup/map/3d/0810.pcd"
    )
    
    # 输出目录，用于生成地图文件(PNG + YAML)
    #happy - pcd 2 map
    out_path = "/home/unicon/nav_ws/src/bringup/map/2d"    
    
    # Z轴高度范围，用于占据分类(单位：米)
    # 低于 z_range[0] 的点标记为 占据 (值 1.0)
    # 介于 z_range[0] 和 z_range[1] 之间的点标记为 空闲 (值 0.0)
    # 高于 z_range[1] 的点被忽略(保持 未知/0.5)
    #happy - boundary for z axis
    z_range = [-10.5, 20.5]
    
    # 栅格分辨率，单位：米/栅格
    # 较小的值创建高分辨率地图，但会增加内存使用
    #happy - 정밀도
    res = 0.05
    
    # 输出文件的基础名称(将生成 {map_name}.png 和 {map_name}.yaml)
    map_name = "0810"

    # ========== 加载点云 ==========
    
    # 使用Open3D库从PCD文件加载点云
    # Open3D支持多种点云格式，包括PCD、PLY、XYZ等
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Open3D PointCloud를 NumPy 배열로 변환
    pcd = np.asarray(pcd.points)

    # ========== 计算栅格边界 ==========
    
    # 计算点云在X-Y平面的边界框
    x_min = np.min(pcd[:, 0])  # X坐标最小值
    x_max = np.max(pcd[:, 0])  # X坐标最大值
    y_min = np.min(pcd[:, 1])  # Y坐标最小值
    y_max = np.max(pcd[:, 1])  # Y坐标最大值
    
    # Z轴统计已计算但当前未使用
    # z_min = np.min(pcd[:, 2])
    # z_max = np.max(pcd[:, 2])

    # ========== 计算栅格维度 ==========
    
    # 根据分辨率计算栅格宽度和高度
    # 加1以确保所有点都包含在栅格内
    # 栅格维度单位为栅格数(非米)
    w = int((x_max - x_min) / res) + 1  # 宽度(X轴)
    h = int((y_max - y_min) / res) + 1  # 高度(Y轴)

    # ========== 栅格初始化 ==========
    
    # 初始化占据栅格，未知值设为0.5
    # 形状：(高度, 宽度) = (h, w)
    # 数据类型稍后转换为uint8(0-255)
    grid = np.full((h, w), 0.5)

    # ========== 点处理和栅格填充 ==========
    
    # 遍历点云中的每个点
    for p in pcd:
        # 将点坐标转换为栅格单元索引
        # X坐标：(point_x - min_x) / resolution 得到列索引
        x_grid = int((p[0] - x_min) / res)
        
        # Y坐标翻转 (h - 1 - ...) 因为：
        # - 图像坐标系原点在左上角(0,0)
        # - ROS地图坐标系原点在左下角
        # - 我们希望地图原点在左下角以符合ROS约定
        y_grid = h - 1 - int((p[1] - y_min) / res)

        # 确保索引在栅格边界内
        if 0 <= x_grid < w and 0 <= y_grid < h:
            # ===== Z轴分类 =====
            
            # 基于点的Z(高度)坐标分类栅格
            if p[2] < z_range[0]:
                # 低于下限的点为 占据 (例如：地面)
                grid[y_grid, x_grid] = 1.0
                
            elif p[2] >= z_range[0] and p[2] <= z_range[1]:
                # 在阈值范围内的点为 空闲 (可导航空间)
                grid[y_grid, x_grid] = 0.0
                
            # 高于上限的点被忽略(保持 未知 = 0.5)
            # 这允许过滤掉天花板或高物体等障碍物

    # ========== 归一化和保存 ==========
    
    # 将占据值从 [0.0, 1.0] 转换为 uint8 [0, 255]
    # 乘以255得到8位灰度值：
    # - 0   = 黑色  = 空闲空间
    # - 255 = 白色  = 占据
    # - 128 = 灰色  = 未知
    grid = (grid * 255).astype(int)

    # 创建ROS地图元数据字典，遵循ROS nav_msgs/OccupancyGrid格式
    map_dict = {
        "image": f"{map_name}.png",          # 灰度地图图像文件名
        "resolution": res,                    # 每栅格米数(栅格大小)
        "origin": [float(x_min), float(y_min), 0.0],  # 地图在世界坐标系中的原点(x, y, yaw)
        "occupied_thresh": 0.6,              # 占据概率阈值，高于此值视为占据(0-1)
        "free_thresh": 0.3,                  # 空闲概率阈值，低于此值视为空闲(0-1)
        "negate": 0,                          # 是否反转占据值(0=否, 1=是)
    }

    # 保存占据栅格为灰度PNG图像
    # Matplotlib的imsave使用cmap='gray'映射：
    # - 0   → 黑色(空闲)
    # - 255 → 白色(占据)
    # - 128 → 灰色(未知)
    imsave(os.path.join(out_path, f"{map_name}.png"), grid, cmap="gray")

    # 保存地图元数据为YAML文件，兼容ROS map_server
    # 该YAML由ROS的map_server节点加载以提供占据栅格
    with open(os.path.join(out_path, f"{map_name}.yaml"), "w") as file:
        yaml.dump(map_dict, file, default_flow_style=None)
