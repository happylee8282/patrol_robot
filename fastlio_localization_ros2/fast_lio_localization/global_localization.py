#!/usr/bin/env python3
"""
基于ICP的全局定位节点

此节点通过将当前LiDAR扫描与预构建的全局点云地图进行匹配，
来估计机器人在map坐标系中的位姿。使用Open3D的ICP配准算法
实现位姿估计。

主要功能:
- 从磁盘加载全局点云地图(PCD格式)
- 执行多尺度ICP配准(从粗到精)
- 根据视场角(FOV)裁剪地图以提高效率
- 发布定位估计为odometry消息
- 支持通过RViz手动设置初始位姿

订阅:
    /cloud_registered: 当前扫描(已注册的点云)
    /Odometry: 当前里程计(odom -> baselink)
    /initialpose: 来自RViz的初始位姿估计(PoseWithCovarianceStamped)

发布:
    /map_to_odom: 估计的变换(map -> odom)作为Odometry发布
    /cur_scan_in_map: 转换到map坐标系的当前扫描(调试用)
    /submap: 当前视场内的全局地图子集(调试用)

作者: FAST-LIO团队
日期: 2024
许可证: MIT
"""

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf_transformations
import ros2_numpy


class FastLIOLocalization(Node):
    """
    基于ICP的全局定位节点。
    
    此节点持续将传入的LiDAR扫描与预构建的全局地图进行匹配，
    以估计机器人相对于map坐标系的位姿。采用多尺度ICP方法
    实现鲁棒的配准。
    
    属性:
        global_map (open3d.geometry.PointCloud): 预构建的全局地图
        T_map_to_odom (numpy.ndarray): 当前map -> odom变换(4x4矩阵)
        cur_odom (nav_msgs.msg.Odometry): 最新的odometry消息
        cur_scan (open3d.geometry.PointCloud): 当前LiDAR扫描
        initialized (bool): 是否已设置初始位姿
        tf_buffer (tf2_ros.Buffer): 坐标变换缓冲区
        tf_listener (tf2_ros.TransformListener): 坐标变换监听器
        
    参数:
        map_voxel_size (float): 全局地图体素下采样大小(默认: 0.4米)
        scan_voxel_size (float): 扫描体素下采样大小(默认: 0.1米)
        freq_localization (float): 定位更新频率(Hz, 默认: 0.5)
        freq_global_map (float): 全局地图发布频率(默认: 0.25)
        localization_threshold (float): 接受ICP配准结果的最小适配度(默认: 0.8)
        fov (float): 地图裁剪的视场角(弧度, 默认: 2π = 360°)
        fov_far (float): 最大视距(米, 默认: 300米)
        pcd_map_topic (str): 地图发布的话题名
        pcd_map_path (str): 全局地图PCD文件的磁盘路径
    """
    
    def __init__(self):
        """初始化FastLIOLocalization节点。"""
        super().__init__("fast_lio_localization")
        
        # 状态变量
        self.global_map = None           # 全局点云地图
        self.T_map_to_odom = np.eye(4)   # 当前map -> odom变换
        self.cur_odom = None             # 最新的odometry消息
        self.cur_scan = None             # 当前LiDAR扫描
        self.initialized = False         # 是否已设置初始位姿

        # 声明并加载ROS参数
        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),      # 全局地图下采样分辨率
                ("scan_voxel_size", 0.1),     # 扫描下采样分辨率
                ("freq_localization", 0.5),   # 定位更新频率
                ("freq_global_map", 0.25),    # 地图可视化频率
                ("localization_threshold", 0.8),  # 最小ICP适配度分数
                ("fov", 6.28319),             # 视场角(~360°)
                ("fov_far", 300),             # 最大视距(米)
                ("pcd_map_topic", "/map"),    # 地图话题名
                ("pcd_map_path", ""),         # 地图文件路径
            ],
        )

        # 设置TF系统用于坐标系变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 创建发布器
        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        # 等待并初始化全局地图
        self.get_logger().info("等待全局地图...")
        self.initialize_global_map()
        self.get_logger().info("全局地图已接收。")
        
        # 创建订阅
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        # 创建定时器用于周期性更新
        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self) -> None:
        """
        定时器回调：周期性发布全局地图。
        
        可用于在RViz中可视化完整的全局地图。
        目前被注释掉以减少带宽占用。
        """
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose) -> np.ndarray:
        """
        将ROS Pose消息转换为4x4齐次变换矩阵。
        
        参数:
            pose (geometry_msgs.msg.Pose): 包含位置和方向的输入位姿
            
        返回:
            numpy.ndarray: 4x4齐次变换矩阵 [R|t; 0 0 0 1]
        """
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg) -> np.ndarray:
        """
        将PointCloud2 ROS消息转换为numpy点数组。
        
        使用ros2_numpy高效转换ROS消息到numpy数组。
        抑制关于缺失intensity/rgb字段的警告，因为我们只需要XYZ坐标。
        
        参数:
            pc_msg (sensor_msgs.msg.PointCloud2): 输入点云消息
            
        返回:
            numpy.ndarray: Nx3或Nx4的点数组(xyz或xyzi)
        """
        import warnings
        # 抑制ros2_numpy关于缺失intensity/rgb字段的警告
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="The 'intensity' field is not present")
            warnings.filterwarnings("ignore", message="The 'rgb' field is not present")
            pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]
    
    def registration_at_scale(self, scan, map, initial, scale) -> tuple:
        """
        在特定尺度(体素大小)下执行ICP配准。
        
        采用由粗到精的方法，通过体素大小对扫描和地图进行下采样。
        这允许多分辨率ICP，提高鲁棒性和速度。
        
        参数:
            scan (open3d.geometry.PointCloud): 源点云(当前扫描)
            map (open3d.geometry.PointCloud): 目标点云(全局地图)
            initial (numpy.ndarray): 初始4x4变换猜测矩阵
            scale (float): 体素大小的缩放因子(值越大越粗糙)
            
        返回:
            tuple: (transformation, fitness)
                - transformation (numpy.ndarray): 4x4变换矩阵
                - fitness (float): ICP适配度分数[0,1]，越高越好
                
        注意:
            适配度是对应点的均方误差。值>0.8通常表示良好对齐。
        """
        result_icp = o3d.pipelines.registration.registration_icp(
            self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
            self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
            1.0 * scale,  # 最大对应点距离(随尺度增加而增大)
            initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans) -> np.ndarray:
        """
        计算SE(3)变换矩阵的逆。
        
        对于刚性变换T = [R|t]，其逆为:
            T_inv = [R^T | -R^T * t]
                   [0 0 0 |     1    ]
        
        其中R是3x3旋转矩阵，t是3x1平移向量。
        
        参数:
            trans (numpy.ndarray): 4x4变换矩阵
            
        返回:
            numpy.ndarray: 逆变换矩阵(4x4)
        """
        trans_inverse = np.eye(4)
        # 逆旋转 = 转置(R^T)
        trans_inverse[:3, :3] = trans[:3, :3].T
        # 逆平移 = -R^T * t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc) -> None:
        """
        将numpy点云数组发布为ROS PointCloud2消息。
        
        使用ros2_numpy将numpy点数组高效转换为ROS PointCloud2消息。
        
        参数:
            publisher (rclpy.publisher.Publisher): 发布器对象
            header (std_msgs.msg.Header): ROS消息头部
            pc (numpy.ndarray): 点云数组(Nx3或Nx4，包含intensity)
        """
        data = dict()
        data["xyz"] = pc[:, :3]
        
        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        # else:
            # data["rgb"] = np.ones_like(pc)
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16  # 4个浮点数(x,y,z,intensity) * 每浮点数4字节
        else:
            msg.point_step = 12  # 3个浮点数(x,y,z) * 每浮点数4字节
            
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation) -> o3d.geometry.PointCloud:
        """
        根据当前视场角(FOV)裁剪全局地图以提高效率。
        
        将全局地图转换到机器人baselink坐标系，并过滤位于当前
        传感器视场角和最大范围内的点。这显著减少ICP计算时间。
        
        变换链:
            T_map_to_base_link = pose_estimation * T_odom_to_baselink
            T_base_link_to_map = inverse(T_map_to_base_link)
            
            map_points_homog (Nx4) * T_base_link_to_map (4x4) = points_in_base_link (Nx4)
        
        然后基于以下条件过滤:
            - 对于前向视场: x > 0 (机器人前方的点)
            - 角度约束: |arctan2(y, x)| < fov/2
            - 距离约束: x < fov_far
        
        参数:
            pose_estimation (numpy.ndarray): 当前map -> odom变换(4x4矩阵)
            
        返回:
            open3d.geometry.PointCloud: 视场内的全局地图子集
            
        注意:
            如果fov > π (180°)，假定为360°LiDAR(无角度过滤)。
        """
        # 获取当前odometry (odom -> baselink)
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        
        # 计算map -> baselink: T_mb = T_mo * T_ob
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        
        # 计算逆变换: baselink -> map
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 加载全局地图点并转换为齐次坐标(Nx4)
        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        
        # 将所有地图点转换到baselink坐标系
        # Points_base = T_base_link_to_map * Points_map (齐次乘法)
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 应用视场角和距离过滤
        if self.get_parameter("fov").value > 3.14:
            # 360° LiDAR: 无前后区分，仅距离限制
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], 
                                     global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            # 前向视场: 仅保留机器人前方的点
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)  # x > 0 (在机器人前方)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)  # 不太远
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], 
                                     global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        
        # 提取视场内的点
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布裁剪后的子地图用于调试(降采样10倍)
        header = self.cur_odom.header
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation) -> None:
        """
        使用ICP配准进行全局定位。
        
        这是主要的定位例程，它:
        1. 将全局地图裁剪到当前视场
        2. 执行多尺度ICP(由粗到精)
        3. 基于适配度分数接受或拒绝结果
        4. 如果成功则更新并发布位姿
        
        ICP流程:
            尺度5(粗略): 体素大小 = 5 * 基础体素
                - 快速，能处理较大的初始误差
                - 初始猜测: 提供的pose_estimation
                
            尺度1(精细): 体素大小 = 1 * 基础体素
                - 精确对齐
                - 初始猜测: 粗略尺度的结果
                
            决策: 如果适配度 > localization_threshold(默认0.8)则接受
        
        参数:
            pose_estimation (numpy.ndarray): 初始位姿估计(4x4矩阵)
                来自上一帧或用户初始化的值。
                
        注意:
            如果适配度过低，则拒绝位姿并保留旧的估计。
            这可以防止灾难性的定位失败。
        """
        # 复制以避免修改原始数据
        scan_tobe_mapped = copy.copy(self.cur_scan)
        
        # 步骤1: 使用当前位姿估计将全局地图裁剪到当前视场
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        
        # 步骤2: 在scale=5执行粗略ICP(更大的体素 = 更快，对较大误差更鲁棒)
        transformation, _ = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
        
        # 步骤3: 在scale=1执行精细ICP(全分辨率)
        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1)
        
        # 步骤4: 检查适配度并接受/拒绝
        if fitness > self.get_parameter("localization_threshold").value:
            # 良好对齐: 更新全局位姿
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
        else:
            # 较差对齐: 保留之前的位姿，警告用户
            self.get_logger().warn(
                f"适配度分数 {fitness:.4f} 小于定位阈值 "
                f"{self.get_parameter('localization_threshold').value}"
            )

    def voxel_down_sample(self, pcd, voxel_size) -> o3d.geometry.PointCloud:
        """
        使用体素网格滤波对点云进行下采样。
        
        通过对每个体素单元内的点进行平均来减少点数。
        这加快了ICP配准速度，同时保留了点云的总体结构。
        
        参数:
            pcd (open3d.geometry.PointCloud): 输入点云
            voxel_size (float): 体素单元大小(米)
            
        返回:
            open3d.geometry.PointCloud: 下采样后的点云
            
        注意:
            处理Open3D API兼容性(voxel_down_sample方法在
            Open3D 0.7版本和之后版本之间有所变化)。
        """
        try:
            # Open3D 0.8+ 版本API
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except Exception as e:
            # Open3D 0.7或更低版本API
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg: Odometry) -> None:
        """
        /Odometry订阅的回调函数。
        
        存储最新的odometry消息(odom -> baselink)。
        用于:
        - 计算map -> body变换
        - 将扫描转换到map坐标系用于裁剪
        
        参数:
            msg (nav_msgs.msg.Odometry): 当前odometry
        """
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg: PointCloud2) -> None:
        """
        /cloud_registered订阅的回调函数。
        
        将传入的PointCloud2消息转换为Open3D点云，
        存储它，并发布转换到map坐标系的扫描用于调试。
        
        参数:
            msg (sensor_msgs.msg.PointCloud2): FAST-LIO注册的点云
        """
        # 将ROS消息转换为numpy数组
        pc = self.msg_to_array(msg)
        
        # 创建Open3D点云对象
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        
        # 发布当前估计下的map坐标系扫描用于RViz可视化
        self.publish_point_cloud(self.pub_pc_in_map, msg.header, pc)
        
    def initialize_global_map(self) -> None: #, pc_msg):
        """
        从磁盘加载并初始化全局点云地图。
        
        读取预构建的全局地图PCD文件并对其进行下采样
        以减少计算量。地图应覆盖整个操作环境且质量高。
        
        注意:
            之前等待通过ROS消息接收地图(已注释)。
            现在直接从参数指定的文件加载。
            
        异常:
            Exception: 如果地图文件无法加载
        """
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        
        # 从PCD文件加载地图(离线预构建)
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        
        # 下采样以减少点数(体素大小=0.4米默认)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        
        # 可选: 保存下采样地图用于检查
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        
        self.get_logger().info("全局地图已接收。")

    def cb_initialize_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """
        /initialpose订阅的回调函数(来自RViz)。
        
        当用户通过RViz的"2D位姿估计"工具设置初始位姿估计时调用。
        使用此初始猜测立即触发全局定位。
        
        参数:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): 带协方差的初始位姿
        """
        # 将ROS位姿转换为变换矩阵
        initial_pose = self.pose_to_mat(msg.pose.pose)
        
        # 标记系统已初始化
        self.initialized = True
        self.get_logger().info("初始位姿已接收。")
        
        # 如果扫描可用，立即执行定位
        if self.cur_scan is not None:
            self.global_localization(initial_pose)
            
    def publish_odom(self, transform) -> None:
        """
        将map -> odom变换发布为Odometry消息。
        
        这是全局定位节点的主要输出。变换表示odometry坐标系
        相对于map坐标系的估计位姿。
        
        参数:
            transform (numpy.ndarray): 4x4变换矩阵(map -> odom)
        """
        odom_msg = Odometry()
        
        # 提取平移(x, y, z)
        xyz = transform[:3, 3]
        
        # 提取旋转为四元数[x, y, z, w]
        quat = tf_transformations.quaternion_from_matrix(transform)
        
        # 构建Pose消息
        odom_msg.pose.pose = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        
        # 设置头部(时间戳和帧ID)
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        
        # 发布odometry消息
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self) -> None:
        """
        周期性定位更新回调。
        
        在配置的频率(默认0.5 Hz)下调用。检查系统是否已初始化
        且扫描是否可用，然后使用当前位姿估计作为初始猜测
        执行全局定位(用于跟踪，而非全局重定位)。
        
        注意:
            这用于初始化后的连续位姿跟踪。
            对于丢失跟踪后的全局重定位，请使用/initialpose。
        """
        if not self.initialized:
            self.get_logger().info("等待初始位姿...")
            return
        
        if self.cur_scan is not None:
            # 使用当前T_map_to_odom作为初始猜测运行ICP
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    """全局定位节点的主入口点。"""
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()