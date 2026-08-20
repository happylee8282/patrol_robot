# import matplotlib
# matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt


"""
IKFOM（流形上的迭代卡尔曼滤波）结果可视化脚本

此脚本加载和可视化 IKFOM SLAM（同步定位与建图）或类似状态估计算法的输出。
它比较"pre"（优化/更新前）和"out"（优化/更新后）的各类状态变量随时间的变化。

脚本期望当前工作目录中有两个数据文件：
    - mat_pre.txt：包含优化前的状态估计
    - mat_out.txt：包含优化后的状态估计

两个文件应具有以下列结构：
    第 0 列：时间戳
    第 1-3 列：第一组状态（如姿态：roll、pitch、yaw）
    第 4-6 列：第二组状态（如平移：x、y、z）
    第 7-9 列：第三组状态，以此类推...

脚本生成 4x2 的子图网格显示不同的状态分量：
    第 0 行：姿态（旋转） - 优化前 vs 优化后
    第 1 行：平移（位置） - 优化前 vs 优化后
    第 2 行：外参旋转 - 优化前 vs 优化后
    第 3 行：外参平移 - 优化前 vs 优化后
    列：X、Y、Z 分量（左列显示全部，右列显示速度/bg/ba/重力）

额外注释掉的区域演示如何绘制：
    - 原始 IMU 数据（陀螺仪和加速度计）
    - 计算时间统计的箱线图

使用方法：
    1. 确保 mat_pre.txt 和 mat_out.txt 在当前目录
    2. 运行：python plot.py
    3. matplotlib 窗口将显示图表
    4. 使用 plt.savefig() 保存图像（在脚本中取消注释）

注意：某些区域被注释掉用于可选可视化。
IKFOM 特定的绘图是默认激活的。
"""


########## IKFOM 状态比较可视化 ##########
# 创建 4x2 子图网格用于显示不同的状态变量
# 4 行 2 列允许显示 8 个不同的图
fig, axs = plt.subplots(4, 2)

# 图例标签前缀
# 'pre' 表示优化前值
# 'out' 表示优化后值
lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
lab_out = ['', 'out-x', 'out-y', 'out-z']

# 绘图索引 - 映射子图网格位置
# plot_ind 当前未使用但可用于自定义索引
plot_ind = range(7, 10)

# 从文本文件加载优化前和优化后的数据
# 这些文件应由 IKFOM 算法生成
a_pre = np.loadtxt('mat_pre.txt')  # 形状：(n_timesteps, n_columns)
a_out = np.loadtxt('mat_out.txt')  # 形状：(n_timesteps, n_columns)

# 从预数据的第 0 列提取时间向量（假设两者相同）
time = a_pre[:, 0]

# 设置 4x2 子图网格的标题
# 第 0 列（左）：主要状态变量
axs[0, 0].set_title('姿态')           # 第 0 行：方向（roll、pitch、yaw）
axs[1, 0].set_title('平移')           # 第 1 行：位置（x、y、z）
axs[2, 0].set_title('外参旋转')       # 第 2 行：外参旋转参数
axs[3, 0].set_title('外参平移')       # 第 3 行：外参平移参数

# 第 1 列（右）：附加状态变量
axs[0, 1].set_title('速度')           # 第 0 行：线速度（vx、vy、 vz）
axs[1, 1].set_title('陀螺仪偏置')     # 第 1 行：陀螺仪偏置（bx、by、bz）
axs[2, 1].set_title('加速度偏置')     # 第 2 行：加速度计偏置（bx、by、bz）
axs[3, 1].set_title('重力')           # 第 3 行：重力大小或方向

# 绘制每个状态变量（x、y、z 分量）在所有 8 个子图上
# 索引模式 j%4, j//4 将线性索引 j 映射到二维网格（行、列）
# j 范围 0-7 覆盖按读取顺序的所有 8 个子图
for i in range(1, 4):        # i = 1,2,3 对应 x、y、z 分量
    for j in range(8):       # j = 0-7 对应所有 8 个子图
        row = j % 4          # 行索引：0,1,2,3,0,1,2,3
        col = j // 4         # 列索引：0,0,0,0,1,1,1,1
        # 用点线绘制优化前数据
        axs[row, col].plot(time, a_pre[:, i + j*3], '.-', label=lab_pre[i])
        # 用点线绘制优化后数据
        axs[row, col].plot(time, a_out[:, i + j*3], '.-', label=lab_out[i])

# 完成每个子图：添加网格和图例
for j in range(8):
    axs[j % 4, j // 4].grid(True, alpha=0.3)  # 添加浅网格提高可读性
    axs[j % 4, j // 4].legend()                # 添加图例显示 pre/out 标签

# 为整个图形添加网格（与子图网格部分冗余）
plt.grid(True, alpha=0.3)

# 注意：文件末尾调用 plt.show()
# 这将显示交互式 matplotlib 窗口
########## IKFOM 可视化结束 ##########


########## IMU 数据可视化（已注释） ##########
# 此区域演示如何绘制原始 IMU 测量值（陀螺仪和加速度计）
# 取消注释以使用：需要 'imu.txt' 文件，列格式为：time, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z

# fig, axs = plt.subplots(2)
# imu = np.loadtxt('imu.txt')
# time = imu[:, 0]
# axs[0].set_title('陀螺仪')
# axs[1].set_title('加速度计')
# lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
# lab_2 = ['acc-x', 'acc-y', 'acc-z']
# for i in range(3):
#     axs[0].plot(time, imu[:, i+1], '.-', label=lab_1[i])
#     axs[1].plot(time, imu[:, i+4], '.-', label=lab_2[i])
# for i in range(2):
#     axs[i].grid(True, alpha=0.3)
#     axs[i].legend()
# plt.grid(True, alpha=0.3)
########## IMU 可视化结束 ##########


########## 计算时间分析（已注释） ##########
# 此区域创建组合箱线图和折线图，显示：
#   1. 算法使用的有效特征数（左 y 轴）
#   2. 每迭代计算时间（毫秒，右 y 轴，红色）
# 比较三种场景：户外场景、室内场景 1、室内场景 2

# 期望的数据文件：
#   - Log/mat_out_time_indoor1.txt
#   - Log/mat_out_time_indoor2.txt
#   - Log/mat_out_time_outdoor.txt
# 每个文件格式：time, feature_count（列可能不同）

# plt.figure(3)
# fig = plt.figure()
# font1 = {'family': 'Times New Roman',
#          'weight': 'normal',
#          'size': 12}
# c = "red"
# a_out1 = np.loadtxt('Log/mat_out_time_indoor1.txt')
# a_out2 = np.loadtxt('Log/mat_out_time_indoor2.txt')
# a_out3 = np.loadtxt('Log/mat_out_time_outdoor.txt')
#
# # 创建 twin 坐标轴：左轴为特征数，右轴为计算时间
# ax1 = fig.add_subplot(111)
# ax1.set_ylabel('有效特征数', font1)
#
# # 在位置 0.9、1.9、2.9 绘制特征数的箱线图
# ax1.boxplot(a_out1[:, 2], showfliers=False, positions=[0.9])
# ax1.boxplot(a_out2[:, 2], showfliers=False, positions=[1.9])
# ax1.boxplot(a_out3[:, 2], showfliers=False, positions=[2.9])
# ax1.set_ylim([0, 3000])
#
# # 第二个 y 轴用于计算时间（毫秒）
# ax2 = ax1.twinx()
# ax2.spines['right'].set_color('red')
# ax2.set_ylabel('计算时间 (ms)', font1)
# ax2.yaxis.label.set_color('red')
# ax2.tick_params(axis='y', colors='red')
#
# # 在位置 1.1、2.1、3.1 绘制计算时间的箱线图（转换为毫秒）
# # 所有箱线图元素使用红色
# ax2.boxplot(a_out1[:, 1]*1000, showfliers=False, positions=[1.1],
#             boxprops=dict(color=c), capprops=dict(color=c), whiskerprops=dict(color=c))
# ax2.boxplot(a_out2[:, 1]*1000, showfliers=False, positions=[2.1],
#             boxprops=dict(color=c), capprops=dict(color=c), whiskerprops=dict(color=c))
# ax2.boxplot(a_out3[:, 1]*1000, showfliers=False, positions=[3.1],
#             boxprops=dict(color=c), capprops=dict(color=c), whiskerprops=dict(color=c))
# ax2.set_xlim([0.5, 3.5])
# ax2.set_ylim([0, 100])
#
# # 设置 x 轴刻度标签
# plt.xticks([1, 2, 3], ('户外场景', '室内场景 1', '室内场景 2'))
# plt.grid(True, alpha=0.3)
#
# # 保存高分辨率图形用于出版
# plt.savefig("time.pdf", dpi=1200)
########## 时间分析结束 ##########


# 显示所有活动的图形
# 这是渲染 matplotlib 窗口的最终命令
# 如果在无头模式（如服务器）上运行，请注释掉
plt.show()
