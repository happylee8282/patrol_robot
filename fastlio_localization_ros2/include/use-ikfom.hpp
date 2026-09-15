/**
 * @file use-ikfom.hpp
 * @brief FAST-LIO自定义状态向量和过程模型定义
 *
 * 本文件定义了IKFOM（Indirect Kalman Filter on Manifold）所需的类型：
 * - state_ikfom：系统状态向量（位置、姿态、速度、偏置、重力）
 * - input_ikfom：IMU输入向量（加速度、角速度）
 * - process_noise_ikfom：过程噪声向量
 *
 * 同时定义了关键函数：
 * - get_f()：状态转移函数（连续时间动力学）
 * - df_dx()：状态转移Jacobian矩阵
 * - df_dw()：过程噪声Jacobian矩阵
 * - process_noise_cov()：过程噪声协方差矩阵
 * - SO3ToEuler()：四元数转欧拉角
 *
 * 这些定义基于MTK（Manifold Toolkit）库，使用李群李代数表示旋转。
 *
 * 状态向量维度：24
 * 输入向量维度：12
 * 过程噪声维度：12
 *
 * 作者: FAST-LIO团队（基于香港大学MTK库）
 * 日期: 2025-04
 */

#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

/**
 * @brief 3维向量类型别名
 *
 * 基于MTK::vect<3, double>，用于表示位置、速度、平移等。
 */
typedef MTK::vect<3, double> vect3;
/**
 * @brief SO3旋转类型别名
 *
 * 基于MTK::SO3<double>，使用单位四元数表示3D旋转。
 * 李群SO(3)的切空间是so(3)（李代数，对应旋转向量）。
 */
typedef MTK::SO3<double> SO3;
/**
 * @brief S2球面类型别名
 *
 * 基于MTK::S2<double, 98090, 10000, 1>，用于表示单位方向（如重力方向）。
 * 参数说明：98090=最大迭代次数，10000=收敛阈值，1=维度
 */
typedef MTK::S2<double, 98090, 10000, 1> S2; 
/**
 * @brief 1维向量类型别名
 */
typedef MTK::vect<1, double> vect1;
/**
 * @brief 2维向量类型别名
 */
typedef MTK::vect<2, double> vect2;

/**
 * @brief 系统状态流形定义
 *
 * 状态向量state_ikfom包含24维状态，按顺序：
 *
 * | 索引 | 字段 | 维度 | 说明 |
 * |------|------|------|------|
 * | 0-2 | pos | 3 | 位置（世界坐标系，米） |
 * | 3-6 | rot | 4 | 姿态（SO3，单位四元数） |
 * | 7-9 | offset_R_L_I | 4 | LiDAR到IMU的旋转（SO3，四元数） |
 * | 10-12 | offset_T_L_I | 3 | LiDAR到IMU的平移（米） |
 * | 13-15 | vel | 3 | 速度（世界坐标系，米/秒） |
 * | 16-18 | bg | 3 | 陀螺仪偏置（弧度/秒） |
 * | 19-21 | ba | 3 | 加速度计偏置（米/秒²） |
 * | 22-23 | grav | 2 | 重力方向（S2球面，实际存储单位向量） |
 *
 * 李群流形：
 * - pos ∈ R³（欧几里得空间）
 * - rot ∈ SO(3)（旋转群，约束：||q||=1）
 * - offset_R_L_I ∈ SO(3)（固定外参，初始化后不变）
 * - offset_T_L_I ∈ R³（固定外参）
 * - vel ∈ R³
 * - bg ∈ R³（无约束，但通常变化缓慢）
 * - ba ∈ R³
 * - grav ∈ S²（单位球面，表示重力方向）
 */
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))          // 位置：世界坐标系下的机器人位置
((SO3, rot))            // 姿态：世界坐标系到IMU坐标系的旋转
((SO3, offset_R_L_I))   // 外参旋转：IMU坐标系到LiDAR坐标系的旋转
((vect3, offset_T_L_I)) // 外参平移：LiDAR原点在IMU坐标系下的坐标
((vect3, vel))          // 速度：世界坐标系下的速度
((vect3, bg))           // 陀螺仪偏置：角速度测量偏差
((vect3, ba))           // 加速度计偏置：比力测量偏差
((S2, grav))            // 重力方向：世界坐标系Z轴在IMU系的表示（单位向量）
);

/**
 * @brief IMU输入流形定义
 *
 * IMU测量值作为EKF的输入向量：
 * - acc：线性加速度（body系，单位：m/s²，已包含重力）
 * - gyro：角速度（body系，单位：rad/s）
 *
 * 注意：加速度计测量的是比力（specific force）：
 * f = a - g（IMU系下的表观加速度 = 真实加速度 - 重力）
 * 但在FAST-LIO中，通常直接使用原始加速度计数据。
 */
MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))   // 加速度计测量值（body系）
((vect3, gyro))  // 陀螺仪测量值（body系）
);

/**
 * @brief 过程噪声流形定义
 *
 * 定义连续时间过程噪声的维度：
 * - ng：陀螺仪噪声（角速度随机游走，rad/s/√s）
 * - na：加速度计噪声（速度随机游走，m/s²/√s）
 * - nbg：陀螺仪偏置噪声（偏置随机游走，rad/s/√s）
 * - nba：加速度计偏置噪声（偏置随机游走，m/s³/√s）
 */
MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))  // 陀螺仪测量噪声
((vect3, na))  // 加速度计测量噪声
((vect3, nbg)) // 陀螺仪偏置噪声
((vect3, nba)) // 加速度计偏置噪声
);

/**
 * @brief 计算过程噪声协方差矩阵Q
 *
 * 返回一个12x12对角矩阵，对角元素对应各过程噪声的方差。
 *
 * 默认值（从esekfom库复制）：
 * - ng: 0.0001 (rad/s)²  —— 陀螺仪测量噪声
 * - na: 0.0001 (m/s²)²  —— 加速度计测量噪声
 * - nbg: 0.00001 (rad/s)² —— 陀螺仪偏置随机游走
 * - nba: 0.00001 (m/s³)²  —— 加速度计偏置随机游走
 *
 * 这些值可在运行时通过ImuProcess::set_*_cov()调整。
 *
 * @return 过程噪声协方差矩阵 Q (12x12, Eigen::Matrix)
 */
MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

/**
 * @brief 系统状态转移函数（连续时间）
 *
 * 定义状态微分方程：dx/dt = f(x, u)
 *
 * 状态向量 x = [pos(3), rot(4), offset_R_L_I(4), offset_T_L_I(3), vel(3), bg(3), ba(3), grav(2)]^T
 * 输入向量 u = [acc(3), gyro(3)]^T
 *
 * 动力学方程：
 * 1. dp/dt = v                                    (位置导数 = 速度)
 * 2. dR/dt = R * hat(ω - b_g)                    (姿态导数 = 旋转 × (角速度-偏置))
 * 3. d(offset_R_L_I)/dt = 0                      (外参旋转常量，不更新)
 * 4. d(offset_T_L_I)/dt = 0                      (外参平移常量，不更新)
 * 5. dv/dt = R * (f - b_a) + g                   (速度导数 = 旋转×比力 + 重力)
 * 6. db_g/dt = n_g                               (陀螺仪偏置随机游走)
 * 7. db_a/dt = n_a                               (加速度计偏置随机游走)
 * 8. d grav/dt = 0                               (重力方向常量)
 *
 * 其中：
 * - hat(·) 将3D向量转换为反对称矩阵（叉乘矩阵）
 * - f = R^T * (a - b_a) 是body系下的比力（去除偏置并旋转到body系）
 * - g = grav（通过S2转换回3D向量）
 *
 * @param s 当前状态（输入/输出：计算导数后写入res）
 * @param in IMU输入（加速度、角速度）
 * @return 状态导数向量 (24x1, Eigen::Matrix)
 */
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);  // ω - b_g（去偏置角速度）
	vect3 a_inertial = s.rot * (in.acc-s.ba); // 旋转到世界系的比力（未加重力）
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];                    // dp/dt = v
		res(i + 3) =  omega[i];               // d(rot)/dt ≈ hat(ω-bg)*rot（隐式）
		res(i + 12) = a_inertial[i] + s.grav[i];  // dv/dt = R*(a-ba) + g
	}
	return res;
}

/**
 * @brief 状态转移Jacobian矩阵（状态维度）
 *
 * 计算连续时间状态转移函数对状态的偏导数：df/dx (24x23)
 *
 * 注意：state_ikfom实际维度为24，但offset_R_L_I(SO3)只有2个自由度（单位四元数约束），
 * 所以有效状态维度为23。Jacobian矩阵的列对应23个自由度。
 *
 * 矩阵结构（稀疏表示）：
 * - 第0-2列（pos）：影响速度（∂v/∂p = 0，此处为零）
 * - 第3-6列（rot）：影响速度、角速度项
 * - 第7-9列（offset_R_L_I）：影响速度（通过R*offset_R_L_I）
 * - 第10-12列（offset_T_L_I）：无直接导数（常量）
 * - 第13-15列（vel）：影响位置（∂p/∂v = I）
 * - 第16-18列（bg）：影响角速度项（∂ω/∂bg = -I）
 * - 第19-21列（ba）：影响速度（∂v/∂ba = -R）
 * - 第22-23列（grav）：影响速度（∂v/∂grav，通过S2_Mx计算）
 *
 * @param s 当前状态
 * @param in IMU输入
 * @return 状态转移Jacobian矩阵 (24x23, Eigen::Matrix)
 */
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();  // ∂p/∂v = I
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);  // a - b_a
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);  // ω - b_g
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);  // ∂v/∂rot = -R*hat(f)
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();  // ∂v/∂ba = -R
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);  // 计算S2流形上的Jacobian（重力方向）
	cov.template block<3, 2>(12, 21) =  grav_matrix;  // ∂v/∂grav
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();  // d(rot)/dbg ≈ -I（简化）
	return cov;
}

/**
 * @brief 过程噪声Jacobian矩阵（噪声维度）
 *
 * 计算状态导数对过程噪声的偏导数：df/dw (24x12)
 *
 * 噪声向量w = [ng(3), na(3), nbg(3), nba(3)]^T
 *
 * 矩阵结构：
 * - 行0-2（位置）：∂p/∂w = 0
 * - 行3-5（姿态）：∂rot/∂ng = 0（间接影响通过bg）
 * - 行6-8（offset_R_L_I）：∂offset_R/∂w = 0（常量）
 * - 行9-11（offset_T_L_I）：∂offset_T/∂w = 0
 * - 行12-14（速度）：∂v/∂na = R（加速度计噪声直接影响速度）
 * - 行15-17（bg）：∂bg/∂nbg = I
 * - 行18-20（ba）：∂ba/∂nba = I
 * - 行21-23（grav）：∂grav/∂w = 0（常量）
 *
 * 实际矩阵内容：
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [0]         [0]         [0]         [0]
 * [ I ]       [0]         [0]         [0]  ← dv/dna = R (block(12,3))
 * [0]         [ I ]       [0]         [0]  ← d(bg)/dnbg = I (block(15,6))
 * [0]         [0]         [ I ]       [0]  ← d(ba)/dnba = I (block(18,9))
 * [0]         [0]         [0]         [0]
 *
 * @param s 当前状态
 * @param in IMU输入
 * @return 过程噪声Jacobian矩阵 (24x12, Eigen::Matrix)
 */
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();  // dv/dna = -R（加速度计噪声到速度）
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();  // d(rot)/dng ≈ -I（陀螺仪噪声到姿态）
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();  // dbg/dnbg = I
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();  // dba/dnba = I
	return cov;
}

/**
 * @brief 将SO3四元数转换为欧拉角（RPY顺序）
 *
 * 将单位四元数 q = [qx, qy, qz, qw]^T 转换为欧拉角（roll, pitch, yaw）。
 * 使用ZYX顺序（yaw-pitch-roll，即内旋顺序）：
 *   R = R_z(yaw) * R_y(pitch) * R(roll)
 *
 * 转换公式（从四元数到欧拉角）：
 *   roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx² + qy²))
 *   pitch = asin(2*(qw*qy - qz*qx))
 *   yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
 *
 * 奇异点处理：
 * - 北极点（pitch ≈ +90°）：当 test = qw*qy - qz*qx > 0.49999*unit
 *   此时yaw任意，设roll = 2*atan2(qx, qw), pitch = +90°
 * - 南极点（pitch ≈ -90°）：当 test < -0.49999*unit
 *   此时yaw任意，设roll = -2*atan2(qx, qw), pitch = -90°
 *
 * 输出角度单位为度（乘以57.3转换弧度→度）。
 *
 * @param orient SO3对象（单位四元数）
 * @return 欧拉角向量 [roll, pitch, yaw] (度)
 */
vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // 归一化因子（若单位四元数应为1）
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole（万向锁：俯仰角+90°）
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole（万向锁：俯仰角-90°）
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),  // roll
			std::asin (2*test/unit),                                                          // pitch
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);  // yaw
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif