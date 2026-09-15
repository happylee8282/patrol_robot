/**
 * @file so3_math.h
 * @brief SO(3)流形上的三维旋转数学操作
 * @author FAST_LIO localization team
 * @date 2026
 * @details 提供使用特殊正交群 SO(3) 操作3D旋转的核心数学函数
 *          。实现指数映射（so(3)→SO(3)）、对数映射（SO(3)→so(3)）
 *          以及欧拉角转换。广泛应用于状态估计和IMU预积分。
 *
 * @section Math_Background
 * SO(3) 是行列式为1的3×3旋转矩阵群。
 * 单位元处的切空间是 so(3)，即3×3反对称矩阵空间，
 * 通过"vee"映射与ℝ³同构：
 *   [  0  -ω₃  ω₂ ]          [ω₁]
 *   [ ω₃   0  -ω₁ ]  ↔  ω =  [ω₂]
 *   [-ω₂  ω₁   0  ]          [ω₃]
 *
 * 指数映射 Exp(ω) 通过Rodrigues公式将轴角转换为旋转矩阵。
 * 对数映射 Log(R) 提取旋转向量。
 */

#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

/**
 * @def SKEW_SYM_MATRX(v)
 * @brief 从3元素数组创建反对称矩阵宏
 * @param v 包含3个元素的数组 [v0, v1, v2]
 * @return 作为逗号分隔值的反对称矩阵，用于Eigen初始化
 * @details 创建矩阵：
 *   [  0  -v[2]  v[1] ]
 *   [ v[2]   0  -v[0] ]
 *   [-v[1]  v[0]   0  ]
 * @note 与Eigen的逗号初始化一起使用：K << SKEW_SYM_MATRX(v);
 */
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 从3D向量创建反对称矩阵
 * @tparam T 标量类型（float/double）
 * @param v 3D向量（Eigen::Matrix<T,3,1>）
 * @return 3×3反对称矩阵K，满足 K * w = v × w
 * @details 反对称矩阵表示叉积为线性变换：
 *          对于任意w，skew_sym_mat(v) * w = v × w。
 *          在Rodrigues公式和so(3)表示中使用。
 *
 * @f[
 * \text{skew}(\mathbf{v}) = \begin{bmatrix}
 * 0 & -v_z & v_y \\
 * v_z & 0 & -v_x \\
 * -v_y & v_x & 0
 * \end{bmatrix}
 * @f]
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0;
    return skew_sym_mat;
}

/**
 * @brief 从 so(3) 到 SO(3) 的指数映射（轴角到旋转）
 * @tparam T 标量类型（float/double）
 * @param ang 旋转向量（轴角，右值引用）
 * @return 旋转矩阵 R ∈ SO(3)
 * @details 使用Rodrigues公式计算 R = exp([ω]×)：
 *          R = I + sin(θ)K + (1-cos(θ))K²，
 *          其中 K = skew(ω/θ)，θ = ||ω||。
 *          当 θ ≈ 0 时使用小角度近似。
 *
 * @note 输入为右值引用（&&）以启用移动语义；调用者的参数被消耗。
 *       对于左值参数，请使用 Exp(const Vector3&, dt) 重载版本。
 *
 * @see Exp(const Eigen::Matrix<T, 3, 1>&, const Ts&) 用于时间步积分
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Rodrigues变换
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief 用于角速度对时间积分的指数映射
 * @tparam T 旋转标量类型（float/double）
 * @tparam Ts 时间标量类型（float/double/int）
 * @param ang_vel 角速度向量 ω（rad/s）
 * @param dt 时间步长 Δt（秒）
 * @return 旋转矩阵 R = exp([ω]× Δt)
 * @details 计算恒定角速度在 Δt 时间间隔上的旋转增量。
 *          用于IMU预积分和传播：R(t+Δt) = R(t) * Exp(ω, Δt)。
 *          等价于 Exp(ω * Δt)，但避免了大 Δt·ω 导致的溢出问题。
 */
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Rodrigues变换
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief 从三个标量到旋转矩阵的指数映射
 * @tparam T 标量类型
 * @param v1 旋转向量的X分量
 * @param v2 旋转向量的Y分量
 * @param v3 旋转向量的Z分量
 * @return 旋转矩阵 R ∈ SO(3)
 * @details 便捷重载版本，接受三个独立参数而非向量。
 *          计算 θ = sqrt(v1²+v2²+v3²)，归一化后应用Rodrigues公式。
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Rodrigues变换
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief 从 SO(3) 到 so(3) 的对数映射（旋转矩阵到轴角）
 * @tparam T 标量类型
 * @param R 旋转矩阵（必须在 SO(3) 上，即正交且行列式=1）
 * @return 3×1旋转向量 ω，满足 R = exp([ω]×)
 * @details 从旋转矩阵提取旋转轴和角度：
 *          1. 计算角度：θ = acos(0.5*(trace(R)-1)，带小角度检查
 *          2. 计算反对称部分：K = (R - R^T)/2
 *          3. 若θ很小：ω ≈ 0.5 * K.vec()（一阶近似）
 *             否则：ω = (θ / sin(θ)) * K.vec()
 *
 * @note 对于 θ ≈ 0（单位旋转），使用级数展开避免除以 sin(θ)。
 * @warning 假设R是有效的旋转矩阵。不进行正交性检查。
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief 将旋转矩阵转换为欧拉角（ZYX顺序 / 偏航-俯仰-横滚）
 * @tparam T 标量类型
 * @param rot 输入旋转矩阵 R ∈ SO(3)
 * @return 向量 [roll (x), pitch (y), yaw (z)]，单位为弧度
 * @details 使用 ZYX 约定提取欧拉角（偏航→俯仰→横滚顺序）：
 *          roll = atan2(R(2,1), R(2,2))
 *          pitch = atan2(-R(2,0), sy)，其中 sy = sqrt(R(0,0)²+R(1,0)²)
 *          yaw = atan2(R(1,0), R(0,0))
 *          处理 pitch ≈ ±90° 时的奇异性（sy < 1e-6）。
 */
template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

#endif
