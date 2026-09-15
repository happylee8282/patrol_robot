/**
 * @file Exp_mat.h
 * @brief SO(3)旋转群的指数映射（独立版本）
 * @author FAST_LIO localization team
 * @date 2026
 * @details 提供与 so3_math.h 相同的 SO(3) 指数和对数函数
 *          ，但作为独立头文件，不依赖 common_lib.h。
 *          包含旋转矩阵指数映射（Exp）和对数映射（Log）。
 *          还包含OpenCV版本（已注释）用于可视化。
 *
 * @note 此文件与 so3_math.h 类似但是自包含的，可用于不包含
 *       common_lib.h 的模块。
 */

#ifndef EXP_MAT_H
#define EXP_MAT_H

#include <math.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
// #include <common_lib.h>

/**
 * @def SKEW_SYM_MATRX(v)
 * @brief 创建反对称矩阵值的逗号分隔列表宏
 * @param v 包含3个元素的数组 [x, y, z]
 * @return 九个逗号分隔的值，用于Eigen矩阵初始化
 * @details 展开为：0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0
 *          使用示例：Eigen::Matrix3d K; K << SKEW_SYM_MATRX(v);
 */
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 指数映射：旋转向量 → 旋转矩阵
 * @tparam T 标量类型（float/double）
 * @param ang 旋转向量（轴角表示，右值引用）
 * @return 3×3旋转矩阵 R = exp([ω]×)
 * @details 使用Rodrigues公式：R = I + sin(θ)K + (1-cos(θ))K²
 *          其中 K = skew(ω/θ) 是反对称矩阵，θ = ||ω||。
 *          当 θ ≈ 0 时，返回单位矩阵。
 *
 * @note 参数为 &&（右值引用）以启用移动语义；调用者的参数可能被移动。
 *       对于左值参数，请使用其他重载版本。
 *
 * @see Exp(const Eigen::Matrix<T,3,1>&, const Ts&) 用于时间步积分
 * @see so3_math.h 获取更详细的文档
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
 * @brief 指数映射：角速度对时间积分 → 旋转增量
 * @tparam T 旋转标量类型（float/double）
 * @tparam Ts 时间标量类型（float/double/int）
 * @param ang_vel 角速度向量 ω（rad/s）
 * @param dt 时间步长 Δt（秒）
 * @return 旋转矩阵 R = exp([ω]× Δt)
 * @details 在时间间隔 Δt 上积分恒定角速度。
 *          计算 θ = ||ω||·Δt，然后应用Rodrigues公式。
 *          推荐用于IMU预积分，避免大角度问题。
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
 * @brief 指数映射：三个标量分量 → 旋转矩阵
 * @tparam T 标量类型
 * @param v1 旋转向量的X分量
 * @param v2 旋转向量的Y分量
 * @param v3 旋转向量的Z分量
 * @return 旋转矩阵 R
 * @details 便捷重载版本，当旋转向量分量是独立变量而非Eigen向量时使用。
 *          计算 θ = sqrt(v1²+v2²+v3²)，然后应用Rodrigues公式。
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
 * @brief 对数映射：旋转矩阵 → 旋转向量
 * @tparam T 标量类型
 * @param R 旋转矩阵（必须在SO(3)流形上）
 * @return 3×1旋转向量 ω（轴角表示）
 * @details 从旋转矩阵提取旋转向量：
 *          1. 计算旋转角：θ = acos(0.5*(trace(R)-1))
 *          2. 计算反对称部分：K = (R - Rᵀ)/2
 *          3. 若θ很小：ω ≈ 0.5·K.vec()（一阶近似：exp(ω)≈I+[ω]×）
 *             否则：ω = (θ/sin(θ))·K.vec()
 *
 * @note 处理单位旋转（θ≈0）时使用小角度近似
 *       以避免除以接近零的sin(θ)。
 * @warning 不检查R是否真的是旋转矩阵（正交且行列式=1）。
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    T &&theta = std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

// 注释掉的OpenCV版本 - 保留供参考
// template<typename T>
// cv::Mat Exp(const T &v1, const T &v2, const T &v3)
// {
//     
//     T norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
//     cv::Mat Eye3 = cv::Mat::eye(3, 3, CV_32F);
//     if (norm > 0.0000001)
//     {
//         T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
//         cv::Mat K = (cv::Mat_<T>(3,3) << SKEW_SYM_MATRX(r_ang));
//
//         /// Rodrigues变换
//         return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
//     }
//     else
//     {
//         return Eye3;
//     }
// }

#endif
