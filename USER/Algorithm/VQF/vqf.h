/**
 * @file vqf.h
 * @brief Simplified VQF (Variational Quaternion Filter)
 */

#ifndef VQF_H
#define VQF_H

/**
 * @brief 初始化 VQF
 */
void VQF_Init(void);

/**
 * @brief VQF 更新
 * @param q_out  输出四元数 [4]
 * @param gx, gy, gz  陀螺仪 (rad/s)
 * @param ax, ay, az  加速度计 (m/s^2 或 g，需要归一化)
 * @param dt  时间间隔 (秒)
 */
void VQF_Update(float *q_out, float gx, float gy, float gz,
                float ax, float ay, float az, float dt);

/**
 * @brief 设置滤波参数
 * @param accel_tau  加速度计修正时间常数(秒)，越小越信任加速度计，默认1.0
 * @param gyro_bias_tau  陀螺仪零偏估计时间常数(秒)，越大估计越慢越稳，默认10.0
 */
void VQF_SetTau(float accel_tau, float gyro_bias_tau);

#endif // VQF_H
