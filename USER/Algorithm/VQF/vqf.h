/**
 * @file vqf.h
 * @brief VQF (Variational Quaternion Filter) - C adaptation
 * @reference https://github.com/DLaidig/VQF
 */

#ifndef VQF_H
#define VQF_H

#include <stddef.h>

/**
 * @brief VQF 状态结构体
 */
typedef struct
{
    /* 参数 */
    float tauAcc;           // 加速度计低通滤波时间常数(秒)，默认3.0
    float tauMag;           // 磁力计更新时间常数(秒)，默认9.0

    /* 系数 */
    float gyrTs;            // 陀螺仪采样周期
    float accTs;            // 加速度计采样周期
    float magTs;            // 磁力计采样周期
    double accLpB[3];       // 加速度低通滤波器分子系数
    double accLpA[2];       // 加速度低通滤波器分母系数
    float kMag;             // 航向修正增益

    /* 状态 */
    float gyrQuat[4];       // 陀螺仪积分四元数
    float accQuat[4];       // 倾斜修正四元数
    float delta;            // 航向偏差角(rad)
    float lastAccLp[3];     // 低通滤波后的加速度
    double accLpState[3*2]; // 加速度低通滤波器状态
    float kMagInit;         // 快速初始收敛增益
} vqf_t;

/**
 * @brief 初始化 VQF
 * @param vqf  VQF 结构体指针
 * @param gyrTs  陀螺仪采样周期(秒)，如 0.001 = 1kHz
 * @param accTs  加速度计采样周期(秒)，-1 表示和 gyrTs 相同
 * @param magTs  磁力计采样周期(秒)，-1 表示和 gyrTs 相同
 */
void VQF_Init(vqf_t *vqf, float gyrTs, float accTs, float magTs);

/**
 * @brief 重置 VQF 状态
 */
void VQF_ResetState(vqf_t *vqf);

/**
 * @brief 陀螺仪更新步骤
 */
void VQF_UpdateGyr(vqf_t *vqf, const float gyr[3]);

/**
 * @brief 加速度计更新步骤
 */
void VQF_UpdateAcc(vqf_t *vqf, const float acc[3]);

/**
 * @brief 磁力计更新步骤
 */
void VQF_UpdateMag(vqf_t *vqf, const float mag[3]);

/**
 * @brief 6D 更新（无磁力计）
 * @param gyr  陀螺仪 (rad/s)
 * @param acc  加速度计 (m/s² 或 g)
 */
void VQF_Update(vqf_t *vqf, const float gyr[3], const float acc[3]);

/**
 * @brief 9D 更新（含磁力计）
 */
void VQF_Update9D(vqf_t *vqf, const float gyr[3], const float acc[3], const float mag[3]);

/**
 * @brief 获取 6D 四元数（无磁力计修正）
 */
void VQF_GetQuat6D(vqf_t *vqf, float out[4]);

/**
 * @brief 获取 9D 四元数（含磁力计修正）
 */
void VQF_GetQuat9D(vqf_t *vqf, float out[4]);

/**
 * @brief 设置加速度计时间常数
 * @param tauAcc  越小越信任加速度计，默认3.0
 */
void VQF_SetTauAcc(vqf_t *vqf, float tauAcc);

/**
 * @brief 设置磁力计时间常数
 * @param tauMag  越小越信任磁力计，默认9.0
 */
void VQF_SetTauMag(vqf_t *vqf, float tauMag);

#endif // VQF_H
