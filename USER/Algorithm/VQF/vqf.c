/**
 * @file vqf.c
 * @brief Simplified VQF (Variational Quaternion Filter) implementation
 * @reference https://github.com/DLaidig/VQF
 */

#include "vqf.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float twoKp = 2.0f * 1.0f;    // 2 * proportional gain
static float twoKi = 2.0f * 0.0f;    // 2 * integral gain

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// 陀螺仪零偏估计
static float bias_x = 0.0f, bias_y = 0.0f, bias_z = 0.0f;
static float bias_tau = 10.0f;  // 零偏估计时间常数(秒)

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void VQF_Init(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    bias_x = 0.0f; bias_y = 0.0f; bias_z = 0.0f;
}

void VQF_Update(float *q_out, float gx, float gy, float gz,
                float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 减去估计的零偏
    gx -= bias_x;
    gy -= bias_y;
    gz -= bias_z;

    // 如果加速度计数据有效（不是纯自由落体）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 归一化加速度计
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计的重力方向（从四元数算出）
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差 = 测量的重力 × 估计的重力（叉积）
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // PI 控制器修正
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;

        // 零偏更新（误差积分）
        bias_x += halfex * dt / bias_tau;
        bias_y += halfey * dt / bias_tau;
        bias_z += halfez * dt / bias_tau;
    }

    // 四元数积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q_out[0] = q0;
    q_out[1] = q1;
    q_out[2] = q2;
    q_out[3] = q3;
}

void VQF_SetTau(float accel_tau, float gyro_bias_tau)
{
    twoKp = 2.0f / accel_tau;
    bias_tau = gyro_bias_tau;
}
