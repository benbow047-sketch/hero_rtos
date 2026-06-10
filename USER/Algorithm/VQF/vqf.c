/**
 * @file vqf.c
 * @brief VQF (Variational Quaternion Filter) - C adaptation
 * @reference https://github.com/DLaidig/VQF
 * @note Adapted from BasicVQF C++ implementation for embedded use (float precision)
 */

#include "vqf.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SQRT2 1.41421356237309504880f
#define EPS 1.19209290e-07f  // FLT_EPSILON

/* ==================== 内部工具函数 ==================== */

static float vqf_square(float x) { return x * x; }

static float vqf_norm(const float vec[], size_t N)
{
    float s = 0;
    for (size_t i = 0; i < N; i++) s += vec[i] * vec[i];
    return sqrtf(s);
}

static void vqf_normalize(float vec[], size_t N)
{
    float n = vqf_norm(vec, N);
    if (n < EPS) return;
    for (size_t i = 0; i < N; i++) vec[i] /= n;
}

static void vqf_quatMultiply(const float q1[4], const float q2[4], float out[4])
{
    float w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    float x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    float y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    float z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

static void vqf_quatRotate(const float q[4], const float v[3], float out[3])
{
    float x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    float y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    float z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

static float vqf_gainFromTau(float tau, float Ts)
{
    if (tau < 0.0f) return 0;
    if (tau == 0.0f) return 1;
    return 1 - expf(-Ts / tau);
}

static void vqf_filterCoeffs(float tau, float Ts, double outB[3], double outA[2])
{
    if (tau < Ts / 2) {
        outB[0] = 1; outB[1] = 0; outB[2] = 0;
        outA[0] = 0; outA[1] = 0;
        return;
    }
    double fc = (SQRT2 / (2.0 * M_PI)) / (double)tau;
    double C = tan(M_PI * fc * (double)Ts);
    double D = C * C + sqrt(2.0) * C + 1.0;
    double b0 = C * C / D;
    outB[0] = b0;
    outB[1] = 2 * b0;
    outB[2] = b0;
    outA[0] = 2 * (C * C - 1) / D;
    outA[1] = (1 - sqrt(2.0) * C + C * C) / D;
}

static float vqf_filterStep(float x, const double b[3], const double a[2], double state[2])
{
    double y = b[0] * (double)x + state[0];
    state[0] = b[1] * (double)x - a[0] * y + state[1];
    state[1] = b[2] * (double)x - a[1] * y;
    return (float)y;
}

/* 初始化阶段：用平均值初始化滤波器状态 */
static void vqf_filterInitialState(float x0, const double b[3], const double a[2], double out[2])
{
    out[0] = (double)x0 * (1 - b[0]);
    out[1] = (double)x0 * (b[2] - a[1]);
}

/* 向量滤波（带初始化） */
static void vqf_filterVec(const float x[], size_t N, float tau, float Ts,
                           const double b[3], const double a[2], double state[], float out[])
{
    // 初始化阶段：前 tau 秒用平均值
    if (state[0] != state[0]) { // NaN check: state[0] is NaN during init
        if (state[1] != state[1]) { // first sample
            state[1] = 0; // sample count
            for (size_t i = 0; i < N; i++) state[2 + i] = 0; // sum
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2 + i] += (double)x[i];
            out[i] = (float)(state[2 + i] / state[1]);
        }
        if ((float)state[1] * Ts >= tau) {
            for (size_t i = 0; i < N; i++) {
                vqf_filterInitialState(out[i], b, a, state + 2 * i);
            }
        }
        return;
    }
    for (size_t i = 0; i < N; i++) {
        out[i] = vqf_filterStep(x[i], b, a, state + 2 * i);
    }
}

/* ==================== 外部 API ==================== */

void VQF_Init(vqf_t *vqf, float gyrTs, float accTs, float magTs)
{
    vqf->tauAcc = 3.0f;
    vqf->tauMag = 9.0f;

    vqf->gyrTs = gyrTs;
    vqf->accTs = (accTs > 0) ? accTs : gyrTs;
    vqf->magTs = (magTs > 0) ? magTs : gyrTs;

    // 计算滤波器系数
    vqf_filterCoeffs(vqf->tauAcc, vqf->accTs, vqf->accLpB, vqf->accLpA);
    vqf->kMag = vqf_gainFromTau(vqf->tauMag, vqf->magTs);

    // 重置状态
    VQF_ResetState(vqf);
}

void VQF_ResetState(vqf_t *vqf)
{
    vqf->gyrQuat[0] = 1; vqf->gyrQuat[1] = 0; vqf->gyrQuat[2] = 0; vqf->gyrQuat[3] = 0;
    vqf->accQuat[0] = 1; vqf->accQuat[1] = 0; vqf->accQuat[2] = 0; vqf->accQuat[3] = 0;
    vqf->delta = 0;

    for (size_t i = 0; i < 3; i++) vqf->lastAccLp[i] = 0;
    // NaN 表示未初始化
    float nan_val = 0.0f / 0.0f;
    for (size_t i = 0; i < 3 * 2; i++) vqf->accLpState[i] = nan_val;

    vqf->kMagInit = 1.0f;
}

void VQF_UpdateGyr(vqf_t *vqf, const float gyr[3])
{
    float gyrNorm = vqf_norm(gyr, 3);
    float angle = gyrNorm * vqf->gyrTs;
    if (gyrNorm > EPS) {
        float c = cosf(angle / 2);
        float s = sinf(angle / 2) / gyrNorm;
        float gyrStepQuat[4] = {c, s * gyr[0], s * gyr[1], s * gyr[2]};
        vqf_quatMultiply(vqf->gyrQuat, gyrStepQuat, vqf->gyrQuat);
        vqf_normalize(vqf->gyrQuat, 4);
    }
}

void VQF_UpdateAcc(vqf_t *vqf, const float acc[3])
{
    if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) return;

    float accEarth[3];

    // 在惯性系中滤波加速度
    vqf_quatRotate(vqf->gyrQuat, acc, accEarth);
    vqf_filterVec(accEarth, 3, vqf->tauAcc, vqf->accTs, vqf->accLpB, vqf->accLpA, vqf->accLpState, vqf->lastAccLp);

    // 转换到 6D 大地系并归一化
    vqf_quatRotate(vqf->accQuat, vqf->lastAccLp, accEarth);
    vqf_normalize(accEarth, 3);

    // 倾斜修正
    float accCorrQuat[4];
    float q_w = sqrtf((accEarth[2] + 1) / 2);
    if (q_w > 1e-6f) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5f * accEarth[1] / q_w;
        accCorrQuat[2] = -0.5f * accEarth[0] / q_w;
        accCorrQuat[3] = 0;
    } else {
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    vqf_quatMultiply(accCorrQuat, vqf->accQuat, vqf->accQuat);
    vqf_normalize(vqf->accQuat, 4);
}

void VQF_UpdateMag(vqf_t *vqf, const float mag[3])
{
    if (mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f) return;

    float magEarth[3];
    float accGyrQuat[4];
    VQF_GetQuat6D(vqf, accGyrQuat);
    vqf_quatRotate(accGyrQuat, mag, magEarth);

    float magDisAngle = atan2f(magEarth[0], magEarth[1]) - vqf->delta;
    if (magDisAngle > M_PI) magDisAngle -= 2 * M_PI;
    else if (magDisAngle < -M_PI) magDisAngle += 2 * M_PI;

    float k = vqf->kMag;

    // 快速初始收敛
    if (vqf->kMagInit != 0.0f) {
        if (k < vqf->kMagInit) k = vqf->kMagInit;
        vqf->kMagInit = vqf->kMagInit / (vqf->kMagInit + 1);
        if (vqf->kMagInit * vqf->tauMag < vqf->magTs) vqf->kMagInit = 0.0f;
    }

    vqf->delta += k * magDisAngle;
    if (vqf->delta > M_PI) vqf->delta -= 2 * M_PI;
    else if (vqf->delta < -M_PI) vqf->delta += 2 * M_PI;
}

void VQF_Update(vqf_t *vqf, const float gyr[3], const float acc[3])
{
    VQF_UpdateGyr(vqf, gyr);
    VQF_UpdateAcc(vqf, acc);
}

void VQF_Update9D(vqf_t *vqf, const float gyr[3], const float acc[3], const float mag[3])
{
    VQF_UpdateGyr(vqf, gyr);
    VQF_UpdateAcc(vqf, acc);
    VQF_UpdateMag(vqf, mag);
}

void VQF_GetQuat6D(vqf_t *vqf, float out[4])
{
    vqf_quatMultiply(vqf->accQuat, vqf->gyrQuat, out);
}

void VQF_GetQuat9D(vqf_t *vqf, float out[4])
{
    vqf_quatMultiply(vqf->accQuat, vqf->gyrQuat, out);
    // apply delta heading rotation
    float c = cosf(vqf->delta / 2);
    float s = sinf(vqf->delta / 2);
    float q0 = c * out[0] - s * out[3];
    float q1 = c * out[1] - s * out[2];
    float q2 = c * out[2] + s * out[1];
    float q3 = c * out[3] + s * out[0];
    out[0] = q0; out[1] = q1; out[2] = q2; out[3] = q3;
}

void VQF_SetTauAcc(vqf_t *vqf, float tauAcc)
{
    if (vqf->tauAcc == tauAcc) return;
    vqf->tauAcc = tauAcc;
    double newB[3], newA[2];
    vqf_filterCoeffs(vqf->tauAcc, vqf->accTs, newB, newA);
    // 更新系数（不调整状态，简化处理）
    vqf->accLpB[0] = newB[0]; vqf->accLpB[1] = newB[1]; vqf->accLpB[2] = newB[2];
    vqf->accLpA[0] = newA[0]; vqf->accLpA[1] = newA[1];
}

void VQF_SetTauMag(vqf_t *vqf, float tauMag)
{
    vqf->tauMag = tauMag;
    vqf->kMag = vqf_gainFromTau(vqf->tauMag, vqf->magTs);
}
