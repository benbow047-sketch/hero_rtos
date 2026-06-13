/**
 * @file ins_task.h
 * @brief INS (Inertial Navigation System) task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef INS_TASK_H
#define INS_TASK_H

#include "rm_algorithm.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_X 0
#define INS_Y 1
#define INS_Z 2

/**
 * @brief IMU 原始数据 + 解算结果
 */
typedef struct
{
    float q[4];             // 四元数

    float motion_accel_b[3]; // 机体系运动加速度
    float motion_accel_n[3]; // 大地系加速度

    float accel_lpf;         // 加速度低通滤波系数

    // 机体坐标系在大地坐标系中的表示
    float xn[3];
    float yn[3];
    float zn[3];

    // IMU 测量值
    float gyro[3];
    float accel[3];

    // 姿态角
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;

    uint8_t init;
} ins_t;

/**
 * @brief IMU 安装校正参数
 */
typedef struct
{
    uint8_t flag;
    float scale[3];
    float yaw;
    float pitch;
    float roll;
} imu_param_t;

#ifdef BSP_USING_INS_TASK
void InsTask_Entry(void *argument);
#endif

#endif // INS_TASK_H
