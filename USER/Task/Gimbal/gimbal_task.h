/**
 * @file gimbal_task.h
 * @brief Gimbal task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_RELAX = 0,        //云台掉电
    GIMBAL_INIT = 1,         //云台初始化
    GIMBAL_GYRO = 2,         //云台使用imu闭环
    GIMBAL_AUTO = 3,         //云台自瞄
    GIMBAL_ECD = 4           //云台使用编码器闭环
} gimbal_mode_e;

/**
 * @brief 云台归中状态
 */
typedef enum
{
    BACK_STEP = 0,             //云台正在归中
    BACK_IS_OK = 1,            //云台归中完成
} gimbal_back_e;

/**
 * @brief 自瞄相对角度处理
 */
typedef enum
{
    RELATIVE_ANGLE_TRANS = 0,  //正在传输
    RELATIVE_ANGLE_OK = 1,     //传输完成
} auto_relative_angle_status_e;

#ifdef BSP_USING_GIMBAL_TASK
void GimbalTask_Entry(void *argument);
#endif

#endif // GIMBAL_TASK_H
