/**
 * @file rm_task.h
 * @brief FreeRTOS task management and message definitions
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef RM_TASK_H
#define RM_TASK_H

#include <stdbool.h>
#include <stdint.h>

/* ==================== Task Switches ==================== */
#define BSP_USING_INS_TASK
#define BSP_USING_MOTOR_TASK
#define BSP_USING_CMD_TASK
#define BSP_USING_CHASSIS_TASK
#define BSP_USING_GIMBAL_TASK
#define BSP_USING_SHOOT_TASK
#define BSP_USING_TRANSMISSION_TASK
#define BSP_USING_USARTREC_TASK
#define BSP_USING_REFEREE_TASK
#define BSP_USING_SUPERCAP_TASK

/* ==================== Task Headers ==================== */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif
#ifdef BSP_USING_MOTOR_TASK
#include "motor_task.h"
#endif
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif
#ifdef BSP_USING_GIMBAL_TASK
#include "gimbal_task.h"
#endif
#ifdef BSP_USING_SHOOT_TASK
#include "shoot_task.h"
#endif
#ifdef BSP_USING_TRANSMISSION_TASK
#include "transmission_task.h"
#endif
#ifdef BSP_USING_USARTREC_TASK
#include "usart_recive_task.h"
#endif
#ifdef BSP_USING_REFEREE_TASK
#include "referee_task.h"
#endif
#ifdef BSP_USING_SUPERCAP_TASK
#include "supercap_task.h"
#endif

/* ==================== Message Structures ==================== */

/**
 * @brief INS (IMU) 数据
 */
struct ins_msg
{
    float gyro[3];          // 角速度
    float accel[3];         // 加速度
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};

/**
 * @brief 底盘控制命令 (cmd → chassis)
 */
struct chassis_cmd_msg
{
    float vx;                       // 前进方向速度
    float vy;                       // 横移方向速度
    float vw;                       // 旋转速度
    float offset_angle;             // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;       // 当前底盘控制模式
    chassis_mode_e last_mode;       // 上一次底盘控制模式
};

/**
 * @brief 云台控制命令 (cmd → gimbal)
 */
struct gimbal_cmd_msg
{
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;        // 当前云台控制模式
    gimbal_mode_e last_mode;        // 上一次云台控制模式
};

/**
 * @brief 发射器控制命令 (cmd → shoot)
 */
struct shoot_cmd_msg
{
    shoot_mode_e ctrl_mode;         // 当前发射器控制模式
    shoot_mode_e last_mode;         // 上一次发射器控制模式
    trigger_mode_e trigger_status;
    int16_t shoot_freq;             // 发射弹频
    int16_t shoot_speed;            // 发射弹速
    uint8_t cover_open;             // 弹仓盖开关
    uint8_t mirror_enable;          // 倍镜使能开关
    bool friction_status;
    bool trigger_clear_flag;        // 拨弹pid清零标志位
};

/**
 * @brief 云台反馈状态 (gimbal → cmd)
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;        // 云台归中情况
    float yaw_offset_angle_total;   // 云台初始yaw轴角度
    float yaw_offset_angle;
    float pit_offset_angle;
    float yaw_relative_angle;       // 云台相对于初始位置的yaw轴角度
};

/**
 * @brief 底盘反馈状态 (chassis → cmd)
 */
struct chassis_fdb_msg
{
    float x_pos_gim;
    float y_pos_gim;
    float vw_ch;                    // 底盘旋转速度
};

/**
 * @brief 发射器反馈状态 (shoot → cmd)
 */
struct shoot_fdb_msg
{
    shoot_back_e trigger_status;    // shoot状态反馈
    int16_t trigger_motor_current;  // 拨弹电机电流
    int shoot_cnt;
};

/**
 * @brief 上位机反馈状态 (transmission → cmd)
 */
struct trans_fdb_msg
{
    float yaw;
    float pitch;
    float roll;
    uint8_t heartbeat;
    float linear_x;
    float linear_y;
    uint8_t linear_z;               // 标志位
    float angular_x;
    float angular_y;
    float angular_z;
};

#endif // RM_TASK_H
