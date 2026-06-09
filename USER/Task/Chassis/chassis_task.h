/**
 * @file chassis_task.h
 * @brief Chassis task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_STOP,          //底盘停止
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY,           //底盘飞坡模式
    CHASSIS_AUTO           //底盘自动模式
} chassis_mode_e;

void ChassisTask_Entry(void *argument);

#endif // CHASSIS_TASK_H
