/**
 * @file shoot_task.h
 * @brief Shoot task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/**
 * @brief 发射器模式
 */
typedef enum
{
    SHOOT_STOP=0,        //发射关闭
    SHOOT_ONE=1,         //单发模式
    SHOOT_THREE=2,       //三连发模式
    SHOOT_COUNTINUE=3,   //自动连发
    SHOOT_REVERSE=4,     //堵弹反转
    SHOOT_AUTO=5,        //自动射击模式
} shoot_mode_e;

/**
 * @brief 拨弹模式
 */
typedef enum
{
    TRIGGER_ON=1,        //拨弹开启状态
    TRIGGER_OFF=0,       //拨弹关闭状态
    TRIGGER_ING=2,       //拨弹进行状态
} trigger_mode_e;

/**
 * @brief 发射弹频
 */
typedef enum
{
    LOW_FREQUENCY=1,
    MIDDLE_FREQUENCY=2,
    HIGH_FREQUENCY=3,
} shoot_frequency_e;

/**
 * @brief 发射器状态反馈
 */
typedef enum
{
    SHOOT_OK=1,          //发射正常
    SHOOT_ERR=0,         //发射异常
    SHOOT_WAITING=2,     //等待中
} shoot_back_e;

/**
 * @brief 发射器角度继承
 */
typedef enum
{
    SHOOT_ANGLE_CONTINUE=0,  //角度为连续状态
    SHOOT_ANGLE_SINGLE=1,    //角度为单次状态
} shoot_angle_inherit_e;

void ShootTask_Entry(void *argument);

#endif // SHOOT_TASK_H
