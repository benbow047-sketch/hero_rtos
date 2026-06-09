/**
 * @file motor_task.c
 * @brief Motor control task
 * @date 2026-06-07
 * @author Jasper
 */

#include "motor_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ------------------------------------------------- 电机控制线程入口 ------------------------------------------------------ */
void MotorTask_Entry(void *argument)
{
    for(;;)
    {
        // TODO: 调用dji_motor_control()发送CAN控制帧
        osDelay(1);
    }
}
