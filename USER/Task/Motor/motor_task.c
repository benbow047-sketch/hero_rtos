/**
 * @file motor_task.c
 * @brief Motor control task
 * @date 2026-06-07
 * @author Jasper
 */

#include "motor_task.h"
#include "cmsis_os2.h"

void MotorTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
