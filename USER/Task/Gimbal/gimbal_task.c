/**
 * @file gimbal_task.c
 * @brief Gimbal task
 * @date 2026-06-07
 * @author Jasper
 */

#include "gimbal_task.h"
#include "cmsis_os2.h"

void GimbalTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
