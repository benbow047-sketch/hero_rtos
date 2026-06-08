/**
 * @file chassis_task.c
 * @brief Chassis task
 * @date 2026-06-07
 * @author Jasper
 */

#include "chassis_task.h"
#include "cmsis_os2.h"

void ChassisTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
