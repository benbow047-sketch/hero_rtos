/**
 * @file shoot_task.c
 * @brief Shoot task
 * @date 2026-06-07
 * @author Jasper
 */

#include "shoot_task.h"
#include "cmsis_os2.h"

void ShootTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
