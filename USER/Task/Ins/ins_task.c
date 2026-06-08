/**
 * @file ins_task.c
 * @brief INS (Inertial Navigation System) task
 * @date 2026-06-07
 * @author Jasper
 */

#include "ins_task.h"
#include "cmsis_os2.h"

void InsTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
