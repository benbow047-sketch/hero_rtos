/**
 * @file referee_task.c
 * @brief Referee system task
 * @date 2026-06-07
 * @author Jasper
 */

#include "referee_task.h"
#include "cmsis_os2.h"

void RefereeTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
