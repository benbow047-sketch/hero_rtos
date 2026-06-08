/**
 * @file cmd_task.c
 * @brief Command task
 * @date 2026-06-07
 * @author Jasper
 */

#include "cmd_task.h"
#include "cmsis_os2.h"

void CmdTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
