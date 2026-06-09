/**
 * @file referee_task.c
 * @brief Referee system task
 * @date 2026-06-07
 * @author Jasper
 */

#include "referee_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ------------------------------------------------- 裁判系统线程入口 ------------------------------------------------------ */
void RefereeTask_Entry(void *argument)
{
    for(;;)
    {
        // TODO: 裁判系统数据解析
        osDelay(100);
    }
}
