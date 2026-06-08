/**
 * @file supercap_task.c
 * @brief Super capacitor task
 * @date 2026-06-07
 * @author Jasper
 */

#include "supercap_task.h"
#include "cmsis_os2.h"

void Supercap_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
