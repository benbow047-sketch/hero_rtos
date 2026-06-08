/**
 * @file transmission_task.c
 * @brief Transmission (USB) task
 * @date 2026-06-07
 * @author Jasper
 */

#include "transmission_task.h"
#include "cmsis_os2.h"

void TransmissionTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
