/**
 * @file usart_recive_task.c
 * @brief USART receive task
 * @date 2026-06-07
 * @author Jasper
 */

#include "usart_recive_task.h"
#include "cmsis_os2.h"

void USARTRecTask_Entry(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}
