/**
 * @file usart_recive_task.c
 * @brief USART receive task
 * @date 2026-06-07
 * @author Jasper
 */

#include "usart_recive_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ------------------------------------------------- 串口接收线程入口 ------------------------------------------------------ */
void USARTRecTask_Entry(void *argument)
{
    for(;;)
    {
        // TODO: DBUS/SBUS遥控器数据接收, 裁判系统数据接收
        osDelay(1);
    }
}
