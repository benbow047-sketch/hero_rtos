/**
 * @file transmission_task.h
 * @brief Transmission (USB) task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef TRANSMISSION_TASK_H
#define TRANSMISSION_TASK_H

#ifdef BSP_USING_TRANSMISSION_TASK
void TransmissionTask_Entry(void *argument);
#endif

#endif // TRANSMISSION_TASK_H
