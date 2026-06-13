/**
 * @file motor_task.h
 * @brief Motor control task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#ifdef BSP_USING_MOTOR_TASK
void MotorTask_Entry(void *argument);
#endif

#endif // MOTOR_TASK_H
