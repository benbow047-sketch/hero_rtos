/**
 * @file rm_task.h
 * @brief FreeRTOS task management
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef RM_TASK_H
#define RM_TASK_H

/* ==================== Task Switches ==================== */
#define BSP_USING_INS_TASK
#define BSP_USING_MOTOR_TASK
#define BSP_USING_CMD_TASK
#define BSP_USING_CHASSIS_TASK
#define BSP_USING_GIMBAL_TASK
#define BSP_USING_SHOOT_TASK
#define BSP_USING_TRANSMISSION_TASK
#define BSP_USING_USARTREC_TASK
#define BSP_USING_REFEREE_TASK
#define BSP_USING_SUPERCAP_TASK

/* ==================== Task Headers ==================== */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif
#ifdef BSP_USING_MOTOR_TASK
#include "motor_task.h"
#endif
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif
#ifdef BSP_USING_GIMBAL_TASK
#include "gimbal_task.h"
#endif
#ifdef BSP_USING_SHOOT_TASK
#include "shoot_task.h"
#endif
#ifdef BSP_USING_TRANSMISSION_TASK
#include "transmission_task.h"
#endif
#ifdef BSP_USING_USARTREC_TASK
#include "usart_recive_task.h"
#endif
#ifdef BSP_USING_REFEREE_TASK
#include "referee_task.h"
#endif
#ifdef BSP_USING_SUPERCAP_TASK
#include "supercap_task.h"
#endif

#endif // RM_TASK_H
