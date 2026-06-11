/**
 * @file rm_module.h
 * @brief Module layer aggregation header
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef RM_MODULE_H
#define RM_MODULE_H

/* ---- 算法层 ---- */
#include "rm_algorithm.h"

/* ---- Pub/Sub 消息系统 ---- */
#include "drv_msg.h"

/* ---- DJI 电机驱动 ---- */
#include "dj_motor.h"

/* ---- FDCAN 驱动 ---- */
#include "bsp_fdcan.h"

/* ---- BMI088 IMU 驱动 ---- */
#include "BMI088driver.h"

/* ---- DWT 定时 ---- */
#include "drv_dwt.h"

/* ---- 遥控器 ---- */
#ifdef BSP_USING_RC_DBUS
#include "rc_dbus.h"
#else
#include "rc_sbus.h"
#endif

#include "keyboard.h"

/* ---- 裁判系统 ---- */
// #include "referee_system.h"

#endif // RM_MODULE_H
