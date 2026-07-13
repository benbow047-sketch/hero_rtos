/**
 * @file transmission_task.h
 * @brief Transmission (USB) task
 * @date 2026-06-07
 * @author Jasper
 */

#ifndef TRANSMISSION_TASK_H
#define TRANSMISSION_TASK_H

#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

/* BCP通信协议定义 */
#define FRAME_MAX_LEN 36
#define FRAME_XYA_LEN 6
#define FRAME_AUTO_LEN 22
#define FRAME_RPY_LEN 25
#define FRAME_ODOM_LEN 36
#define FRAME_CTRL_LEN 24
#define FRAME_SHOOT_LEN 7
#define FRAME_IMU_LEN 24

/* 目标地址 */
#define BROADCAST   0x00
#define MAINFLOD    0x01
#define SENTRY_UP   0x02
#define SENTRY_DOWN 0x03
#define INFANTRY    0x04
#define ENGINEER    0x05
#define HERO        0x06
#define AIR         0x07
#define RADAR       0x08
#define GATHER      0x09
#define STANDARD    0x10

/* 数据类型 */
#define CHASSIS                 0x10
#define CHASSIS_ODOM            0x11
#define CHASSIS_CTRL            0x12
#define CHASSIS_IMU             0x13
#define GIMBAL                  0x20
#define GAME_STATUS             0x30
#define ROBOT_HP                0x31
#define ICRA_BUFF_DEBUFF_ZONE   0x32
#define GAME_MODE               0x33
#define ROBOT_COMMAND           0x34
#define CLIENT_MAP_COMMAND      0x35
#define BARREL                  0x40
#define MANIFOLD_CTRL           0x50
#define MODE                    0x60
#define DEV_ERROR               0xE0
#define HEARTBEAT               0xF0
#define FIRE                    0xF1

/**
 * @brief 通信帧结构体
 */
typedef struct
{
    uint8_t HEAD;
    uint8_t D_ADDR;
    uint8_t ID;
    uint8_t LEN;
    uint8_t DATA[FRAME_MAX_LEN];
    uint8_t SC;
    uint8_t AC;
} __attribute__((packed)) BCPFrameTypeDef;

/**
 * @brief 欧拉角RPY格式通信帧结构体
 */
typedef struct
{
    uint8_t HEAD;
    uint8_t D_ADDR;
    uint8_t ID;
    uint8_t LEN;
    int8_t DATA[FRAME_RPY_LEN];
    uint8_t SC;
    uint8_t AC;
} __attribute__((packed)) RpyTypeDef;

/**
 * @brief 队伍颜色
 */
typedef enum
{
    RED = 1,
    BLUE = 0,
    UNKNOWN = -1
} TeamColor;

#ifdef BSP_USING_TRANSMISSION_TASK
void TransmissionTask_Entry(void *argument);
#endif

void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch, float openfire, int team_color, float yaw_vel, float yaw_acc);
void Check_Rpy(RpyTypeDef *frame);
void Send_to_pc(RpyTypeDef data_r);
void process_usb_data(uint8_t *Buf, uint32_t *Len);
struct trans_fdb_msg *get_trans_fdb(void);

#endif // TRANSMISSION_TASK_H
