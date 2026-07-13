/**
 * @file transmission_task.c
 * @brief Transmission (USB) task - 自瞄通信
 * @date 2026-06-07
 * @author Jasper
 */

#include <stdio.h>
#include "transmission_task.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"

extern void MX_USB_DEVICE_Init(void);

#define HEART_BEAT 500  // ms

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;
static struct trans_fdb_msg trans_fdb;

static publisher_t *pub_trans;
static subscriber_t *sub_cmd, *sub_ins, *sub_gim;

static void trans_sub_init(void);
static void trans_pub_init(void);
static void trans_sub_pull(void);
static void trans_pub_push(void);

/* ----------------------------------------------- MPC方案B：安全读取小端int32 --------------------------------------------------- */
static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t value =
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

/* ----------------------------------------------- MPC方案B：接收校验 --------------------------------------------------- */
static uint8_t verify_frame(const uint8_t *frame, uint8_t len)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        sum += frame[i];
        add += sum;
    }

    if (frame[len] != sum || frame[len + 1] != add)
        return 0;

    return 1;
}

/* ----------------------------------------------- 自瞄通信相关 --------------------------------------------------- */
uint8_t *r_buffer_point;

static uint8_t frame_buffer[sizeof(RpyTypeDef)];
static uint32_t frame_index = 0;
static enum {
    WAIT_FOR_HEADER,
    RECEIVING_DATA
} receive_state = WAIT_FOR_HEADER;

uint8_t buf[31] = {0};
RpyTypeDef rpy_tx_data = {
    .HEAD = 0XFF,
    .D_ADDR = MAINFLOD,
    .ID = GIMBAL,
    .LEN = FRAME_RPY_LEN,
    .DATA = {0},
    .SC = 0,
    .AC = 0,
};
RpyTypeDef rpy_rx_data;
static uint32_t heart_dt;
uint32_t vision_last_rx_ms;  // MPC方案B：最后一次收到有效视觉数据的时间
TeamColor team_color;

extern auto_relative_angle_status_e auto_relative_angle_status;

static float trans_dt;
static float openfire;

/* ----------------------------------------------- 线程入口 --------------------------------------------------- */
void TransmissionTask_Entry(void *argument)
{
    static float trans_start;

    MX_USB_DEVICE_Init();

    trans_sub_init();
    trans_pub_init();
    r_buffer_point = frame_buffer;

    for(;;)
    {
        trans_start = dwt_get_time_ms();

        trans_sub_pull();
        trans_pub_push();

        if ((dwt_get_time_ms() - heart_dt) >= HEART_BEAT)
        {
            heart_dt = dwt_get_time_ms();
        }

        Send_to_pc(rpy_tx_data);

        trans_dt = dwt_get_time_ms() - trans_start;
        if (trans_dt > 1)
            printf("Transmission Task is being DELAY! dt = [%f]\n", trans_dt);

        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static void trans_sub_init(void)
{
    sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
}

static void trans_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_gim, &gim_fdb);
}

static void trans_pub_init(void)
{
    pub_trans = pub_register("trans_fdb", sizeof(struct trans_fdb_msg));
}

static void trans_pub_push(void)
{
    pub_push_msg(pub_trans, &trans_fdb);
}

/* ----------------------------------------------- 发送数据到上位机 --------------------------------------------------- */
void Send_to_pc(RpyTypeDef data_r)
{
    float yaw_vel = ins_data.gyro[INS_Z];  // yaw轴角速度（rad/s）
    float yaw_acc = 1;  // yaw轴角加速度（暂未计算）
    pack_Rpy(&data_r, (gim_fdb.yaw_offset_angle - ins_data.yaw), ins_data.pitch - gim_fdb.pit_offset_angle, openfire, team_color, yaw_vel, yaw_acc);
    Check_Rpy(&data_r);
    CDC_Transmit_HS((uint8_t *)&data_r,  sizeof(data_r));
    if (gim_cmd.ctrl_mode == GIMBAL_AUTO && auto_relative_angle_status == RELATIVE_ANGLE_TRANS)
    {
        auto_relative_angle_status = RELATIVE_ANGLE_OK;
    }
}

/* ----------------------------------------------- 数据打包（MPC方案B：增加yaw_vel/yaw_acc） --------------------------------------------------- */
void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch, float openfire, int team_color, float yaw_vel, float yaw_acc)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0};
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = openfire * 1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;
    rpy_data = team_color * 1000;
    rpy_tx_buffer[13] = *gimbal_rpy;
    rpy_tx_buffer[14] = *gimbal_rpy >> 8;
    rpy_tx_buffer[15] = *gimbal_rpy >> 16;
    rpy_tx_buffer[16] = *gimbal_rpy >> 24;
    /* MPC方案B：yaw_vel/yaw_acc 放在保留字节 */
    rpy_data = yaw_vel * 1000;
    rpy_tx_buffer[17] = *gimbal_rpy;
    rpy_tx_buffer[18] = *gimbal_rpy >> 8;
    rpy_tx_buffer[19] = *gimbal_rpy >> 16;
    rpy_tx_buffer[20] = *gimbal_rpy >> 24;
    rpy_data = yaw_acc * 1000;
    rpy_tx_buffer[21] = *gimbal_rpy;
    rpy_tx_buffer[22] = *gimbal_rpy >> 8;
    rpy_tx_buffer[23] = *gimbal_rpy >> 16;
    rpy_tx_buffer[24] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer, FRAME_RPY_LEN);
    frame->LEN = FRAME_RPY_LEN;
}

/* ----------------------------------------------- 校验（MPC方案B：逐字节计算） --------------------------------------------------- */
void Check_Rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;
    uint8_t *raw = (uint8_t *)frame;

    // 头4字节(HEAD+D_ADDR+ID+LEN) + DATA，逐字节计算
    for (int i = 0; i < 4 + frame->LEN; i++)
    {
        sum += raw[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

/* ----------------------------------------------- USB 接收处理 --------------------------------------------------- */
static void usb_input(uint8_t *Buf, uint32_t *Len)
{
    for (uint32_t i = 0; i < *Len; i++)
    {
        uint8_t current_byte = Buf[i];

        switch (receive_state)
        {
            case WAIT_FOR_HEADER:
                if (current_byte == 0xFF)
                {
                    frame_index = 0;
                    frame_buffer[frame_index++] = current_byte;
                    receive_state = RECEIVING_DATA;
                }
                break;

            case RECEIVING_DATA:
                frame_buffer[frame_index++] = current_byte;

                if (frame_index >= sizeof(RpyTypeDef))
                {
                    memcpy(&rpy_rx_data, frame_buffer, sizeof(rpy_rx_data));

                    switch (rpy_rx_data.ID)
                    {
                        case GIMBAL:
                        {
                            // MPC方案B：解析 vision_mode + yaw/yaw_vel/yaw_acc + pitch
                            trans_fdb.vision_mode = rpy_rx_data.DATA[0];

                            trans_fdb.yaw     = -(read_i32_le(&rpy_rx_data.DATA[1])  / 1000.0f);
                            trans_fdb.yaw_vel = -(read_i32_le(&rpy_rx_data.DATA[5])  / 1000.0f);
                            trans_fdb.yaw_acc = -(read_i32_le(&rpy_rx_data.DATA[9])  / 1000.0f);

                            trans_fdb.pitch   =  read_i32_le(&rpy_rx_data.DATA[13]) / 1000.0f;

                            vision_last_rx_ms = dwt_get_time_ms();
                            break;
                        }

                        case HEARTBEAT:
                            trans_fdb.heartbeat = (*(uint8_t *)&rpy_rx_data.DATA[0]);
                            heart_dt = dwt_get_time_ms();
                            break;

                        case CHASSIS_CTRL:
                            trans_fdb.linear_x = (*(int32_t *)&rpy_rx_data.DATA[0] / 10000.0);
                            trans_fdb.linear_y = (*(int32_t *)&rpy_rx_data.DATA[4] / 10000.0);
                            trans_fdb.linear_z = (*(int32_t *)&rpy_rx_data.DATA[8] / 10000.0);
                            trans_fdb.angular_x = (*(int32_t *)&rpy_rx_data.DATA[12] / 10000.0);
                            trans_fdb.angular_y = (*(int32_t *)&rpy_rx_data.DATA[16] / 10000.0);
                            trans_fdb.angular_z = (*(int32_t *)&rpy_rx_data.DATA[20] / 10000.0);
                            break;
                    }

                    memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
                    memset(frame_buffer, 0, sizeof(frame_buffer));
                    receive_state = WAIT_FOR_HEADER;
                }
                break;
        }
    }
}

void process_usb_data(uint8_t *Buf, uint32_t *Len)
{
    usb_input(Buf, Len);
}

struct trans_fdb_msg *get_trans_fdb(void)
{
    return &trans_fdb;
}
