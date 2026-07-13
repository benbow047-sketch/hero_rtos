/**
 * @file cmd_task.c
 * @brief Command task - 遥控器状态机 + 控制指令生成
 * @date 2026-06-07
 * @author Jasper
 */

#include <stdio.h>
#include <string.h>
#include "rm_config.h"
#include "rm_task.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static publisher_t *pub_gim, *pub_chassis, *pub_shoot;
static subscriber_t *sub_gim, *sub_shoot, *sub_trans, *sub_ins, *sub_chassis;
static struct gimbal_cmd_msg gim_cmd;
static struct gimbal_fdb_msg gim_fdb;
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;
static struct trans_fdb_msg trans_fdb;
static struct ins_msg ins_data;
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;

static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/* ---------------------------------------------------- 底盘相关 ------------------------------------------------------ */
static int spin_cnt = 0;
float follow_err, vw;
static pid_obj_t *follow_pid;
static pid_config_t chassis_follow_config = INIT_PID_CONFIG(
    CHASSIS_KP_V_FOLLOW, CHASSIS_KI_V_FOLLOW, CHASSIS_KD_V_FOLLOW,
    CHASSIS_INTEGRAL_V_FOLLOW, CHASSIS_MAX_V_FOLLOW,
    (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

/* ---------------------------------------------------- 云台相关 ------------------------------------------------------ */
static int mirror_servo_flag = 0;
static float mouse_accumulate_x = 0;
static float mouse_accumulate_y = 0;
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;

/* --------------------------------------------------- 发射机构相关 ---------------------------------------------------- */
static int trigger_flag = 0;
static int reverse_cnt;

/* --------------------------------------------------- 遥控器相关 ---------------------------------------------------- */
#ifdef BSP_USING_RC_DBUS
extern rc_dbus_obj_t dbus_data_fdb;
extern volatile uint32_t dbus_heartbeat;
static rc_dbus_obj_t *rc_now, *rc_last;
static uint32_t last_dbus_heartbeat = 0;
static uint32_t last_dbus_tick = 0;
#define DBUS_TIMEOUT_MS  500  // 遥控器信号丢失超时
#else
extern sbus_data_t sbus_data_fdb;
#endif

/* --------------------------------------------------- 键鼠相关 ---------------------------------------------------- */
static int key_e_status = -1;
static int key_f_status = -1;
static int rc_f_status = -1;

ramp_obj_t *km_vx_ramp = NULL;
ramp_obj_t *km_vy_ramp = NULL;
ramp_obj_t *km_vw_ramp = NULL;

First_Order_Filter_t mouse_y_lpf, mouse_x_lpf;
float Ballistic;

/* ----------------------------------------------- 其余变量与函数声明 ---------------------------------------------------- */
static int cnt_flag = 0;
static int deploy_flag = 0;

extern auto_relative_angle_status_e auto_relative_angle_status;
extern uint8_t *r_buffer_point;
SemaphoreHandle_t sbus_cmd_mutex = NULL;

static void remote_to_cmd_dbus(void);
static void remote_to_cmd_sbus(void);

/* ------------------------------------------------- cmd线程入口 ------------------------------------------------------ */
static float cmd_dt;

void CmdTask_Entry(void *argument)
{
    static float cmd_start;

    dbus_data_init();
    sbus_cmd_mutex = xSemaphoreCreateMutex();
    rc_last = (rc_now + 1);

    cmd_pub_init();
    cmd_sub_init();

    follow_pid = pid_register(&chassis_follow_config);

    km_vx_ramp = ramp_register(0, 200);
    km_vy_ramp = ramp_register(0, 200);
    km_vw_ramp = ramp_register(0, 200);

    First_Order_Filter_Init(&mouse_x_lpf, 0.014f, 0.1f);
    First_Order_Filter_Init(&mouse_y_lpf, 0.014f, 0.1f);

#ifdef BSP_USING_RC_DBUS
    rc_now->sw1 = RC_UP;
    rc_now->sw2 = RC_UP;
#else
    sbus_data_fdb.sw1 = RC_UP;
    sbus_data_fdb.sw2 = RC_UP;
    sbus_data_fdb.sw3 = RC_UP;
    sbus_data_fdb.sw4 = RC_UP;
#endif

    for(;;)
    {
        cmd_start = dwt_get_time_ms();
        cmd_sub_pull();

#ifdef BSP_USING_RC_DBUS
        remote_to_cmd_dbus();
#else
        remote_to_cmd_sbus();
#endif

        cmd_pub_push();
        cmd_dt = dwt_get_time_ms() - cmd_start;
        osDelay(1);
    }
}

/* ----------------------------------------------- DBUS 遥控器处理 --------------------------------------------------- */
#ifdef BSP_USING_RC_DBUS
static void remote_to_cmd_dbus(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode = shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    *rc_now = dbus_data_fdb;
    xSemaphoreGive(sbus_cmd_mutex);

    float fx = First_Order_Filter_Calculate(&mouse_x_lpf, rc_now->mouse.x);
    float fy = First_Order_Filter_Calculate(&mouse_y_lpf, rc_now->mouse.y);
    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf, rc_now->mouse.y) * 0.05f;

    /* 底盘命令 */
    chassis_cmd.vx = (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
    chassis_cmd.vy = (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;

    /* 遥控器信号丢失检测 */
    if (dbus_heartbeat != last_dbus_heartbeat)
    {
        last_dbus_heartbeat = dbus_heartbeat;
        last_dbus_tick = xTaskGetTickCount();
    }
    else if ((xTaskGetTickCount() - last_dbus_tick) > pdMS_TO_TICKS(DBUS_TIMEOUT_MS))
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode = SHOOT_STOP;
        shoot_cmd.friction_status = 0;
        gim_cmd.pitch = 0;
        gim_cmd.yaw = 0;
        gyro_yaw_inherit = 0;
        gyro_pitch_inherit = 0;
        key_e_status = -1;
        key_f_status = -1;
        memset(r_buffer_point, 0, sizeof(*r_buffer_point));
        return;
    }

#ifndef BSP_USING_GIMBAL_TASK
    /* 云台任务未启用时，底盘开环直控 */
    if (rc_now->sw2 == RC_UP)
    {
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        chassis_cmd.vx = 0;
        chassis_cmd.vy = 0;
        chassis_cmd.vw = 0;
    }
    else
    {
        chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
        chassis_cmd.vw = (float)rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED;
    }
    return;
#endif

    /* 云台命令 */
    if (gim_cmd.ctrl_mode == GIMBAL_GYRO)
    {
        gim_cmd.yaw += (float)rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW + fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += (float)rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit = gim_cmd.yaw;
        gyro_pitch_inherit = gim_cmd.pitch;
        mouse_accumulate_x = 0;
        mouse_accumulate_y = 0;
    }
    if (gim_cmd.ctrl_mode == GIMBAL_AUTO)
    {
        if (auto_relative_angle_status == RELATIVE_ANGLE_TRANS)
        {
            trans_fdb.yaw = 0;
            trans_fdb.pitch = 0;
        }
        mouse_accumulate_y -= fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        if (trans_fdb.roll != 2)
        {
            gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + mouse_accumulate_x;
            gim_cmd.pitch = trans_fdb.pitch + gyro_pitch_inherit + mouse_accumulate_y;
        }
    }

    /* 限制云台pitch轴角度 */
    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    if (gim_cmd.ctrl_mode == GIMBAL_INIT || gim_cmd.ctrl_mode == GIMBAL_RELAX)
    {
        gim_cmd.pitch = 0;
        gyro_pitch_inherit = 0;
        gim_cmd.yaw = 0;
        gyro_yaw_inherit = 0;
        mouse_accumulate_x = 0;
        mouse_accumulate_y = 0;
    }

    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    if (rc_now->mouse.r == 0)
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == BACK_IS_OK)
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                memset(r_buffer_point, 0, sizeof(*r_buffer_point));
            }
            else if (gim_fdb.back_mode == BACK_STEP)
            {
                chassis_cmd.ctrl_mode = CHASSIS_RELAX;
                gim_cmd.ctrl_mode = GIMBAL_INIT;
            }
        }
    }

    if (rc_now->mouse.r == 1 || rc_now->sw2 == RC_DN)
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == 1)
            {
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else if (gim_fdb.back_mode == BACK_STEP)
            {
                chassis_cmd.ctrl_mode = CHASSIS_RELAX;
                gim_cmd.ctrl_mode = GIMBAL_INIT;
            }
        }
    }

    /* 小陀螺 */
    if (km.e_sta == KEY_PRESS_ONCE)
        key_e_status = 1;
    if (key_e_status == 1 || rc_now->sw1 == RC_DN)
    {
        if (gim_fdb.back_mode == BACK_IS_OK)
            chassis_cmd.ctrl_mode = CHASSIS_SPIN;
        else
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
    }
    if (rc_now->kb.bit.Q == 1)
        key_e_status = 0;
    if (key_e_status == 0)
        chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;

    if (chassis_cmd.ctrl_mode == CHASSIS_SPIN)
    {
        chassis_cmd.vw = 3;
        if (chassis_fdb.vw_ch < chassis_cmd.vw * 0.85f)
        {
            spin_cnt++;
            if (spin_cnt > 2000)
            {
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                spin_cnt = 0;
            }
        }
        else
            spin_cnt = 0;
    }
    else if (chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL)
    {
        follow_err = chassis_cmd.offset_angle;
        if (follow_err < 5 && follow_err >= 0)
            chassis_cmd.offset_angle = follow_err * follow_err / 5;
        else if (follow_err < 0 && follow_err > -5)
            chassis_cmd.offset_angle = -follow_err * follow_err / 5;

        vw = -pid_calculate(follow_pid, chassis_cmd.offset_angle, SIDEWAYS_ANGLE);
        chassis_cmd.vw = vw;
    }

    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    if (km.f_sta == KEY_PRESS_ONCE)
        key_f_status = 1;
    if (rc_now->sw1 == RC_MI)
        rc_f_status = 1;
    if (key_f_status == 1 || rc_f_status == 1)
        shoot_cmd.friction_status = 1;
    else
        shoot_cmd.friction_status = 0;

    if (rc_now->kb.bit.G == 1)
        key_f_status = 0;
    if (rc_now->sw1 != RC_MI)
    {
        if (rc_f_status == 1)
            rc_f_status = 0;
    }

    /* 连发模式 */
    if ((km.v_sta != KEY_RELEASE && km.v_sta != KEY_WAIT_EFFECTIVE) || (rc_now->wheel <= -300))
    {
        if (shoot_cmd.friction_status == 1)
        {
            shoot_cmd.ctrl_mode = SHOOT_COUNTINUE;
            shoot_cmd.shoot_freq = 2000;
        }
    }
    else if (km.lk_sta == KEY_PRESS_ONCE || rc_now->wheel >= 400 && (shoot_cmd.friction_status == 1))
    {
        trigger_flag = 1;
        shoot_cmd.ctrl_mode = SHOOT_ONE;
        shoot_cmd.trigger_status = TRIGGER_OFF;
    }
    else if (km.lk_sta == KEY_RELEASE || rc_now->wheel == 0 && (shoot_cmd.friction_status == 1))
    {
        if (shoot_fdb.trigger_status == SHOOT_OK)
            trigger_flag = 0;
        if (trigger_flag == 0)
        {
            shoot_cmd.ctrl_mode = SHOOT_ONE;
            shoot_cmd.trigger_status = TRIGGER_OFF;
        }
        else if (trigger_flag == 1)
        {
            shoot_cmd.ctrl_mode = SHOOT_ONE;
            shoot_cmd.trigger_status = TRIGGER_ON;
        }
    }
    else
    {
        shoot_cmd.ctrl_mode = SHOOT_STOP;
        shoot_cmd.shoot_freq = 0;
    }

    /* 堵弹反转检测 */
    if (shoot_fdb.trigger_motor_current >= 16300)
    {
        reverse_cnt++;
        if (reverse_cnt < 300)
            reverse_cnt++;
        else
        {
            reverse_cnt = 0;
            shoot_cmd.ctrl_mode = SHOOT_REVERSE;
        }
    }

    /* 倍镜舵机控制 */
    if (rc_now->kb.bit.R == 1 || rc_now->wheel <= -600 && shoot_cmd.friction_status == 0)
        mirror_servo_flag = 1;
    else if (rc_now->wheel == 0 && mirror_servo_flag == 1)
    {
        mirror_servo_flag = 0;
        if (shoot_cmd.mirror_enable == 1)
            shoot_cmd.mirror_enable = 0;
        else
            shoot_cmd.mirror_enable = 1;
    }

    /* 手动模式下清空自瞄传过来的角度buffer */
    if (gim_cmd.ctrl_mode == GIMBAL_GYRO)
        memset(r_buffer_point, 0, sizeof(*r_buffer_point));

    /* 使能判断 */
    if (rc_now->wheel >= 600 && rc_now->sw2 == RC_MI)
    {
        cnt_flag++;
        if (cnt_flag >= 400)
        {
            cnt_flag = 0;
            deploy_flag = (deploy_flag == 0) ? 1 : 0;
        }
    }
    else
        cnt_flag = 0;

    if (deploy_flag == 1)
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;

    /* 使能判断放最后，防止抽风 */
    if (rc_now->sw2 != RC_MI && rc_now->sw2 != RC_DN)
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode = SHOOT_STOP;
        shoot_cmd.friction_status = 0;
        gim_cmd.pitch = 0;
        gim_cmd.yaw = 0;
        gyro_yaw_inherit = 0;
        gyro_pitch_inherit = 0;
        key_e_status = -1;
        key_f_status = -1;
        memset(r_buffer_point, 0, sizeof(*r_buffer_point));
    }
}
#endif

/* ----------------------------------------------- SBUS 遥控器处理 --------------------------------------------------- */
#ifndef BSP_USING_RC_DBUS
static void remote_to_cmd_sbus(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode = shoot_cmd.ctrl_mode;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    sbus_data_t tmp_data = sbus_data_fdb;
    xSemaphoreGive(sbus_cmd_mutex);

    /* 底盘命令 */
    chassis_cmd.vx = (float)tmp_data.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
    chassis_cmd.vy = (float)tmp_data.ch3 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED;
    chassis_cmd.vw = (float)tmp_data.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;

    /* 云台命令 */
    if (gim_cmd.ctrl_mode == GIMBAL_GYRO)
    {
        gim_cmd.yaw += (float)tmp_data.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;
        gim_cmd.pitch += (float)tmp_data.ch2 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;
        gyro_yaw_inherit = gim_cmd.yaw;
        gyro_pitch_inherit = gim_cmd.pitch;
    }
    if (gim_cmd.ctrl_mode == GIMBAL_AUTO)
    {
        if (auto_relative_angle_status == RELATIVE_ANGLE_TRANS)
        {
            trans_fdb.yaw = 0;
            trans_fdb.pitch = 0;
        }
        if (trans_fdb.roll != 2)
        {
            gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit;
            gim_cmd.pitch = trans_fdb.pitch + gyro_pitch_inherit;
        }
    }

    /* 限制云台pitch轴角度 */
    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    if (gim_cmd.ctrl_mode == GIMBAL_INIT || gim_cmd.ctrl_mode == GIMBAL_RELAX)
    {
        gim_cmd.pitch = 0;
        gim_cmd.yaw = 0;
    }

    /*-------------------------------------------------底盘_云台状态机---------------------------------------------------------*/
    if (gim_cmd.last_mode == GIMBAL_RELAX)
    {
        gim_cmd.ctrl_mode = GIMBAL_INIT;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
    }
    else
    {
        if (gim_fdb.back_mode == BACK_IS_OK)
        {
            gim_cmd.ctrl_mode = GIMBAL_GYRO;
            chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            memset(r_buffer_point, 0, sizeof(*r_buffer_point));
        }
        else if (gim_fdb.back_mode == BACK_STEP)
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
    }

    if (tmp_data.sw3 == RC_DN)
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == 1)
            {
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else if (gim_fdb.back_mode == BACK_STEP)
            {
                chassis_cmd.ctrl_mode = CHASSIS_RELAX;
                gim_cmd.ctrl_mode = GIMBAL_INIT;
            }
        }
    }

    /* 小陀螺 */
    if (tmp_data.sw2 == RC_DN)
    {
        if (gim_fdb.back_mode == BACK_IS_OK)
            chassis_cmd.ctrl_mode = CHASSIS_SPIN;
        else
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
    }

    if (chassis_cmd.ctrl_mode == CHASSIS_SPIN)
    {
        chassis_cmd.vw = 3;
        if (chassis_fdb.vw_ch < chassis_cmd.vw * 0.85f)
        {
            spin_cnt++;
            if (spin_cnt > 2000)
            {
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                spin_cnt = 0;
            }
        }
        else
            spin_cnt = 0;
    }

    /*------------------------------------------------发射模块状态机-------------------------------------------------------*/
    if (tmp_data.sw1 == RC_DN)
        rc_f_status = 1;
    if (key_f_status == 1 || rc_f_status == 1)
        shoot_cmd.friction_status = 1;
    else
        shoot_cmd.friction_status = 0;

    if (tmp_data.sw1 == RC_UP)
    {
        if (rc_f_status == 1)
            rc_f_status = 0;
    }

    /* 扳机连发模式 */
    if (tmp_data.sw4 == RC_DN && (shoot_cmd.friction_status == 1))
    {
        trigger_flag = 1;
        shoot_cmd.ctrl_mode = SHOOT_ONE;
        shoot_cmd.trigger_status = TRIGGER_OFF;
    }
    else if (tmp_data.sw4 == RC_UP && (shoot_cmd.friction_status == 1))
    {
        if (shoot_fdb.trigger_status == SHOOT_OK)
            trigger_flag = 0;
        if (trigger_flag == 0)
        {
            shoot_cmd.ctrl_mode = SHOOT_ONE;
            shoot_cmd.trigger_status = TRIGGER_OFF;
        }
        else if (trigger_flag == 1)
        {
            shoot_cmd.ctrl_mode = SHOOT_ONE;
            shoot_cmd.trigger_status = TRIGGER_ON;
        }
    }
    else
    {
        shoot_cmd.ctrl_mode = SHOOT_STOP;
        shoot_cmd.shoot_freq = 0;
    }

    /* 堵弹反转检测 */
    if (shoot_fdb.trigger_motor_current >= 16300)
    {
        reverse_cnt++;
        if (reverse_cnt < 300)
            reverse_cnt++;
        else
        {
            reverse_cnt = 0;
            shoot_cmd.ctrl_mode = SHOOT_REVERSE;
        }
    }

    /* 手动模式下清空自瞄传过来的角度buffer */
    if (gim_cmd.ctrl_mode == GIMBAL_GYRO)
        memset(r_buffer_point, 0, sizeof(*r_buffer_point));

    /* 使能判断 */
    if (tmp_data.sw3 == RC_UP)
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode = SHOOT_STOP;
        shoot_cmd.friction_status = 0;
        gim_cmd.pitch = 0;
        gim_cmd.yaw = 0;
        gyro_yaw_inherit = 0;
        gyro_pitch_inherit = 0;
        key_e_status = -1;
        key_f_status = -1;
        memset(r_buffer_point, 0, sizeof(*r_buffer_point));
    }
}
#endif

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static void cmd_pub_init(void)
{
    pub_gim = pub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot = pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

static void cmd_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_cmd);
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
}

static void cmd_sub_init(void)
{
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
    sub_shoot = sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_trans = sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_chassis = sub_register("chassis_fdb", sizeof(struct chassis_fdb_msg));
}

static void cmd_sub_pull(void)
{
    sub_get_msg(sub_gim, &gim_fdb);
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_trans, &trans_fdb);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_chassis, &chassis_fdb);
}
