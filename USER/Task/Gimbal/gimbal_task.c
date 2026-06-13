/**
 * @file gimbal_task.c
 * @brief Gimbal task
 * @date 2026-06-07
 * @author Jasper
 */

#include <stdlib.h>
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_task.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "cmsis_os2.h"

#define GIM_MOTOR_NUM 2

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;

static publisher_t *pub_gim;
static subscriber_t *sub_cmd, *sub_ins;

static void gimbal_pub_init(void);
static void gimbal_sub_init(void);
static void gimbal_pub_push(void);
static void gimbal_sub_pull(void);

/* ------------------------------------------------- 电机控制相关 ------------------------------------------------------ */
static struct gimbal_controller_t {
    pid_obj_t *pid_speed_imu;
    pid_obj_t *pid_angle_imu;
    pid_obj_t *pid_speed_auto;
    pid_obj_t *pid_angle_auto;
} gim_controller[GIM_MOTOR_NUM];

static motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
    {
        .motor_type = GM6020,
        .can_name = CAN_CHASSIS,
        .rx_id = YAW_MOTOR_ID,
        .controller = &gim_controller[YAW],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = PITCH_MOTOR_ID,
        .controller = &gim_controller[PITCH],
    }
};

/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
static int16_t yaw_motor_relive, pitch_motor_relive;
static ramp_obj_t *yaw_ramp;
static ramp_obj_t *pit_ramp;
static dji_motor_object_t *gim_motor[GIM_MOTOR_NUM];
static float gim_motor_ref[GIM_MOTOR_NUM];

static void gimbal_motor_init(void);
static int16_t motor_control_yaw(dji_motor_measure_t measure);
static int16_t motor_control_pitch(dji_motor_measure_t measure);
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
static int auto_staus = 1;
auto_relative_angle_status_e auto_relative_angle_status = RELATIVE_ANGLE_TRANS;

/* ------------------------------------------------ 云台线程入口 ----------------------------------------------------- */
static float gim_dt;

void GimbalTask_Entry(void *argument)
{
    static float gim_start;
    static uint32_t init_start_time;
    uint32_t init_dt = 0;

    gimbal_pub_init();
    gimbal_sub_init();
    gimbal_motor_init();
    yaw_ramp = ramp_register(0, BACK_CENTER_TIME / GIMBAL_PERIOD);
    pit_ramp = ramp_register(0, BACK_CENTER_TIME / GIMBAL_PERIOD);

    for(;;)
    {
        gim_start = dwt_get_time_ms();
        gimbal_sub_pull();

        // 云台电机相对于归中值的角度
        yaw_motor_relive = -(int16_t)get_relative_pos(gim_motor[YAW]->measure.ecd, CENTER_ECD_YAW) / 22.75f;
        pitch_motor_relive = ins_data.pitch;

        for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
            dji_motor_enable(gim_motor[i]);

        switch (gim_cmd.ctrl_mode)
        {
            case GIMBAL_RELAX:
                for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
                    dji_motor_relax(gim_motor[i]);
                gim_fdb.back_mode = BACK_STEP;
                yaw_ramp->reset(yaw_ramp, 0, BACK_CENTER_TIME / GIMBAL_PERIOD);
                pit_ramp->reset(pit_ramp, 0, BACK_CENTER_TIME / GIMBAL_PERIOD);
                break;

            case GIMBAL_INIT:
                if (gim_cmd.last_mode != GIMBAL_INIT)
                    init_start_time = dwt_get_time_ms();
                else
                    init_dt = dwt_get_time_ms() - init_start_time;

                gim_motor_ref[YAW] = yaw_motor_relive * (1 - yaw_ramp->calc(yaw_ramp));
                gim_motor_ref[PITCH] = pitch_motor_relive * (1 - pit_ramp->calc(pit_ramp));

                if ((abs(ins_data.pitch) <= (20 / 22.75f)
                     && (abs(gim_motor[YAW]->measure.ecd - CENTER_ECD_YAW) <= 20))
                    || ((abs(ins_data.pitch) <= (200 / 22.75f))
                        && (abs(gim_motor[YAW]->measure.ecd - CENTER_ECD_YAW) <= 200)
                        && (init_dt > INIT_TIMEOUT)))
                {
                    gim_fdb.back_mode = BACK_IS_OK;
                    gim_fdb.yaw_offset_angle_total = ins_data.yaw_total_angle;
                    gim_fdb.yaw_offset_angle = ins_data.yaw;
                    gim_fdb.pit_offset_angle = ins_data.pitch;
                    auto_staus = 1;
                }
                else
                {
                    gim_fdb.back_mode = BACK_STEP;
                }
                break;

            case GIMBAL_GYRO:
                gim_motor_ref[YAW] = gim_cmd.yaw;
                gim_motor_ref[PITCH] = gim_cmd.pitch;
                gim_fdb.yaw_relative_angle = -yaw_motor_relive;
                gim_fdb.yaw_offset_angle = ins_data.yaw;
                gim_fdb.pit_offset_angle = ins_data.pitch;
                if (auto_staus == 0)
                {
                    auto_staus = 1;
                    auto_relative_angle_status = RELATIVE_ANGLE_TRANS;
                }
                break;

            case GIMBAL_AUTO:
                if (auto_staus == 1)
                {
                    auto_staus = 0;
                    auto_relative_angle_status = RELATIVE_ANGLE_TRANS;
                }
                gim_motor_ref[YAW] = gim_cmd.yaw;
                gim_motor_ref[PITCH] = gim_cmd.pitch;
                gim_fdb.yaw_relative_angle = -yaw_motor_relive;
                break;

            case GIMBAL_ECD:
                gim_motor_ref[YAW] = gim_cmd.yaw;
                gim_motor_ref[PITCH] = gim_cmd.pitch;
                gim_fdb.yaw_relative_angle = -yaw_motor_relive;
                gim_fdb.yaw_offset_angle = ins_data.yaw;
                gim_fdb.pit_offset_angle = ins_data.pitch;
                if (auto_staus == 0)
                {
                    auto_staus = 1;
                    auto_relative_angle_status = RELATIVE_ANGLE_TRANS;
                }
                break;

            default:
                for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
                    dji_motor_relax(gim_motor[i]);
                break;
        }

        gimbal_pub_push();

        gim_dt = dwt_get_time_ms() - gim_start;
        if (gim_dt > 3)
            printf("Gimbal Task is being DELAY! dt = [%f]\n", gim_dt);

        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void gimbal_pub_init(void)
{
    pub_gim = pub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
}

static void gimbal_sub_init(void)
{
    sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

static void gimbal_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_fdb);
}

static void gimbal_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
}

/* ----------------------------------------------- 电机控制相关 ------------------------------------------------------- */
static void gimbal_motor_init(void)
{
    /* YAW */
    pid_config_t yaw_speed_imu_config = INIT_PID_CONFIG(YAW_KP_V_IMU, YAW_KI_V_IMU, YAW_KD_V_IMU, YAW_INTEGRAL_V_IMU, YAW_MAX_V_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_imu_config = INIT_PID_CONFIG(YAW_KP_A_IMU, YAW_KI_A_IMU, YAW_KD_A_IMU, YAW_INTEGRAL_A_IMU, YAW_MAX_A_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_speed_auto_config = INIT_PID_CONFIG(YAW_KP_V_AUTO, YAW_KI_V_AUTO, YAW_KD_V_AUTO, YAW_INTEGRAL_V_AUTO, YAW_MAX_V_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_auto_config = INIT_PID_CONFIG(YAW_KP_A_AUTO, YAW_KI_A_AUTO, YAW_KD_A_AUTO, YAW_INTEGRAL_A_AUTO, YAW_MAX_A_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[YAW].pid_speed_imu = pid_register(&yaw_speed_imu_config);
    gim_controller[YAW].pid_angle_imu = pid_register(&yaw_angle_imu_config);
    gim_controller[YAW].pid_speed_auto = pid_register(&yaw_speed_auto_config);
    gim_controller[YAW].pid_angle_auto = pid_register(&yaw_angle_auto_config);
    gim_motor[YAW] = dji_motor_register(&gimbal_motor_config[YAW], motor_control_yaw);

    /* PITCH */
    pid_config_t pitch_speed_imu_config = INIT_PID_CONFIG(PITCH_KP_V_IMU, PITCH_KI_V_IMU, PITCH_KD_V_IMU, PITCH_INTEGRAL_V_IMU, PITCH_MAX_V_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_imu_config = INIT_PID_CONFIG(PITCH_KP_A_IMU, PITCH_KI_A_IMU, PITCH_KD_A_IMU, PITCH_INTEGRAL_A_IMU, PITCH_MAX_A_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_speed_auto_config = INIT_PID_CONFIG(PITCH_KP_V_AUTO, PITCH_KI_V_AUTO, PITCH_KD_V_AUTO, PITCH_INTEGRAL_V_AUTO, PITCH_MAX_V_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_auto_config = INIT_PID_CONFIG(PITCH_KP_A_AUTO, PITCH_KI_A_AUTO, PITCH_KD_A_AUTO, PITCH_INTEGRAL_A_AUTO, PITCH_MAX_A_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[PITCH].pid_speed_imu = pid_register(&pitch_speed_imu_config);
    gim_controller[PITCH].pid_angle_imu = pid_register(&pitch_angle_imu_config);
    gim_controller[PITCH].pid_speed_auto = pid_register(&pitch_speed_auto_config);
    gim_controller[PITCH].pid_angle_auto = pid_register(&pitch_angle_auto_config);
    gim_motor[PITCH] = dji_motor_register(&gimbal_motor_config[PITCH], motor_control_pitch);
}

/* ----------------------------------------------- YAW 电机控制 ------------------------------------------------------- */
static int16_t motor_control_yaw(dji_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;
    static float pid_out_angle;
    static int16_t send_data;

    switch (gim_cmd.ctrl_mode)
    {
        case GIMBAL_INIT:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = ins_data.gyro[INS_Z];
            get_angle = -yaw_motor_relive;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = ins_data.gyro[INS_Z];
            get_angle = ins_data.yaw_total_angle - gim_fdb.yaw_offset_angle_total;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[YAW].pid_speed_auto;
            pid_angle = gim_controller[YAW].pid_angle_auto;
            get_speed = ins_data.gyro[INS_Z];
            get_angle = ins_data.yaw_total_angle - gim_fdb.yaw_offset_angle_total;
            break;
        case GIMBAL_ECD:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = ins_data.gyro[INS_Z];
            get_angle = -yaw_motor_relive;
            break;
        default:
            break;
    }

    // 切换模式时清除PID历史
    if (gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[YAW].pid_angle_imu);
        pid_clear(gim_controller[YAW].pid_speed_imu);
        pid_clear(gim_controller[YAW].pid_angle_auto);
        pid_clear(gim_controller[YAW].pid_speed_auto);
    }

    if (gim_cmd.ctrl_mode == GIMBAL_INIT || gim_cmd.ctrl_mode == GIMBAL_ECD)
    {
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);
    }
    else if (gim_cmd.ctrl_mode != GIMBAL_RELAX && gim_cmd.ctrl_mode != GIMBAL_INIT)
    {
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);
    }

    if (gim_cmd.ctrl_mode == GIMBAL_RELAX)
    {
        send_data = 0;
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[YAW].pid_angle_imu);
        pid_clear(gim_controller[YAW].pid_speed_imu);
        pid_clear(gim_controller[YAW].pid_angle_auto);
        pid_clear(gim_controller[YAW].pid_speed_auto);
    }

    return send_data;
}

/* ----------------------------------------------- PITCH 电机控制 ----------------------------------------------------- */
static int16_t motor_control_pitch(dji_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;
    static float pid_out_angle;
    static int16_t send_data;

    switch (gim_cmd.ctrl_mode)
    {
        case GIMBAL_INIT:
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = ins_data.gyro[INS_Y];
            get_angle = ins_data.pitch;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = ins_data.gyro[INS_Y];
            get_angle = ins_data.pitch;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[PITCH].pid_speed_auto;
            pid_angle = gim_controller[PITCH].pid_angle_auto;
            get_speed = ins_data.gyro[INS_Y];
            get_angle = ins_data.pitch;
            break;
        default:
            break;
    }

    // 切换模式时清除PID历史
    if (gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[PITCH].pid_angle_imu);
        pid_clear(gim_controller[PITCH].pid_speed_imu);
        pid_clear(gim_controller[PITCH].pid_angle_auto);
        pid_clear(gim_controller[PITCH].pid_speed_auto);
    }

    if (gim_cmd.ctrl_mode == GIMBAL_AUTO)
    {
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);
        float feedforward = 25 * pid_out_angle;
        send_data = -pid_calculate(pid_speed, get_speed, pid_out_angle) - feedforward;
    }
    else
    {
        VAL_LIMIT(gim_motor_ref[PITCH], PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);
        float feedforward = 50 * pid_out_angle;
        send_data = -pid_calculate(pid_speed, get_speed, pid_out_angle) - feedforward;
    }

    if (gim_cmd.ctrl_mode == GIMBAL_RELAX)
    {
        send_data = 0;
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[PITCH].pid_angle_imu);
        pid_clear(gim_controller[PITCH].pid_speed_imu);
        pid_clear(gim_controller[PITCH].pid_angle_auto);
        pid_clear(gim_controller[PITCH].pid_speed_auto);
    }

    return send_data;
}

/* ----------------------------------------------- 编码器相对位置计算 ------------------------------------------------------- */
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4095)
    {
        if (raw_ecd > center_offset - 4095)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8191 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4095)
            tmp = raw_ecd - 8191 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}
