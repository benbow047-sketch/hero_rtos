/**
 * @file shoot_task.c
 * @brief Shoot task
 * @date 2026-06-07
 * @author Jasper
 */

#include "shoot_task.h"
#include "rm_config.h"
#include "rm_task.h"
#include "motor_def.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;

static publisher_t *pub_shoot;
static subscriber_t *sub_cmd;

static void shoot_pub_init(void);
static void shoot_sub_init(void);
static void shoot_pub_push(void);
static void shoot_sub_pull(void);

/* ------------------------------------------------- 发射机构相关 ----------------------------------------------------- */
#define SHT_MOTOR_NUM 4

#define RIGHT_FRICTION 0
#define LEFT_FRICTION 1
#define MIDDLE_FRICTION 2
#define TRIGGER_MOTOR 3

static struct shoot_controller_t {
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
} sht_controller[SHT_MOTOR_NUM];

static motor_config_t shoot_motor_config[SHT_MOTOR_NUM] = {
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = RIGHT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[RIGHT_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = LEFT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[LEFT_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = MIDDLE_FRICTION_MOTOR_ID,
        .controller = &sht_controller[MIDDLE_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = TRIGGER_MOTOR_ID,
        .controller = &sht_controller[TRIGGER_MOTOR],
    }
};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];
static float shoot_motor_ref[SHT_MOTOR_NUM];

/* ------------------------------------------------- 电机控制相关 ----------------------------------------------------- */
static int total_angle_flag = SHOOT_ANGLE_CONTINUE;
static void shoot_motor_init(void);
static int16_t motor_control_right(dji_motor_measure_t measure);
static int16_t motor_control_left(dji_motor_measure_t measure);
static int16_t motor_control_middle(dji_motor_measure_t measure);
static int16_t motor_control_trigger(dji_motor_measure_t measure);
static int reverse_ref;

/* ----------------------------------------------- 线程入口 --------------------------------------------------- */
static float sht_dt;

void ShootTask_Entry(void *argument)
{
    static float sht_start;
    static float sht_gap_time;
    static float sht_gap_start_time;

    shoot_motor_init();
    shoot_pub_init();
    shoot_sub_init();

    /* 状态初始化 */
    shoot_cmd.ctrl_mode = SHOOT_STOP;
    shoot_cmd.trigger_status = TRIGGER_OFF;
    shoot_motor_ref[TRIGGER_MOTOR] = 0;

    for(;;)
    {
        sht_start = dwt_get_time_ms();
        shoot_sub_pull();

        /* 使能电机 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
            dji_motor_enable(sht_motor[i]);

        shoot_fdb.trigger_motor_current = sht_motor[TRIGGER_MOTOR]->measure.real_current;

        /* 控制摩擦轮 */
        if (shoot_cmd.friction_status == SHOOT_ONE)
        {
            shoot_motor_ref[RIGHT_FRICTION] = -FRICTION_SPEED_ONE;
            shoot_motor_ref[MIDDLE_FRICTION] = -FRICTION_SPEED_ONE;
            shoot_motor_ref[LEFT_FRICTION] = FRICTION_SPEED_ONE;
            if (shoot_cmd.ctrl_mode == SHOOT_COUNTINUE)
            {
                shoot_motor_ref[RIGHT_FRICTION] = -FRICTION_SPEED_CONTINUE;
                shoot_motor_ref[MIDDLE_FRICTION] = -FRICTION_SPEED_CONTINUE;
                shoot_motor_ref[LEFT_FRICTION] = FRICTION_SPEED_CONTINUE;
            }
        }
        else
        {
            shoot_motor_ref[TRIGGER_MOTOR] = 0;
            shoot_motor_ref[RIGHT_FRICTION] = 0;
            shoot_motor_ref[MIDDLE_FRICTION] = 0;
            shoot_motor_ref[LEFT_FRICTION] = 0;
            total_angle_flag = SHOOT_ANGLE_CONTINUE;
        }

        switch (shoot_cmd.ctrl_mode)
        {
            case SHOOT_STOP:
                shoot_motor_ref[TRIGGER_MOTOR] = 0;
                total_angle_flag = 0;
                shoot_fdb.trigger_status = SHOOT_WAITING;
                break;

            case SHOOT_ONE:
                if (shoot_cmd.trigger_status == TRIGGER_OFF)
                    shoot_fdb.trigger_status = SHOOT_WAITING;

                if (total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag = SHOOT_ANGLE_SINGLE;
                }

                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    sht_gap_time = dwt_get_time_ms() - sht_gap_start_time;
                    sht_gap_start_time = dwt_get_time_ms();

                    if (sht_gap_time >= 300)
                    {
                        if (shoot_motor_ref[TRIGGER_MOTOR] - sht_controller[TRIGGER_MOTOR].pid_angle->Measure > TRIGGER_MOTOR_120_TO_ANGLE * 19 * 0.3)
                        {
                            shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR];
                        }
                        else if (reverse_ref != 0 && shoot_cmd.last_mode == SHOOT_REVERSE)
                        {
                            if (sht_controller[TRIGGER_MOTOR].pid_angle->Measure - reverse_ref > TRIGGER_MOTOR_120_TO_ANGLE * 19 * 0.4)
                            {
                                shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR];
                            }
                        }
                        else
                        {
                            shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_120_TO_ANGLE * 19;
                        }
                    }
                    shoot_cmd.trigger_status = TRIGGER_OFF;
                    shoot_fdb.trigger_status = SHOOT_OK;
                }
                break;

            case SHOOT_THREE:
                if (total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag = SHOOT_ANGLE_SINGLE;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_120_TO_ANGLE * 19;
                    shoot_cmd.trigger_status = TRIGGER_OFF;
                    shoot_fdb.trigger_status = SHOOT_OK;
                }
                break;

            case SHOOT_COUNTINUE:
                shoot_motor_ref[TRIGGER_MOTOR] = shoot_cmd.shoot_freq;
                total_angle_flag = SHOOT_ANGLE_CONTINUE;
                shoot_fdb.trigger_status = SHOOT_OK;
                break;

            case SHOOT_REVERSE:
                if (shoot_motor_ref[TRIGGER_MOTOR] - reverse_ref < TRIGGER_MOTOR_120_TO_ANGLE * 19 * 1.8)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR];
                }
                else
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = shoot_motor_ref[TRIGGER_MOTOR] - TRIGGER_MOTOR_120_TO_ANGLE * 19 * 1.5;
                    reverse_ref = shoot_motor_ref[TRIGGER_MOTOR];
                }
                total_angle_flag = SHOOT_ANGLE_SINGLE;
                break;

            default:
                for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
                    dji_motor_relax(sht_motor[i]);
                shoot_fdb.trigger_status = SHOOT_ERR;
                break;
        }

        shoot_pub_push();

        sht_dt = dwt_get_time_ms() - sht_start;
        if (sht_dt > 1)
            printf("Shoot Task is being DELAY! dt = [%f]\n", sht_dt);

        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static void shoot_pub_init(void)
{
    pub_shoot = pub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
}

static void shoot_sub_init(void)
{
    sub_cmd = sub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

static void shoot_pub_push(void)
{
    pub_push_msg(pub_shoot, &shoot_fdb);
}

static void shoot_sub_pull(void)
{
    sub_get_msg(sub_cmd, &shoot_cmd);
}

/* ----------------------------------------------- 电机初始化 ------------------------------------------------------- */
static void shoot_motor_init(void)
{
    /* 右摩擦轮 */
    pid_config_t right_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V, RIGHT_INTEGRAL_V, RIGHT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[RIGHT_FRICTION].pid_speed = pid_register(&right_speed_config);

    /* 左摩擦轮 */
    pid_config_t left_speed_config = INIT_PID_CONFIG(LEFT_KP_V, LEFT_KI_V, LEFT_KD_V, LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                     (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[LEFT_FRICTION].pid_speed = pid_register(&left_speed_config);

    /* 中摩擦轮 */
    pid_config_t middle_speed_config = INIT_PID_CONFIG(MIDDLE_KP_V, MIDDLE_KI_V, MIDDLE_KD_V, MIDDLE_INTEGRAL_V, MIDDLE_MAX_V,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[MIDDLE_FRICTION].pid_speed = pid_register(&middle_speed_config);

    /* 拨弹电机 */
    pid_config_t toggle_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V, TRIGGER_KI_V, TRIGGER_KD_V, TRIGGER_INTEGRAL_V, TRIGGER_MAX_V,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t toggle_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A, TRIGGER_MAX_A,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[TRIGGER_MOTOR].pid_speed = pid_register(&toggle_speed_config);
    sht_controller[TRIGGER_MOTOR].pid_angle = pid_register(&toggle_angle_config);

    /* 注册电机 */
    sht_motor[TRIGGER_MOTOR] = dji_motor_register(&shoot_motor_config[TRIGGER_MOTOR], motor_control_trigger);
    sht_motor[LEFT_FRICTION] = dji_motor_register(&shoot_motor_config[LEFT_FRICTION], motor_control_left);
    sht_motor[MIDDLE_FRICTION] = dji_motor_register(&shoot_motor_config[MIDDLE_FRICTION], motor_control_middle);
    sht_motor[RIGHT_FRICTION] = dji_motor_register(&shoot_motor_config[RIGHT_FRICTION], motor_control_right);
}

/* ----------------------------------------------- 电机控制 ------------------------------------------------------- */
static int16_t motor_control_right(dji_motor_measure_t measure)
{
    return (int16_t)pid_calculate(sht_controller[RIGHT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[RIGHT_FRICTION]);
}

static int16_t motor_control_left(dji_motor_measure_t measure)
{
    return (int16_t)pid_calculate(sht_controller[LEFT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[LEFT_FRICTION]);
}

static int16_t motor_control_middle(dji_motor_measure_t measure)
{
    return (int16_t)pid_calculate(sht_controller[MIDDLE_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[MIDDLE_FRICTION]);
}

static int16_t motor_control_trigger(dji_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;
    static float pid_out_angle;
    static int16_t send_data;

    pid_speed = sht_controller[TRIGGER_MOTOR].pid_speed;
    pid_angle = sht_controller[TRIGGER_MOTOR].pid_angle;
    get_angle = measure.total_angle;
    get_speed = measure.speed_rpm;

    /* 切换模式时清除PID历史 */
    if (shoot_cmd.ctrl_mode != shoot_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    /* 单发/三发/反转模式：双环PID */
    if (shoot_cmd.ctrl_mode == SHOOT_ONE || shoot_cmd.ctrl_mode == SHOOT_THREE || shoot_cmd.ctrl_mode == SHOOT_REVERSE)
    {
        pid_out_angle = (int16_t)pid_calculate(pid_angle, get_angle, shoot_motor_ref[TRIGGER_MOTOR]);
        send_data = (int16_t)pid_calculate(pid_speed, get_speed, pid_out_angle);
    }
    /* 连发/停止模式：只用速度环 */
    else if (shoot_cmd.ctrl_mode == SHOOT_COUNTINUE || shoot_cmd.ctrl_mode == SHOOT_STOP)
    {
        send_data = (int16_t)pid_calculate(pid_speed, get_speed, shoot_motor_ref[TRIGGER_MOTOR]);
    }

    return send_data;
}
