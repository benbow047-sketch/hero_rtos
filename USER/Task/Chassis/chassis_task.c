/**
 * @file chassis_task.c
 * @brief Chassis task
 * @date 2026-06-07
 * @author Jasper
 */

#include <string.h>
#include "rm_config.h"
#include "rm_task.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
static struct ins_msg ins_data;

static publisher_t *pub_chassis;
static subscriber_t *sub_cmd, *sub_ins;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* ------------------------------------------------- 电机控制相关 ------------------------------------------------------ */
static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
} chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];
static int16_t motor_ref[4];

/* ------------------------------------------------ 底盘运动学解算 ----------------------------------------------------- */
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t *out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t *out_speed) = mecanum_calc;

/* ---------------------------------------------- 其余变量与函数声明 ---------------------------------------------------- */
static void chassis_motor_init(void);
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
static float chassis_dt;

/* ------------------------------------------------- 底盘线程入口 ------------------------------------------------------ */
void ChassisTask_Entry(void *argument)
{
    static float chassis_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
    bsp_can_init();

    for(;;)
    {
        chassis_start = dwt_get_time_ms();

        chassis_sub_pull();

        for (uint8_t i = 0; i < 4; i++)
            dji_motor_enable(chassis_motor[i]);

        switch (chassis_cmd.ctrl_mode)
        {
            case CHASSIS_RELAX:
                for (uint8_t i = 0; i < 4; i++)
                    dji_motor_relax(chassis_motor[i]);
                break;
            case CHASSIS_FOLLOW_GIMBAL:
                absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                break;
            case CHASSIS_SPIN:
                absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                break;
            case CHASSIS_OPEN_LOOP:
                chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                break;
            case CHASSIS_STOP:
                memset(motor_ref, 0, sizeof(motor_ref));
                break;
            default:
                for (uint8_t i = 0; i < 4; i++)
                    dji_motor_relax(chassis_motor[i]);
                break;
        }

        chassis_pub_push();

        chassis_dt = dwt_get_time_ms() - chassis_start;
        if (chassis_dt > 3)
            printf("Chassis Task is being DELAY! dt = [%f]\n", chassis_dt);

        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("chassis_fdb", sizeof(struct chassis_fdb_msg));
}

static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

static void chassis_pub_push(void)
{
    pub_push_msg(pub_chassis, &chassis_fdb);
}

static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
    sub_get_msg(sub_ins, &ins_data);
}

/* ----------------------------------------------- 电机控制相关 ------------------------------------------------------- */
static int16_t motor_control_0(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_ref[0]);
    return set;
}

static int16_t motor_control_1(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_ref[1]);
    return set;
}

static int16_t motor_control_2(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_ref[2]);
    return set;
}

static int16_t motor_control_3(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_ref[3]);
    return set;
}

static void *motor_control[4] = {
    motor_control_0,
    motor_control_1,
    motor_control_2,
    motor_control_3,
};

/* 底盘电机配置 */
static motor_config_t chassis_motor_config[4] = {
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x201,
        .tx_id = 0x201,
        .controller = &chassis_controller[0],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x202,
        .tx_id = 0x202,
        .controller = &chassis_controller[1],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x203,
        .tx_id = 0x203,
        .controller = &chassis_controller[2],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x204,
        .tx_id = 0x204,
        .controller = &chassis_controller[3],
    },
};

/* ----------------------------------------------- 底盘运动学解算 ----------------------------------------------------- */
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t *out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);

    wheel_rpm[0] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[1] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[2] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[3] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

    chassis_fdb.vw_ch = (chassis_motor[0]->measure.speed_rpm + chassis_motor[1]->measure.speed_rpm
                         + chassis_motor[2]->measure.speed_rpm + chassis_motor[3]->measure.speed_rpm)
                        / (4.0f * wheel_rpm_ratio * LENGTH_RADIUS);

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}

/* ----------------------------------------------- 电机初始化 ------------------------------------------------------- */
static void chassis_motor_init(void)
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(
        CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR,
        CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);
    }
}

/* ----------------------------------------------- 坐标变换 ------------------------------------------------------- */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle)
{
    float angle_hd = -(angle / RADIAN_COEF);
    float vx = cmd->vx;
    float vy = cmd->vy;

    cmd->vx = vx * cos(angle_hd) - vy * sin(angle_hd);
    cmd->vy = vx * sin(angle_hd) + vy * cos(angle_hd);
}
