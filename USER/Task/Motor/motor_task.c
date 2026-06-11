/**
 * @file motor_task.c
 * @brief Motor control task - 测试单个电机
 * @date 2026-06-07
 * @author Jasper
 */

#include "motor_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ---- 测试用：单个底盘电机 ---- */
static pid_obj_t *test_speed_pid;
static dji_motor_object_t *test_motor;
static int16_t test_motor_ref = 500;  // 测试转速（RPM）

static int16_t test_motor_control(dji_motor_measure_t measure)
{
    return pid_calculate(test_speed_pid, measure.speed_rpm, test_motor_ref);
}

static float motor_dt;

void MotorTask_Entry(void *argument)
{
    static float motor_start;

    // 注册测试电机（底盘左前轮，CAN1，rx_id=0x201）
    pid_config_t speed_config = INIT_PID_CONFIG(
        3.0f, 0.0f, 0.07f, 0, 16384,
        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    test_speed_pid = pid_register(&speed_config);

    motor_config_t motor_config = {
        .motor_type = M3508,
        .can_name = "hfdcan1",
        .rx_id = 0x201,
        .tx_id = 0x201,
    };
    test_motor = dji_motor_register(&motor_config, test_motor_control);

    for(;;)
    {
        motor_start = dwt_get_time_ms();

#ifdef BSP_USING_DJI_MOTOR
        dji_motor_control();
#endif

        motor_dt = dwt_get_time_ms() - motor_start;
        osDelay(1);
    }
}
