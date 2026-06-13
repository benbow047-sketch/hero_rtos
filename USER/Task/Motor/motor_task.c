/**
 * @file motor_task.c
 * @brief Motor control task
 * @date 2026-06-07
 * @author Jasper
 */

#include "motor_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

static float motor_dt;
uint32_t tec, rec;

void MotorTask_Entry(void *argument)
{
    static float motor_start;

    bsp_can_init();

    for(;;)
    {
        motor_start = dwt_get_time_ms();

        tec = (hfdcan1.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
        rec = (hfdcan1.Instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;

#ifdef BSP_USING_DJI_MOTOR
        dji_motor_control();
#endif

        motor_dt = dwt_get_time_ms() - motor_start;
        if (motor_dt > 3)
            printf("Motor Task is being DELAY! dt = [%f]\n", motor_dt);

        osDelay(1);
    }
}
