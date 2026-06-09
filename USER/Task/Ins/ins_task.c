/**
 * @file ins_task.c
 * @brief INS (Inertial Navigation System) task
 * @date 2026-06-07
 * @author Jasper
 */

#include "ins_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct ins_msg ins_data;

static publisher_t *ins_pub;

static void ins_pub_init(void);

/* ------------------------------------------------- ins线程入口 ------------------------------------------------------ */
void InsTask_Entry(void *argument)
{
    ins_pub_init();

    for(;;)
    {
        // TODO: 读取BMI088, 运行EKF, 更新ins_data

        pub_push_msg(ins_pub, &ins_data);
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void ins_pub_init(void)
{
    ins_pub = pub_register("ins_msg", sizeof(struct ins_msg));
}
