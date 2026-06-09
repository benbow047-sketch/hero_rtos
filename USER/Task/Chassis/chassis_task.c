/**
 * @file chassis_task.c
 * @brief Chassis task
 * @date 2026-06-07
 * @author Jasper
 */

#include "chassis_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
static struct ins_msg ins_data;

static publisher_t *pub_chassis;
static subscriber_t *sub_cmd, *sub_ins;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_sub_pull(void);

/* ------------------------------------------------- 底盘线程入口 ------------------------------------------------------ */
void ChassisTask_Entry(void *argument)
{
    chassis_pub_init();
    chassis_sub_init();

    for(;;)
    {
        chassis_sub_pull();

        // TODO: 底盘运动学解算 + follow PID

        pub_push_msg(pub_chassis, &chassis_fdb);
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("chassis_fdb", sizeof(struct chassis_fdb_msg));
}

static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
    sub_get_msg(sub_ins, &ins_data);
}
