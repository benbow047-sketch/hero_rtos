/**
 * @file gimbal_task.c
 * @brief Gimbal task
 * @date 2026-06-07
 * @author Jasper
 */

#include "gimbal_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct gimbal_fdb_msg gim_fdb;
static struct ins_msg ins_data;

static publisher_t *pub_gim;
static subscriber_t *sub_cmd, *sub_ins;

static void gimbal_pub_init(void);
static void gimbal_sub_init(void);
static void gimbal_pub_push(void);
static void gimbal_sub_pull(void);

/* ------------------------------------------------- 云台线程入口 ------------------------------------------------------ */
void GimbalTask_Entry(void *argument)
{
    gimbal_pub_init();
    gimbal_sub_init();

    for(;;)
    {
        gimbal_sub_pull();

        // TODO: 云台PID控制

        gimbal_pub_push();
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

static void gimbal_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
}

static void gimbal_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_fdb);
}
