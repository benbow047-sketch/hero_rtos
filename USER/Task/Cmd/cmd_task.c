/**
 * @file cmd_task.c
 * @brief Command task
 * @date 2026-06-07
 * @author Jasper
 */

#include "cmd_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct chassis_cmd_msg chassis_cmd;
static struct shoot_cmd_msg shoot_cmd;

static struct gimbal_fdb_msg gim_fdb;
static struct chassis_fdb_msg chassis_fdb;
static struct shoot_fdb_msg shoot_fdb;
static struct trans_fdb_msg trans_fdb;
static struct ins_msg ins_data;

static publisher_t *pub_gim, *pub_chassis, *pub_shoot;
static subscriber_t *sub_gim, *sub_shoot, *sub_trans, *sub_ins, *sub_chassis;

static void cmd_pub_init(void);
static void cmd_sub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_pull(void);

/* ------------------------------------------------- cmd线程入口 ------------------------------------------------------ */
void CmdTask_Entry(void *argument)
{
    cmd_pub_init();
    cmd_sub_init();

    for(;;)
    {
        cmd_sub_pull();

        // TODO: 遥控器状态机处理

        cmd_pub_push();
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void cmd_pub_init(void)
{
    pub_gim = pub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot = pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
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

static void cmd_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_cmd);
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
}
