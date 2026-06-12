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

/* ----------------------------------------------- 遥控器测试变量 --------------------------------------------------- */
volatile int16_t rc_ch1 = 0, rc_ch2 = 0, rc_ch3 = 0, rc_ch4 = 0;
volatile uint8_t rc_sw1 = 0, rc_sw2 = 0;
volatile int16_t rc_mouse_x = 0, rc_mouse_y = 0;
volatile uint8_t rc_mouse_l = 0, rc_mouse_r = 0;

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
    dbus_data_init();
    cmd_pub_init();
    cmd_sub_init();

    for(;;)
    {
        cmd_sub_pull();

        // 读取遥控器数据（调试用）
        rc_ch1 = dbus_data_fdb.ch1;
        rc_ch2 = dbus_data_fdb.ch2;
        rc_ch3 = dbus_data_fdb.ch3;
        rc_ch4 = dbus_data_fdb.ch4;
        rc_sw1 = dbus_data_fdb.sw1;
        rc_sw2 = dbus_data_fdb.sw2;
        rc_mouse_x = dbus_data_fdb.mouse.x;
        rc_mouse_y = dbus_data_fdb.mouse.y;
        rc_mouse_l = dbus_data_fdb.mouse.l;
        rc_mouse_r = dbus_data_fdb.mouse.r;

        // 测试：sw2==RC_MI 时底盘前进
        if (dbus_data_fdb.sw2 == RC_MI)
        {
            chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
            chassis_cmd.vx = 300;
        }
        else
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
            chassis_cmd.vx = 0;
        }

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
