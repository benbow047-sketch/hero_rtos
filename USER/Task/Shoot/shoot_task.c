/**
 * @file shoot_task.c
 * @brief Shoot task
 * @date 2026-06-07
 * @author Jasper
 */

#include "shoot_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;

static publisher_t *pub_shoot;
static subscriber_t *sub_cmd;

static void shoot_pub_init(void);
static void shoot_sub_init(void);
static void shoot_sub_pull(void);

/* ------------------------------------------------- 发射线程入口 ------------------------------------------------------ */
void ShootTask_Entry(void *argument)
{
    shoot_pub_init();
    shoot_sub_init();

    for(;;)
    {
        shoot_sub_pull();

        // TODO: 摩擦轮 + 拨弹控制

        pub_push_msg(pub_shoot, &shoot_fdb);
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void shoot_pub_init(void)
{
    pub_shoot = pub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
}

static void shoot_sub_init(void)
{
    sub_cmd = sub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

static void shoot_sub_pull(void)
{
    sub_get_msg(sub_cmd, &shoot_cmd);
}
