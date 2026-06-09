/**
 * @file transmission_task.c
 * @brief Transmission (USB) task
 * @date 2026-06-07
 * @author Jasper
 */

#include "transmission_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct trans_fdb_msg trans_fdb;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;

static publisher_t *pub_trans;
static subscriber_t *sub_ins, *sub_gim;

static void trans_pub_init(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);

/* ------------------------------------------------- transmission线程入口 ------------------------------------------------------ */
void TransmissionTask_Entry(void *argument)
{
    trans_pub_init();
    trans_sub_init();

    // TODO: MX_USB_DEVICE_Init();

    for(;;)
    {
        trans_sub_pull();

        // TODO: USB自瞄通信

        pub_push_msg(pub_trans, &trans_fdb);
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void trans_pub_init(void)
{
    pub_trans = pub_register("trans_fdb", sizeof(struct trans_fdb_msg));
}

static void trans_sub_init(void)
{
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
}

static void trans_sub_pull(void)
{
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_gim, &gim_fdb);
}
