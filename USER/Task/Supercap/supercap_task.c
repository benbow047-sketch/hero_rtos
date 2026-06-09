/**
 * @file supercap_task.c
 * @brief Super capacitor task
 * @date 2026-06-07
 * @author Jasper
 */

#include "supercap_task.h"
#include "rm_task.h"
#include "rm_module.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
// static struct supercap_fdb_msg cap_fdb;
// static publisher_t *pub_cap;
// static void supercap_pub_init(void);

/* ------------------------------------------------- 超电线程入口 ------------------------------------------------------ */
void Supercap_Entry(void *argument)
{
    // supercap_pub_init();

    for(;;)
    {
        // TODO: 超级电容CAN通信

        // pub_push_msg(pub_cap, &cap_fdb);
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
// static void supercap_pub_init(void)
// {
//     pub_cap = pub_register("cap_fdb", sizeof(struct supercap_fdb_msg));
// }
