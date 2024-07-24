//
// Created by tom on 7/9/24.
//

#include <ranging.h>
#include <resp_main.h>
#include <thread_priorities.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>

/**
 * @brief SS TWR Initiator task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
NO_RETURN static void responder_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_msleep(20);
        watchdog_red_rocket();

        // Check if responding is suspended, return 0 means suspended
        unsigned int suspend_start = k_sem_count_get(&k_sus_resp);

        if (suspend_start != 0) {
            if (get_twr_mode()) {
                ds_resp_run();
            } else {
                ss_resp_run();
            }
        }
    }
}

#if ENABLE_THREADS && ENABLE_RESPONDER
K_THREAD_STACK_DEFINE(responder_stack, CONFIG_RESPONDER_STACK_SIZE);
static struct k_thread responder_data;
static k_tid_t responder_task_id;

void init_responder_thread(void) {
    responder_task_id = k_thread_create(
        &responder_data, responder_stack,
        K_THREAD_STACK_SIZEOF(responder_stack), responder_task_function, NULL,
        NULL, NULL, CONFIG_BELUGA_RESPONDER_PRIO, 0, K_NO_WAIT);
    printk("Started responder\n");
}
#else
void init_responder_thread(void) {}
#endif
