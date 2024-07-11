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
void responder_task_function(void) {

    //    if (leds_mode == 0)
    //        dwt_setleds(DWT_LEDS_ENABLE);
    //    if (leds_mode == 1)
    //        dwt_setleds(DWT_LEDS_DISABLE);

    while (true) {
        watchdog_red_rocket();

        // Check if responding is suspended, return 0 means suspended
        unsigned int suspend_start = k_sem_count_get(&k_sus_resp);

        if (suspend_start == 0) {
            if (get_twr_mode()) {
                ds_resp_run();
            } else {
                ss_resp_run();
            }
        }
    }
}

#if ENABLE_THREADS && ENABLE_RESPONDER
K_THREAD_DEFINE(responder_task_id, STACK_SIZE, responder_task_function, NULL,
                NULL, NULL, RESPONDER_PRIO, 0, 0);
#endif
