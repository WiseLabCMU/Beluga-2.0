//
// Created by tom on 7/9/24.
//

#include <list_monitor.h>

#include <ble_app.h>
#include <init_main.h>
#include <stdint.h>
#include <thread_priorities.h>
//#include <timestamp.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>

#define ENABLE_NODE_ADD_SEM 0

K_MUTEX_DEFINE(timeout_mutex);

#if ENABLE_NODE_ADD_SEM
K_SEM_DEFINE(node_add_sem, 0, 1);
#define SEM_NODE_TAKE k_sem_take(&node_add_sem, K_FOREVER)
#define SEM_NODE_GIVE k_sem_give(&node_add_sem)
#else
#define SEM_NODE_TAKE                                                          \
    do {                                                                       \
    } while (0)
#define SEM_NODE_GIVE                                                          \
    do {                                                                       \
    } while (0)
#endif

static uint64_t timeout = UINT64_C(9000);
static bool _node_added = false;

void set_node_timeout(uint32_t value) {
    k_mutex_lock(&timeout_mutex, K_FOREVER);
    timeout = value;
    k_mutex_unlock(&timeout_mutex);
}

uint64_t get_node_timeout(void) {
    uint64_t retVal;
    k_mutex_lock(&timeout_mutex, K_FOREVER);
    retVal = timeout;
    k_mutex_unlock(&timeout_mutex);
    return retVal;
}

void node_added(void) {
    SEM_NODE_TAKE;
    _node_added = true;
    SEM_NODE_GIVE;
}

static void reset_node_added(void) {
    SEM_NODE_TAKE;
    _node_added = false;
    SEM_NODE_GIVE;
}

bool check_node_added(void) {
    bool retVal;
    SEM_NODE_TAKE;
    retVal = _node_added;
    SEM_NODE_GIVE;
    return retVal;
}

/**
 * @brief Task to check nodes eviction and re-sorting
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
NO_RETURN void monitor_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    uint32_t count = 0;
    bool removed = false;

    while (1) {
        k_sleep(K_MSEC(1000));

        watchdog_red_rocket();

        k_sem_take(&k_sus_init, K_FOREVER);
        removed = false;
        count += 1;

        // Check for timeout eviction
        for (int x = 0; x < MAX_ANCHOR_COUNT; x++) {
            if (seen_list[x].UUID != 0) {
                if ((k_uptime_get() - seen_list[x].ble_time_stamp) >=
                    get_node_timeout()) {
                    removed = true;
                    memset(&seen_list[x], 0, sizeof(seen_list[0]));
                }
            }
        }

        // Re-sort seen list by RSSI value when a node is removed, added, or a
        // period of time
        if (removed || check_node_added() || ((count % 5) == 0)) {
            disable_bluetooth();

            for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {
                for (int k = j + 1; k < MAX_ANCHOR_COUNT; k++) {
                    if (seen_list[j].RSSI < seen_list[k].RSSI) {
                        node A = seen_list[j];
                        seen_list[j] = seen_list[k];
                        seen_list[k] = A;
                    }
                }
            }

            // Resume scanning/building up neighbor list
            reset_node_added();
            removed = false;
            count = 0;
            enable_bluetooth();
        }

        k_sem_give(&k_sus_init);
    }
}

#if ENABLE_THREADS && ENABLE_MONITOR
K_THREAD_STACK_DEFINE(monitor_stack, CONFIG_MONITOR_STACK_SIZE);
static struct k_thread monitor_data;
static k_tid_t monitor_task_id;

void init_monitor_thread(void) {
    monitor_task_id = k_thread_create(&monitor_data, monitor_stack,
                                      K_THREAD_STACK_SIZEOF(monitor_stack),
                                      monitor_task_function, NULL, NULL, NULL,
                                      CONFIG_BELUGA_MONITOR_PRIO, 0, K_NO_WAIT);
    printk("Started monitor\n");
}
#else
void init_monitor_thread(void) { printk("Monitor disabled\n"); }
#endif
