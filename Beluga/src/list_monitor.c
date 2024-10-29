//
// Created by tom on 7/9/24.
//

#include <ble_app.h>
#include <init_main.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <stdint.h>
#include <stdio.h>
#include <thread_priorities.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(list_monitor, CONFIG_LIST_MONITOR_LOG_LEVEL);

#define LIST_SORT_PERIOD_S     5
#define LIST_MONITOR_PERIOD_MS 1000

#define ENABLE_NODE_ADD_SEM    0

K_MUTEX_DEFINE(timeout_mutex);

#if ENABLE_NODE_ADD_SEM
K_SEM_DEFINE(node_add_sem, 0, 1);
#define SEM_NODE_TAKE k_sem_take(&node_add_sem, K_FOREVER)
#define SEM_NODE_GIVE k_sem_give(&node_add_sem)
#else
#define SEM_NODE_TAKE (void)0
#define SEM_NODE_GIVE (void)0
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
    struct task_wdt_attr watchdogAttr = {.period = 3000};

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        LOG_ERR("Unable to spawn watchdog for monitor thread.\n");
        while (1)
            ;
    }

    while (1) {
        k_msleep(LIST_MONITOR_PERIOD_MS);

        watchdog_red_rocket(&watchdogAttr);

        if (k_sem_take(&k_sus_init, K_NO_WAIT) < 0) {
            continue;
        }
        removed = false;
        count += 1;

        // Check for timeout eviction
        for (int x = 0; x < MAX_ANCHOR_COUNT; x++) {
            if (seen_list[x].UUID != 0) {
                if ((k_uptime_get() - seen_list[x].ble_time_stamp) >= timeout) {
                    LOG_INF("Removing node %" PRId16, seen_list[x].UUID);
                    if (get_format_mode()) {
                        printf("rm %" PRId16 "\r\n", seen_list[x].UUID);
                    }
                    removed = true;
                    memset(&seen_list[x], 0, sizeof(seen_list[0]));
                }
            }
        }

        // Re-sort seen list by RSSI value when a node is removed, added, or a
        // period of time
        if (removed || check_node_added() ||
            ((count % LIST_SORT_PERIOD_S) == 0)) {
            LOG_INF("Sorting list");
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
    k_thread_name_set(monitor_task_id, "Neighbors monitor");
    LOG_INF("Started monitor");
}
#else
void init_monitor_thread(void) { LOG_INF("Monitor disabled"); }
#endif
