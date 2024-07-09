//
// Created by tom on 7/9/24.
//

#include <ble_app.h>
#include <list_monitor.h>
#include <stdint.h>
#include <timestamp.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <thread_priorities.h>
#include <utils.h>

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
void monitor_task_function(void) {
    uint32_t count = 0;
    bool removed = false;

    while (1) {
        k_sleep(K_MSEC(1000));

        watchdog_red_rocket();

        // TODO: xSemaphoreTake(sus_init, portMAX_DELAY);
        removed = false;
        count += 1;

        // Check for timeout eviction
        for (int x = 0; x < MAX_ANCHOR_COUNT; x++) {
            if (seen_list[x].UUID != 0) {
                if ((get_timestamp() - seen_list[x].ble_time_stamp) >=
                    get_node_timeout()) {
                    removed = true;
                    memset(&seen_list[x], 0, sizeof(seen_list[0]));
                }
            }
        }

        // Re-sort seen list by RSSI value when a node is removed, added, or a
        // period of time
        if (removed || check_node_added() || ((count % 5) == 0)) {
            ble_disable_scan();

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
            ble_enable_scan();
        }

        // TODO: xSemaphoreGive(sus_init);
    }
}

#if ENABLE_THREADS
K_THREAD_DEFINE(monitor_list_task_id, STACK_SIZE, monitor_task_function, NULL, NULL,
                NULL, MONITOR_PRIO, 0, 0);
#endif
