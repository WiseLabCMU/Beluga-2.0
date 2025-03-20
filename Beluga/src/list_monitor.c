/**
 * @file list_monitor.c
 *
 * @brief List monitor module for neighbor node management.
 *
 * This module is responsible for monitoring and managing a list of neighboring
 * devices. It handles evicting timed out neighbors, sorting the list by RSSI,
 * and ensuring the list is up-to-date by periodically checking the nodes.
 *
 * @date 7/9/2024
 *
 * @author WiSeLab CMU
 * @author Tom Schmitz
 */

#include <beluga_message.h>
#include <ble_app.h>
#include <initiator.h>
#include <list_monitor.h>
#include <list_neighbors.h>
#include <serial/comms.h>
#include <stdint.h>
#include <stdio.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the list monitor
 */
LOG_MODULE_REGISTER(list_monitor, CONFIG_LIST_MONITOR_LOG_LEVEL);

/**
 * The number of cycles that are needed before the list gets sorted if there
 * have been no neighbor evictions or additions
 */
#define LIST_SORT_PERIOD_S 5

/**
 * The period of the list monitor task
 */
#define LIST_MONITOR_PERIOD_MS 1000

/**
 * Routine to suspend neighbor scanning
 */
#define SUSPEND_NEIGHBOR_SCANNING() disable_bluetooth()

/**
 * Routine to resume neighbor scanning
 */
#define RESUME_NEIGHBOR_SCANNING() enable_bluetooth()

/**
 * Number of milliseconds allowed to elapse before evicting a neighbor
 */
static uint64_t timeout = UINT64_C(9000);

/**
 * Flag indicating if there was a neighbor addition
 */
static bool _node_added = false;

/**
 * @brief Set the timeout value for node eviction.
 *
 * @param value The new timeout value in milliseconds.
 */
void set_node_timeout(uint32_t value) { timeout = value; }

/**
 * @brief Get the current timeout value for node eviction.
 *
 * @return The current timeout value in milliseconds.
 */
uint64_t get_node_timeout(void) {
    uint64_t retVal;
    retVal = timeout;
    return retVal;
}

/**
 * @brief Mark that a new node has been added.
 */
void node_added(void) { _node_added = true; }

/**
 * @brief Resets the flag indicating that a new node has been added
 */
static void reset_node_added(void) { _node_added = false; }

/**
 * @brief Checks if a node has been added to the list
 * @return `true` if a node has been added
 * @return `false` if no nodes have been added
 */
bool check_node_added(void) {
    bool retVal;
    retVal = _node_added;
    return retVal;
}

/**
 * @brief Sends an eviction message
 * @param[in] comms Pointer to the comms instance
 * @param[in] uuid The node that got evicted
 */
static void evict_node(const struct comms *comms, uint16_t uuid) {
    struct beluga_msg msg = {.type = NEIGHBOR_DROP,
                             .payload.dropped_neighbor = uuid};
    comms_write_msg(comms, &msg);
}

/**
 * @brief Evict expired neighbor nodes from the list.
 *
 * Checks each neighbor in the list and removes any neighbors that have not been
 * scanned by BLE recently.
 *
 * @return `true` if any nodes were removed
 * @return `false` otherwise
 */
static bool evict_nodes(const struct comms *comms) {
    bool removed = false;

    for (size_t x = 0; x < MAX_ANCHOR_COUNT; x++) {
        if (seen_list[x].UUID != 0) {
            if ((k_uptime_get() - seen_list[x].ble_time_stamp) >= timeout) {
                LOG_INF("Removing node %" PRId16, seen_list[x].UUID);
                evict_node(comms, seen_list[x].UUID);
                removed = true;
                seen_list[x].UUID = 0;
            }
        }
    }

    return removed;
}

/**
 * @brief Sorts the neighbors list by RSSI value in descending order.
 */
static void sort_nodes(void) {
    SUSPEND_NEIGHBOR_SCANNING();

    for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {
        for (int k = j + 1; k < MAX_ANCHOR_COUNT; k++) {
            if (seen_list[j].RSSI < seen_list[k].RSSI) {
                struct node A = seen_list[j];
                seen_list[j] = seen_list[k];
                seen_list[k] = A;
            }
        }
    }

    reset_node_added();
    RESUME_NEIGHBOR_SCANNING();
}

/**
 * Message queue for nodes that have been evicted during the scanning routine
 */
K_MSGQ_DEFINE(evicted_nodes, sizeof(uint16_t), 2 * MAX_ANCHOR_COUNT, 1);

/**
 * Retrieves items from the evicted nodes message queue and indicates that they
 * have been evicted
 * @param[in] comms Pointer to the comms instance
 * @return `true` if any nodes were evicted by the scan event
 * @return `false` otherwise
 */
static bool notify_msgq_nodes(const struct comms *comms) {
    uint16_t uuid;
    int err = k_msgq_get(&evicted_nodes, &uuid, K_NO_WAIT);
    bool ret = false;

    while (err == 0) {
        // Check if node got re-inserted
        if (!in_seen_list(uuid)) {
            ret = true;
            evict_node(comms, uuid);
        }
        err = k_msgq_get(&evicted_nodes, &uuid, K_NO_WAIT);
    }
    return ret;
}

/**
 * @brief Task that maintains the neighbor list.
 *
 * Task that checks for neighbors that have timed out and re-sorts the neighbors
 * list.
 *
 * This is the main function for the monitor task that runs periodically to
 * check for expired neighbors, re-sort the list, and ensure the system is
 * running smoothly. It uses a watchdog to ensure the task operates correctly
 * and is executed periodically.
 *
 * @param p1 Unused parameter.
 * @param p2 Unused parameter.
 * @param p3 Unused parameter.
 */
NO_RETURN static void monitor_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    uint32_t count = 0;
    bool removed;
    struct task_wdt_attr watchdogAttr = {.period = 3000};
    const struct comms *comms = comms_backend_uart_get_ptr();

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        LOG_ERR("Unable to spawn watchdog for monitor thread.\n");
        while (1)
            ;
    }

    while (1) {
        k_sleep(K_MSEC(LIST_MONITOR_PERIOD_MS));

        watchdog_red_rocket(&watchdogAttr);

        removed = notify_msgq_nodes(comms);

        if (k_sem_take(&k_sus_init, K_NO_WAIT) < 0) {
            continue;
        }
        BOUND_INCREMENT(count, LIST_SORT_PERIOD_S);

        removed = evict_nodes(comms) || removed;

        if (removed || check_node_added() || count == 0) {
            LOG_INF("Sorting list");
            sort_nodes();
        }

        k_sem_give(&k_sus_init);
    }
}

#if defined(CONFIG_ENABLE_BELUGA_THREADS) && defined(CONFIG_ENABLE_MONITOR)
/**
 * List monitor stack allocation
 */
K_THREAD_STACK_DEFINE(monitor_stack, CONFIG_MONITOR_STACK_SIZE);

/**
 * Thread data for the list monitor
 */
static struct k_thread monitor_data;

/**
 * List monitor thread ID
 */
static k_tid_t monitor_task_id;

/**
 * @brief Initializes and starts the list monitor thread
 */
void init_monitor_thread(void) {
    monitor_task_id = k_thread_create(&monitor_data, monitor_stack,
                                      K_THREAD_STACK_SIZEOF(monitor_stack),
                                      monitor_task_function, NULL, NULL, NULL,
                                      CONFIG_BELUGA_MONITOR_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(monitor_task_id, "Neighbors monitor");
    LOG_INF("Started monitor");
}
#else
/**
 * @brief Placeholder for the initialization function for when the list monitor
 * is disabled.
 */
void init_monitor_thread(void) { LOG_INF("Monitor disabled"); }
#endif // defined(CONFIG_ENABLE_BELUGA_THREADS) &&
       // defined(CONFIG_ENABLE_MONITOR)
