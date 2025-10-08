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
#include <ble/ble_app.h>
#include <ble/scan.h>
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
#define SUSPEND_NEIGHBOR_SCANNING() suspend_scanning()

/**
 * Routine to resume neighbor scanning
 */
#define RESUME_NEIGHBOR_SCANNING() resume_scanning()

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

#if defined(CONFIG_ENABLE_MONITOR)
K_THREAD_DEFINE(monitor_task, CONFIG_MONITOR_STACK_SIZE, monitor_task_function,
                NULL, NULL, NULL, CONFIG_BELUGA_MONITOR_PRIO, 0, -1);

/**
 * @brief Initializes and starts the list monitor thread
 */
void init_monitor_thread(void) {
    k_thread_name_set(monitor_task, "Neighbors monitor");
    k_thread_start(monitor_task);
    LOG_INF("Started monitor");
}
#else
/**
 * @brief Placeholder for the initialization function for when the list monitor
 * is disabled.
 */
void init_monitor_thread(void) { LOG_INF("Monitor disabled"); }
#endif // defined(CONFIG_ENABLE_MONITOR)

/**
 * Fetches the neighbor list index where a neighbor node data is stored
 * @param[in] uuid The neighbor node ID
 * @return The index to the neighbor node
 * @return -1 if the neighbor is not present in the list
 */
static ssize_t get_seen_list_index(uint16_t uuid) {
    for (ssize_t i = 0; i < (ssize_t)MAX_ANCHOR_COUNT; i++) {
        if (seen_list[i].UUID == uuid) {
            return i;
        }
    }
    return -1;
}

/**
 * Checks if specified neighbor is in the neighbor list
 * @param[in] uuid The neighbor node ID
 * @return true if in neighbor list
 * @return false if not in neighbor list
 */
bool in_seen_list(uint16_t uuid) { return get_seen_list_index(uuid) != -1; }

/**
 * Marks a node as updated when evaluating BLE signal strength
 */
#define EVAL_STRENGTH()                                                        \
    COND_CODE_1(IS_ENABLED(CONFIG_BELUGA_EVAL_BLE_STRENGTH),                   \
                (seen_list[index].update_flag = true), ((void)0))

/**
 * Generic code for inserting a node into the neighbor list
 *
 * @param[in] evict_call A variable or function return value to set index to
 * when evicting a node
 */
#define INSERT_NODE(evict_call, _poll_flag)                                    \
    do {                                                                       \
        ssize_t index = get_seen_list_index(0);                                \
        if (index < 0) {                                                       \
            index = evict_call;                                                \
            if (index < 0) {                                                   \
                return;                                                        \
            }                                                                  \
            (void)k_msgq_put(&evicted_nodes, &seen_list[index].UUID,           \
                             K_NO_WAIT);                                       \
        }                                                                      \
        seen_list[index].UUID = data->uuid;                                    \
        seen_list[index].RSSI = rssi;                                          \
        EVAL_STRENGTH();                                                       \
        seen_list[index].ble_time_stamp = k_uptime_get();                      \
        seen_list[index].polling_flag = _poll_flag;                            \
        node_added();                                                          \
    } while (0)

#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define _STATIC static
#else
#define _STATIC
#endif

#if (IS_ENABLED(CONFIG_BELUGA_EVICT_RR) ||                                     \
     IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT))
#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define RR_FUNC_NAME insert_seen_list_rr
#else
#define RR_FUNC_NAME insert_into_seen_list
#endif

/**
 * Inserts a new neighbor into the neighbor list using the index round-robin
 * method
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
_STATIC void RR_FUNC_NAME(struct ble_data *data, int8_t rssi, bool polling) {
    static ssize_t last_seen_index = 0;
    INSERT_NODE(last_seen_index, polling);
    BOUND_INCREMENT(last_seen_index, MAX_ANCHOR_COUNT);
}
#endif

#if (IS_ENABLED(CONFIG_BELUGA_EVICT_RSSI) ||                                   \
     IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT))
#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define RSSI_FUNC_NAME insert_seen_list_rssi
#else
#define RSSI_FUNC_NAME insert_into_seen_list
#endif

/**
 * Searches the neighbor list for the neighbor with the smallest RSSI
 * @param[in] rssi The RSSI of the scanned node
 * @return index of the smallest RSSI
 * @return -1 if all the RSSI values are larger than the input RSSI value
 */
static ssize_t find_smallest_rssi(int8_t rssi) {
    ssize_t evict_index = -1;
    int8_t lowest_rssi = rssi;

    for (ssize_t index = MAX_ANCHOR_COUNT - 1; index >= 0; index--) {
        if (seen_list[index].RSSI < lowest_rssi) {
            lowest_rssi = seen_list[index].RSSI;
            evict_index = index;
        }
    }

    return evict_index;
}

/**
 * Inserts a new neighbor into the neighbor list and evicts based on RSSI
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
_STATIC void RSSI_FUNC_NAME(struct ble_data *data, int8_t rssi, bool polling) {
    INSERT_NODE(find_smallest_rssi(rssi), polling);
}
#endif

#if (IS_ENABLED(CONFIG_BELUGA_EVICT_RANGE) ||                                  \
     IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT))

#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define RANGE_FUNC_NAME insert_seen_list_range
#else
#define RANGE_FUNC_NAME insert_into_seen_list
#endif

#include <math.h>

/**
 * Finds the node with the longest range
 * @return index of the node with the longest range
 */
static ssize_t find_largest_range(void) {
    ssize_t evict_index = -1;
    float largest_range = -1.0f;

    for (ssize_t index = 0; index < MAX_ANCHOR_COUNT; index++) {
        if (isgreater(seen_list[index].range, largest_range)) {
            largest_range = seen_list[index].range;
            evict_index = index;
        }
    }

    return evict_index;
}

/**
 * Inserts a new neighbor into the neighbor list and evicts based on range
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
_STATIC void RANGE_FUNC_NAME(struct ble_data *data, int8_t rssi, bool polling) {
    INSERT_NODE(find_largest_range(), polling);
}
#endif

#if (IS_ENABLED(CONFIG_BELUGA_EVICT_BLE_TS) ||                                 \
     IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT))
#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define BLE_TS_FUNC_NAME insert_seen_list_ble_ts
#else
#define BLE_TS_FUNC_NAME insert_into_seen_list
#endif

/**
 * Finds the least recently scanned node
 * @return index of the least recently scanned node
 */
static ssize_t find_oldest_ble_ts(void) {
    ssize_t evict_index = -1;
    int64_t timestamp = k_uptime_get();

    for (ssize_t index = 0; index < MAX_ANCHOR_COUNT; index++) {
        if (seen_list[index].ble_time_stamp < timestamp) {
            timestamp = seen_list[index].ble_time_stamp;
            evict_index = index;
        }
    }

    return evict_index;
}

/**
 * Inserts a new neighbor into the neighbor list and evicts based on BLE
 * timestamp
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
_STATIC void BLE_TS_FUNC_NAME(struct ble_data *data, int8_t rssi,
                              bool polling) {
    INSERT_NODE(find_oldest_ble_ts(), polling);
}
#endif

#if (IS_ENABLED(CONFIG_BELUGA_EVICT_RANGE_TS) ||                               \
     IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT))
#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
#define RANGE_TS_FUNC_NAME insert_seen_list_range_ts
#else
#define RANGE_TS_FUNC_NAME insert_into_seen_list
#endif

/**
 * Finds the node that has the oldest range value
 * @return index of the node with the oldest range value
 */
static ssize_t find_oldest_range_ts(void) {
    ssize_t evict_index = -1;
    int64_t timestamp = k_uptime_get();

    for (ssize_t index = 0; index < MAX_ANCHOR_COUNT; index++) {
        if (seen_list[index].time_stamp < timestamp) {
            timestamp = seen_list[index].time_stamp;
            evict_index = index;
        }
    }

    return evict_index;
}

/**
 * Inserts a new neighbor into the neighbor list and evicts based on range
 * timestamp
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
_STATIC void RANGE_TS_FUNC_NAME(struct ble_data *data, int8_t rssi,
                                bool polling) {
    INSERT_NODE(find_oldest_range_ts(), polling);
}
#endif

#if IS_ENABLED(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)

/**
 * The current node eviction policy
 */
static enum node_eviction_policy policy = EVICT_POLICY_RR;

/**
 * Inserts a new neighbor into the neighbor list and evicts based on the current
 * eviction policy
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
void insert_into_seen_list(struct ble_data *data, int8_t rssi, bool polling) {
    switch (policy) {
    case EVICT_POLICY_RR:
        insert_seen_list_rr(data, rssi, polling);
        break;
    case EVICT_POLICY_RSSI:
        insert_seen_list_rssi(data, rssi, polling);
        break;
    case EVICT_POLICY_RANGE:
        insert_seen_list_range(data, rssi, polling);
        break;
    case EVICT_POLICY_BLE_TS:
        insert_seen_list_ble_ts(data, rssi, polling);
        break;
    case EVICT_POLICY_RANGE_TS:
        insert_seen_list_range_ts(data, rssi, polling);
        break;
    default:
        break;
    }
}

/**
 * Updates the eviction policy
 * @param[in] policy The new policy
 */
void set_node_eviction_policy(enum node_eviction_policy new_policy) {
    if (new_policy >= EVICT_POLICY_INVALID) {
        return;
    }
    policy = new_policy;
}

/**
 * Prints the eviction policy in human readable text
 * @param[in] comms Pointer to the comms instance
 * @return 0 upon success
 * @return -EINVAL if input parameters are invalid
 * @return -EFAULT if the current eviction policy is unknown
 * @return -ENOTSUP if disabled
 * @return negative error code otherwise
 */
int print_eviction_scheme(const struct comms *comms) {
    struct beluga_msg msg = {.type = START_EVENT};
    int ret = 0;

    if (comms == NULL || comms->ctx == NULL) {
        return -EINVAL;
    }

    switch (policy) {
    case EVICT_POLICY_RR: {
        msg.payload.node_version = "  Eviction Scheme: Index Round Robin";
        break;
    }
    case EVICT_POLICY_RSSI: {
        msg.payload.node_version = "  Eviction Scheme: Lowest RSSI";
        break;
    }
    case EVICT_POLICY_RANGE: {
        msg.payload.node_version = "  Eviction Scheme: Longest Range";
        break;
    }
    case EVICT_POLICY_BLE_TS: {
        msg.payload.node_version =
            "  Eviction Scheme: Least Recent BLE Timestamp";
        break;
    }
    case EVICT_POLICY_RANGE_TS: {
        msg.payload.node_version =
            "  Eviction Scheme: Least Recent Range Timestamp";
        break;
    }
    default: {
        ret = -EFAULT;
        break;
    }
    }

    if (ret == 0) {
        ret = comms_write_msg(comms, &msg);
    }

    return ret;
}
#endif

/**
 * Update a neighbor in the neighbor list
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
void update_seen_neighbor(struct ble_data *data, int8_t rssi, bool polling) {
    int32_t index = get_seen_list_index(data->uuid);
    seen_list[index].RSSI = rssi;
    EVAL_STRENGTH();
    seen_list[index].ble_time_stamp = k_uptime_get();
    seen_list[index].polling_flag = polling;
}
