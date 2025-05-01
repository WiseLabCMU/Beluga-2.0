/**
 * @file list_monitor.h
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

#ifndef BELUGA_LIST_MONITOR_H
#define BELUGA_LIST_MONITOR_H

#include <serial/comms.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * The maximum length needed for the advertising name
 */
#define NAME_LEN 30

/**
 * Beluga scan data
 */
struct ble_data {
    uint16_t uuid;                      ///< The node ID
    uint8_t manufacturerData[NAME_LEN]; ///< The manufacturer data for the node
    bool beluga_node; ///< Flag indicating that the scanned device is a Beluga
    ///< node
};

/**
 * @brief Set the timeout value for node eviction.
 *
 * @param value The new timeout value in milliseconds.
 */
void set_node_timeout(uint32_t value);

/**
 * @brief Get the current timeout value for node eviction.
 *
 * @return The current timeout value in milliseconds.
 */
uint64_t get_node_timeout(void);

/**
 * @brief Mark that a new node has been added.
 */
void node_added(void);

/**
 * @brief Checks if a node has been added to the list
 * @return `true` if a node has been added
 * @return `false` if no nodes have been added
 */
bool check_node_added(void);

/**
 * @brief Initializes and starts the list monitor thread
 */
void init_monitor_thread(void);

/**
 * Inserts a new neighbor into the seen list
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 * @param[in] polling Flag indicating if the neighbor is polling
 */
void insert_into_seen_list(struct ble_data *data, int8_t rssi, bool polling);

/**
 * Update a neighbor in the neighbor list
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 * @param[in] polling Flag indicating if the neighbor is polling
 */
void update_seen_neighbor(struct ble_data *data, int8_t rssi, bool polling);

#if defined(CONFIG_BELUGA_EVICT_RUNTIME_SELECT)
/**
 * Updates the eviction policy
 * @param[in] policy The new policy
 */
void set_node_eviction_policy(enum node_eviction_policy new_policy);

/**
 * Prints the eviction policy in human readable text
 * @param[in] comms Pointer to the comms instance
 * @return 0 upon success
 * @return -EINVAL if input parameters are invalid
 * @return -EFAULT if the current eviction policy is unknown
 * @return negative error code otherwise
 */
int print_eviction_scheme(const struct comms *comms);
#else
/**
 * Updates the eviction policy
 * @param[in] policy The new policy
 */
#define set_node_eviction_policy(...) (void)0

/**
 * Prints the eviction policy in human readable text
 * @param[in] comms Pointer to the comms instance
 * @return 0 upon success
 * @return -EINVAL if input parameters are invalid
 * @return -EFAULT if the current eviction policy is unknown
 * @return -ENOTSUP if disabled
 * @return negative error code otherwise
 */
#define print_eviction_scheme(...)    (-ENOTSUP)
#endif

extern struct k_msgq evicted_nodes;

#endif // BELUGA_LIST_MONITOR_H
