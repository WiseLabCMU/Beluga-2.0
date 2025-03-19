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

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

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

extern struct k_msgq evicted_nodes;

#endif // BELUGA_LIST_MONITOR_H
