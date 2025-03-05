/**
 * @file list_neighbors.h
 * @brief Neighbor listing module for printing neighbor node information.
 *
 * This module provides functionalities to print out the list of neighbor nodes
 * in either CSV or JSON format. It supports both full list printing and
 * streaming updates, depending on the selected mode.
 *
 * @date 7/9/2024
 *
 * @author WiSeLab CMU
 * @author Tom Schmitz
 */

#ifndef BELUGA_LIST_NEIGHBORS_H
#define BELUGA_LIST_NEIGHBORS_H

#include <zephyr/kernel.h>

/**
 * @brief Sets the stream mode for neighbor printing.
 *
 * This function sets whether the program should print the entire list of
 * neighbors or only updated nodes.
 *
 * @param[in] value If true, only updated nodes will be printed. If false, the
 * entire list will be printed.
 */
void set_stream_mode(bool value);

/**
 * @brief Gets the current stream mode.
 *
 * This function returns the current stream mode, indicating whether only
 * updates or the entire neighbor list is being printed.
 *
 * @return `true` if only updates are printed
 * @return `false` if the entire list is printed
 */
bool get_stream_mode(void);

/**
 * @brief Initializes the task that prints the neighbor list.
 */
void init_print_list_task(void);

/**
 * Semaphore for suspending the task that prints the neighbors to the console
 */
extern struct k_sem print_list_sem;

#endif // BELUGA_LIST_NEIGHBORS_H
