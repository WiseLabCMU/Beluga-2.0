/**
 * @file list_neighbors.c
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

#include <beluga_message.h>
#include <ble_app.h>
#include <inttypes.h>
#include <list_neighbors.h>
#include <serial/comms.h>
#include <stdio.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the neighbor report module
 */
LOG_MODULE_REGISTER(neighbor_listing, CONFIG_NEIGHBOR_LISTING_LOG_LEVEL);

/**
 * Semaphore for suspending the task that prints the neighbors to the console
 */
K_SEM_DEFINE(print_list_sem, 0, 1);

/**
 * Indicates whether to print the whole list or only updates
 */
static bool stream_mode = false;

/**
 * @brief Sets the stream mode for neighbor printing.
 *
 * This function sets whether the program should print the entire list of
 * neighbors or only updated nodes.
 *
 * @param[in] value If true, only updated nodes will be printed. If false, the
 * entire list will be printed.
 */
void set_stream_mode(bool value) { stream_mode = value; }

/**
 * @brief Gets the current stream mode.
 *
 * This function returns the current stream mode, indicating whether only
 * updates or the entire neighbor list is being printed.
 *
 * @return `true` if only updates are printed
 * @return `false` if the entire list is printed
 */
bool get_stream_mode(void) { return stream_mode; }

/**
 * @brief Task to print out visible nodes information.
 *
 * This function runs as a task to periodically print out the neighbor nodes
 * information in the selected format (CSV or JSON). It prints either the full
 * list or only updated neighbors based on the stream mode.
 */
NO_RETURN static void list_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    const struct comms *comms = comms_backend_uart_get_ptr();
    struct beluga_msg msg = {.type = NEIGHBOR_UPDATES,
                             .payload.neighbor_list = seen_list,
                             .payload.stream = stream_mode};

    while (true) {
        k_sleep(K_MSEC(50));

        k_sem_take(&print_list_sem, K_FOREVER);

        msg.payload.stream = stream_mode;

        comms_write_msg(comms, &msg);
        if (stream_mode) {
            ARRAY_FOR_EACH(seen_list, i) { seen_list[i].update_flag = false; }
        }

        k_sem_give(&print_list_sem);
    }
}

#if defined(CONFIG_ENABLE_BELUGA_THREADS) && defined(CONFIG_ENABLE_LIST)

/**
 * Neighbor list reporter stack allocation
 */
K_THREAD_STACK_DEFINE(list_stack, CONFIG_LIST_STACK_SIZE);

/**
 * Thread data for the neighbor list reporter
 */
static struct k_thread list_thread_data;

/**
 * Thread ID for the neighbor list reporter
 */
static k_tid_t print_list_task_id;

/**
 * @brief Initializes the task that prints the neighbor list.
 */
void init_print_list_task(void) {
    print_list_task_id = k_thread_create(&list_thread_data, list_stack,
                                         K_THREAD_STACK_SIZEOF(list_stack),
                                         list_task_function, NULL, NULL, NULL,
                                         CONFIG_BELUGA_LIST_PRIO, 0, K_NO_WAIT);
    LOG_INF("Started neighbors list");
}
#else

/**
 * @brief Placeholder for the initialization function for when the list
 * is disabled.
 */
void init_print_list_task(void) { LOG_INF("Neighbors list disabled"); }
#endif // defined(CONFIG_ENABLE_BELUGA_THREADS) && defined(CONFIG_ENABLE_LIST)
