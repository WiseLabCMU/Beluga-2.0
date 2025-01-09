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

#include <ble_app.h>
#include <inttypes.h>
#include <list_neighbors.h>
#include <stdio.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the neighbor report module
 */
LOG_MODULE_REGISTER(neighbor_listing, CONFIG_NEIGHBOR_LISTING_LOG_LEVEL);

/**
 * Alias to indicate that printing is done in CSV format
 */
#define csv_mode false

/**
 * Alias to indicate that printing is done in JSON format
 */
#define json_mode true

#if defined(CONFIG_UWB_LOGIC_CLK)

/**
 * Format string for JSON format
 */
#define JSON_FORMAT                                                            \
    "{\"ID\": %" PRIu16 ", \"RANGE\": %f, \"RSSI\": %" PRId32                  \
    ", \"TIMESTAMP\": %" PRId64 ", \"EXCHANGE\": %" PRId32 "} \r\n"

/**
 * Format string for CSV format
 */
#define CSV_FORMAT "%" PRIu16 ", %f, %" PRId32 ", %" PRId64 ", %" PRId32 " \r\n"

/**
 * First string printed while in CSV mode
 */
#define CSV_HEADER "# ID, RANGE, RSSI, TIMESTAMP, EXCHANGE\r\n"

/**
 * Prints entry in the neighbor list to the console
 */
#define PRINT_ENTRY(entry)                                                     \
    printf(format_str(format_mode), (entry).UUID, (double)(entry).range,       \
           (entry).RSSI, (entry).time_stamp, (entry).exchange_id)
#else

/**
 * Format string for JSON format
 */
#define JSON_FORMAT                                                            \
    "{\"ID\": %" PRIu16 ", \"RANGE\": %f, \"RSSI\": %" PRId32                  \
    ", \"TIMESTAMP\": %" PRId64 "} \r\n"

/**
 * Format string for CSV format
 */
#define CSV_FORMAT "%" PRIu16 ", %f, %" PRId32 ", %" PRId64 " \r\n"

/**
 * First string printed while in CSV mode
 */
#define CSV_HEADER "# ID, RANGE, RSSI, TIMESTAMP\r\n"

/**
 * Prints entry in the neighbor list to the console
 */
#define PRINT_ENTRY(entry)                                                     \
    printf(format_str(format_mode), (entry).UUID, (entry).range, (entry).RSSI, \
           (entry).time_stamp)
#endif // defined(CONFIG_UWB_LOGIC_CLK)

/**
 * Determines what format string to use depending on the mode
 *
 * @param[in] mode The current format mode
 *
 * @return The format string based on the mode
 */
#define format_str(mode) ((mode) == json_mode) ? JSON_FORMAT : CSV_FORMAT

/**
 * Semaphore for suspending the task that prints the neighbors to the console
 */
K_SEM_DEFINE(print_list_sem, 0, 1);

/**
 * Mutex for locking the format mode
 */
K_MUTEX_DEFINE(format_mutex);

/**
 * Indicates whether to print the whole list or only updates
 */
static bool stream_mode = false;

/**
 * Indicates what format to print the neighbors in
 */
static bool format_mode = csv_mode;

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
 * @brief Sets the format mode for printing neighbors.
 *
 * This function allows switching between JSON and CSV output formats.
 *
 * @param[in] json If true, sets the format to JSON. If false, sets it to CSV.
 */
void set_format_mode(bool json) {
    k_mutex_lock(&format_mutex, K_FOREVER);
    format_mode = json;
    k_mutex_unlock(&format_mutex);
}

/**
 * @brief Gets the current format mode.
 *
 * This function returns the current output format, either JSON or CSV.
 *
 * @return `true` if the format is JSON
 * @return `false` if it is CSV
 */
bool get_format_mode(void) { return format_mode; }

/**
 * @brief Helper macro that substitutes additional conditions if there are any
 * provided. If there are no conditions provided, then true is automatically
 * substituted.
 */
#define PRINT_CONDITION(...)                                                   \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (true), (GET_ARG_N(1, __VA_ARGS__)))

/**
 * @brief Helper macro that resets the update flag if there is an argument
 * present.
 */
#define UPDATE_AFTER_PRINT(...)                                                \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (seen_list[j].update_flag = 0;))

/**
 * @brief Helper macro that initializes a flag to keep track if the header has
 * been printed or not. If no arguments are present, then nothing is generated.
 */
#define HEADER_VAR(...)                                                        \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (int header_flag = 0;))

/**
 * @brief Helper macro that generates an if statement to check if the CSV header
 * has been printed while in CSV mode. If no argument is provided, then the if
 * statement is not generated.
 */
#define PRINT_HEADER(...)                                                      \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (),                                     \
                (if (header_flag == 0 && format_mode == csv_mode) {            \
                    printf(CSV_HEADER);                                        \
                    header_flag = 1;                                           \
                }))

/**
 * @brief Generator macro that loops through the neighbor list and prints
 * entries. If no condition is provided, it will print all neighbors in the
 * list. If a condition is provided, then that condition will be used to
 * determine which neighbors get printed.
 */
#define PRINT_LIST(...)                                                        \
    do {                                                                       \
        HEADER_VAR(__VA_ARGS__)                                                \
        for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {                           \
            if (seen_list[j].UUID != 0 && PRINT_CONDITION(__VA_ARGS__)) {      \
                PRINT_HEADER(__VA_ARGS__)                                      \
                PRINT_ENTRY(seen_list[j]);                                     \
                UPDATE_AFTER_PRINT(__VA_ARGS__)                                \
            }                                                                  \
        }                                                                      \
    } while (0)

/**
 * @brief Custom print function for printing the format setting
 * @param[in] format The format saved in settings
 */
void print_output_format(int32_t format) {
    printf("Output Format: %s ", (format == 1) ? "JSON" : "CSV");
}

/**
 * @brief Prints all the neighbors in the list
 */
static void normal_print(void) {
    LOG_INF("Dumping all neighbors");
    if (format_mode == csv_mode) {
        LOG_INF("Logging in CSV mode");
        printf(CSV_HEADER);
    }

    PRINT_LIST();
}

/**
 * @brief Prints the neighbors that have been updated since the last run
 */
static void stream_print(void) {
    LOG_INF("Dumping updated neighbors");
    PRINT_LIST(seen_list[j].update_flag != 0);
}

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

    while (true) {
        k_sleep(K_MSEC(50));

        k_sem_take(&print_list_sem, K_FOREVER);
        k_mutex_lock(&format_mutex, K_FOREVER);

        /* Normal mode to print all neighbor nodes */
        if (!get_stream_mode()) {
            normal_print();
        } else {
            /* Streaming mode to print only new updated nodes */
            stream_print();
        }

        k_mutex_unlock(&format_mutex);
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
