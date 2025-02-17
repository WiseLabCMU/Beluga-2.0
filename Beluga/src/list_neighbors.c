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
#include <stdio.h>
#include <uart.h>
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

/**
 * Semaphore for suspending the task that prints the neighbors to the console
 */
K_SEM_DEFINE(print_list_sem, 0, 1);

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
void set_format_mode(bool json) { format_mode = json; }

/**
 * @brief Gets the current format mode.
 *
 * This function returns the current output format, either JSON or CSV.
 *
 * @return `true` if the format is JSON
 * @return `false` if it is CSV
 */
bool get_format_mode(void) { return format_mode; }

#define _STREAM_CONDITION(...)                                                 \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (true), (GET_ARG_N(1, __VA_ARGS__)))

#define _STREAM_FLAG_RESET(idx_, ...)                                          \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (seen_list[(idx_)].update_flag = 0))

#define _ID_FORMAT        "%" PRIu16
#define _RSSI_FORMAT      "%" PRId8
#define _RANGE_FORMAT     "%f"
#define _TIMESTAMP_FORMAT "%" PRId64
#define _EXCHANGE_FORMAT  "%" PRIu32
#define _SEP              ","

#define _BASE_CSV_FORMAT                                                       \
    _ID_FORMAT _SEP _RSSI_FORMAT _SEP _RANGE_FORMAT _SEP _TIMESTAMP_FORMAT
#define _BASE_CSV_HEADER "ID,RSSI,RANGE,TIMESTAMP"
#define _BASE_JSON_FORMAT                                                      \
    "{ID:" _ID_FORMAT ",RSSI:" _RSSI_FORMAT ",RANGE:" _RANGE_FORMAT            \
    ",TIMESTAMP:" _TIMESTAMP_FORMAT
#define _BASE_PRINT_ARGS(idx_)                                                 \
    seen_list[(idx_)].UUID, seen_list[(idx_)].RSSI,                            \
        (double)seen_list[(idx_)].range, seen_list[(idx_)].time_stamp

#if defined(CONFIG_UWB_LOGIC_CLK)
#define _CSV_FORMAT_STR   _BASE_CSV_FORMAT _SEP _EXCHANGE_FORMAT "\n"
#define _CSV_HEADER       _BASE_CSV_HEADER ",EXCHANGE\n"
#define _JSON_FORMAT_STR  _BASE_JSON_FORMAT ",EXCHANGE:" _EXCHANGE_FORMAT "}\n"
#define _PRINT_ARGS(idx_) _BASE_PRINT_ARGS(idx_), seen_list[(idx_)].exchange_id
#else
#define _CSV_FORMAT_STR   _BASE_CSV_FORMAT "\n"
#define _CSV_HEADER       _BASE_CSV_HEADER "\n"
#define _JSON_FORMAT_STR  _BASE_JSON_FORMAT "}\n"
#define _PRINT_ARGS(idx_) _BASE_PRINT_ARGS(idx_)
#endif

#define _PRINT_NORMAL(idx_, ...)                                               \
    do {                                                                       \
        bool header_printed = false;                                           \
        ARRAY_FOR_EACH(seen_list, idx_) {                                      \
            if (!header_printed && format_mode == csv_mode &&                  \
                _STREAM_CONDITION(__VA_ARGS__)) {                              \
                printf(_CSV_HEADER);                                           \
                header_printed = true;                                         \
            }                                                                  \
            if (_STREAM_CONDITION(__VA_ARGS__)) {                              \
                printf((format_mode == json_mode) ? _JSON_FORMAT_STR           \
                                                  : _CSV_FORMAT_STR,           \
                       _PRINT_ARGS(idx_));                                     \
                _STREAM_FLAG_RESET(idx_, __VA_ARGS__);                         \
            }                                                                  \
        }                                                                      \
    } while (0)

#define _FRAME_PRINT()                                                         \
    do {                                                                       \
        struct beluga_msg msg = {.type = NEIGHBOR_UPDATES,                     \
                                 .payload.neighbor_list = seen_list,           \
                                 .payload.stream = stream_mode};               \
        (void)write_message_frame(&msg);                                       \
        if (stream_mode) {                                                     \
            ARRAY_FOR_EACH(seen_list, i) { seen_list[i].update_flag = 0; }     \
        }                                                                      \
    } while (0)

#define PRINT_LIST(idx_, ...)                                                  \
    COND_CODE_1(IS_ENABLED(CONFIG_BELUGA_FRAMES), (_FRAME_PRINT()),            \
                (_PRINT_NORMAL(idx_, __VA_ARGS__)))

/**
 * @brief Custom print function for printing the format setting
 * @param[in] format The format saved in settings
 */
void print_output_format(int32_t format) {
    printf("Output Format: %s ", (format == 1) ? "JSON" : "CSV");
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

        /* Normal mode to print all neighbor nodes */
        if (!stream_mode) {
            LOG_INF("Dumping all neighbors");
            PRINT_LIST(j);
        } else {
            /* Streaming mode to print only new updated nodes */
            LOG_INF("Dumping updated neighbors");
            PRINT_LIST(j, seen_list[j].update_flag != 0);
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
