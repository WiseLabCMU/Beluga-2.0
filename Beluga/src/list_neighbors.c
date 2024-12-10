//
// Created by tom on 7/9/24.
//

#include <uart.h>

#include <ble_app.h>
#include <inttypes.h>
#include <list_neighbors.h>
#include <stdio.h>
#include <thread_priorities.h>
#include <utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(neighbor_listing, CONFIG_NEIGHBOR_LISTING_LOG_LEVEL);

#define csv_mode  false
#define json_mode true

#define format_str(mode)                                                       \
    ((mode) == json_mode) ? "{\"ID\": %" PRIu16                                \
                            ", \"RANGE\": %f, \"RSSI\": %" PRId32              \
                            ", \"TIMESTAMP\": %" PRId64 "} \r\n"               \
                          : "%" PRIu16 ", %f, %" PRId32 ", %" PRId64 " \r\n"

K_SEM_DEFINE(print_list_sem, 0, 1);
K_MUTEX_DEFINE(format_mutex);

static bool stream_mode = false;
static bool format_mode = csv_mode;

void set_stream_mode(bool value) { stream_mode = value; }

bool get_stream_mode(void) { return stream_mode; }

void set_format_mode(bool json) {
    k_mutex_lock(&format_mutex, K_FOREVER);
    format_mode = json;
    k_mutex_unlock(&format_mutex);
}

bool get_format_mode(void) { return format_mode; }

#define PRINT_CONDITION(...)                                                   \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (true), (GET_ARG_N(1, __VA_ARGS__)))
#define UPDATE_AFTER_PRINT(...)                                                \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (seen_list[j].update_flag = 0;))
#define HEADER_VAR(...)                                                        \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (), (int header_flag = 0;))
#define PRINT_HEADER(...)                                                      \
    COND_CODE_1(IS_EMPTY(__VA_ARGS__), (),                                     \
                (if (header_flag == 0 && format_mode == csv_mode) {            \
                    printf("# ID, RANGE, RSSI, TIMESTAMP\r\n");                \
                    header_flag = 1;                                           \
                }))

#define PRINT_LIST(...)                                                        \
    do {                                                                       \
        HEADER_VAR(__VA_ARGS__)                                                \
        for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {                           \
            if (seen_list[j].UUID != 0 && PRINT_CONDITION(__VA_ARGS__)) {      \
                PRINT_HEADER(__VA_ARGS__)                                      \
                printf(format_str(format_mode), seen_list[j].UUID,             \
                       seen_list[j].range, seen_list[j].RSSI,                  \
                       seen_list[j].time_stamp);                               \
                UPDATE_AFTER_PRINT(__VA_ARGS__)                                \
            }                                                                  \
        }                                                                      \
    } while (0)

static void normal_print(void) {
    LOG_INF("Dumping all neighbors");
    if (format_mode == csv_mode) {
        LOG_INF("Logging in CSV mode");
        printf("# ID, RANGE, RSSI, TIMESTAMP\r\n");
    }

    PRINT_LIST();
}

static void stream_print(void) {
    LOG_INF("Dumping updated neighbors");
    PRINT_LIST(seen_list[j].update_flag != 0);
}

/**
 * @brief Task to print out visible nodes information
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
NO_RETURN void list_task_function(void *p1, void *p2, void *p3) {
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

#if ENABLE_THREADS && ENABLE_LIST
K_THREAD_STACK_DEFINE(list_stack, CONFIG_LIST_STACK_SIZE);
static struct k_thread list_thread_data;
static k_tid_t print_list_task_id;

void init_print_list_task(void) {
    print_list_task_id = k_thread_create(&list_thread_data, list_stack,
                                         K_THREAD_STACK_SIZEOF(list_stack),
                                         list_task_function, NULL, NULL, NULL,
                                         CONFIG_BELUGA_LIST_PRIO, 0, K_NO_WAIT);
    LOG_INF("Started neighbors list");
}
#else
void init_print_list_task(void) { LOG_INF("Neighbors list disabled"); }
#endif
