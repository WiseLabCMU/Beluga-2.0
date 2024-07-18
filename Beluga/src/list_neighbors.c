//
// Created by tom on 7/9/24.
//

#include <ble_app.h>
#include <list_neighbors.h>
#include <thread_priorities.h>
#include <utils.h>
#include <zephyr/kernel.h>

K_SEM_DEFINE(print_list_sem, 1, 1);
K_MUTEX_DEFINE(stream_mode_mutex);

static bool stream_mode = false;

void set_stream_mode(bool value) {
    k_mutex_lock(&stream_mode_mutex, K_FOREVER);
    stream_mode = value;
    k_mutex_unlock(&stream_mode_mutex);
}

bool get_stream_mode(void) {
    bool retVal;
    k_mutex_lock(&stream_mode_mutex, K_FOREVER);
    retVal = stream_mode;
    k_mutex_unlock(&stream_mode_mutex);
    return retVal;
}

static void normal_print(void) {
    printf("# ID, RANGE, RSSI, TIMESTAMP\r\n");

    for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {
        if (seen_list[j].UUID != 0)
            printf("%d, %f, %d, %d \r\n", seen_list[j].UUID, seen_list[j].range,
                   seen_list[j].RSSI, seen_list[j].time_stamp);
    }
}

static void stream_print(void) {
    int count_flag = 0;

    // Check whether alive nodes have update flag or not
    for (int i = 0; i < MAX_ANCHOR_COUNT; i++) {
        if (seen_list[i].UUID != 0 && seen_list[i].update_flag != 0) {
            count_flag++;
        }
    }
    // If one of node has update flag, print it
    if (count_flag != 0) {
        printf("# ID, RANGE, RSSI, TIMESTAMP\r\n");

        for (int j = 0; j < MAX_ANCHOR_COUNT; j++) {
            if (seen_list[j].UUID != 0 && seen_list[j].update_flag == 1)
                printf("%d, %f, %d, %d \r\n", seen_list[j].UUID,
                       seen_list[j].range, seen_list[j].RSSI,
                       seen_list[j].time_stamp);

            // Reset update flag of the node
            seen_list[j].update_flag = 0;
        }
    }
}

/**
 * @brief Task to print out visible nodes information
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
void list_task_function(void) {
    while (true) {
        k_sleep(K_MSEC(50));

        k_sem_take(&print_list_sem, K_FOREVER);

        /* Normal mode to print all neighbor nodes */
        if (!get_stream_mode()) {
            normal_print();
        } else {
            /* Streaming mode to print only new updated nodes */
            stream_print();
        }

        k_sem_give(&print_list_sem);
    }
}

#if ENABLE_THREADS && ENABLE_LIST
K_THREAD_DEFINE(print_list_task_id, CONFIG_LIST_STACK_SIZE, list_task_function, NULL, NULL,
                NULL, LIST_PRIO, 0, 0);
#endif
