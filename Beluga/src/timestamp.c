//
// Created by tom on 6/28/24.
//

#include <timestamp.h>
#include <zephyr/kernel.h>
#include <utils.h>

static volatile uint64_t timestamp = UINT64_C(0);

static void update_timestamp(UNUSED struct k_timer *timer) {
    timestamp += UINT64_C(1);
}

K_TIMER_DEFINE(timestamp_timer, update_timestamp, NULL);

void init_timekeeper(uint32_t ms_delay_start, uint32_t ms_period) {
    if (ms_delay_start == 0) {
        ms_delay_start = 1;
    }
    k_timer_start(&timestamp_timer, K_MSEC(ms_delay_start), K_MSEC(ms_period));
}

uint64_t get_timestamp(void) {
    return timestamp;
}
