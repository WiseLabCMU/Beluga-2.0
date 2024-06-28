//
// Created by tom on 6/28/24.
//

#ifndef BELUGA_TIMESTAMP_H
#define BELUGA_TIMESTAMP_H

#include <stdint.h>

void init_timekeeper(uint32_t ms_delay_start, uint32_t ms_period);
uint64_t get_timestamp(void);

#endif //BELUGA_TIMESTAMP_H
