//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_LIST_MONITOR_H
#define BELUGA_LIST_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

void set_node_timeout(uint32_t value);
uint64_t get_node_timeout(void);
void node_added(void);
bool check_node_added(void);

#endif // BELUGA_LIST_MONITOR_H
