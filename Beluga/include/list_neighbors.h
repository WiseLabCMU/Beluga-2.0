//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_LIST_NEIGHBORS_H
#define BELUGA_LIST_NEIGHBORS_H

#include <zephyr/kernel.h>

void set_stream_mode(bool value);
bool get_stream_mode(void);

extern struct k_sem print_list_sem;

#endif // BELUGA_LIST_NEIGHBORS_H