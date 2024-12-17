//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_LIST_NEIGHBORS_H
#define BELUGA_LIST_NEIGHBORS_H

#include <zephyr/kernel.h>

void print_output_format(int32_t format);
void set_stream_mode(bool value);
bool get_stream_mode(void);
void set_format_mode(bool json);
bool get_format_mode(void);
void init_print_list_task(void);

extern struct k_sem print_list_sem;

#endif // BELUGA_LIST_NEIGHBORS_H
