//
// Created by tom on 11/22/24.
//

#ifndef BELUGA_INITIATOR_H
#define BELUGA_INITIATOR_H

#include "deca_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

extern int debug_print;

int set_initiator_id(uint16_t id);
int ds_init_run(uint16_t id, double *distance, uint32_t *logic_clock);
int ss_init_run(uint16_t id, double *distance, uint32_t *logic_clock);

extern struct k_sem k_sus_init;

#endif // BELUGA_INITIATOR_H
