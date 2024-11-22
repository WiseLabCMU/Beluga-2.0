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

double ds_init_run(uint8 id);
double ss_init_run(uint8 id);

extern struct k_sem k_sus_init;

#endif // BELUGA_INITIATOR_H
