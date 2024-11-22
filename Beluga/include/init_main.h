/*----------------------------------------------------------------------------
 *  @file    init_main.h
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) initiator
 * code -- Header file
 *
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#ifndef DECA_INIT_MAIN_H
#define DECA_INIT_MAIN_H

#include "deca_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

extern int debug_print;

int ds_init_run(uint8 id, double *distance);
int ss_init_run(uint8 id, double *distance);

extern struct k_sem k_sus_init;

#endif // DECA_INIT_MAIN_H
