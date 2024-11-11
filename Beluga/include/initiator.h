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

int set_pan_id(uint16_t id);
int set_initializer_id(uint16_t id);
int double_sided_init(uint16_t id, double *distance);
int single_sided_init(uint16_t id, double *distance, uint8_t channel);

extern struct k_sem k_sus_init;

#endif // DECA_INIT_MAIN_H
