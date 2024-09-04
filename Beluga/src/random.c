/*! ----------------------------------------------------------------------------
 *  @file   flash.h
 *
 *  @brief  Helper functions to generate random delay number for ALOHA MAC
 * protocol
 *
 *  @date   2020/06
 *
 *  @author WiseLab-CMU
 */

#include "random.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/random/random.h>

/*
 * Get an exponential distribution random number determined by polling frequency
 */
uint16_t get_rand_num_exp_collision(uint32_t freq) {
    int lower = MIN_DELAY;
    double lambda = 5.0 / (double)freq;
    double u;
    u = sys_rand32_get() / ((double)RAND_MAX + 1.0);
    return (-log(1 - u) / lambda) + lower;
}
