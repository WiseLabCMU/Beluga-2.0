/**
 * @file random.c
 *
 * @brief Helper functions to generate random delay number for ALOHA MAC
 * protocol
 *
 * @data 2020/06
 *
 * @author WiSeLab CMU
 */

#include "random.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/random/random.h>

/**
 * The minimum return value
 */
#define MIN_DELAY UINT16_C(10)

/**
 * @brief Get an exponential distribution random number determined by the
 * polling frequency
 *
 * @param[in] freq The polling frequency
 *
 * @return An exponential distribution random number
 */
uint16_t get_rand_num_exp_collision(uint32_t freq) {
    double lambda = 5.0 / (double)freq;
    double u;
    u = sys_rand32_get() / ((double)RAND_MAX + 1.0);
    return (uint16_t)(-log(1 - u) / lambda) + MIN_DELAY;
}
