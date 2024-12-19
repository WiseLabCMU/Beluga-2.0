/**
 * @file random.h
 *
 * @brief Helper functions to generate random delay number for ALOHA MAC
 * protocol
 *
 * @data 2020/06
 *
 * @author WiSeLab CMU
 */

#ifndef ALOHA_MAC_RANDOM_H_
#define ALOHA_MAC_RANDOM_H_

#include <stdint.h>

/**
 * @brief Get an exponential distribution random number determined by the
 * polling frequency
 *
 * @param[in] freq The polling frequency
 *
 * @return An exponential distribution random number
 */
uint16_t get_rand_num_exp_collision(uint32_t freq);

#endif // ALOHA_MAC_RANDOM_H_
