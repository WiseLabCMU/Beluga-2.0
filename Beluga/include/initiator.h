/**
 * @file initiator.h
 *
 * @brief This module implements the functionality for the initiator role in a
 * UWB-based ranging system.
 *
 * The primary objective of this module is to perform the communication sequence
 * required to initiate a ranging process and compute the distance between the
 * initiator and a responder in a UWB-based system.
 *
 * The module handles:
 * - Sending a poll message to the responder.
 * - Receiving a response from the responder.
 * - Sending a final message containing timestamp data (two-way/double-sided
 * ranging only).
 * - Receiving a report message containing the time-of-flight (ToF) data, which
 * is used to calculate the distance (two-way/double-sided ranging only).
 *
 * @date 6/1/2024
 *
 * @author Decawave
 * @author Tome Schmitz
 */

#ifndef BELUGA_INITIATOR_H
#define BELUGA_INITIATOR_H

#include <deca_types.h>
#include <ranging.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * @brief Sets the source IDs for the messages that the initiator sends and the
 * destination ID for the messages the initiator receives
 *
 * @param[in] id The ID of the node
 *
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_initiator_id(uint16_t id);

int set_initiator_antenna_rx_delay(enum uwb_pulse_rate prf, uint16_t delay);

int set_initiator_antenna_tx_delay(enum uwb_pulse_rate prf, uint16_t delay);

int set_initiator_prf(enum uwb_pulse_rate prf);

void set_initiator_power_mode(bool enable);

/**
 * @brief Sets the personal area network (PAN) ID for the initiator messages
 *
 * @param[in] id
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_initiator_pan_id(uint16_t id);

/**
 * @brief Sets the multiplication factor to convert carrier integrator value to
 * a frequency offset in Hertz depending on the data rate.
 *
 * @param[in] datarate_110k `true` if data rate is 110K, `false` otherwise
 */
void set_freq_offset_multiplier(bool datarate_110k);

/**
 * @brief Sets the multiplication factor to convert frequency offset in Hertz to
 * PPM crystal offset depending on the channel.
 *
 * @param[in] channel The channel being used for UWB
 */
void set_hertz_to_ppm_multiplier(uint8_t channel);

/**
 * @brief Initiates a two-way/double-sided ranging measurement to a certain
 * node.
 *
 * @param[in] id The node to range to
 * @param[out] distance The estimated distance between the nodes
 * @param[out] logic_clock The ID associated with the successful exchange.
 *
 * @return 0 upon a successful ranging run
 * @return -EINVAL if distance parameter is NULL
 * @return negative error code otherwise
 *
 * @note logic_clock will be updated after a successful two-way/double-sided
 * ranging exchange. If an error occurred, then logic_clock will not be updated
 * and thus will retain its original value. Also, if logic_clock is NULL, it is
 * assumed that the logic_clock output is not desired and the run will still be
 * initiated.
 */
int ds_init_run(uint16_t id, double *distance, uint32_t *logic_clock);

/**
 * @brief Initiates a single-sided ranging measurement to a certain node.
 *
 * @param[in] id The node to range to
 * @param[out] distance The estimated distance between the nodes
 * @param[out] logic_clock The ID associated with the successful exchange.
 *
 * @return 0 upon a successful ranging run
 * @return -EINVAL if distance parameter is NULL
 * @return negative error code otherwise
 *
 * @note logic_clock will be updated after a successful two-way/double-sided
 * ranging exchange. If an error occurred, then logic_clock will not be updated
 * and thus will retain its original value. Also, if logic_clock is NULL, it is
 * assumed that the logic_clock output is not desired and the run will still be
 * initiated.
 */
int ss_init_run(uint16_t id, double *distance, uint32_t *logic_clock);

/**
 * @brief Retrieves the stage that the UWB ranging exchange failed at
 * @return The failed stage
 */
int dropped_stage(void);

/**
 * Semaphore for suspending the initiator task
 */
extern struct k_sem k_sus_init;

#endif // BELUGA_INITIATOR_H
