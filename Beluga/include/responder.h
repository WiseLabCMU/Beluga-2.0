/**
 * @file responder.h
 *
 * @brief This module implements the functionality for the responder role in a
 * UWB-based ranging system.
 *
 * The primary objective of this module is to perform the communication sequence
 * required to respond to a ranging process.
 *
 * The module handles:
 * - Receiving a poll message to the responder.
 * - Sending a response from the responder.
 * - Receiving a final message containing timestamp data (double-sided
 * TWR only).
 * - Sending a report message containing the time-of-flight (ToF) data, which
 * is used to calculate the distance (double-sided TWR only).
 *
 * @date 11/7/2024
 *
 * @author Decawave
 * @author Tome Schmitz
 */

#ifndef BELUGA_RESPONDER_H
#define BELUGA_RESPONDER_H

#include <ranging.h>
#include <zephyr/kernel.h>

/**
 * @brief Sets the personal area network (PAN) ID for the responder messages
 *
 * @param[in] id
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_responder_pan_id(uint16_t id);

/**
 * @brief Set the antenna RX delay calibration value for the given pulse
 * repetition frequency.
 *
 * @param[in] prf Pulse repetition frequency associated with the value.
 * @param[in] delay The antenna dealy calibration value.
 * @return 0 upon success.
 * @return -EBUSY if UWB is active.
 * @return -EINVAL if prf argument is invalid
 */
int set_responder_antenna_rx_delay(enum uwb_pulse_rate prf, uint16_t delay);

/**
 * @brief Set the antenna TX delay calibration value for the given pulse
 * repetition frequency.
 *
 * @param[in] prf Pulse repetition frequency associated with the value.
 * @param[in] delay The antenna dealy calibration value.
 * @return 0 upon success.
 * @return -EBUSY if UWB is active.
 * @return -EINVAL if prf argument is invalid
 */
int set_responder_antenna_tx_delay(enum uwb_pulse_rate prf, uint16_t delay);

/**
 * Set the pulse repetition frequency that is being used.
 * @param[in] prf The current PRF.
 * @return 0 upon success.
 * @return -EBUSY if UWB is active.
 * @return -EINVAL if prf argument is invalid
 */
int set_responder_prf(enum uwb_pulse_rate prf);

/**
 * Set the power amp state in the module.
 * @param[in] enable The current power amplifier state.
 */
void set_responder_power_mode(bool enable);

/**
 * @brief Sets the source IDs for the messages that the responder sends and the
 * destination ID for the messages the responder receives
 *
 * @param[in] id The ID of the node
 *
 * @return 0 upon success
 * @return -EBUSY if UWB is active
 */
int set_responder_id(uint16_t id);

/**
 * @brief Responds to double-sided two-way ranging requests
 *
 * @param[out] id The node that got responded to.
 * @param[out] logic_clk The exchange ID of the ranging transaction
 *
 * @return 0 upon success
 * @return -EBUSY if the responder gets suspended
 * @return negative error code otherwise
 */
int ds_resp_run(uint16_t *id, uint32_t *logic_clk);

/**
 * @brief Responds to single-sided two-way ranging requests
 *
 * @param[out] id The node that got responded to.
 * @param[out] logic_clk The exchange ID of the ranging transaction
 *
 * @return 0 upon success
 * @return -EBUSY if the responder gets suspended
 * @return negative error code otherwise
 */
int ss_resp_run(uint16_t *id, uint32_t *logic_clk);

/**
 * Semaphore for suspending the responder
 */
extern struct k_sem k_sus_resp;

#endif // BELUGA_RESPONDER_H
