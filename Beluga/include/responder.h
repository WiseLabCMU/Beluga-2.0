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
