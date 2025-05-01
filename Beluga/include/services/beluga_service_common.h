/**
 * @file beluga_service_common.h
 *
 * @brief
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BELUGA_SERVICE_COMMON_H
#define BELUGA_DTS_BELUGA_SERVICE_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Payload size of a sync request
 */
#define BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE 7

/**
 * UWB Parameters for synchronization
 */
struct beluga_uwb_params {
    /**
     * Two-way ranging mode
     */
    bool TWR;

    /**
     * Start frame delimiter mode
     */
    bool SFD;

    /**
     * Pulse rate
     * - false: 16 MHz
     * - true: 64 MHz
     */
    bool PULSE_RATE;

    /**
     * Physical layer header mode
     */
    bool PHR;

    /**
     * UWB channel
     */
    uint8_t CHANNEL;

    /**
     * UWB data rate
     */
    uint8_t DATA_RATE;

    /**
     * Preamble acquisition chunk size
     */
    uint8_t PAC;

    /**
     * External power amp setting
     */
    uint8_t POWER_AMP;

    /**
     * Preamble length
     */
    uint16_t PREAMBLE;

    /**
     * DW1000 transmit power
     */
    uint32_t TX_POWER;
};

/**
 * Encodes the UWB parameters into a buffer
 * @param[in] buf The buffer to store the UWB parameters in
 * @param[in] len The size of the buffer
 * @param[in] configs Pointer to the UWB configurations to encode into a buffer
 * @return 0 upon success
 * @return -EINVAL if any of the parameters are invalid
 * @return -EBADMSG if an invalid preamble length is received
 */
int serialize_uwb_configurations(char *buf, size_t len,
                                 const struct beluga_uwb_params *configs);

/**
 * Decodes a buffer into UWB parameters
 * @param[in] configs Pointer to the UWB configurations object to decode the
 * parameters into
 * @param[in] buf The buffer to decode
 * @param[in] len The length of the buffer
 * @return 0 upon success
 * @return -EINVAL if any of the parameters are invalid
 */
int deserialize_uwb_configurations(struct beluga_uwb_params *configs,
                                   const char *buf, size_t len);

#endif // BELUGA_DTS_BELUGA_SERVICE_COMMON_H
