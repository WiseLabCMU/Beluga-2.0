/**
 * @file beluga_service.h
 *
 * @brief
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BELUGA_SERVICE_H
#define BELUGA_DTS_BELUGA_SERVICE_H

#include <services/beluga_service_common.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

/**
 * @brief Beluga service UUIDs.
 *
 * These UUIDs are used to define the Beluga service.
 *
 * @note When adding a new attribute, take the previous UUID and increment the
 * 32-bit UUID (The first field in BT_UUID_128_ENCODE) by 1.
 *
 * @{
 */

/**
 * Beluga Service UUID
 */
#define BT_UUID_BELUGA_SVC_VAL                                                 \
    BT_UUID_128_ENCODE(0x89b468da, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)

/**
 * Range Characteristic UUID
 */
#define BT_UUID_BELUGA_RANGE_VAL                                               \
    BT_UUID_128_ENCODE(0x89b468db, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)

/**
 * Sync Characteristic UUID
 */
#define BT_UUID_BELUGA_SYNC_VAL                                                \
    BT_UUID_128_ENCODE(0x89b468dc, 0x535a, 0x4268, 0xa703, 0x87ddc32450b8)

#define BT_UUID_BELUGA_SVC   BT_UUID_DECLARE_128(BT_UUID_BELUGA_SVC_VAL)
#define BT_UUID_BELUGA_RANGE BT_UUID_DECLARE_128(BT_UUID_BELUGA_RANGE_VAL)
#define BT_UUID_BELUGA_SYNC  BT_UUID_DECLARE_128(BT_UUID_BELUGA_SYNC_VAL)

/**
 * @}
 */

/**
 * Callback type for when a SYNC request is received
 */
typedef void (*beluga_uwb_sync)(struct bt_conn *conn,
                                const struct beluga_uwb_params *configs);

/**
 * Callback type for when the range notification has been sent
 */
typedef void (*beluga_range_sent)(struct bt_conn *conn);

/**
 * Callbacks for service events
 */
struct beluga_service_cb {
    /**
     * @brief UWB setting sync request callback.
     *
     * @param[in] conn Pointer to the connection object that has received data
     * @param[in] configs Pointer to UWB configurations the connection is
     * requesting the modem to set the UWB to.
     *
     * @note This callback is required.
     */
    beluga_uwb_sync sync_cb;

    /**
     * @brief Range sent callback
     *
     * @param[in] conn Pointer to the connection object that sent the data. NULL
     * if sent to all connected peers.
     */
    beluga_range_sent sent_cb;
};

/**
 * Initializes the service.
 *
 * @param[in] cb Struct with function pointers to callbacks for service events.
 * @return 0 upon success
 * @return -EINVAL if no callbacks are specified
 */
int beluga_service_init(struct beluga_service_cb *cb);

/**
 * @brief Sends a range notification.
 *
 * @param[in] conn Pointer to connection object, or NULL to send to all
 * connected peers.
 * @param[in] id The id of the node ranged to.
 * @param[in] range The measured distance between the nodes.
 * @return 0 if the data is sent.
 * @return negative error code otherwise.
 */
int range_notify(struct bt_conn *conn, uint16_t id, float range);

#endif // BELUGA_DTS_BELUGA_SERVICE_H
