/**
 * @file beluga_client.h
 *
 * @brief Client API for the Beluga Service.
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BELUGA_CLIENT_H
#define BELUGA_DTS_BELUGA_CLIENT_H

#include <bluetooth/gatt_dm.h>
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include <ble/services/beluga_service_common.h>

/**
 * @brief Handles on the connected peer device that are needed to interact with
 * the device.
 */
struct bt_beluga_client_handles {
    /**
     * Handle of the beluga service range characteristic, as provided by a
     * discovery.
     */
    uint16_t range;

    /**
     * Handle of the CCC descriptor of the beluga service range characteristic,
     * as provided by a discovery.
     */
    uint16_t range_ccc;

    /**
     * Handle of the beluga service sync characteristic, as provided by a
     * discovery.
     */
    uint16_t sync;
};

struct bt_beluga_client;

/**
 * @brief Beluga Client callback structure.
 */
struct bt_beluga_client_cb {
    /**
     * @brief Range received callback.
     *
     * The range has been received as a notification of the beluga service range
     * characteristic.
     *
     * @param[in] client The Beluga client instance
     * @param[in] id The ID of the ranged to node
     * @param[in] range The measured distance between the nodes
     *
     * @return BT_GATT_ITER_CONTINUE to keep notifications enabled.
     * @return BT_GATT_ITER_STOP to disable notifications.
     */
    uint8_t (*range)(struct bt_beluga_client *client, uint16_t id,
                     double range);

    /**
     * @brief UWB configurations synced callback.
     *
     * The UWB configuration sync request has been sent and written to the
     * Beluga service sync characteristic.
     *
     * @param[in] client The Beluga client instance
     * @param[in] err ATT error code
     * @param[in] config The UWB configurations sent
     */
    void (*synced)(struct bt_beluga_client *client, uint8_t err,
                   const struct beluga_uwb_params *config);

    /**
     * @brief Range notifications disabled callback.
     * @param[in] client Beluga client instance
     */
    void (*unsubscribed)(struct bt_beluga_client *client);
};

/**
 * Beluga service client structure
 */
struct bt_beluga_client {

    /**
     * Connection object.
     */
    struct bt_conn *conn;

    /**
     * Internal state.
     */
    atomic_t state;

    /**
     * Handles on the connected peer device that are needed to interact with the
     * device.
     */
    struct bt_beluga_client_handles handles;

    /**
     * GATT subscribe parameters for Beluga service range characteristic.
     */
    struct bt_gatt_subscribe_params range_notif_params;

    /**
     * GATT write parameters for Beluga service sync characteristic.
     */
    struct bt_gatt_write_params sync_write_params;

    /**
     * Application callbacks.
     */
    struct bt_beluga_client_cb cb;
};

/**
 * @brief Initializes the beluga client module.
 *
 * @param[in,out] client The Beluga client instance.
 * @param[in] callbacks Beluga client application callbacks.
 * @return 0 if the instantiation is successful
 * @return negative error code otherwise
 */
int bt_beluga_client_init(struct bt_beluga_client *client,
                          const struct bt_beluga_client_cb *callbacks);

/**
 * @brief Sync the server UWB configurations with the client UWB configurations
 * @param[in,out] client The Beluga client instance.
 * @param[in] config Pointer to the UWB parameters to send to the server.
 * @return 0 if operation was successful
 * @return negative error code otherwise
 */
int bt_beluga_client_sync(struct bt_beluga_client *client,
                          const struct beluga_uwb_params *config);

/**
 * @brief Assign handles to the Beluga client instance.
 *
 * This function should be called when a link with a peer has been established
 * to associate the link to this instance of the module. This makes it
 * possible to handle several links and associate each link to a particular
 * instance of this module. The GATT attribute handles are provided by the
 * GATT DB discovery module.
 *
 * @param[in] dm Discovery object
 * @param[in,out] client Beluga client instance.
 * @return 0 If the operation was successful.
 * @return (-ENOTSUP) Special error code used when UUID of the service does not
 * match the expected UUID.
 * @return negative error code otherwise.
 */
int bt_beluga_client_assign(struct bt_gatt_dm *dm,
                            struct bt_beluga_client *client);

/**
 * @brief Request the peer to start sending notifications for the range
 * characteristic.
 * @param[in,out] client Beluga client instance.
 * @return 0 if the operation was successful
 * @return negative error code otherwise.
 */
int bt_beluga_client_subscribe_ranging(struct bt_beluga_client *client);

#endif // BELUGA_DTS_BELUGA_CLIENT_H
