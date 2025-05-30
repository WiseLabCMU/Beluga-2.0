/**
 * @file beluga_client.c
 *
 * @brief Client API for the Beluga Service.
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <ble/services/beluga_client.h>
#include <ble/services/beluga_service.h>

#include <zephyr/logging/log.h>

/**
 * Logger for the Beluga service client
 */
LOG_MODULE_REGISTER(beluga_client, CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL);

/**
 * Client state bits
 */
enum {
    BELUGA_C_INITIALIZED,           ///< Client initialized
    BELUGA_C_RANGING_NOTIF_ENABLED, ///< Ranging notifications enabled
    BELUGA_C_SYNC_PENDING,          ///< Synchronization is pending
};

/**
 * Handler for receiving ranging notifications
 * @param[in] conn The BLE connection that received the range
 * @param[in] params The GATT subscription parameters
 * @param[in] data The received data
 * @param[in] length The amount of bytes received
 * @return BT_GATT_ITER_STOP to stop receiving notifications (unsubscribing)
 * @return BT_GATT_ITER_CONTINUE to continue receiving notifications
 */
static uint8_t on_received(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length) {
    ARG_UNUSED(conn);
    struct bt_beluga_client *client =
        CONTAINER_OF(params, struct bt_beluga_client, range_notif_params);

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0;
        atomic_clear_bit(&client->state, BELUGA_C_RANGING_NOTIF_ENABLED);
        if (client->cb.unsubscribed) {
            client->cb.unsubscribed(client);
        }
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[NOTIFICATION] data %p length %u", data, length);
    if (client->cb.range && length == (sizeof(uint16_t) + sizeof(double))) {
        uint16_t id;
        double range;
        memcpy(&id, data, sizeof(uint16_t));
        memcpy(&range, (char *)data + sizeof(uint16_t), sizeof(double));
        return client->cb.range(client, id, range);
    }

    return BT_GATT_ITER_CONTINUE;
}

/**
 * Handler for indicating that sync data got sent
 * @param[in] conn The connection that data was sent on
 * @param[in] err The error from the operation
 * @param[in] params The GATT write parameters
 */
static void on_sent(struct bt_conn *conn, uint8_t err,
                    struct bt_gatt_write_params *params) {
    ARG_UNUSED(conn);
    const void *data;
    size_t len;
    struct bt_beluga_client *client =
        CONTAINER_OF(params, struct bt_beluga_client, sync_write_params);

    data = params->data;
    len = params->length;

    atomic_clear_bit(&client->state, BELUGA_C_SYNC_PENDING);
    if (client->cb.synced) {
        struct beluga_uwb_params config;
        (void)deserialize_uwb_configurations(&config, data, len);
        client->cb.synced(client, err, &config);
    }
}

/**
 * @brief Initializes the beluga client module.
 *
 * @param[in,out] client The Beluga client instance.
 * @param[in] callbacks Beluga client application callbacks.
 * @return 0 if the instantiation is successful
 * @return negative error code otherwise
 */
int bt_beluga_client_init(struct bt_beluga_client *client,
                          const struct bt_beluga_client_cb *callbacks) {
    if (!client || !callbacks) {
        return -EINVAL;
    }

    if (atomic_test_and_set_bit(&client->state, BELUGA_C_INITIALIZED)) {
        return -EALREADY;
    }

    memcpy(&client->cb, callbacks, sizeof(client->cb));
    return 0;
}

/**
 * @brief Sync the server UWB configurations with the client UWB configurations
 * @param[in,out] client The Beluga client instance.
 * @param[in] config Pointer to the UWB parameters to send to the server.
 * @return 0 if operation was successful
 * @return negative error code otherwise
 */
int bt_beluga_client_sync(struct bt_beluga_client *client,
                          const struct beluga_uwb_params *config) {
    int err;
    static char buf[BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE];

    if (!client) {
        return -EINVAL;
    }

    if (!client->conn) {
        return -ENOTCONN;
    }

    if (atomic_test_and_set_bit(&client->state, BELUGA_C_SYNC_PENDING)) {
        return -EALREADY;
    }

    err = serialize_uwb_configurations(buf, sizeof(buf), config);
    if (err) {
        return err;
    }

    client->sync_write_params.func = on_sent;
    client->sync_write_params.handle = client->handles.sync;
    client->sync_write_params.offset = 0;
    client->sync_write_params.data = buf;
    client->sync_write_params.length = BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE;

    err = bt_gatt_write(client->conn, &client->sync_write_params);
    if (err) {
        atomic_clear_bit(&client->state, BELUGA_C_SYNC_PENDING);
    }

    return err;
}

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
                            struct bt_beluga_client *client) {
    const struct bt_gatt_dm_attr *gatt_service_attr;
    const struct bt_gatt_service_val *gatt_service;
    const struct bt_gatt_dm_attr *gatt_chrc;
    const struct bt_gatt_dm_attr *gatt_desc;

    if (!dm || !client) {
        return -EFAULT;
    }

    gatt_service_attr = bt_gatt_dm_service_get(dm);
    gatt_service = bt_gatt_dm_attr_service_val(gatt_service_attr);

    if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_BELUGA_SVC)) {
        return -ENOTSUP;
    }
    LOG_DBG("Getting handles from Beluga service");
    memset(&client->handles, 0xFF, sizeof(client->handles));

    /* Range characteristic */
    gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_BELUGA_RANGE);
    if (!gatt_chrc) {
        LOG_ERR("Missing ranging characteristic");
        return -EINVAL;
    }

    /* Ranging */
    gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_BELUGA_RANGE);
    if (!gatt_desc) {
        LOG_ERR("Missing Beluga Ranging value descriptor in characteristic.");
        return -EINVAL;
    }
    LOG_DBG("Found handle for ranging characteristic.");
    client->handles.range = gatt_desc->handle;
    /* Ranging CCC */
    gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
    if (!gatt_desc) {
        LOG_ERR("Missing ranging CCC in characteristics");
        return -EINVAL;
    }
    LOG_DBG("Found handle for CCC of ranging characteristic");
    client->handles.range_ccc = gatt_desc->handle;

    /* Sync characteristic */
    gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_BELUGA_SYNC);
    if (!gatt_chrc) {
        LOG_ERR("Missing sync characteristic");
        return -EINVAL;
    }
    /* Sync */
    gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_BELUGA_SYNC);
    if (!gatt_desc) {
        LOG_ERR("Missing sync value descriptor in characteristic");
        return -EINVAL;
    }
    LOG_DBG("Found handle for sync characteristic");
    client->handles.sync = gatt_desc->handle;

    /* Assign connection instance */
    client->conn = bt_gatt_dm_conn_get(dm);
    return 0;
}

/**
 * @brief Request the peer to start sending notifications for the range
 * characteristic.
 * @param[in,out] client Beluga client instance.
 * @return 0 if the operation was successful
 * @return negative error code otherwise.
 */
int bt_beluga_client_subscribe_ranging(struct bt_beluga_client *client) {
    int err;

    if (!client) {
        return -EINVAL;
    }

    if (atomic_test_and_set_bit(&client->state,
                                BELUGA_C_RANGING_NOTIF_ENABLED)) {
        return -EALREADY;
    }

    client->range_notif_params.notify = on_received;
    client->range_notif_params.value = BT_GATT_CCC_NOTIFY;
    client->range_notif_params.value_handle = client->handles.range;
    client->range_notif_params.ccc_handle = client->handles.range_ccc;
    atomic_set_bit(client->range_notif_params.flags,
                   BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

    err = bt_gatt_subscribe(client->conn, &client->range_notif_params);
    if (err) {
        LOG_ERR("Subscribe failed (err %d)", err);
        atomic_clear_bit(&client->state, BELUGA_C_RANGING_NOTIF_ENABLED);
    } else {
        LOG_DBG("[SUBSCRIBED]");
    }

    return err;
}
