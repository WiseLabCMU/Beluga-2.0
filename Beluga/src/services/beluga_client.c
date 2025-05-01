/**
 * @file beluga_client.c
 *
 * @brief
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <services/beluga_client.h>
#include <services/beluga_service.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(beluga_client, LOG_LEVEL_INF);

enum {
    BELUGA_C_INITIALIZED,
    BELUGA_C_RANGING_NOTIF_ENABLED,
    BELUGA_C_SYNC_PENDING,
};

static uint8_t on_received(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length) {
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
    if (client->cb.range && length == (sizeof(uint16_t) + sizeof(float))) {
        uint16_t id;
        float range;
        memcpy(&id, data, sizeof(uint16_t));
        memcpy(&range, (char *)data + sizeof(uint16_t), sizeof(float));
        return client->cb.range(client, id, range);
    }

    return BT_GATT_ITER_CONTINUE;
}

static void on_sent(struct bt_conn *conn, uint8_t err,
                    struct bt_gatt_write_params *params) {
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
