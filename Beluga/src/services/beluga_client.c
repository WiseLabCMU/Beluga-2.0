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
