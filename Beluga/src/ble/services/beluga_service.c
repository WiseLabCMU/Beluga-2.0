/**
 * @file beluga_service.c
 *
 * @brief Server side of the Beluga Service.
 *
 * @date 5/1/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <ble/services/beluga_client.h>
#include <ble/services/beluga_service.h>

/**
 * Logger for the Beluga service
 */
LOG_MODULE_REGISTER(beluga_service, CONFIG_BELUGA_SERVICE_LOG_LEVEL);

/**
 * Beluga Service callbacks for various actions.
 */
static struct beluga_service_cb service_cb;

/**
 * Handler for indicating the ranging notification status has changed.
 * @param[in] attr GATT attributes.
 * @param[in] value The new configuration value.
 */
static void bs_ccc_ranging_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
    ARG_UNUSED(attr);
    if (service_cb.send_range_enabled) {
        LOG_DBG("Notification has been turned %s",
                value == BT_GATT_CCC_NOTIFY ? "on" : "off");
        service_cb.send_range_enabled(value == BT_GATT_CCC_NOTIFY
                                          ? BT_BELUGA_SERVICE_STATUS_ENABLED
                                          : BT_BELUGA_SERVICE_STATUS_DISABLED);
    }
}

/**
 * Handles UWB sync write requests.
 * @param[in] conn The BLE connection that received the write request
 * @param[in] attr BLE GATT attributes
 * @param[in] buf The serial data for the GATT write request
 * @param[in] len The length of the data
 * @param[in] offset The offset of the data
 * @param[in] flags Flags associated with the request
 * @return len upon success
 * @return negative ATT error otherwise
 */
static ssize_t bs_write_uwb_sync(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,
                                 uint8_t flags) {
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    struct beluga_uwb_params configs;

    if (len != BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE) {
        LOG_ERR("Invalid payload length for UWB sync request");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0) {
        LOG_ERR("Invalid offset for UWB sync request");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (service_cb.sync == NULL) {
        LOG_DBG("Service not initialized");
        return len;
    }

    if (deserialize_uwb_configurations(&configs, buf, len) != 0) {
        LOG_ERR("Invalid UWB sync data received");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (service_cb.sync(conn, &configs) != 0) {
        LOG_ERR("UWB sync is not ready");
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    return len;
}

/**
 * Handles range sent events.
 * @param[in] conn The BLE connection the range was sent on
 * @param[in] user_data The data that was sent in the notification
 */
static void on_range_sent(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(user_data);

    LOG_DBG("Range sent, conn %p", (void *)conn);

    if (service_cb.sent_range) {
        service_cb.sent_range(conn);
    }
}

/**
 * Beluga Service declaration
 */
BT_GATT_SERVICE_DEFINE(
    beluga_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_BELUGA_SVC),
    BT_GATT_CHARACTERISTIC(BT_UUID_BELUGA_RANGE, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bs_ccc_ranging_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_BELUGA_SYNC, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, NULL, bs_write_uwb_sync,
                           NULL), );

/**
 * Initializes the service.
 *
 * @param[in] cb Struct with function pointers to callbacks for service events.
 * @return 0 upon success
 * @return -EINVAL if no callbacks are specified
 */
int beluga_service_init(struct beluga_service_cb *cb) {
    if (cb == NULL || cb->sync == NULL) {
        return -EINVAL;
    }
    service_cb.sync = cb->sync;
    service_cb.sent_range = cb->sent_range;
    service_cb.send_range_enabled = cb->send_range_enabled;
    return 0;
}

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
int range_notify(struct bt_conn *conn, uint16_t id, float range) {
    struct bt_gatt_notify_params params = {0};
    const struct bt_gatt_attr *attr = &beluga_svc.attrs[2];
    static char data[sizeof(id) + sizeof(range)];

    memcpy(data, &id, sizeof(id));
    memcpy(data + sizeof(id), &range, sizeof(range));

    params.attr = attr;
    params.data = data;
    params.len = sizeof(data);
    params.func = on_range_sent;

    if (!conn) {
        LOG_DBG("Notification sent to all connected peers");
        return bt_gatt_notify_cb(NULL, &params);
    } else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
        return bt_gatt_notify_cb(conn, &params);
    }
    return -EINVAL;
}
