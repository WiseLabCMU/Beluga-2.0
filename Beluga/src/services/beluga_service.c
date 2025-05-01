/**
 * @file beluga_service.c
 *
 * @brief
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

#include <services/beluga_service.h>
#include <services/beluga_service_common.h>

LOG_MODULE_REGISTER(beluga_service);

static struct beluga_service_cb service_cb;

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

static ssize_t bs_write_uwb_sync(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,
                                 uint8_t flags) {
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    struct beluga_uwb_params configs;

    if (len != BT_BELUGA_SVC_SYNC_PAYLOAD_SIZE) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (service_cb.sync == NULL) {
        return len;
    }

    if (deserialize_uwb_configurations(&configs, buf, len) != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    service_cb.sync(conn, &configs);

    return len;
}

BT_GATT_SERVICE_DEFINE(
    beluga_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_BELUGA_SVC),
    BT_GATT_CHARACTERISTIC(BT_UUID_BELUGA_RANGE, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bs_ccc_ranging_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_BELUGA_SYNC, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, NULL, bs_write_uwb_sync,
                           NULL), );

int beluga_service_init(struct beluga_service_cb *cb) {
    if (cb == NULL || cb->sync == NULL) {
        return -EINVAL;
    }
    service_cb.sync = cb->sync;
    service_cb.sent_range = cb->sent_range;
    service_cb.send_range_enabled = cb->send_range_enabled;
    return 0;
}
