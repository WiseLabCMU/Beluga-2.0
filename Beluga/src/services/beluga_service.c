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
static bool ranging_enabled = false;

static void bs_ccc_ranging_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
    ranging_enabled = value == BT_GATT_CCC_NOTIFY;
}

static ssize_t bs_write_uwb_sync(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,
                                 uint8_t flags) {
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
