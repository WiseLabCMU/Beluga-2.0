/**
 * @file scan.c
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>

#include <ble/services/beluga_service.h>

#include <ble/ble_app_internal.h>
#include <ble/scan.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ble_app);

#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

static bool data_cb(struct bt_data *data, void *user_data) {
    struct ble_data *_data = user_data;
    struct bt_uuid_128 service = {0};

    switch (data->type) {
    case BT_DATA_UUID128_ALL: {
        bt_uuid_create((struct bt_uuid *)&service, data->data, data->data_len);
        if (bt_uuid_cmp((struct bt_uuid *)&service, BT_UUID_BELUGA_SVC) == 0) {
            _data->beluga_node = true;
        }
        break;
    }
    case BT_DATA_UUID16_ALL: {
        UUID16_EXTRACT(&_data->uuid, data->data);
        break;
    }
    case BT_DATA_MANUFACTURER_DATA: {
        memcpy(_data->manufacturerData, data->data,
               MIN(data->data_len, NAME_LEN));
        break;
    }
    default:
        break;
    }

    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *adv_info) {
    struct ble_data data_;
    bt_data_parse(adv_info, data_cb, &data_);

    if (data_.beluga_node && type == BT_GAP_ADV_TYPE_SCAN_RSP) {
        LOG_DBG("Found node advertising Beluga service: 0x%" PRIX16,
                data_.uuid);
        update_seen_list(&data_, rssi, addr);
    }
}

int start_active_scanning(void) {
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (err) {
        LOG_ERR("Failed to start active scanning (%d)", err);
    }
    return err;
}

int start_passive_scanning(void) {
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err) {
        LOG_ERR("Failed to start passive scanning (%d)", err);
    }
    return err;
}

int stop_scanning(void) {
    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Unable to stop scanning (%d)", err);
    }
    return err;
}
