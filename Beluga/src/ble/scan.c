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

LOG_MODULE_DECLARE(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

/**
<<<<<<< HEAD
 * Semaphore for locking the neighbor list.
 */
K_SEM_DEFINE(scanning_allowed, 1, 1);

/**
=======
>>>>>>> master
 * Extracts a 16-bit UUID from a buffer.
 * @param[in] DST Pointer to the integer to store the result in.
 * @param[in] SRC The buffer that contains the 16-bit UUID.
 */
#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

/**
 * Helper function for extracting data from advertising and scan response
 * packets.
 * @param[in] data The Bluetooth data from a packet.
 * @param[in] user_data The BLE data storage structure
 * @return `true` to continue parsing, `false` to stop parsing.
 */
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

/**
 * @brief Callback type for reporting LE scan results.
 *
 * A function of this type is given to the bt_le_scan_start() function
 * and will be called for any discovered LE device.
 *
 * @param[in] addr Advertiser LE address and type.
 * @param[in] rssi Strength of advertiser signal.
 * @param[in] adv_type Type of advertising response from advertiser. Uses the
 * BT_GAP_ADV_TYPE_* values.
 * @param[in] buf Buffer containing advertiser data.
 */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *adv_info) {
    struct ble_data data_ = {0};
    bt_data_parse(adv_info, data_cb, &data_);

    if (data_.beluga_node && type == BT_GAP_ADV_TYPE_SCAN_RSP) {
        LOG_DBG("Found node advertising Beluga service: %d", data_.uuid);
<<<<<<< HEAD

        if (k_sem_take(&scanning_allowed, K_NO_WAIT)) {
            return;
        }

        update_seen_list(&data_, rssi);
        k_sem_give(&scanning_allowed);
=======
        update_seen_list(&data_, rssi);
>>>>>>> master
    } else if (data_.beluga_node && type == BT_GAP_ADV_TYPE_ADV_IND) {
        LOG_DBG("Found connectable node: %d", data_.uuid);
        check_advertiser(&data_, addr);
    }
}

/**
 * Starts scanning for other BLE devices.
 * @return 0 upon success.
 * @return negative error code on error.
 * @note This must be called after advertising is started.
 */
int start_active_scanning(void) {
    LOG_DBG("Starting active scanning");
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (err) {
        LOG_ERR("Failed to start active scanning (%d)", err);
    }
    return err;
}

/**
 * Stops scanning for other BLE devices.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int stop_scanning(void) {
    LOG_DBG("Stopping scanning");
    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Unable to stop scanning (%d)", err);
    }
    return err;
}
<<<<<<< HEAD

/**
 * Suspends neighbor list updates from BLE.
 */
void suspend_scanning(void) { k_sem_take(&scanning_allowed, K_FOREVER); }

/**
 * Resumes neighbor list updates from BLE.
 */
void resume_scanning(void) { k_sem_give(&scanning_allowed); }
=======
>>>>>>> master
