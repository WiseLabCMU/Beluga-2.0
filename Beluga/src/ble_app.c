/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <led_config.h>
#include <utils.h>

#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>

#include <ble_app.h>
#include <list_monitor.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <range_extension.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

K_MUTEX_DEFINE(UUID_mutex);
K_SEM_DEFINE(ble_state, 1, 1);

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream.
 */
#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

#define BELUGA_SERVICE_UUID BT_UUID_HRS_VAL

#define NAME_LEN            30
#define UUID_INDEX          4
#define MANF_INDEX          3
#define POLLING_FLAG_INDEX  2

enum adv_mode {
    ADVERTISING_CONNECTABLE,
    ADVERTISING_NONCONNECTABLE,
    ADVERTISING_OFF
};

static uint16_t NODE_UUID = 0;

static char const m_target_peripheral_name[] = "BN ";

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(
        BT_DATA_UUID16_ALL,
        BT_UUID_16_ENCODE(BELUGA_SERVICE_UUID)), /* Heart Rate Service */
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
};

static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME)};

static const struct bt_data polling_0 =
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30");
static const struct bt_data polling_1 =
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x31");

static bool bluetooth_on = false;
static struct bt_conn *central_conn;

static int32_t last_seen_index = 0;
node seen_list[MAX_ANCHOR_COUNT];
static enum adv_mode currentAdvMode = ADVERTISING_OFF;

struct ble_data {
    uint16_t uuid;
    uint8_t manufacturerData[NAME_LEN];
    bool beluga_node;
};

ALWAYS_INLINE static void set_NODE_UUID(uint16_t uuid) {
    k_mutex_lock(&UUID_mutex, K_FOREVER);
    NODE_UUID = uuid;
    k_mutex_unlock(&UUID_mutex);
}

static bool data_cb(struct bt_data *data, void *user_data) {
    struct ble_data *_data = user_data;
    uint16_t uuid = 0;

    switch (data->type) {
    case BT_DATA_UUID16_ALL:
        UUID16_EXTRACT(&uuid, data->data);
        if (uuid == BELUGA_SERVICE_UUID) {
            _data->beluga_node = true;
        } else {
            _data->uuid = uuid;
        }
        break;
    case BT_DATA_MANUFACTURER_DATA:
        memcpy(_data->manufacturerData, data->data,
               MIN(data->data_len, NAME_LEN - 1));
        break;
    default:
        break;
    }

    return true;
}

static int32_t get_seen_list_index(uint16_t uuid) {
    for (uint32_t i = 0; i < (uint32_t)MAX_ANCHOR_COUNT; i++) {
        if (seen_list[i].UUID == uuid) {
            return (int32_t)i;
        }
    }
    return INT32_C(-1);
}

STATIC_INLINE bool in_seen_list(uint16_t uuid) {
    return get_seen_list_index(uuid) != INT32_C(-1);
}

static void insert_into_seen_list(struct ble_data *data, int8_t rssi) {
    int32_t index = get_seen_list_index(0);
    if (index < 0) {
        // List is full
        index = MAX_ANCHOR_COUNT - 1;
    }
    seen_list[last_seen_index].UUID = data->uuid;
    seen_list[last_seen_index].RSSI = rssi;
    if (IS_ENABLED(CONFIG_BELUGA_EVAL_BLE_STRENGTH)) {
        seen_list[index].update_flag = 1;
    }
    seen_list[last_seen_index].ble_time_stamp = k_uptime_get();
    if (data->manufacturerData[POLLING_FLAG_INDEX] == '0') {
        seen_list[last_seen_index].polling_flag = 0;
    } else if (data->manufacturerData[POLLING_FLAG_INDEX] == '1') {
        seen_list[last_seen_index].polling_flag = 1;
    }

    last_seen_index = (last_seen_index + 1) % MAX_ANCHOR_COUNT;
    node_added();
}

static void update_seen_neighbor(struct ble_data *data, int8_t rssi) {
    int32_t index = get_seen_list_index(data->uuid);
    seen_list[index].RSSI = rssi;
    if (IS_ENABLED(CONFIG_BELUGA_EVAL_BLE_STRENGTH)) {
        seen_list[index].update_flag = 1;
    }
    seen_list[index].ble_time_stamp = k_uptime_get();

    if (data->manufacturerData[POLLING_FLAG_INDEX] == '0') {
        seen_list[last_seen_index].polling_flag = 0;
    } else if (data->manufacturerData[POLLING_FLAG_INDEX] == '1') {
        seen_list[last_seen_index].polling_flag = 1;
    }
}

static void update_seen_list(struct ble_data *data, int8_t rssi) {
    if (data->uuid == NODE_UUID) {
        return;
    }

    if (!in_seen_list(data->uuid)) {
        insert_into_seen_list(data, rssi);
    } else {
        update_seen_neighbor(data, rssi);
    }
}

static void device_found_callback(const bt_addr_le_t *addr, int8_t rssi,
                                  UNUSED uint8_t type,
                                  struct net_buf_simple *adv_info) {
    char addr_str[BT_ADDR_LE_STR_LEN];
    struct ble_data _data;

    memset(&_data, 0, sizeof(_data));

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    bt_data_parse(adv_info, data_cb, &_data);

    if (_data.beluga_node) {
        update_seen_list(&_data, rssi);
    }
}

static int32_t scan_start(void) {
    BLE_LED_OFF(CENTRAL_SCANNING_LED);

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found_callback);

    if (err != 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
    }
    return err;
}

static int32_t adv_scan_start(void) {
    int32_t err;

    BLE_LED_ON(CENTRAL_SCANNING_LED);

    err =
        bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err != 0) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        BLE_LED_OFF(CENTRAL_SCANNING_LED);
        currentAdvMode = ADVERTISING_OFF;
        return 1;
    }
    currentAdvMode = ADVERTISING_CONNECTABLE;

    err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found_callback);

    if (err != 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return 1;
    }
    return 0;
}

static void adv_no_connect_start(void) {
    if (bluetooth_on) {
        int32_t err;

        BLE_LED_ON(CENTRAL_SCANNING_LED);

        err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd,
                              ARRAY_SIZE(sd));

        if (err != 0) {
            LOG_ERR("Advertising failed to start (err %d)", err);
            BLE_LED_OFF(CENTRAL_SCANNING_LED);
            currentAdvMode = ADVERTISING_OFF;
        }
        currentAdvMode = ADVERTISING_CONNECTABLE;
    }
}

static void discovery_completed_cb(struct bt_gatt_dm *dm,
                                   UNUSED void *context) {
    bt_gatt_dm_data_print(dm);

    // TODO: Figure out how to implement this

    bt_gatt_dm_data_release(dm);
}

static void discovery_not_found_cb(UNUSED struct bt_conn *conn,
                                   UNUSED void *context) {
    LOG_WRN("Heart Rate Service could not be found during the discovery");
}

static void discovery_error_found_cb(UNUSED struct bt_conn *conn, int err,
                                     UNUSED void *context) {
    LOG_ERR("The discovery procedure failed with %d", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_not_found_cb,
    .error_found = discovery_error_found_cb};

static void gatt_discover(struct bt_conn *conn) {
    int err;

    err = bt_gatt_dm_start(conn, BT_UUID_GAP, &discovery_cb, NULL);

    if (err) {
        LOG_ERR("Could not start the discovery procedure, error "
                "code: %d",
                err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params) {
    if (!err) {
        LOG_INF("MTU exchange done\n");
    } else {
        LOG_ERR("MTU exchange failed (err %d)", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err) {
    int err;
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, conn_err);

        if (conn == central_conn) {
            bt_conn_unref(central_conn);
            central_conn = NULL;

            scan_start();
        }

        return;
    }

    LOG_INF("Connected: %s", addr);

    static struct bt_gatt_exchange_params exchange_params;
    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err != 0) {
        LOG_ERR("MTU exchange failed (err %d)", err);
    }

    bt_conn_get_info(conn, &info);

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        BLE_LED_ON(CENTRAL_CONNECTED_LED);

        err = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (err) {
            LOG_ERR("Failed to set security (err %d)", err);

            gatt_discover(conn);
        }
    } else {
        BLE_LED_ON(PERIPHERAL_CONNECTED_LED);
        bt_le_adv_stop();
        adv_no_connect_start();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    if (conn == central_conn) {
        BLE_LED_OFF(CENTRAL_CONNECTED_LED);

        bt_conn_unref(central_conn);
        central_conn = NULL;

        scan_start();
    } else {
        BLE_LED_OFF(PERIPHERAL_CONNECTED_LED);
        disable_bluetooth();
        enable_bluetooth();
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", addr, level, err);
    }

    if (conn == central_conn) {
        gatt_discover(conn);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected,
                                     .security_changed = security_changed};

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info) {
    LOG_INF("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn) {
    central_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error,
                scan_connecting);

static int32_t scan_init(void) {
    int err;

    struct bt_scan_init_param param = {.scan_param = NULL,
                                       .conn_param = BT_LE_CONN_PARAM_DEFAULT,
                                       .connect_if_match = 0};

    bt_scan_init(&param);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
    if (err) {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);
        return 1;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return 1;
    }
    return 0;
}

int32_t init_bt_stack(void) {
    int32_t err;

    err = bt_enable(NULL);
    if (err) {
        return 1;
    }
    LOG_INF("Bluetooth stack loaded");

    return scan_init();
}

int32_t deinit_bt_stack(void) {
    bt_le_adv_stop();
    bt_le_scan_stop();

    return bt_disable();
}

static int32_t _enable_bluetooth(void) {
    if (adv_scan_start() != 0) {
        return 1;
    }
    bluetooth_on = true;

    return 0;
}

int32_t enable_bluetooth(void) {
    int32_t retVal = 1;
    k_sem_take(&ble_state, K_FOREVER);
    if (!bluetooth_on) {
        if (!update_fem_shutdown_state(false)) {
            retVal = -EFAULT;
        } else {
            retVal = _enable_bluetooth();
        }
    }
    k_sem_give(&ble_state);
    return retVal;
}

static int32_t _disable_bluetooth(void) {
    int err;
    if ((err = bt_le_adv_stop()) != 0) {
        LOG_ERR("Unable to stop advertising (err: %d)", err);
        return err;
    }
    currentAdvMode = ADVERTISING_OFF;

    if ((err = bt_le_scan_stop()) != 0) {
        LOG_ERR("Unable to stop scanning (err: %d)", err);
        return err;
    }
    bluetooth_on = false;

    return 0;
}

int32_t disable_bluetooth(void) {
    int32_t retVal = 1;
    k_sem_take(&ble_state, K_FOREVER);
    if (bluetooth_on) {
        retVal = _disable_bluetooth();
        if (!retVal && !update_fem_shutdown_state(true)) {
            retVal = -EFAULT;
        }
    }
    k_sem_give(&ble_state);
    return retVal;
}

void update_node_id(uint16_t uuid) {
    static uint8_t uuid_encoded[2];
    static uint8_t advName[NAME_LEN];
    memcpy(uuid_encoded, &uuid, sizeof(uuid));
    size_t len =
        snprintf(advName, NAME_LEN, "%s%d", m_target_peripheral_name, uuid);
    struct bt_data uuid_data = {
        .type = BT_DATA_UUID16_ALL, .data = uuid_encoded, .data_len = 2};
    struct bt_data name_data = {
        .type = BT_DATA_NAME_COMPLETE, .data = advName, .data_len = len};

    set_NODE_UUID(uuid);

    if (currentAdvMode != ADVERTISING_OFF) {
        bt_le_adv_stop();
    }

    ad[UUID_INDEX] = uuid_data;
    sd[0] = name_data;

    switch (currentAdvMode) {
    case ADVERTISING_CONNECTABLE:
        adv_scan_start();
        break;
    case ADVERTISING_NONCONNECTABLE:
        adv_no_connect_start();
        break;
    case ADVERTISING_OFF:
        // do nothing
        break;
    default:
        assert_print("Bad advertising mode");
        break;
    }
}

uint16_t get_NODE_UUID(void) {
    uint16_t retVal;
    k_mutex_lock(&UUID_mutex, K_FOREVER);
    retVal = NODE_UUID;
    k_mutex_unlock(&UUID_mutex);
    return retVal;
}

void advertising_reconfig(int32_t change) {
    bt_le_adv_stop();

    if (change == 0) {
        ad[MANF_INDEX] = polling_0;
    } else {
        ad[MANF_INDEX] = polling_1;
    }

    switch (currentAdvMode) {
    case ADVERTISING_CONNECTABLE:
        adv_scan_start();
        break;
    case ADVERTISING_NONCONNECTABLE:
        adv_no_connect_start();
        break;
    case ADVERTISING_OFF:
        // do nothing
        break;
    default:
        assert_print("Bad advertising mode");
        break;
    }
}

bool check_ble_enabled(void) { return bluetooth_on; }

// void update_ble_service(float range) {
//     union {
//         uint16_t uuid;
//         uint8_t uuid_split[sizeof(uint16_t)];
//     } uuid = {
//             .uuid = NODE_UUID
//     };
//     union {
//         uint8_t bytes[sizeof(uint32_t)];
//         int32_t range;
//     } uRange;
//
//     uRange.range = (int32_t)(range * 1000);
//
//     //bt_gatt_notify(NULL,
// }

bool save_and_disable_bluetooth(void) {
    bool retVal;
    k_sem_take(&ble_state, K_FOREVER);
    retVal = bluetooth_on;
    if (bluetooth_on) {
        _disable_bluetooth();
    }
    return retVal;
}

void restore_bluetooth(bool state) {
    if (state) {
        _enable_bluetooth();
    }
    k_sem_give(&ble_state);
}
