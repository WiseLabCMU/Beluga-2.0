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
#include <stdbool.h>
#include <stdint.h>
#include <utils.h>

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream.
 */
#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

#define NAME_LEN 30
#define UUID_INDEX 2
#define MANF_INDEX 3
#define POLLING_FLAG_INDEX 2

enum adv_mode {
    ADVERTISING_CONNECTABLE,
    ADVERTISING_NONCONNECTABLE,
    ADVERTISING_OFF
};

static uint16_t NODE_UUID = BT_UUID_HRS_VAL;

static char const m_target_peripheral_name[] = "BN ";

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)), /* Heart Rate Service */
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
};

static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME)};

static bool bluetooth_on = false;
static struct bt_conn *central_conn;
static uint64_t timekeeper;

bool node_added = false;
static int32_t last_seen_index = 0;
node seen_list[MAX_ANCHOR_COUNT];
static enum adv_mode currentAdvMode = ADVERTISING_OFF;

struct ble_data {
    char name[NAME_LEN];
    uint16_t uuid;
    uint8_t manufacturerData[NAME_LEN];
};

static bool data_cb(struct bt_data *data, void *user_data) {
    struct ble_data *_data = user_data;

    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        memcpy(_data->name, data->data, MIN(data->data_len, NAME_LEN - 1));
        break;
    case BT_DATA_UUID16_ALL:
        UUID16_EXTRACT(&(_data->uuid), data->data);
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

ALWAYS_INLINE bool in_seen_list(uint16_t uuid) {
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
    seen_list[last_seen_index].ble_time_stamp = timekeeper;
    if (data->manufacturerData[POLLING_FLAG_INDEX] == '0') {
        seen_list[last_seen_index].polling_flag = 0;
    } else if (data->manufacturerData[POLLING_FLAG_INDEX] == '1') {
        seen_list[last_seen_index].polling_flag = 1;
    }

    last_seen_index = (last_seen_index + 1) % MAX_ANCHOR_COUNT;
    node_added = true;
}

static void update_seen_neighbor(struct ble_data *data, int8_t rssi) {
    int32_t index = get_seen_list_index(data->uuid);
    seen_list[index].RSSI = rssi;
    seen_list[index].ble_time_stamp = timekeeper;

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

    if (strncmp(_data.name, m_target_peripheral_name, 3) == 0) {
        update_seen_list(&_data, rssi);
    }
}

static void scan_start(void) {
    LED_OFF(CENTRAL_SCANNING_LED);

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found_callback);

    if (err != 0) {
        printk("Scanning failed to start (err %d)\n", err);
    }
}

static int32_t adv_scan_start(void) {
    if (bluetooth_on) {
        int32_t err;

        LED_ON(CENTRAL_SCANNING_LED);

        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
                              ARRAY_SIZE(sd));

        if (err != 0) {
            printk("Advertising failed to start (err %d)\n", err);
            LED_OFF(CENTRAL_SCANNING_LED);
            currentAdvMode = ADVERTISING_OFF;
            return 1;
        }
        currentAdvMode = ADVERTISING_CONNECTABLE;
        return 0;
    }
    return 1;
}

static void adv_no_connect_start(void) {
    if (bluetooth_on) {
        int32_t err;

        LED_ON(CENTRAL_SCANNING_LED);

        err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

        if (err != 0) {
            printk("Advertising failed to start (err %d)\n", err);
            LED_OFF(CENTRAL_SCANNING_LED);
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
    printk("Heart Rate Service could not be found during the discovery\n");
}

static void discovery_error_found_cb(UNUSED struct bt_conn *conn, int err,
                                     UNUSED void *context) {
    printk("The discovery procedure failed with %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_not_found_cb,
    .error_found = discovery_error_found_cb};

static void gatt_discover(struct bt_conn *conn) {
    int err;

    err = bt_gatt_dm_start(conn, BT_UUID_GAP, &discovery_cb, NULL);

    if (err) {
        printk("Could not start the discovery procedure, error "
               "code: %d\n",
               err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params) {
    if (!err) {
        printk("MTU exchange done\n");
    } else {
        printk("MTU exchange failed (err %d)\n", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err) {
    int err;
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        printk("Failed to connect to %s (%u)\n", addr, conn_err);

        if (conn == central_conn) {
            bt_conn_unref(central_conn);
            central_conn = NULL;

            scan_start();
        }

        return;
    }

    printk("Connected: %s\n", addr);

    static struct bt_gatt_exchange_params exchange_params;
    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err != 0) {
        printk("MTU exchange failed (err %d)\n", err);
    }

    bt_conn_get_info(conn, &info);

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        LED_ON(CENTRAL_CONNECTED_LED);

        err = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (err) {
            printk("Failed to set security (err %d)\n", err);

            gatt_discover(conn);
        }
    } else {
        LED_ON(PERIPHERAL_CONNECTED_LED);
        bt_le_adv_stop();
        adv_no_connect_start();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason %u)\n", addr, reason);

    if (conn == central_conn) {
        LED_OFF(CENTRAL_CONNECTED_LED);

        bt_conn_unref(central_conn);
        central_conn = NULL;

        scan_start();
    } else {
        LED_OFF(PERIPHERAL_CONNECTED_LED);
        bt_le_adv_stop();
        adv_scan_start();
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        printk("Security failed: %s level %u err %d\n", addr, level, err);
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

    printk("Filters matched. Address: %s connectable: %d\n", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info) {
    printk("Connecting failed\n");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn) {
    central_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error,
                scan_connecting);

static void scan_init(void) {
    int err;

    struct bt_scan_init_param param = {.scan_param = NULL,
                                       .conn_param = BT_LE_CONN_PARAM_DEFAULT,
                                       .connect_if_match = 0};

    bt_scan_init(&param);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
    if (err) {
        printk("Scanning filters cannot be set (err %d)\n", err);
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        printk("Filters cannot be turned on (err %d)\n", err);
    }
}

void update_node_id(uint16_t uuid) {
    static uint8_t uuid_encoded[2];
    static uint8_t advName[NAME_LEN];
    memcpy(uuid_encoded, &uuid, sizeof(uuid));
    size_t len = snprintf(advName, NAME_LEN, "%s%d", m_target_peripheral_name, uuid);
    struct bt_data uuid_data = {.type = BT_DATA_UUID16_ALL, .data = uuid_encoded, .data_len = 2};
    struct bt_data name_data = {.type = BT_DATA_NAME_COMPLETE, .data = advName, .data_len = len};

    NODE_UUID = uuid;

    if (currentAdvMode != ADVERTISING_OFF) {
        bt_le_adv_stop();
    }

    ad[UUID_INDEX] = uuid_data;
    sd[0] = name_data;

    switch(currentAdvMode) {
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

void advertising_reconfig(int32_t change) {
    struct bt_data data_poll_0 = BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"), data_poll_1 = BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x31");

    bt_le_adv_stop();

    if (change == 0) {
        ad[MANF_INDEX] = data_poll_0;
    } else {
        ad[MANF_INDEX] = data_poll_1;
    }

    switch(currentAdvMode) {
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

int32_t init_bt_stack(void) {
    int32_t err;

    err = bt_enable(NULL);
    if (err) {
        return 1;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    scan_init();

    scan_start();

    printk("Scanning started\n");

    LED_OFF(PERIPHERAL_ADVERTISING_LED);

    bluetooth_on = true;
    if (!adv_scan_start()) {
        LED_ON(PERIPHERAL_ADVERTISING_LED);
    }

    printk("Advertising started\n");

    return 0;
}

void disable_bluetooth(void) {
    int err;

    err = bt_le_adv_stop();

    if (err) {
        printk("Failed to stop advertising (err %d)\n", err);
    }

    err = bt_le_scan_stop();

    if (err) {
        printk("Failed to stop scanning (err %d)\n", err);
    }

    err = bt_disable();

    if (err) {
        printk("Failed to disable bluetooth (err %d)\n", err);
    }

    currentAdvMode = ADVERTISING_OFF;
    bluetooth_on = false;
}

void enable_bluetooth(void) {
    init_bt_stack();
}
