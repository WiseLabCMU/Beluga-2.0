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

#include <settings.h>

#include <range_extension.h>
#include <zephyr/logging/log.h>

#include <services/beluga_client.h>
#include <services/beluga_service.h>

LOG_MODULE_REGISTER(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

/**
 * Mutex for the UUID
 */
K_MUTEX_DEFINE(UUID_mutex);

/**
 * Semaphore for locking and unlocking the BLE on/off state
 */
K_SEM_DEFINE(ble_state, 1, 1);

/**
 * @brief Macro to unpack 16bit unsigned UUID from an octet stream.
 *
 * @param[out] DST Pointer to location where the UUID is stored
 * @param[in] SRC Array that holds the UUID
 */
#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

/**
 * The service UUID that Beluga uses
 */
#define BELUGA_SERVICE_UUID BT_UUID_HRS_VAL

/**
 * The index the UUID is stored at in the advertising data
 */
#define UUID_INDEX 4

/**
 * The index the manufacturer data is stored at in the advertising data
 */
#define MANF_INDEX                      3

#define BLE_PAN_OVERHEAD                2
#define BLE_UWB_METADATA_OVERHEAD       2
#define BLE_MANF_DATA_OVERHEAD          (BLE_PAN_OVERHEAD + BLE_UWB_METADATA_OVERHEAD)

#define BLE_PAN_OFFSET                  0
#define BLE_UWB_METADATA_OFFSET         (BLE_PAN_OFFSET + BLE_PAN_OVERHEAD)
#define BLE_UWB_METADATA_CHANNEL_BYTE   BLE_UWB_METADATA_OFFSET
#define BLE_UWB_METADATA_TWR_BYTE       BLE_UWB_METADATA_OFFSET
#define BLE_UWB_METADATA_SFD_BYTE       BLE_UWB_METADATA_OFFSET
#define BLE_UWB_METADATA_DATARATE_BYTE  BLE_UWB_METADATA_OFFSET
#define BLE_UWB_METADATA_PULSERATE_BYTE BLE_UWB_METADATA_OFFSET
#define BLE_UWB_METADATA_PHR_BYTE       (BLE_UWB_METADATA_OFFSET + 1)
#define BLE_UWB_METADATA_PREAMBLE_BYTE  BLE_UWB_METADATA_PHR_BYTE
#define BLE_UWB_METADATA_PAC_BYTE       BLE_UWB_METADATA_PHR_BYTE
#define BLE_UWB_METADATA_POLLING_BYTE   BLE_UWB_METADATA_PHR_BYTE
#define BLE_UWB_METADATA_ACTIVE_BYTE    BLE_UWB_METADATA_PHR_BYTE

#define UWB_CHANNEL_SHIFT               0
#define UWB_TWR_SHIFT                   3
#define UWB_SFD_SHIFT                   4
#define UWB_DATARATE_SHIFT              5
#define UWB_PULSERATE_SHIFT             7

#define UWB_PHR_SHIFT                   0
#define UWB_PREAMBLE_SHIFT              1
#define UWB_PAC_SHIFT                   4
#define UWB_POLL_SHIFT                  6
#define UWB_ACTIVE_SHIFT                7

#define UWB_CHANNEL_MASK                UINT8_C(0x7)
#define UWB_TWR_MASK                    (UINT8_C(0x1) << UWB_TWR_SHIFT)
#define UWB_SFD_MASK                    (UINT8_C(0x1) << UWB_SFD_SHIFT)
#define UWB_DATARATE_MASK               (UINT8_C(0x3) << UWB_DATARATE_SHIFT)
#define UWB_PULSERATE_MASK              (UINT8_C(0x1) << UWB_PULSERATE_SHIFT)

#define UWB_PHR_MASK                    UINT8_C(0x1)
#define UWB_PREAMBLE_MASK               (UINT8_C(0x7) << UWB_PREAMBLE_SHIFT)
#define UWB_PAC_MASK                    (UINT8_C(0x3) << UWB_PAC_SHIFT)
#define UWB_POLLING_MASK                (UINT8_C(0x1) << UWB_POLL_SHIFT)
#define UWB_ACTIVE_MASK                 (UINT8_C(0x1) << UWB_ACTIVE_SHIFT)

#define SET_UWB_METADATA(meta, field)                                          \
    set_uwb_metadata(BLE_UWB_METADATA_##field##_BYTE, UWB_##field##_MASK,      \
                     UWB_##field##_SHIFT, (meta)->field)

/**
 * The different advertising modes Beluga can be in
 */
enum adv_mode {
    ADVERTISING_CONNECTABLE,    ///< Advertising as a connectable node
    ADVERTISING_NONCONNECTABLE, ///< Advertising as a node that cannot be
                                ///< connected to
    ADVERTISING_OFF             ///< Advertising is turned off
};

struct bt_connect_work {
    bt_addr_le_t addr;
    struct k_work work;
};

/**
 * The node identifier
 */
static uint16_t NODE_UUID = 0;

/**
 * Prefix for the node advertising name
 */
static char const m_target_peripheral_name[] = "BN ";

/**
 * The BLE advertising data
 */
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

/**
 * The BLE scan response data
 */
static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME)};

/**
 * Flag indicating that the bluetooth is on
 */
static bool bluetooth_on = false;

/**
 * Bluetooth connection data
 */
static struct bt_conn *central_conn;

/**
 * The neighbor list
 */
struct node seen_list[MAX_ANCHOR_COUNT];

/**
 * The current BLE advertising mode
 */
static enum adv_mode currentAdvMode = ADVERTISING_OFF;

static struct bt_beluga_client client;

static struct bt_connect_work connect_work;

static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};
static struct bt_data beluga_data;

//// Advertising data
static inline void set_ble_pan(uint16_t pan) {
    memcpy(beluga_manufacturer_data + BLE_PAN_OFFSET, &pan, BLE_PAN_OVERHEAD);
}

static inline void set_uwb_metadata(size_t byte, uint8_t mask, uint8_t shift,
                                    uint8_t value) {
    uint8_t val = beluga_manufacturer_data[byte];
    val &= ~mask;
    val |= (value << shift) & mask;
    beluga_manufacturer_data[byte] = val;
}

static void update_manufacturer_info(struct advertising_info *uwb_metadata) {
    uint8_t preamble;
    set_ble_pan(uwb_metadata->pan);
    SET_UWB_METADATA(uwb_metadata, CHANNEL);
    SET_UWB_METADATA(uwb_metadata, TWR);
    SET_UWB_METADATA(uwb_metadata, SFD);
    SET_UWB_METADATA(uwb_metadata, DATARATE);
    SET_UWB_METADATA(uwb_metadata, PULSERATE);
    SET_UWB_METADATA(uwb_metadata, PHR);
    SET_UWB_METADATA(uwb_metadata, PAC);
    SET_UWB_METADATA(uwb_metadata, ACTIVE);

    switch (uwb_metadata->preamble) {
    case 64: {
        preamble = 0;
        break;
    }
    case 128: {
        preamble = 1;
        break;
    }
    case 256: {
        preamble = 2;
        break;
    }
    case 512: {
        preamble = 3;
        break;
    }
    case 1024: {
        preamble = 4;
        break;
    }
    case 1536: {
        preamble = 5;
        break;
    }
    case 2048: {
        preamble = 6;
        break;
    }
    case 4096: {
        preamble = 7;
        break;
    }
    default:
        __ASSERT_UNREACHABLE;
    }

    set_uwb_metadata(BLE_UWB_METADATA_PREAMBLE_BYTE, UWB_PREAMBLE_MASK,
                     UWB_PREAMBLE_SHIFT, preamble);
    set_uwb_metadata(BLE_UWB_METADATA_POLLING_BYTE, UWB_POLLING_MASK,
                     UWB_POLL_SHIFT, uwb_metadata->poll_rate != 0);
}

/**
 * Sets the node ID
 * @param[in] uuid The new node ID
 */
ALWAYS_INLINE static void set_NODE_UUID(uint16_t uuid) { NODE_UUID = uuid; }

/**
 * Retrieves the current node ID
 * @return The current node ID
 */
uint16_t get_NODE_UUID(void) { return NODE_UUID; }

////

//// Client stuff
/**
 * Callback indicating that GATT discovery was completed
 * @param[in] dm GATT discovery manager
 * @param[in] context Additional context
 */
static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context) {
    int err;
    struct bt_beluga_client *bsc = context;

    LOG_INF("The discovery procedure succeeded");

    bt_gatt_dm_data_print(dm);

    err = bt_beluga_client_assign(dm, bsc);
    if (err) {
        LOG_INF("Could not init Beluga client object (err %d)", err);
        return;
    }
    // If we want range, then we subscribe here
    // bt_beluga_client_subscribe_ranging(bsc);

    err = bt_gatt_dm_data_release(dm);
    if (err) {
        LOG_INF("Could not release the discovery data (err %d)", err);
    }
}

/**
 * Callback for handling service not found problems
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] context Additional context
 */
static void discovery_not_found_cb(struct bt_conn *conn, void *context) {
    LOG_INF("Beluga Service could not be found during the discovery");
}

/**
 * Callback for handling GATT discovery errors
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] err The error code
 * @param[in] context Additional context
 */
static void discovery_error_found_cb(struct bt_conn *conn, int err,
                                     void *context) {
    LOG_INF("The discovery procedure failed with %d", err);
}

static void beluga_uwb_settings_synced(struct bt_beluga_client *bt_client,
                                       uint8_t err,
                                       const struct beluga_uwb_params *config) {

}

static int init_beluga_client(void) {
    struct bt_beluga_client_cb cb = {
        .synced = beluga_uwb_settings_synced,
    };

    return bt_beluga_client_init(&client, &cb);
}

int sync_uwb_parameters(uint16_t id) {
    struct beluga_uwb_params config;
    int ret;
    // TODO: Put this stuff on the work q
    config.TX_POWER = retrieveSetting(BELUGA_TX_POWER);
    config.PREAMBLE = (uint16_t)retrieveSetting(BELUGA_UWB_PREAMBLE);
    config.POWER_AMP = (uint8_t)retrieveSetting(BELUGA_RANGE_EXTEND);
    config.CHANNEL = (uint8_t)retrieveSetting(BELUGA_UWB_CHANNEL);
    config.TWR = (bool)retrieveSetting(BELUGA_TWR);
    config.PHR = (bool)retrieveSetting(BELUGA_UWB_PHR);
    config.SFD = (bool)retrieveSetting(BELUGA_UWB_NSSFD);
    config.DATA_RATE = (uint8_t)retrieveSetting(BELUGA_UWB_DATA_RATE);
    config.PAC = (uint8_t)retrieveSetting(BELUGA_UWB_PAC);
    config.PULSE_RATE = (bool)retrieveSetting(BELUGA_UWB_PULSE_RATE);

    // TODO: Signal connect
    // TODO: Wait connection (timeout if one could not be established)
    ret = bt_beluga_client_sync(&client, &config);
    // TODO: Wait sent
    return ret;
}

////

/**
 * Callback function for assisting with scan data parsing
 * @param[in] data The bt data that got parsed by the OS
 * @param[in,out] user_data Pointer to a ble_data struct that stores the parsed
 * data
 * @return true always
 */
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

/**
 * Updates the neighbor list by either, updating or inserting the scanned node
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
static void update_seen_list(struct ble_data *data, int8_t rssi) {
    bool polling;
    if (data->uuid == NODE_UUID) {
        return;
    }

    // Check if node is polling
    polling = (data->manufacturerData[BLE_UWB_METADATA_POLLING_BYTE] &
               UWB_POLLING_MASK) != 0;

    // Filter out nodes that do not have the same settings
    data->manufacturerData[BLE_UWB_METADATA_POLLING_BYTE] &= ~UWB_POLLING_MASK;
    data->manufacturerData[BLE_UWB_METADATA_POLLING_BYTE] |=
        beluga_manufacturer_data[BLE_UWB_METADATA_POLLING_BYTE] &
        UWB_POLLING_MASK;
#if !defined(CONFIG_RANGE_TO_ACTIVE_ONLY)
    // Range to everything advertising
    data->manufacturerData[BLE_UWB_METADATA_ACTIVE_BYTE] &= ~UWB_ACTIVE_MASK;
    data->manufacturerData[BLE_UWB_METADATA_ACTIVE_BYTE] |=
        beluga_manufacturer_data[BLE_UWB_METADATA_POLLING_BYTE] &
        UWB_ACTIVE_MASK;
#endif
    // TODO: Check if syncing. If so, check the ID and submit work if ID matches
    if (memcmp(data->manufacturerData, beluga_manufacturer_data,
               sizeof(beluga_manufacturer_data)) != 0) {
        // UWB parameters do not match
        return;
    }

    if (!in_seen_list(data->uuid)) {
        insert_into_seen_list(data, rssi, polling);
    } else {
        update_seen_neighbor(data, rssi, polling);
    }
}

/**
 * Callback function that is called if a scanned device is found. It will
 * determine if the scanned device is a Beluga node and update the neighbor
 * list if it is
 * @param[in] addr Bluetooth LE device address (unused)
 * @param[in] rssi The RSSI of the scanned device
 * @param[in] type The device type
 * @param[in] adv_info Advertising information for the device
 */
static void device_found_callback(UNUSED const bt_addr_le_t *addr, int8_t rssi,
                                  UNUSED uint8_t type,
                                  struct net_buf_simple *adv_info) {
    struct ble_data _data = {};
    bt_data_parse(adv_info, data_cb, &_data);

    if (_data.beluga_node) {
        update_seen_list(&_data, rssi);
    }
}

/**
 * Starts BLE passive scan. Used for starting scanning after a connection error
 * or after a disconnection.
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int32_t scan_start(void) {
    BLE_LED_OFF(CENTRAL_SCANNING_LED);

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found_callback);

    if (err != 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
    }
    return err;
}

/**
 * Starts advertising as a connectable device and starts active scanning for
 * other nodes
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int32_t adv_scan_start(void) {
    int32_t err;

    BLE_LED_ON(CENTRAL_SCANNING_LED);

    LOG_INF("Start connectable advertising");
    err =
        bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err != 0) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        BLE_LED_OFF(CENTRAL_SCANNING_LED);
        currentAdvMode = ADVERTISING_OFF;
        return err;
    }
    currentAdvMode = ADVERTISING_CONNECTABLE;

    err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found_callback);

    if (err != 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }
    return 0;
}

/**
 * Starts advertising as a non-connectable device
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int adv_no_connect_start(void) {
    if (bluetooth_on) {
        int err;

        BLE_LED_ON(CENTRAL_SCANNING_LED);

        LOG_INF("Start non-connectable advertising");
        err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd,
                              ARRAY_SIZE(sd));

        if (err != 0) {
            LOG_ERR("Advertising failed to start (err %d)", err);
            BLE_LED_OFF(CENTRAL_SCANNING_LED);
            currentAdvMode = ADVERTISING_OFF;
            return err;
        }
        currentAdvMode = ADVERTISING_CONNECTABLE;
    }
    return 0;
}

/**
 * GATT discovery manager callbacks
 */
static const struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_not_found_cb,
    .error_found = discovery_error_found_cb};

/**
 * Starts the GATT discovery process
 * @param[in] conn Pointer to Bluetooth connection data
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int gatt_discover(struct bt_conn *conn) {
    int err;

    err = bt_gatt_dm_start(conn, BT_UUID_GAP, &discovery_cb, NULL);

    if (err) {
        LOG_ERR("Could not start the discovery procedure, error "
                "code: %d",
                err);
        return err;
    }
    return 0;
}

/**
 * Callback for the MTU exchange interaction
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] err The error code
 * @param[in] params Pointer to the GATT exchange MTU parameters
 */
static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params) {
    if (!err) {
        LOG_INF("MTU exchange done\n");
    } else {
        LOG_ERR("MTU exchange failed (err %d)", err);
    }
}

/**
 * Callback function for a bluetooth connection event
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] conn_err Connection error code
 */
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

/**
 * Callback function for a Bluetooth disconnection event
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] reason Code indicating the reason why the disconnection
 * event occurred
 */
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

/**
 * Callback function indicating the security level changed
 * @param[in] conn Pointer to Bluetooth connection data
 * @param[in] level The new security level
 * @param[in] err Security error
 */
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

/**
 * Bluetooth connection event callbacks
 */
BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected,
                                     .security_changed = security_changed};

/**
 * Callback function if a scan filter matched
 * @param[in] device_info Pointer to scanned device data
 * @param[in] filter_match Pointer to filter match data
 * @param[in] connectable Flag indicating that the scanned device is
 * connectable
 */
static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    // LOG_INF("Filters matched. Address: %s connectable: %d", addr,
    // connectable);
}

/**
 * Callback for scanned device connection errors
 * @param[in] device_info Pointer to scanned device data
 */
static void scan_connecting_error(struct bt_scan_device_info *device_info) {
    LOG_INF("Connecting failed");
}

/**
 * Callback for scanned devices connecting
 * @param[in] device_info Pointer to scanned device data
 * @param[in] conn Pointer to Bluetooth connection data
 */
static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn) {
    central_conn = bt_conn_ref(conn);
}

/**
 * Scan callbacks
 */
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error,
                scan_connecting);

/**
 * Initializes the Bluetooth scanning
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int scan_init(void) {
    int err;

    struct bt_scan_init_param param = {.scan_param = NULL,
                                       .conn_param = BT_LE_CONN_PARAM_DEFAULT,
                                       .connect_if_match = 0};

    bt_scan_init(&param);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HRS);
    if (err) {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }
    return 0;
}

/**
 * Initializes the Bluetooth stack
 * @return 0 upon success
 * @return negative error code otherwise
 */
int init_bt_stack(void) {
    int32_t err;

    err = bt_enable(NULL);
    if (err) {
        return err;
    }
    LOG_INF("Bluetooth stack loaded");

    return scan_init();
}

/**
 * Stops Bluetooth scanning and advertising and unloads the bluetooth stack
 * @return 0 upon success
 * @return negative error code otherwise
 */
int deinit_bt_stack(void) {
    bt_le_adv_stop();
    bt_le_scan_stop();

    return bt_disable();
}

/**
 * Starts Bluetooth advertising and scanning and marks the Bluetooth as active
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int _enable_bluetooth(void) {
    int err = adv_scan_start();
    if (err != 0) {
        return err;
    }
    bluetooth_on = true;

    return 0;
}

/**
 * Starts the Bluetooth FEM (if applicable) and starts Bluetooth
 * scanning/advertising
 * @return 0 upon success
 * @return 1 if already on
 * @return -EFAULT if unable to start the FEM
 * @return negative error code otherwise
 */
int enable_bluetooth(void) {
    int retVal = 1;
    k_sem_take(&ble_state, K_FOREVER);
    if (!bluetooth_on) {
        if (update_fem_shutdown_state(false) != 0) {
            retVal = -EFAULT;
        } else {
            retVal = _enable_bluetooth();
        }
    }
    k_sem_give(&ble_state);
    return retVal;
}

/**
 * Stops Bluetooth scanning and advertising
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int _disable_bluetooth(void) {
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

/**
 * Stops Bluetooth advertising/scanning and powers down the Bluetooth FEM
 * @return 0 upon success
 * @return 1 if Bluetooth is already off
 * @return -EFAULT if unable to power down the Bluetooth FEM
 * @return negative error code otherwise
 */
int disable_bluetooth(void) {
    int retVal = 1;
    k_sem_take(&ble_state, K_FOREVER);
    if (bluetooth_on) {
        retVal = _disable_bluetooth();
        if (!retVal && update_fem_shutdown_state(true) != 0) {
            retVal = -EFAULT;
        }
    }
    k_sem_give(&ble_state);
    return retVal;
}

/**
 * Updates the node ID and publishes the update to the Bluetooth advertising
 * data
 * @param[in] uuid The new node ID
 */
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
        bt_le_scan_stop();
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

/**
 * Update the advertising data to indicate that the node is polling UWB or not
 * @param[in] change If 0, updates the advertising data to indicate that the
 * node has stopped polling; otherwise, indicates that the node is polling UWB
 */
void advertising_reconfig(struct advertising_info *uwb_metadata) {
    if (currentAdvMode != ADVERTISING_OFF) {
        bt_le_adv_stop();
        bt_le_scan_stop();
    }

    update_manufacturer_info(uwb_metadata);
    beluga_data.data = beluga_manufacturer_data;
    beluga_data.data_len = sizeof(beluga_manufacturer_data);
    beluga_data.type = BT_DATA_MANUFACTURER_DATA;
    ad[MANF_INDEX] = beluga_data;

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

/**
 * Checks if Bluetooth is currently advertising/scanning
 * @return `true` if advertising/scanning
 * @return `false` otherwise
 */
bool check_ble_enabled(void) { return bluetooth_on; }

#if IS_ENABLED(CONFIG_BELUGA_GATT)
#warning "Beluga's GATT service is experimental and may not work"
#define BELUGA_GATT_VAL_LEN 14
void update_ble_service(uint16_t uuid, float range) {
    static uint8_t gatt_value[BELUGA_GATT_VAL_LEN];

    union {
        int32_t i32;
        uint8_t bytes[sizeof(int32_t)];
    } range_int;

    struct bt_gatt_attr attr;
    int err;

    range_int.i32 = (int32_t)range * INT32_C(1000);

    memset(gatt_value, 1, 2);
    memcpy(gatt_value + 2, &uuid, sizeof(uuid));
    memcpy(gatt_value + 4, range_int.bytes, sizeof(range_int));
    gatt_value[8] = 10;
    memset(gatt_value + 9, 0, 5);

    // TODO: Fix parameters 1 and 2
    err = bt_gatt_notify(NULL, NULL, gatt_value, BELUGA_GATT_VAL_LEN);
    if (err != 0) {
        LOG_ERR("Cannot perform gatt notify (%d)", err);
    }
}
#endif

/**
 * Returns the current Bluetooth state and disables scanning/advertising.
 * Additionally, blocks other threads from enabling Bluetooth until the state is
 * restored. See `restore_bluetooth()`
 * @return The current Bluetooth state
 */
bool save_and_disable_bluetooth(void) {
    bool retVal;
    k_sem_take(&ble_state, K_FOREVER);
    retVal = bluetooth_on;
    if (bluetooth_on) {
        _disable_bluetooth();
    }
    return retVal;
}

/**
 * Restores the Bluetooth state to its original setting. Additionally, releases
 * the Bluetooth lock to allow other threads to update the Bluetooth state. See
 * `save_and_disable-bluetooth()`
 * @param[in] state The saved Bluetooth state
 */
void restore_bluetooth(bool state) {
    if (state) {
        _enable_bluetooth();
    }
    k_sem_give(&ble_state);
}
