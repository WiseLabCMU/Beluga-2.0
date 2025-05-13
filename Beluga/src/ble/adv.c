/**
 * @file adv.c
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include <list_monitor.h>
#include <range_extension.h>

#include <ble/ble_app.h>
#include <ble/ble_app_internal.h>
#include <ble/scan.h>
#include <ble/services/beluga_service.h>

LOG_MODULE_DECLARE(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

/**
 * Index for the non-connectable advertising handle
 */
#define NCONN_ADV_IDX 0

/**
 * Index for the connectable advertising handle
 */
#define CONN_ADV_IDX 1

/**
 * Advertising name index in the non-connectable advertising data
 */
#define NCONN_ADV_NAME_IDX 1

/**
 * Manufacturer data index in the non-connectable scan data
 */
#define NCONN_SCAN_MANF_DATA_IDX 0

/**
 * Node UUID index in the non-connectable scan data
 */
#define NCONN_SCAN_UUID_IDX 1

/**
 * Node UUID index in the connectable advertising data
 */
#define CONN_ADV_UUID_IDX 2

/**
 * Manufacturer data index in the connectable advertising data
 */
#define CONN_ADV_MANF_DATA_IDX 3

/**
 * Advertising name index in the connectable scan data
 */
#define CONN_SCAN_NAME_IDX 0

/**
 * Helper macro for setting UWB metadata in the manufacturer data.
 * @param[in] meta The struct that contains the metadata.
 * @param[in] field The struct field being encoded.
 */
#define SET_UWB_METADATA(meta, field)                                          \
    set_uwb_metadata(BLE_UWB_METADATA_##field##_BYTE, UWB_##field##_MASK,      \
                     UWB_##field##_SHIFT, (meta)->field)

/**
 * Advertising states for the BLE
 */
enum advertising_state {
    ADVERTISING_OFF,         ///< Advertising disabled
    ADVERTISING_NO_CONNECT,  ///< Non-connectable advertising only
    ADVERTISING_CONNECTABLE, ///< Both connectable and non-connectable
                             ///< advertising
};

/**
 * Semaphore that indicates the BLE connection state
 */
static K_SEM_DEFINE(ble_connect_status, 1, 1);

static void advertising_work_handle(struct k_work *work);

/**
 * Work for starting connectable advertising after disconnecting as a
 * peripheral.
 */
static K_WORK_DEFINE(advertising_work, advertising_work_handle);

static void restart_ble_work_handle(struct k_work *work);

/**
 * Work for restarting BLE scanning and advertising after being connected as a
 * central.
 */
static K_WORK_DEFINE(restart_ble_work, restart_ble_work_handle);

/**
 * Advertising sets
 */
static struct bt_le_ext_adv *ext_adv[CONFIG_BT_EXT_ADV_MAX_ADV_SET];

/**
 * Advertising parameters for the non-connectable advertising set
 */
static const struct bt_le_adv_param *nconn_adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_SCANNABLE, 0x140, 0x190, NULL);

/**
 * Advertising parameters for the connectable advertising set
 */
static const struct bt_le_adv_param *conn_adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2, NULL);

/**
 * Non-connectable advertising data
 */
static struct bt_data nconn_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
};

/**
 * Connectable advertising data
 */
static struct bt_data conn_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
};

/**
 * Non-connectable scan data
 */
static struct bt_data nconn_sd_data[] = {
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
};

/**
 * Connectable scan data
 */
static struct bt_data conn_sd_data[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
};

/**
 * Reference to a central connection.
 */
static struct bt_conn *central_conn = NULL;

/**
 * Reference to a peripheral connection.
 */
static struct bt_conn *peripheral_conn = NULL;

/**
 * The manufacturer data for the node
 */
static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};

/**
 * The advertising name prefix
 */
static char const m_target_peripheral_name[] = "BN ";

/**
 * The advertising display name
 */
static char adv_name[10];

/**
 * The buffer encoded representation of the node UUID.
 */
static uint8_t uuid_encoded[sizeof(uint16_t)];

/**
 * The current advertising state
 */
static enum advertising_state adv_state = ADVERTISING_OFF;

/**
 * The node's unique ID on the network.
 */
static uint16_t NODE_UUID = 0;

/**
 * Handler for when the advertising set connects as a peripheral
 * @param[in] adv The advertising set that connected
 * @param[in] info The connection info
 */
static void adv_connected_cb(struct bt_le_ext_adv *adv,
                             struct bt_le_ext_adv_connected_info *info) {
    ARG_UNUSED(adv);
    ARG_UNUSED(info);
    LOG_INF("Advertiser[%d] %p connected conn %p", bt_le_ext_adv_get_index(adv),
            adv, info->conn);
}

/**
 * Connectable advertising callbacks
 */
static const struct bt_le_ext_adv_cb adv_cb = {
    .connected = adv_connected_cb,
};

/**
 * Starts connectable advertising and starts scanning
 */
static void connectable_adv_start(void) {
    LOG_DBG("Starting connectable advertising from work");
    int err =
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
    adv_state =
        (err == 0 || err == -EALREADY) ? ADVERTISING_CONNECTABLE : adv_state;
    if (err) {
        LOG_ERR(
            "Advertising work: Failed to start connectable advertising (%d)",
            err);
    }
    start_active_scanning();
}

/**
 * Work handle for starting connectable advertising after a disconnection
 * @param[in] work Work item
 */
static void advertising_work_handle(struct k_work *work) {
    ARG_UNUSED(work);
    connectable_adv_start();
}

/**
 * BLE connection handler. Handles the event when a connection is established,
 * either as a peripheral or a central device.
 * @param[in] conn Pointer to the connection
 * @param[in] conn_err Connection error
 */
static void connected(struct bt_conn *conn, uint8_t conn_err) {
    int err;
    struct bt_conn_info info;

    if (conn_err) {
        LOG_ERR("Failed to connect (%u) %s", conn_err,
                bt_hci_err_to_str(conn_err));

        if (conn == central_conn) {
            bt_conn_unref(conn);
            central_conn = NULL;
        }
        start_active_scanning();
        k_poll_signal_raise(
            &connect_signalling.connect_signals[CONNECT_CONNECTED], 1);
        return;
    }

    LOG_INF("Connected");
    err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info (%d)", err);
    }

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        LOG_INF("Connected as central");
        gatt_discover(conn);
    } else {
        adv_state = ADVERTISING_NO_CONNECT;
        k_sem_take(&ble_connect_status, K_NO_WAIT);
        peripheral_conn = conn;
        bt_conn_ref(conn);
        LOG_INF("Connected as peripheral");
    }
}

/**
 * BLE disconnection handler. Handles the event when a BLE connection gets
 * terminated, either as a peripheral or a central device.
 * @param[in] conn The connection that got terminated
 * @param[in] reason The reason why the connection got terminated
 */
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02X %s", addr, reason,
            bt_hci_err_to_str(reason));
    ARG_UNUSED(reason);

    bt_conn_get_info(conn, &info);
    if (info.role == BT_CONN_ROLE_CENTRAL) {
        bt_conn_unref(central_conn);
        central_conn = NULL;
        k_work_submit(&restart_ble_work);
    } else {
        bt_le_scan_stop();
        k_work_submit(&advertising_work);
        bt_conn_unref(peripheral_conn);
        peripheral_conn = NULL;
        k_sem_give(&ble_connect_status);
    }
}

/**
 * BLE connection callbacks
 */
BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected};

/**
 * Helper for creating new advertising sets
 * @param[in,out] adv Pointer to the advertising set being created
 * @param[in] param Advertising parameters
 * @param[in] ad Advertising data
 * @param[in] ad_len Advertising data len (in elements)
 * @param[in] sd Scan data
 * @param[in] sd_len Scan data length (in elements)
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int advertising_set_create(struct bt_le_ext_adv **adv,
                                  const struct bt_le_adv_param *param,
                                  const struct bt_data *ad, size_t ad_len,
                                  const struct bt_data *sd, size_t sd_len) {
    int err;
    struct bt_le_ext_adv *adv_set;

    err = bt_le_ext_adv_create(param, &adv_cb, adv);

    if (err) {
        LOG_ERR("Failed to create advertising set %d", err);
        return err;
    }

    adv_set = *adv;

    LOG_INF("Created adv: %p", adv_set);

    err = bt_le_ext_adv_set_data(adv_set, ad, ad_len, sd, sd_len);
    if (err) {
        LOG_ERR("Failed to set advertising data (%d)", err);
        return err;
    }

    return 0;
}

/**
 * Creates the non-connectable advertising set
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int non_connectable_adv_create(void) {
    int err = advertising_set_create(&ext_adv[NCONN_ADV_IDX], nconn_adv_param,
                                     nconn_ad_data, ARRAY_SIZE(nconn_ad_data),
                                     nconn_sd_data, ARRAY_SIZE(nconn_sd_data));

    if (err) {
        LOG_ERR("Failed to create a non-connectable advertising set (%d)", err);
        return err;
    }

    return 0;
}

/**
 * Creates the connectable advertising set
 * @return 0 upon success
 * @return negative error code otherwise
 */
static int connectable_adv_create(void) {
    int err = advertising_set_create(&ext_adv[CONN_ADV_IDX], conn_adv_param,
                                     conn_ad_data, ARRAY_SIZE(conn_ad_data),
                                     conn_sd_data, ARRAY_SIZE(conn_sd_data));

    if (err) {
        LOG_ERR("Failed to create a connectable advertising set (%d)", err);
    }

    return err;
}

/**
 * Initializes the advertising sets for Beluga
 * @return 0 upon success
 * @return negative error code otherwise
 */
int init_advertising(void) {
    int err;

    err = non_connectable_adv_create();
    if (err) {
        return err;
    }

    return connectable_adv_create();
}

/**
 * Stops connectable and non-connectable advertising.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int stop_advertising(void) {
    int err;
    LOG_DBG("Attempting to stop advertising");

    err = bt_le_ext_adv_stop(ext_adv[CONN_ADV_IDX]);
    if (err) {
        LOG_ERR("Unable to stop advertising for the non-connectable set (%d)",
                err);
        return err;
    }

    err = bt_le_ext_adv_stop(ext_adv[NCONN_ADV_IDX]);
    if (err) {
        LOG_ERR("Unable to stop advertising for the non-connectable set (%d)",
                err);
        return err;
    }

    return 0;
}

/**
 * Starts connectable and non-connectable advertising. Does nothing if already
 * advertising.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int start_advertising(void) {
    int err;
    LOG_DBG("Attempting to start advertising");

    err = bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX],
                              BT_LE_EXT_ADV_START_DEFAULT);
    if (err != 0 && err != -EALREADY) {
        LOG_ERR("Unable to start non-connectable advertising (%d)", err);
        return err;
    }
    adv_state = ADVERTISING_NO_CONNECT;

    err =
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
    if (err != 0 && err != -EALREADY) {
        LOG_ERR("Unable to start connectable advertising (%d)", err);
        return err;
    }
    adv_state = ADVERTISING_CONNECTABLE;

    return 0;
}

/**
 * Refreshes the advertising and scan data for both the non-connectable and the
 * connectable advertising sets.
 */
static void refresh_advertising_data(void) {
    int err;

    err = bt_le_ext_adv_set_data(ext_adv[NCONN_ADV_IDX], nconn_ad_data,
                                 ARRAY_SIZE(nconn_ad_data), nconn_sd_data,
                                 ARRAY_SIZE(nconn_sd_data));
    if (err) {
        LOG_ERR("Unable to update non-connectable advertising data (%d)", err);
    }

    err = bt_le_ext_adv_set_data(ext_adv[CONN_ADV_IDX], conn_ad_data,
                                 ARRAY_SIZE(conn_ad_data), conn_sd_data,
                                 ARRAY_SIZE(conn_sd_data));
    if (err) {
        LOG_ERR("Unable to update connectable advertising data (%d)", err);
    }
}

/**
 * Sets the PAN ID for the BLE manufacturer data
 * @param[in] pan The new PAN ID
 */
static inline void set_ble_pan(uint16_t pan) {
    memcpy(beluga_manufacturer_data + BLE_PAN_OFFSET, &pan, BLE_PAN_OVERHEAD);
}

/**
 * Generic function for setting UWB metadata in the manufacturer data
 * @param[in] byte The byte being modified
 * @param[in] mask The bitmask for the field being modified
 * @param[in] shift The left shift needed to get to the bit field of the UWB
 * metadata
 * @param[in] value The value to write to the bitfield
 */
static inline void set_uwb_metadata(size_t byte, uint8_t mask, uint8_t shift,
                                    uint8_t value) {
    uint8_t val = beluga_manufacturer_data[byte];
    val &= ~mask;
    val |= (value << shift) & mask;
    beluga_manufacturer_data[byte] = val;
}

/**
 * Helper function for updating the manufacturer info
 * @param[in] uwb_metadata The node's UWB metadata
 */
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
 * Updates the BLE manufacturer data with the new UWB metadata
 * @param[in] uwb_metadata The new UWB metadata
 */
void advertising_reconfig(struct advertising_info *uwb_metadata) {
    struct bt_data manf_info;

    LOG_DBG("Reconfiguring the manufacturer data");
    internal_stop_ble();

    update_manufacturer_info(uwb_metadata);
    manf_info.type = BT_DATA_MANUFACTURER_DATA;
    manf_info.data = beluga_manufacturer_data;
    manf_info.data_len = sizeof(beluga_manufacturer_data);
    nconn_sd_data[NCONN_SCAN_MANF_DATA_IDX] = manf_info;
    conn_ad_data[CONN_ADV_MANF_DATA_IDX] = manf_info;

    refresh_advertising_data();

    internal_start_ble();
}

/**
 * Updates the advertising name with the new UUID
 * @param[in] uuid The new UUID
 */
static void update_adv_name(uint16_t uuid) {
    LOG_DBG("Updating advertising name");
    size_t len;
    memcpy(uuid_encoded, &uuid, sizeof(uuid));
    len = snprintk(adv_name, sizeof(adv_name), "%s%d", m_target_peripheral_name,
                   uuid);
    struct bt_data uuid_data = {
        .type = BT_DATA_UUID16_ALL,
        .data = uuid_encoded,
        .data_len = sizeof(uint16_t),
    };
    struct bt_data name_data = {
        .type = BT_DATA_NAME_COMPLETE,
        .data = adv_name,
        .data_len = len,
    };

    internal_stop_ble();
    nconn_ad_data[NCONN_ADV_NAME_IDX] = name_data;
    conn_sd_data[CONN_SCAN_NAME_IDX] = name_data;
    nconn_sd_data[NCONN_SCAN_UUID_IDX] = uuid_data;
    conn_ad_data[CONN_ADV_UUID_IDX] = uuid_data;

    refresh_advertising_data();

    internal_start_ble();
}

/**
 * Retrieves a reference central connection object
 * @return reference to the central connection object
 */
struct bt_conn **get_central_connection_obj(void) {
    return &central_conn;
}

/**
 * Stops advertising and scanning without updating the advertising state
 */
void internal_stop_ble(void) {
    LOG_DBG("Stopping BLE");
    bt_le_ext_adv_stop(ext_adv[CONN_ADV_IDX]);
    bt_le_ext_adv_stop(ext_adv[NCONN_ADV_IDX]);
    stop_scanning();
}

/**
 * Restores advertising and scanning according to the current advertising state
 */
void internal_start_ble(void) {
    LOG_DBG("Resuming BLE");
    switch (adv_state) {
    case ADVERTISING_CONNECTABLE: {
        LOG_DBG("Starting connectable and non-connectable advertising");
        bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX],
                            BT_LE_EXT_ADV_START_DEFAULT);
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
        start_active_scanning();
        break;
    }
    case ADVERTISING_NO_CONNECT: {
        LOG_DBG("Starting non-connectable advertising");
        bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX],
                            BT_LE_EXT_ADV_START_DEFAULT);
        start_active_scanning();
        break;
    }
    case ADVERTISING_OFF:
        LOG_DBG("Doing nothing since advertising is off");
        break;
    default:
        __ASSERT_UNREACHABLE;
    }
}

/**
 * Work handler for restarting BLE after disconnecting as a central device
 * @param[in] work The work item
 */
static void restart_ble_work_handle(struct k_work *work) {
    ARG_UNUSED(work);
    stop_advertising();
    stop_scanning();
    start_advertising();
    start_active_scanning();
}

/**
 * Wait until a BLE disconnection occurs.
 * @param[in] timeout The maximum time to wait for a BLE disconnection to occur
 * @return 0 if BLE connection disconnected
 * @return -EBUSY if returned without waiting
 * @return -EAGAIN if timed out
 */
int wait_ble_disconnect(k_timeout_t timeout) {
    int ret = k_sem_take(&ble_connect_status, timeout);
    if (ret) {
        return ret;
    }
    k_sem_give(&ble_connect_status);
    return 0;
}

/**
 * Terminates the peripheral connection, if one exists
 */
void disconnect_ble_connections(void) {
    int ret;
    if (k_sem_take(&ble_connect_status, K_NO_WAIT) == 0) {
        k_sem_give(&ble_connect_status);
        return;
    }
    __ASSERT_NO_MSG(peripheral_conn != NULL);
    ret = bt_conn_disconnect(peripheral_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if (ret) {
        LOG_ERR("Failed to disconnect (%d)", ret);
        k_panic();
    }
    wait_ble_disconnect(K_FOREVER);
    stop_advertising();
    stop_scanning();
}

/**
 * Updates the neighbor list by either, updating or inserting the scanned node
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
void update_seen_list(struct ble_data *data, int8_t rssi) {
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
 * Helper function for finding a neighbor node to connect to.
 * @param[in] data The found node data.
 * @param[in] addr The BLE address of the scanned node.
 */
void check_advertiser(struct ble_data *data, const bt_addr_le_t *addr) {
    int ret, signaled, search_id;
    ret = k_poll(&connect_signalling.connect_events[CONNECT_SEARCH_ID], 1,
                 K_NO_WAIT);

    if (ret != 0) {
        return;
    }

    k_poll_signal_check(&connect_signalling.connect_signals[CONNECT_SEARCH_ID],
                        &signaled, &search_id);
    if (signaled && search_id == (int)data->uuid) {
        k_poll_signal_reset(
            &connect_signalling.connect_signals[CONNECT_SEARCH_ID]);
        bt_addr_le_copy(&connect_signalling.addr, addr);
        k_poll_signal_raise(
            &connect_signalling.connect_signals[CONNECT_SEARCH_FOUND], 0);
    }
}

/**
 * Updates the node ID and publishes the update to the Bluetooth advertising
 * data
 * @param[in] uuid The new node ID
 */
void update_node_id(uint16_t uuid) {
    NODE_UUID = uuid;
    update_adv_name(uuid);
}

/**
 * Retrieves the current node ID
 * @return The current node ID
 */
uint16_t get_NODE_UUID(void) { return NODE_UUID; }
