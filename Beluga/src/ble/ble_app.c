/**
 * @file ble_app.c
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include <ble/adv.h>
#include <ble/ble_app.h>
#include <ble/ble_app_internal.h>
#include <ble/scan.h>

#include <ble/services/beluga_client.h>
#include <ble/services/beluga_service.h>

#include <list_monitor.h>
#include <range_extension.h>
#include <settings.h>

LOG_MODULE_REGISTER(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

K_SEM_DEFINE(ble_state, 1, 1);

static uint16_t NODE_UUID = 0;

static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};

struct bt_connect connect_signalling;
struct uwb_sync_configs sync_configs;
static struct bt_beluga_client client;
static bool bluetooth_on = false;
struct node seen_list[MAX_ANCHOR_COUNT];

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

void check_advertiser(struct ble_data *data, const bt_addr_le_t *addr) {
    int ret, signaled, search_id;
    ret = k_poll(&connect_signalling.connect_events[CONNECT_SEARCH_ID], 1,
                 K_NO_WAIT);

    if (ret != 0) {
        return;
    }

    k_poll_signal_check(&connect_signalling.connect_signals[CONNECT_SEARCH_ID], &signaled,&search_id);
    if (signaled && search_id == (int)data->uuid) {
        k_poll_signal_reset(&connect_signalling.connect_signals[CONNECT_SEARCH_ID]);
        bt_addr_le_copy(&connect_signalling.addr, addr);
        k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_SEARCH_FOUND], 0);
    }
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context) {
    int err;
    struct bt_beluga_client *bsc = context;

    LOG_INF("The discovery procedure succeeded");

    bt_gatt_dm_data_print(dm);

    err = bt_beluga_client_assign(dm, bsc);
    if (err) {
        LOG_INF("Could not init Beluga client object (err %d)", err);
        k_poll_signal_raise(
            &connect_signalling.connect_signals[CONNECT_CONNECTED], 1);
        return;
    }
    // If we want range, then we subscribe here
    // bt_beluga_client_subscribe_ranging(bsc);

    err = bt_gatt_dm_data_release(dm);
    if (err) {
        LOG_INF("Could not release the discovery data (err %d)", err);
    }
    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_CONNECTED],
                        0);
    LOG_INF("Client data updated");
}

static void discovery_not_found_cb(struct bt_conn *conn, void *context) {
    ARG_UNUSED(conn);
    ARG_UNUSED(context);
    LOG_INF("Beluga Service could not be found during the discovery");
    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_CONNECTED],
                        1);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err,
                                     void *context) {
    ARG_UNUSED(conn);
    ARG_UNUSED(context);
    ARG_UNUSED(err);
    LOG_INF("The discovery procedure failed with %d", err);
    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_CONNECTED],
                        1);
}

static const struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_not_found_cb,
    .error_found = discovery_error_found_cb,
};

int gatt_discover(struct bt_conn *conn) {
    int err =
        bt_gatt_dm_start(conn, BT_UUID_BELUGA_SVC, &discovery_cb, &client);
    if (err) {
        LOG_ERR("Could not start the discovery procedure, error "
                "code: %d",
                err);
    }
    return err;
}

static void beluga_uwb_settings_synced(struct bt_beluga_client *bt_client,
                                       uint8_t err,
                                       const struct beluga_uwb_params *config) {
    ARG_UNUSED(bt_client);
    ARG_UNUSED(err);
    ARG_UNUSED(config);
    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECTED_SYNCED],
                        0);
}

static int init_beluga_client(void) {
    struct bt_beluga_client_cb cb = {
        .synced = beluga_uwb_settings_synced,
    };
    int err = bt_beluga_client_init(&client, &cb);
    if (err) {
        return err;
    }

    for (size_t i = 0; i < CONNECT_LAST_ENUMERATOR; i++) {
        k_poll_signal_init(&connect_signalling.connect_signals[i]);
        k_poll_event_init(&connect_signalling.connect_events[i],
                          K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                          &connect_signalling.connect_signals[i]);
    }

    return bt_beluga_client_init(&client, &cb);
}

static void prep_configs(struct beluga_uwb_params *config) {
    LOG_DBG("Loading UWB settings");
    config->TX_POWER = retrieveSetting(BELUGA_TX_POWER);
    config->PREAMBLE = (uint16_t)retrieveSetting(BELUGA_UWB_PREAMBLE);
    config->POWER_AMP = (uint8_t)retrieveSetting(BELUGA_RANGE_EXTEND);
    config->CHANNEL = (uint8_t)retrieveSetting(BELUGA_UWB_CHANNEL);
    config->TWR = (bool)retrieveSetting(BELUGA_TWR);
    config->PHR = (bool)retrieveSetting(BELUGA_UWB_PHR);
    config->SFD = (bool)retrieveSetting(BELUGA_UWB_NSSFD);
    config->DATA_RATE = (uint8_t)retrieveSetting(BELUGA_UWB_DATA_RATE);
    config->PAC = (uint8_t)retrieveSetting(BELUGA_UWB_PAC);
    config->PULSE_RATE = (bool)retrieveSetting(BELUGA_UWB_PULSE_RATE);
}

static int wait_connection(bool *stopped) {
    struct bt_conn **central_conn;
    int ret;

    LOG_DBG("Waiting for node to be found");
    ret = k_poll(&connect_signalling.connect_events[CONNECT_SEARCH_FOUND], 1,
                 K_SECONDS(1));

    if (ret != 0) {
        // Timed out
        k_poll_signal_reset(
            &connect_signalling.connect_signals[CONNECT_SEARCH_ID]);
        LOG_ERR("Search timed out");
        return -EAGAIN;
    }

    k_poll_signal_reset(
        &connect_signalling.connect_signals[CONNECT_SEARCH_FOUND]);

    LOG_DBG("Found node. Stopping advertising and scanning");
    internal_stop_ble();
    *stopped = true;

    LOG_DBG("Attempting to establish a connection");
    central_conn = get_central_connection_obj();
    ret = bt_conn_le_create(&connect_signalling.addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, central_conn);

    if (ret != 0) {
        LOG_ERR("Connection failed: %d", ret);
    }

    return ret;
}

static int perform_gatt_exchange(struct beluga_uwb_params *config) {
    int ret, set, val;
    LOG_DBG("Waiting for GATT exchange");
    (void)k_poll(&connect_signalling.connect_events[CONNECT_CONNECTED], 1,
                 K_FOREVER);
    k_poll_signal_check(&connect_signalling.connect_signals[CONNECT_CONNECTED],
                        &set, &val);
    k_poll_signal_reset(&connect_signalling.connect_signals[CONNECT_CONNECTED]);
    if (val) {
        LOG_ERR("GATT exchange failed");
        return -ENOTCONN;
    }

    LOG_DBG("Now syncing other node's UWB settings");
    ret = bt_beluga_client_sync(&client, config);
    if (ret != 0) {
        LOG_ERR("Beluga client sync failed: %d", ret);
        return ret;
    }

    LOG_DBG("Waiting for target to be synced");
    (void)k_poll(&connect_signalling.connect_events[CONNECTED_SYNCED], 1,
                 K_FOREVER);
    k_poll_signal_reset(&connect_signalling.connect_signals[CONNECTED_SYNCED]);
    return 0;
}

int sync_uwb_parameters(uint16_t id) {
    struct bt_conn **central_conn;
    struct beluga_uwb_params config;
    int ret, disconnect_ret;
    bool ble_stopped = false;
    LOG_DBG("Searching for node");

    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_SEARCH_ID],
                        (int)id);

    prep_configs(&config);
    ret = wait_connection(&ble_stopped);
    if (ret != 0) {
        if (ble_stopped) {
            goto finish;
        }
        return ret;
    }

    ret = perform_gatt_exchange(&config);

    LOG_DBG("Now attempting to disconnect");
    central_conn = get_central_connection_obj();
    disconnect_ret =
        bt_conn_disconnect(*central_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

    if (disconnect_ret) {
        LOG_ERR("Disconnect failed: %d", ret);
    }

    if (ret == 0 && disconnect_ret != 0) {
        ret = disconnect_ret;
    }

finish:
    internal_start_ble();
    return ret;
}

static int sync_uwb_settings(struct bt_conn *conn,
                             const struct beluga_uwb_params *configs) {
    ARG_UNUSED(conn);
    LOG_INF("Syncing UWB parameters");
    // This is probably running in a cooperative thread. Move it over
    // into the main thread. This is to make sure that this does not
    // run into a deadlock issue...
    int set, val;

    k_poll_signal_check(&sync_configs.ready_sig, &set, &val);
    if (set) {
        return -EAGAIN;
    }

    memcpy(&sync_configs.config, configs, sizeof(sync_configs.config));
    k_poll_signal_raise(&sync_configs.ready_sig, 0);
    return 0;
}

static int init_beluga_service(void) {
    struct beluga_service_cb cb = {
        .sync = sync_uwb_settings,
    };

    k_poll_signal_init(&sync_configs.ready_sig);
    k_poll_event_init(&sync_configs.ready, K_POLL_TYPE_SIGNAL,
                      K_POLL_MODE_NOTIFY_ONLY, &sync_configs.ready_sig);

    return beluga_service_init(&cb);
}

int init_bt_stack(void) {
    int32_t err;

    err = bt_enable(NULL);
    if (err) {
        return err;
    }
    LOG_INF("Bluetooth stack loaded");

    err = init_advertising();
    if (err) {
        return err;
    }

    err = init_beluga_service();
    if (err) {
        return err;
    }

    err = init_beluga_client();
    if (err != 0 && err != -EALREADY) {
        return err;
    }

    return 0;
}

int deinit_bt_stack(void) {
    stop_advertising();
    stop_scanning();

    return bt_disable();
}

static int _enable_bluetooth(void) {
    LOG_DBG("_enable_bluetooth()");
    int err = start_advertising();
    if (err != 0) {
        return err;
    }

    err = start_active_scanning();
    if (err != 0) {
        stop_advertising();
        return err;
    }

    bluetooth_on = true;
    return 0;
}

static int _disable_bluetooth(void) {
    int err;
    LOG_DBG("_disable_bluetooth()");

    err = stop_advertising();
    if (err) {
        LOG_ERR("Unable to stop advertising (err: %d)", err);
        return err;
    }

    err = stop_scanning();
    if (err) {
        LOG_ERR("Unable to stop scanning (err: %d)", err);
        return err;
    }

    bluetooth_on = false;
    return 0;
}

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

int disable_bluetooth(void) {
    int retVal = -EALREADY;
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

void update_node_id(uint16_t uuid) {
    NODE_UUID = uuid;
    update_adv_name(uuid);
}

uint16_t get_NODE_UUID(void) { return NODE_UUID; }

bool check_ble_enabled(void) { return bluetooth_on; }

bool save_and_disable_bluetooth(void) {
    bool ret;
    k_sem_take(&ble_state, K_FOREVER);
    ret = bluetooth_on;
    if (bluetooth_on) {
        _disable_bluetooth();
    }
    return ret;
}

void restore_bluetooth(bool state) {
    if (state) {
        _enable_bluetooth();
    }
    k_sem_give(&ble_state);
}
