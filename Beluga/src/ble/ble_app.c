/**
 * @file ble_app.c
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#include <ble/ble_app_internal.h>
#include <bluetooth/gatt_dm.h>
#include <list_monitor.h>
#include <zephyr/bluetooth/gatt.h>

#include <ble/services/beluga_client.h>
#include <ble/services/beluga_service.h>

#include <zephyr/logging/log.h>

static uint16_t NODE_UUID = 0;

static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};

struct bt_connect connect_signalling;

/**
 * Updates the neighbor list by either, updating or inserting the scanned node
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
void update_seen_list(struct ble_data *data, int8_t rssi,
                      const bt_addr_le_t *addr) {
    bool polling;
    int ret;
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

    ret = k_poll(&connect_signalling.connect_events[CONNECT_SEARCH_ID], 1,
                 K_NO_WAIT);
    if (ret == 0) {
        int signaled, search_id;
        k_poll_signal_check(
            &connect_signalling.connect_signals[CONNECT_SEARCH_ID], &signaled,
            &search_id);
        if (signaled && search_id == (int)data->uuid) {
            k_poll_signal_reset(
                &connect_signalling.connect_signals[CONNECT_SEARCH_ID]);
            bt_addr_le_copy(&connect_signalling.addr, addr);
            k_poll_signal_raise(
                &connect_signalling.connect_signals[CONNECT_SEARCH_FOUND], 0);
        }
    }

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

static struct bt_beluga_client client;

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
    LOG_INF("Beluga Service could not be found during the discovery");
    k_poll_signal_raise(&connect_signalling.connect_signals[CONNECT_CONNECTED],
                        1);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err,
                                     void *context) {
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
