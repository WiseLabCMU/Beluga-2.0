/**
 * @file ble_app_internal.h
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BLE_APP_INTERNAL_H
#define BELUGA_DTS_BLE_APP_INTERNAL_H

#include <list_monitor.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>

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

enum connect_signals {
    CONNECT_SEARCH_ID,
    CONNECT_SEARCH_FOUND,
    CONNECT_CONNECTED,
    CONNECTED_SYNCED,
    CONNECT_LAST_ENUMERATOR
};

struct bt_connect {
    bt_addr_le_t addr;
    struct k_poll_signal connect_signals[CONNECT_LAST_ENUMERATOR];
    struct k_poll_event connect_events[CONNECT_LAST_ENUMERATOR];
};

void update_seen_list(struct ble_data *data, int8_t rssi);
void check_advertiser(struct ble_data *data, const bt_addr_le_t *addr);
int gatt_discover(struct bt_conn *conn);

/**
 * Stops advertising and scanning without updating the advertising state
 */
void internal_stop_ble(void);

/**
 * Restores advertising and scanning according to the current advertising state
 */
void internal_start_ble(void);

/**
 * Retrieves a reference central connection object
 * @return reference to the central connection object
 */
struct bt_conn **get_central_connection_obj(void);

/**
 * Terminates the peripheral connection, if one exists
 */
void disconnect_ble_connections(void);

extern struct bt_connect connect_signalling;

#endif // BELUGA_DTS_BLE_APP_INTERNAL_H
