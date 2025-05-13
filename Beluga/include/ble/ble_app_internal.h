/**
 * @file ble_app_internal.h
 *
 * @brief I know this is a complete mess, but fuck it, so is the BLE stack.
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

/**
 * @{
 * The UWB metadata and manufacturer data overheads.
 */
#define BLE_PAN_OVERHEAD          2
#define BLE_UWB_METADATA_OVERHEAD 2
#define BLE_MANF_DATA_OVERHEAD    (BLE_PAN_OVERHEAD + BLE_UWB_METADATA_OVERHEAD)
/**
 * @}
 */

/**
 * @{
 * UWB metadata byte offsets.
 */
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
/**
 * @}
 */

/**
 * @{
 * UWB metadata bitfield shifts
 */
#define UWB_CHANNEL_SHIFT   0
#define UWB_TWR_SHIFT       3
#define UWB_SFD_SHIFT       4
#define UWB_DATARATE_SHIFT  5
#define UWB_PULSERATE_SHIFT 7

#define UWB_PHR_SHIFT       0
#define UWB_PREAMBLE_SHIFT  1
#define UWB_PAC_SHIFT       4
#define UWB_POLL_SHIFT      6
#define UWB_ACTIVE_SHIFT    7
/**
 * @}
 */

/**
 * @{
 * Bitmasks for the UWB metadata
 */
#define UWB_CHANNEL_MASK   UINT8_C(0x7)
#define UWB_TWR_MASK       (UINT8_C(0x1) << UWB_TWR_SHIFT)
#define UWB_SFD_MASK       (UINT8_C(0x1) << UWB_SFD_SHIFT)
#define UWB_DATARATE_MASK  (UINT8_C(0x3) << UWB_DATARATE_SHIFT)
#define UWB_PULSERATE_MASK (UINT8_C(0x1) << UWB_PULSERATE_SHIFT)

#define UWB_PHR_MASK       UINT8_C(0x1)
#define UWB_PREAMBLE_MASK  (UINT8_C(0x7) << UWB_PREAMBLE_SHIFT)
#define UWB_PAC_MASK       (UINT8_C(0x3) << UWB_PAC_SHIFT)
#define UWB_POLLING_MASK   (UINT8_C(0x1) << UWB_POLL_SHIFT)
#define UWB_ACTIVE_MASK    (UINT8_C(0x1) << UWB_ACTIVE_SHIFT)
/**
 * @}
 */

/**
 * Connection state signals
 */
enum connect_signals {
    CONNECT_SEARCH_ID,    ///< Searching for a node by ID
    CONNECT_SEARCH_FOUND, ///< Node with specific ID found
    CONNECT_CONNECTED,    ///< Connected to the node as a central
    CONNECTED_SYNCED,     ///< UWB settings between nodes synchronized

    CONNECT_LAST_ENUMERATOR ///< Last enumerator. Don not use
};

/**
 * BLE search and connect synchronization signals.
 */
struct bt_connect {
    bt_addr_le_t addr; ///< The BLE address to connect to
    struct k_poll_signal
        connect_signals[CONNECT_LAST_ENUMERATOR]; ///< The signals for
                                                  ///< connection
                                                  ///< synchronization
    struct k_poll_event
        connect_events[CONNECT_LAST_ENUMERATOR]; ///< The connection events
};

/**
 * Updates the neighbor list by either, updating or inserting the scanned node
 * @param[in] data The BLE scan data
 * @param[in] rssi The RSSI of the scanned node
 */
void update_seen_list(struct ble_data *data, int8_t rssi);

/**
 * Helper function for finding a neighbor node to connect to.
 * @param[in] data The found node data.
 * @param[in] addr The BLE address of the scanned node.
 */
void check_advertiser(struct ble_data *data, const bt_addr_le_t *addr);

/**
 * Starts the GATT discovery procedure for the client connection.
 * @param[in] conn The connection object to start discovery for.
 * @return 0 upon success.
 * @return negative error code otherwise.
 */
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

/**
 * Synchronization signals for transferring UWB configurations from this node to
 * the other connected node.
 */
extern struct bt_connect connect_signalling;

#endif // BELUGA_DTS_BLE_APP_INTERNAL_H
