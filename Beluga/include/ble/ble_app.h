/**
 * @file ble_app.h
 *
 * @brief
 *
 * @date 5/7/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_BLE_APP_H
#define BELUGA_DTS_BLE_APP_H

#include <ble/services/beluga_service_common.h>
#include <deca_device_api.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * The maximum length needed for the advertising name
 */
#define NAME_LEN 30

/**
 * Beluga scan data
 */
struct ble_data {
    uint16_t uuid;                      ///< The node ID
    uint8_t manufacturerData[NAME_LEN]; ///< The manufacturer data for the node
    bool beluga_node; ///< Flag indicating that the scanned device is a Beluga
    ///< node
};

/**
 * Eviction policies for when the neighbor list is full
 */
enum node_eviction_policy {
    EVICT_POLICY_RR = 0,       ///< Index round robin
    EVICT_POLICY_RSSI = 1,     ///< Lowest RSSI
    EVICT_POLICY_RANGE = 2,    ///< Largest range
    EVICT_POLICY_BLE_TS = 3,   ///< Least recently scanned node
    EVICT_POLICY_RANGE_TS = 4, ///< Least recently updated range
    EVICT_POLICY_INVALID       ///< Last enumerator
};

/**
 * UWB metadata to be advertised
 */
struct advertising_info {
    uint8_t CHANNEL; ///< The UWB channel.
    bool TWR; ///< The ranging mode. `true` for DS-TWR, `false` for SS-TWR.
    bool SFD; ///< Start Frame Delimiter mode. `true` for non-standard SFD,
              ///< `false` for standard SFD.
    uint8_t DATARATE; ///< The UWB data rate.
    bool PULSERATE;   ///< The UWB pulse repetition frequency. `true` if 64MHz,
                      ///< `false` for 16MHz.
    bool PHR; ///< Physical header mode. `true` for extended mode, `false` for
              ///< standard mode.
    uint8_t PAC;  ///< UWB Packet Acquisition Chunk Size.
    bool ACTIVE;  ///< `true` if UWB is turned on, `false` if UWB is turned off.
    uint16_t pan; ///< UWB PAN ID.
    uint32_t preamble;  ///< UWB Preamble length.
    uint32_t poll_rate; ///< UWB poll rate.
};

/**
 * BLE neighbor node structure
 */
struct node {
    uint16_t UUID;      /** node ID */
    int8_t RSSI;        /** node RSSI value */
    int64_t time_stamp; /** Last timestamp updated ranging value */
    float range;        /** Last updated ranging value */
    bool update_flag;  /** Flag to indicate the ranging value is updated or not,
                         1  if the node get updated */
    bool polling_flag; /** Flag to indicate the node is passive or not, 1 if the
                         node will init uwb signal*/
    int64_t
        ble_time_stamp; /** Last timestamp get the BLE package from this node */

#if IS_ENABLED(CONFIG_UWB_LOGIC_CLK)
    uint32_t exchange_id; /** Ranging exchange ID (Logic clock) */
#endif

    dwt_rxdiag_t uwb_diagnostics;
    dwt_deviceentcnts_t uwb_counts;
};

struct uwb_sync_configs {
    struct beluga_uwb_params config;
    struct k_poll_signal ready_sig;
    struct k_poll_event ready;
};

/**
 * Neighbor list size
 */
#define MAX_ANCHOR_COUNT CONFIG_BELUGA_NETWORK_SIZE

/**
 * Checks if specified neighbor is in the neighbor list
 * @param[in] uuid The neighbor node ID
 * @return true if in neighbor list
 * @return false if not in neighbor list
 */
bool in_seen_list(uint16_t uuid);

/**
 * Initializes the Bluetooth stack
 * @return 0 upon success
 * @return negative error code otherwise
 */
int init_bt_stack(void);

/**
 * Stops Bluetooth scanning and advertising and unloads the bluetooth stack
 * @return 0 upon success
 * @return negative error code otherwise
 */
int deinit_bt_stack(void);

/**
 * Starts the Bluetooth FEM (if applicable) and starts Bluetooth
 * scanning/advertising
 * @return 0 upon success
 * @return 1 if already on
 * @return -EFAULT if unable to start the FEM
 * @return negative error code otherwise
 */
int enable_bluetooth(void);

/**
 * Stops Bluetooth advertising/scanning and powers down the Bluetooth FEM
 * @return 0 upon success
 * @return 1 if Bluetooth is already off
 * @return -EFAULT if unable to power down the Bluetooth FEM
 * @return negative error code otherwise
 */
int disable_bluetooth(void);

/**
 * Updates the node ID and publishes the update to the Bluetooth advertising
 * data
 * @param[in] uuid The new node ID
 */
void update_node_id(uint16_t uuid);

/**
 * Retrieves the current node ID
 * @return The current node ID
 */
uint16_t get_NODE_UUID(void);

/**
 * Updates the BLE manufacturer data with the new UWB metadata
 * @param[in] uwb_metadata The new UWB metadata
 */
void advertising_reconfig(struct advertising_info *uwb_metadata);

/**
 * Checks if Bluetooth is currently advertising/scanning
 * @return `true` if advertising/scanning
 * @return `false` otherwise
 */
bool check_ble_enabled(void);

/**
 * Returns the current Bluetooth state and disables scanning/advertising.
 * Additionally, blocks other threads from enabling Bluetooth until the state is
 * restored. See `restore_bluetooth()`
 * @return The current Bluetooth state
 */
bool save_and_disable_bluetooth(void);

/**
 * Restores the Bluetooth state to its original setting. Additionally, releases
 * the Bluetooth lock to allow other threads to update the Bluetooth state. See
 * `save_and_disable-bluetooth()`
 * @param[in] state The saved Bluetooth state
 */
void restore_bluetooth(bool state);

#if defined(CONFIG_BELUGA_GATT)
/**
 * Publish a range to the connected peers.
 * @param[in] uuid The node that was ranged to.
 * @param[in] range The distance between self and the ranged to node.
 */
void update_ble_service(uint16_t uuid, double range);
#else
/**
 * Publish a range to the connected peers.
 * @param[in] uuid The node that was ranged to.
 * @param[in] range The distance between self and the ranged to node.
 */
#define update_ble_service(x, y) (void)0
#endif // defined(CONFIG_BELUGA_GATT)

/**
 * Searches for the given UUID when scanning for neighbors and syncs that
 * neighbor's UWB settings to the current node's UWB settings.
 *
 * @param[in] id The neighbor ID to search for.
 * @return 0 upon successfully syncing .
 * @return negative error code otherwise.
 */
int sync_uwb_parameters(uint16_t id);

/**
 * Wait until a BLE disconnection occurs.
 * @param[in] timeout The maximum time to wait for a BLE disconnection to occur
 * @return 0 if BLE connection disconnected
 * @return -EBUSY if returned without waiting
 * @return -EAGAIN if timed out
 */
int wait_ble_disconnect(k_timeout_t timeout);

/**
 * The neighbor list
 */
extern struct node seen_list[MAX_ANCHOR_COUNT];

/**
 * Synced UWB configurations.
 */
extern struct uwb_sync_configs sync_configs;

#endif // BELUGA_DTS_BLE_APP_H
