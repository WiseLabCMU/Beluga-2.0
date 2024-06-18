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

#include "ble_app.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_cus.h"
#include "ble_db_discovery.h"
#include "ble_nus.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_fstorage.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PERIPHERAL_ADVERTISING_LED BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED   BSP_BOARD_LED_3
#define CENTRAL_SCANNING_LED       BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED      BSP_BOARD_LED_1

#define DEVICE_NAME                "Node" /**< Name of device used for advertising. */
#define MANUFACTURER_NAME                                                      \
    "NordicSemiconductor" /**< Manufacturer. Will be passed to Device          \
                             Information Service. */
#define APP_ADV_INTERVAL                                                       \
    300 /**< The advertising interval (in units of 0.625 ms). This value       \
           corresponds to 187.5 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS                                             \
    180 /**< The advertising timeout in units of seconds. */

#define APP_BLE_CONN_CFG_TAG                                                   \
    1 /**< A tag identifying the SoftDevice BLE configuration. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY                                         \
    APP_TIMER_TICKS(                                                           \
        5000) /**< Time from initiating event (connect or start of             \
                 notification) to first time sd_ble_gap_conn_param_update is   \
                 called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                                          \
    APP_TIMER_TICKS(                                                           \
        30000) /**< Time between each call to sd_ble_gap_conn_param_update     \
                  after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                                           \
    3 /**< Number of attempts before giving up the connection parameter        \
         negotiation. */

#define APP_FEATURE_NOT_SUPPORTED                                              \
    BLE_GATT_STATUS_ATTERR_APP_BEGIN +                                         \
        2 /**< Reply when unsupported features are requested. */

#define SEC_PARAM_BOND     1 /**< Perform bonding. */
#define SEC_PARAM_MITM     0 /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC     0 /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0 /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES                                              \
    BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE                                                 \
    7 /**< Minimum encryption key size in octets.                              \
       */
#define SEC_PARAM_MAX_KEY_SIZE                                                 \
    16 /**< Maximum encryption key size in octets.                             \
        */

#define SCAN_INTERVAL                                                          \
    0x00140 /**< Determines scan interval in units of 0.625 millisecond.       \
               0x00A0 140*/
#define SCAN_WINDOW                                                            \
    0x00140 /**< Determines scan window in units of 0.625 millisecond.         \
               0x0050*/
#define SCAN_TIMEOUT 0

#define MIN_CONNECTION_INTERVAL                                                \
    (uint16_t)                                                                 \
        MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection    \
                                            interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL                                                \
    (uint16_t)                                                                 \
        MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Determines maximum connection     \
                                           interval in milliseconds. */
#define SLAVE_LATENCY                                                          \
    0 /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT                                                    \
    (uint16_t)                                                                 \
        MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out   \
                                           in units of 10 milliseconds. */

#define UUID16_SIZE             2 /**< Size of a UUID, in bytes. */

#define NON_CONNECTABLE_SET     0
#define NON_CONNECTABLE_RECOVER 1

#define NODE_UUID_START         0x00FF
#define UUID_INDICATOR          0x1800

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO 3

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream.
 */
#define UUID16_EXTRACT(DST, SRC)                                               \
    do {                                                                       \
        (*(DST)) = (SRC)[1];                                                   \
        (*(DST)) <<= 8;                                                        \
        (*(DST)) |= (SRC)[0];                                                  \
    } while (0)

// Create CUS instance
BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery,
2); /**< Database discovery module instances. */

/**@brief Variable length data encapsulation in terms of length and pointer to
 * data. */
typedef struct {
    uint8_t *p_data;   /**< Pointer to data. */
    uint16_t data_len; /**< Length of data. */
} data_t;

static bool in_seen_list(uint16_t UUID);
static int get_seen_list_idx(uint16_t UUID);

/**@brief names which the central applications will scan for, and which will be
 * advertised by the peripherals. if these are set to empty strings, the UUIDs
 * defined below will be used
 */
// static char const m_target_periph_name[] = "Node";
static char const m_target_periph_name[] = "BN ";

uint16_t NODE_UUID = 1;
uint32_t time_keeper;
int ble_started;
int node_added;
int last_seen_idx = 0;
node seen_list[MAX_ANCHOR_COUNT];
ble_uuid_t m_adv_uuids[2];

/**@brief Advertising UUID list */
ble_uuid_t m_adv_uuids[2] = {
        {UUID_INDICATOR, BLE_UUID_TYPE_BLE}, {NODE_UUID_START, BLE_UUID_TYPE_BLE},
        //{CUSTOM_SERVICE_UUID           , BLE_UUID_TYPE_VENDOR_BEGIN},
};

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params = {
        .active = 1,
        .interval = SCAN_INTERVAL,
        .window = SCAN_WINDOW,
        .timeout = SCAN_TIMEOUT,
#if (NRF_SD_BLE_API_VERSION <= 2)
        .selective = 0,
        .p_whitelist = NULL,
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
#endif
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param = {
        MIN_CONNECTION_INTERVAL, MAX_CONNECTION_INTERVAL, SLAVE_LATENCY,
        SUPERVISION_TIMEOUT};

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 * You need to analyze how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went
 * wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Parses advertisement data, providing length and location of the field
 * in case matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data
 * length and pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t *p_advdata,
                                 data_t *p_typedata) {
    uint32_t index = 0;
    uint8_t *p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len) {
        uint8_t field_length = p_data[index];
        uint8_t field_type = p_data[index + 1];

        if (field_type == type) {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for initiating scanning.
 */
void scan_start(void) {
    ret_code_t err_code;

    (void)sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initiating advertising and scanning.
 */
void adv_scan_start(void) {
    ret_code_t err_code;

    // check if there are no flash operations in progress
    if (!nrf_fstorage_is_busy(NULL)) {
        // Start scanning for peripherals and initiate connection to devices
        // which advertise Heart Rate or Running speed and cadence UUIDs.
        if (ble_started == 1) {
            scan_start();

            // Start advertising.
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
    ret_code_t err_code;

    switch (p_evt->evt_id) {
        case PM_EVT_BONDED_PEER_CONNECTED: {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED: {
            NRF_LOG_INFO(
                    "Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                    ble_conn_state_role(p_evt->conn_handle), p_evt->conn_handle,
                    p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED: {
            /* Often, when securing fails, it shouldn't be restarted, for security
             * reasons. Other times, it can be restarted directly. Sometimes it can
             * be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and
             * reconnected. Sometimes it is impossible, to secure the link, or the
             * peer device does not support it. How to handle this error is highly
             * application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ: {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL: {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY ||
                err_code == FDS_ERR_NO_SPACE_IN_QUEUES) {
                // Retry.
            } else {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED: {
            adv_scan_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED: {
            // The local database has likely changed, send service changed
            // indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED: {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED: {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED: {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED: {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a
 * given name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(ble_gap_evt_adv_report_t const *p_adv_report,
                          char const *name_to_find) {
    ret_code_t err_code;
    data_t adv_data;
    data_t dev_name;

    // Initialize advertisement report for parsing
    adv_data.p_data = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    // search for advertising names
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data,
                                &dev_name);
    if (err_code == NRF_SUCCESS) {
        // NOTE TODO: This line needs to be modified after change advertising
        // name
        // printf("data len: %d \n", dev_name.data_len);
        // if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len)== 0)

        if (memcmp(name_to_find, dev_name.p_data, 2) == 0) {

            return true;
        }
    } else {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data,
                                    &dev_name);
        if (err_code != NRF_SUCCESS) {
            return false;
        }
        // if (memcmp(m_target_periph_name, dev_name.p_data,
        // dev_name.data_len)== 0)
        if (memcmp(m_target_periph_name, dev_name.p_data, 2) == 0) {
            return true;
        }
    }
    return false;
}

/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a
 * given UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool find_adv_uuid(ble_gap_evt_adv_report_t const *p_adv_report,
                          uint16_t uuid_to_find) {
    ret_code_t err_code;
    data_t adv_data;
    data_t type_data;

    // Initialize advertisement report for parsing.
    adv_data.p_data = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    err_code =
            adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                             &adv_data, &type_data);

    if (err_code != NRF_SUCCESS) {
        // Look for the services in 'complete' if it was not found in 'more
        // available'.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                    &adv_data, &type_data);

        if (err_code != NRF_SUCCESS) {
            // If we can't parse the data, then exit.
            return false;
        }
    }

    // Verify if any UUID match the given UUID.
    for (uint32_t i = 0; i < (type_data.data_len / UUID16_SIZE); i++) {
        uint16_t extracted_uuid;
        UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[i * UUID16_SIZE]);

        if (extracted_uuid == uuid_to_find) {
            return true;
        }
    }
    return false;
}

/**@brief Function to find UUID value in advertising package
 */
static uint16_t
find_adv_uuid_next(ble_gap_evt_adv_report_t const *p_adv_report) {
    ret_code_t err_code;
    data_t adv_data;
    data_t type_data;

    // Initialize advertisement report for parsing.
    adv_data.p_data = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    err_code =
            adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                             &adv_data, &type_data);

    if (err_code != NRF_SUCCESS) {
        // Look for the services in 'complete' if it was not found in 'more
        // available'.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                    &adv_data, &type_data);

        if (err_code != NRF_SUCCESS) {
            // If we can't parse the data, then exit.
            return -1;
        }
    }

    // Verify if any UUID match the given UUID.
    for (uint32_t i = 0; i < (type_data.data_len / UUID16_SIZE); i++) {
        uint16_t extracted_uuid;
        UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[i * UUID16_SIZE]);

        if (extracted_uuid == UUID_INDICATOR) {
            UUID16_EXTRACT(&extracted_uuid,
                           &type_data.p_data[(i + 1) * UUID16_SIZE]);

            return extracted_uuid;
            // if(extracted_uuid == uuid_to_find) return true;
        }
    }
    return -1;
}

/**@brief Function to find polling rate bit encodeed in manufactor data
 */
static uint8_t find_adv_manu(ble_gap_evt_adv_report_t const *p_adv_report) {
    ret_code_t err_code;
    data_t adv_data;
    data_t type_data;

    // Initialize advertisement report for parsing.
    adv_data.p_data = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    err_code = adv_report_parse(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                &adv_data, &type_data);
    return type_data.p_data[2];
}

/**@brief   Function for handling BLE events from central applications.
 *
 * @details This function parses scanning reports and initiates a connection to
 * peripherals when a target UUID is found. It updates the status of LEDs used
 * to report central applications activity.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const *p_ble_evt) {
    ret_code_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
        // No peripheral devices will connect to node
        case BLE_GAP_EVT_CONNECTED: {
        } break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED: {
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_RSSI_CHANGED: {
            NRF_LOG_INFO("%d", &p_gap_evt->params.rssi_changed.rssi);
        } break;

        case BLE_GAP_EVT_ADV_REPORT: {

            if (strlen(m_target_periph_name) != 0) {

                if (find_adv_name(&p_gap_evt->params.adv_report,
                                  m_target_periph_name)) {
                    // Node Found
                    /*
                    Add to List

                     - Get UUID
                     - Get RSSI
                     - Get Current Timestamp
                     */

                    // Parse polling flag data from manufacturer data
                    uint8_t found_pollflag =
                            find_adv_manu(&p_gap_evt->params.adv_report);

                    // Find UUID in advertising package
                    uint16_t found_UUID =
                            find_adv_uuid_next(&p_gap_evt->params.adv_report);
                    // printf("found UUID: %d \n", found_UUID);

                    // Check found UUID already in neighbor list or not
                    if (found_UUID != NODE_UUID && !in_seen_list(found_UUID)) {

                        uint16_t index = get_seen_list_idx(0); // Get empty index
                        if (index == -1)                       // If list is full
                        {
                            index = MAX_ANCHOR_COUNT - 1; // Evict least RSSI
                        }
                        seen_list[last_seen_idx].UUID = found_UUID;
                        seen_list[last_seen_idx].RSSI =
                                p_ble_evt->evt.gap_evt.params.adv_report.rssi;
                        seen_list[last_seen_idx].ble_time_stamp = time_keeper;

                        // Update rate flag of the node in neighbor list
                        if (found_pollflag == '1') {
                            seen_list[last_seen_idx].polling_flag = 1;
                        }
                        if (found_pollflag == '0') {
                            seen_list[last_seen_idx].polling_flag = 0;
                        }

                        last_seen_idx += 1;
                        last_seen_idx %= MAX_ANCHOR_COUNT;
                        node_added = 1;
                    }

                    // Update neighbor list node value if found node already in list
                    if ((found_UUID != NODE_UUID) && in_seen_list(found_UUID)) {
                        int index = get_seen_list_idx(found_UUID);
                        seen_list[index].RSSI =
                                p_ble_evt->evt.gap_evt.params.adv_report.rssi;
                        seen_list[index].ble_time_stamp = time_keeper;

                        // Update rate flag of the node in neighbor list
                        if (found_pollflag == '1') {
                            seen_list[index].polling_flag = 1;
                        }
                        if (found_pollflag == '0') {
                            seen_list[index].polling_flag = 0;
                        }
                    }
                }
            } else {
            }
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT: {
            // We have not specified a timeout for scanning, so only connection
            // attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(
                    p_gap_evt->conn_handle,
                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys = {
                    .rx_phys = BLE_GAP_PHY_AUTO,
                    .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code =
                    sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code =
                    sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code =
                    sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the
 * peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const *p_ble_evt) {
    ret_code_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:

            // Update service connection handle
            m_cus.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            // Set gatts to enable create connection
            sd_ble_gatts_sys_attr_set(m_cus.conn_handle, NULL, 0, 0);

            // Keep advertising but with non-connectable package
            non_connectable_advertising_init(NON_CONNECTABLE_SET);
            scan_start();

            printf("Peripheral connected \n");
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            // Update service connection handle to invalid
            m_cus.conn_handle = BLE_CONN_HANDLE_INVALID;

            // Stop non-connectable advertising and begin original/connectable
            // advertising
            sd_ble_gap_adv_stop();
            adv_scan_start();

            printf("Peripheral disconnected \n");
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys = {
                    .rx_phys = BLE_GAP_PHY_AUTO,
                    .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code =
                    sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code =
                    sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code =
                    sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code =
                    sd_ble_user_mem_reply(p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
            ble_gatts_evt_rw_authorize_request_t req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID) {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    } else {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(
                            p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for checking if a bluetooth stack event is an advertising
 * timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const *p_ble_evt) {
    return ((p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT) &&
            (p_ble_evt->evt.gap_evt.params.timeout.src ==
             BLE_GAP_TIMEOUT_SRC_ADVERTISING));
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role = ble_conn_state_role(conn_handle);

    // Based on the role this device plays in the connection, dispatch to the
    // right handler.
    if (role == BLE_GAP_ROLE_PERIPH ||
        ble_evt_is_advertising_timeout(p_ble_evt)) {
        // printf("peripheral \r\n");
        on_ble_peripheral_evt(p_ble_evt);
    } else if ((role == BLE_GAP_ROLE_CENTRAL) ||
               (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT)) {
        // printf("central \r\n");
        on_ble_central_evt(p_ble_evt);
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler,
                         NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init(void) {
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
void delete_bonds(void) {
    ret_code_t err_code;

    // NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was
 * pressed to wake the application up.
 */
void buttons_leds_init(bool *p_erase_bonds) {
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 * parameters of the device including the device name, appearance, and the
 * preferred connection parameters.
 */
void gap_params_init(char *device_name) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(
            &sec_mode, (const uint8_t *)device_name, strlen(device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle =
            BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail = true;
    cp_init.evt_handler = NULL; // Ignore events.
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST: {
            NRF_LOG_INFO("Fast advertising.");
            // bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
        } break;

        case BLE_ADV_EVT_IDLE: {
            ret_code_t err_code =
                    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            // printf("ADV IDLE \r\n");
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            // printf("ADV Default \r\n");
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void) {

    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    // Set manufacturing data
    ble_advdata_manuf_data_t
            manuf_data;        // Variable to hold manufacturer specific data
    uint8_t data[2] = "0"; // Our data to advertise
    manuf_data.company_identifier = 0x0059; // Nordics company ID
    manuf_data.data.p_data = data;
    manuf_data.data.size = sizeof(data);
    init.advdata.p_manuf_specific_data = &manuf_data;

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // Edit service data section
    //    ble_advdata_service_data_t           service_data;
    //    uint8_array_t               data_array;
    //    uint8_t                     data_input[6];
    //    data_input[0] = 0xAA;
    //    data_input[1] = 0xBB;
    //    data_input[2] = 0xCC;
    //    data_input[3] = 0xDD;
    //    data_input[4] = 0xEE;
    //    data_input[5] = 0xFF;
    //    data_array.p_data = data_input;
    //    data_array.size = sizeof(data_input);
    //    service_data.service_uuid = CUSTOM_SERVICE_UUID;
    //    service_data.data = data_array;
    //    init.advdata.service_data_count = 1;
    //    init.advdata.p_service_data_array = &service_data;

    init.advdata.uuids_complete.uuid_cnt =
            sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;
    // init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) /
    // sizeof(m_adv_uuids[0]); init.srdata.uuids_complete.p_uuids  =
    // m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for re-config the Advertising package.
 */
void advertising_reconfig(int change) {

    ret_code_t err_code;
    ble_advdata_t advdata;

    ble_advdata_service_data_t service_data;
    uint8_array_t data_array;
    uint8_t data_input[1];
    data_input[0] = 0xFF;
    data_array.p_data = data_input;
    data_array.size = sizeof(data_input);
    service_data.service_uuid = CUSTOM_SERVICE_UUID;
    service_data.data = data_array;

    uint8_t data_0[2] = "0";
    uint8_t data_1[2] = "1";

    memset(&advdata, 0, sizeof(advdata));

    // Set manufacturing data
    ble_advdata_manuf_data_t
            manuf_data; // Variable to hold manufacturer specific data

    uint8_t data[2]; // Our data to advertise
    if (change == 0) {
        strcpy(data, data_0);
    } else {
        strcpy(data, data_1);
    }

    manuf_data.company_identifier = 0x0059; // Nordics company ID
    manuf_data.data.p_data = data;
    manuf_data.data.size = sizeof(data);
    advdata.p_manuf_specific_data = &manuf_data;

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt =
            sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids = m_adv_uuids;

    // init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) /
    // sizeof(m_adv_uuids[0]); init.srdata.uuids_complete.p_uuids  = m_adv_uuids

    advdata.service_data_count = 1;
    advdata.p_service_data_array = &service_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start non-connectable advertising
 */
void non_connectable_advertising_init(int mode) {

    uint32_t err_code;
    ble_gap_adv_params_t adv_params;

    memset(&adv_params, 0, sizeof(ble_gap_adv_params_t));
    if (mode == NON_CONNECTABLE_SET) {
        adv_params.type = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    }
    if (mode == NON_CONNECTABLE_RECOVER) {
        adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
    }
    adv_params.p_peer_addr = NULL;
    adv_params.fp = BLE_GAP_ADV_FP_ANY;
    adv_params.interval = 200;
    adv_params.timeout = 0;

    err_code = sd_ble_gap_adv_start(&adv_params, BLE_CONN_CFG_TAG_DEFAULT);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing logging.
 */
void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/** @brief Function to sleep until a BLE event is received by the application.
 */
void power_manage(void) {
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/** @brief Function to check the given UUID in neighbor list
 */
static bool in_seen_list(uint16_t UUID) {

    for (int i = 0; i < MAX_ANCHOR_COUNT; i++) {

        if (seen_list[i].UUID == UUID) {

            return true;
        }
    }
    return false;
}

/** @brief Function to retrive the corresponding neighbor list index by UUID
 */
static int get_seen_list_idx(uint16_t UUID) {

    for (int i = 0; i < MAX_ANCHOR_COUNT; i++) {

        if (seen_list[i].UUID == UUID) {

            return i;
        }
    }
    return -1;
}

/**@brief Function for initializing services that will be used by the
 * application.
 */
void services_init(void) {
    ret_code_t err_code;
    ble_cus_init_t cus_init;

    // Initialize CUS Service init structure to zero.
    memset(&cus_init, 0, sizeof(cus_init));

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);
}

void change_char_value(void) {
    ble_gatts_value_t gatts_value;
    static uint8_t value[14];
    static uint16_t length = 14;
    memset(&gatts_value, 0, sizeof(gatts_value));

    static int x = 1;
    if (x == 1) {
        value[0] = 1;    // Distance type
        value[1] = 1;    // Distance count
        value[2] = 0x1;  //
        value[3] = 0;    // Node 1 ID
        value[4] = 0xdc; //
        value[5] = 0x5;  //
        value[6] = 0;    //
        value[7] = 0;    // node 1 range
        value[8] = 10;   // node 1 quality
        value[9] = 0;
        value[10] = 0; //
        value[11] = 0; //
        value[12] = 0; //
        value[13] = 0;
    }
    if (x == 2) {
        value[0] = 1;    // Distance type
        value[1] = 1;    // Distance count
        value[2] = 0x2;  //
        value[3] = 0;    // Node 2 ID
        value[4] = 0xd0; //
        value[5] = 0x7;  //
        value[6] = 0;    //
        value[7] = 0;    // node 2 range
        value[8] = 20;   // node 2 quality
        value[9] = 0;
        value[10] = 0; //
        value[11] = 0; //
        value[12] = 0; //
        value[13] = 0;
    }
    if (x == 3) {
        value[0] = 1;    // Distance type
        value[1] = 1;    // Distance count
        value[2] = 0x3;  //
        value[3] = 0;    // Node 3 ID
        value[4] = 0xb8; //
        value[5] = 0xb;  //
        value[6] = 0;    //
        value[7] = 0;    // node 3 range
        value[8] = 30;   // node 3 quality
        value[9] = 0;
        value[10] = 0; //
        value[11] = 0; //
        value[12] = 0; //
        value[13] = 0;
    }

    gatts_value.len = 14;
    gatts_value.offset = 0;
    gatts_value.p_value = value;

    int err = sd_ble_gatts_value_set(
            BLE_CONN_HANDLE_INVALID, m_cus.dt_handles.value_handle, &gatts_value);
    // printf("%x \r", err);

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = m_cus.dt_handles.value_handle;
    hvx_params.p_data = NULL;
    hvx_params.offset = 0;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(m_cus.conn_handle, &hvx_params);

    x += 1;
    if (x == 4)
        x = 1;
}

/** @brief Function to change characteristic value in BLE service and notify
 */
void update_char_value(uint8 UUID, float range) {

    ble_gatts_value_t gatts_value;
    static uint8_t value[14];
    static uint16_t length = 14;
    memset(&gatts_value, 0, sizeof(gatts_value));

    int32 range_int = range * 1000;
    uint8_t *array;
    array = (uint8_t *)&range_int;

    value[0] = 1;            // Distance type
    value[1] = 1;            // Distance count
    value[2] = UUID;         //
    value[3] = 0;            // Node ID
    value[4] = *(array + 0); //
    value[5] = *(array + 1); //
    value[6] = *(array + 2); //
    value[7] = *(array + 3); // node range
    value[8] = 10;           // node quality
    value[9] = 0;
    value[10] = 0;
    value[11] = 0;
    value[12] = 0;
    value[13] = 0;

    // printf("value[4]: %x \n", array[0]);
    // printf("value[5]: %x \n", array[1]);
    // printf("value[6]: %x \n", array[2]);
    // printf("value[7]: %x \n", array[3]);

    gatts_value.len = 14;
    gatts_value.offset = 0;
    gatts_value.p_value = value;

    int err = sd_ble_gatts_value_set(
            BLE_CONN_HANDLE_INVALID, m_cus.dt_handles.value_handle, &gatts_value);
    // printf("%x \r", err);

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = m_cus.dt_handles.value_handle;
    hvx_params.p_data = NULL;
    hvx_params.offset = 0;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(m_cus.conn_handle, &hvx_params);
}
