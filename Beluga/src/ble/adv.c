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
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include <range_extension.h>

#include <ble/ble_app.h>
#include <ble/ble_app_internal.h>
#include <ble/scan.h>
#include <ble/services/beluga_service.h>

LOG_MODULE_REGISTER(ble_test, LOG_LEVEL_DBG);

#define NCONN_ADV_IDX      0
#define CONN_ADV_IDX       1

#define ADV_NAME_IDX       1
#define SCAN_MANF_DATA_IDX 0
#define SCAN_UUID_IDX      1

////////////////////////////////////////////////////////////////

static void advertising_work_handle(struct k_work *work);
static K_WORK_DEFINE(advertising_work, advertising_work_handle);

static struct bt_le_ext_adv *ext_adv[CONFIG_BT_EXT_ADV_MAX_ADV_SET];
static const struct bt_le_adv_param *nconn_adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_SCANNABLE, 0x140, 0x190, NULL);

static const struct bt_le_adv_param *conn_adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2, NULL);

static struct bt_data nconn_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
};

static struct bt_data conn_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
};

static struct bt_data scan_data[] = {
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
};

static struct bt_conn *central_conn;

static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};

static char const m_target_peripheral_name[] = "BN ";
static char adv_name[10];
static uint8_t uuid_encoded[sizeof(uint16_t)];

/////////////////////////////////////////////////////////////////////////////

static void adv_connected_cb(struct bt_le_ext_adv *adv,
                             struct bt_le_ext_adv_connected_info *info) {
    ARG_UNUSED(adv);
    ARG_UNUSED(info);
    LOG_INF("Advertiser[%d] %p connected conn %p", bt_le_ext_adv_get_index(adv),
            adv, info->conn);
}

static const struct bt_le_ext_adv_cb adv_cb = {
    .connected = adv_connected_cb,
};

static void connectable_adv_start(void) {
    // TODO: Update advertising mode
    int err =
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Failed to start connectable advertising (%d)", err);
    }
    start_active_scanning();
}

static void advertising_work_handle(struct k_work *work) {
    ARG_UNUSED(work);
    connectable_adv_start();
}

static void connected(struct bt_conn *conn, uint8_t conn_err) {
    int err;
    struct bt_conn_info info;

    if (conn_err) {
        LOG_ERR("Failed to connect (%u)", err);

        if (conn == central_conn) {
            bt_conn_unref(conn);
            central_conn = NULL;
            start_passive_scanning();
        }
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
        // TODO: Update advertising mode
        LOG_INF("Connected as peripheral");
    }
}

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
        start_passive_scanning();
    } else {
        bt_le_scan_stop();
        k_work_submit(&advertising_work);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected};

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

static int non_connectable_adv_create(void) {
    int err = advertising_set_create(&ext_adv[NCONN_ADV_IDX], nconn_adv_param,
                                     nconn_ad_data, ARRAY_SIZE(nconn_ad_data),
                                     scan_data, ARRAY_SIZE(scan_data));

    if (err) {
        LOG_ERR("Failed to create a non-connectable advertising set (%d)", err);
        return err;
    }

    return 0;
}

static int connectable_adv_create(void) {
    int err = advertising_set_create(&ext_adv[CONN_ADV_IDX], conn_adv_param,
                                     conn_ad_data, ARRAY_SIZE(nconn_ad_data),
                                     NULL, 0);

    if (err) {
        LOG_ERR("Failed to create a connectable advertising set (%d)", err);
    }

    return err;
}

int init_advertising(void) {
    int err;

    err = non_connectable_adv_create();
    if (err) {
        return err;
    }

    return connectable_adv_create();
}

void stop_advertising(void) {
    int err;

    err = bt_le_ext_adv_stop(ext_adv[NCONN_ADV_IDX]);
    if (err) {
        LOG_ERR("Unable to stop advertising for the non-connectable set (%d)",
                err);
    }

    err = bt_le_ext_adv_stop(ext_adv[CONN_ADV_IDX]);
    if (err) {
        LOG_ERR("Unable to stop advertising for the non-connectable set (%d)",
                err);
    }
}

void start_advertising(void) {
    // TODO Check advertising mode
    int err;

    err = bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX],
                              BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Unable to start non-connectable advertising (%d)", err);
    }

    err =
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Unable to start connectable advertising (%d)", err);
    }
}

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

void advertising_reconfig(struct advertising_info *uwb_metadata) {
    struct bt_data manf_info;
    // TODO internal_stop_ble();

    update_manufacturer_info(uwb_metadata);
    manf_info.type = BT_DATA_MANUFACTURER_DATA;
    manf_info.data = beluga_manufacturer_data;
    manf_info.data_len = sizeof(beluga_manufacturer_data);
    scan_data[SCAN_MANF_DATA_IDX] = manf_info;

    // TODO internal_start_ble();
}

void update_adv_name(uint16_t uuid) {
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

    // TODO internal_stop_ble();
    nconn_ad_data[ADV_NAME_IDX] = name_data;
    conn_ad_data[ADV_NAME_IDX] = name_data;
    scan_data[SCAN_UUID_IDX] = uuid_data;
    // TODO internal_start_ble();
}
