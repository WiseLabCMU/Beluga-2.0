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
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include <range_extension.h>

#include <ble/ble_app.h>
#include <ble/ble_app_internal.h>
#include <ble/scan.h>
#include <ble/services/beluga_service.h>

LOG_MODULE_DECLARE(ble_app, CONFIG_BLE_APP_LOG_LEVEL);

#define NCONN_ADV_IDX      0
#define CONN_ADV_IDX       1

#define NCONN_ADV_NAME_IDX       1
#define NCONN_SCAN_MANF_DATA_IDX 0
#define NCONN_SCAN_UUID_IDX      1

#define CONN_ADV_UUID_IDX 2
#define CONN_ADV_MANF_DATA_IDX 3
#define CONN_SCAN_NAME_IDX 0

enum advertising_state {
    ADVERTISING_OFF,
    ADVERTISING_NO_CONNECT,
    ADVERTISING_CONNECTABLE,
};

////////////////////////////////////////////////////////////////

K_SEM_DEFINE(ble_connect_status, 1, 1);

static void advertising_work_handle(struct k_work *work);
static K_WORK_DEFINE(advertising_work, advertising_work_handle);

static void restart_ble_work_handle(struct k_work *work);
static K_WORK_DEFINE(restart_ble_work, restart_ble_work_handle);

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
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
};

static struct bt_data nconn_sd_data[] = {
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "\x59\x00\x30"),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1234)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BELUGA_SVC_VAL),
};

static struct bt_data conn_sd_data[] = {
        BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
};

static struct bt_conn *central_conn;

static uint8_t beluga_manufacturer_data[BLE_MANF_DATA_OVERHEAD] = {0};

static char const m_target_peripheral_name[] = "BN ";
static char adv_name[10];
static uint8_t uuid_encoded[sizeof(uint16_t)];
static enum advertising_state adv_state = ADVERTISING_OFF;

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
    LOG_DBG("Starting connectable advertising from work");
    int err =
        bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
    adv_state = (err == 0 || err == -EALREADY) ? ADVERTISING_CONNECTABLE : adv_state;
    if (err) {
        LOG_ERR("Advertising work: Failed to start connectable advertising (%d)", err);
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
        LOG_ERR("Failed to connect (%u) %s", conn_err, bt_hci_err_to_str(conn_err));

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
        k_work_submit(&restart_ble_work);
    } else {
        bt_le_scan_stop();
        k_work_submit(&advertising_work);
        k_sem_give(&ble_connect_status);
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
                                     nconn_sd_data, ARRAY_SIZE(nconn_sd_data));

    if (err) {
        LOG_ERR("Failed to create a non-connectable advertising set (%d)", err);
        return err;
    }

    return 0;
}

static int connectable_adv_create(void) {
    int err = advertising_set_create(&ext_adv[CONN_ADV_IDX], conn_adv_param,
                                     conn_ad_data, ARRAY_SIZE(conn_ad_data),
                                     conn_sd_data, ARRAY_SIZE(conn_sd_data));

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

static void refresh_advertising_data(void) {
    int err;

    err = bt_le_ext_adv_set_data(ext_adv[NCONN_ADV_IDX], nconn_ad_data, ARRAY_SIZE(nconn_ad_data), nconn_sd_data,
                                 ARRAY_SIZE(nconn_sd_data));
    if (err) {
        LOG_ERR("Unable to update non-connectable advertising data (%d)", err);
    }

    err = bt_le_ext_adv_set_data(ext_adv[CONN_ADV_IDX], conn_ad_data, ARRAY_SIZE(conn_ad_data), conn_sd_data,
                                 ARRAY_SIZE(conn_sd_data));
    if (err) {
        LOG_ERR("Unable to update connectable advertising data (%d)", err);
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

void update_adv_name(uint16_t uuid) {
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

struct bt_conn **get_central_connection_obj(void) {
    return &central_conn;
}

void internal_stop_ble(void) {
    LOG_DBG("Stopping BLE");
    bt_le_ext_adv_stop(ext_adv[CONN_ADV_IDX]);
    bt_le_ext_adv_stop(ext_adv[NCONN_ADV_IDX]);
    stop_scanning();
}

void internal_start_ble(void) {
    LOG_DBG("Resuming BLE");
    switch(adv_state) {
        case ADVERTISING_CONNECTABLE: {
            LOG_DBG("Starting connectable and non-connectable advertising");
            bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
            bt_le_ext_adv_start(ext_adv[CONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
            start_active_scanning();
            break;
        }
        case ADVERTISING_NO_CONNECT: {
            LOG_DBG("Starting non-connectable advertising");
            bt_le_ext_adv_start(ext_adv[NCONN_ADV_IDX], BT_LE_EXT_ADV_START_DEFAULT);
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

static void restart_ble_work_handle(struct k_work *work) {
    ARG_UNUSED(work);
    stop_advertising();
    stop_scanning();
    start_advertising();
    start_active_scanning();
}

int wait_ble_disconnect(k_timeout_t timeout) {
    int ret = k_sem_take(&ble_connect_status, timeout);
    if (ret) {
        return ret;
    }
    k_sem_give(&ble_connect_status);
    return 0;
}
