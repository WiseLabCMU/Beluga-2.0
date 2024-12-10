//
// Created by tom on 7/11/24.
//

#include <settings.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(settings_logger, CONFIG_SETTINGS_MODULE_LOG_LEVEL);

#define DEFAULT_RATE          250
#define DEFAULT_TIMEOUT       9000
#define DEFAULT_STREAMMODE    0
#define DEFAULT_LEDMODE       0
#define DEFAULT_BOOTMODE      0
#define DEFAULT_CHANNEL       5
#define DEFAULT_TXPOWER       0
#define DEFAULT_TWR           1
#define DEFAULT_OUT_FORMAT    0
#define DEFAULT_AMPLIFICATION 0
#define DEFAULT_PHR           0
#define DEFAULT_DATARATE      0 // 6M8
#define DEFAULT_PULSERATE     1 // 64M
#define DEFAULT_PREAMBLE      128
#define DEFAULT_PAC           0 // 8
#define DEFAULT_NSSFD         0 // Standard SFD
#define DEFAULT_PAN_ID        0xDECA

#if defined(CONFIG_SETTINGS_FILE)
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#endif

#define STORAGE_PARTITION    storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

struct beluga_settings_dict {
    const char *key;
    int32_t value;
};

static const int32_t default_settings[] = {
    DEFAULT_ID_SETTING, DEFAULT_BOOTMODE,      DEFAULT_RATE,
    DEFAULT_CHANNEL,    DEFAULT_TIMEOUT,       DEFAULT_TXPOWER,
    DEFAULT_STREAMMODE, DEFAULT_TWR,           DEFAULT_LEDMODE,
    DEFAULT_OUT_FORMAT, DEFAULT_AMPLIFICATION, DEFAULT_PHR,
    DEFAULT_DATARATE,   DEFAULT_PULSERATE,     DEFAULT_PREAMBLE,
    DEFAULT_PAC,        DEFAULT_NSSFD,         DEFAULT_PAN_ID};

static struct beluga_settings_dict settingValues[] = {
    {"id", DEFAULT_ID_SETTING},
    {"boot_mode", DEFAULT_BOOTMODE},
    {"poll_rate", DEFAULT_RATE},
    {"uwb_channel", DEFAULT_CHANNEL},
    {"ble_timeout", DEFAULT_TIMEOUT},
    {"tx_power", DEFAULT_TXPOWER},
    {"stream_mode", DEFAULT_STREAMMODE},
    {"twr", DEFAULT_TWR},
    {"led_mode", DEFAULT_LEDMODE},
    {"out_format", DEFAULT_OUT_FORMAT},
    {"range_ext", DEFAULT_AMPLIFICATION},
    {"phr", DEFAULT_PHR},
    {"datarate", DEFAULT_DATARATE},
    {"pulserate", DEFAULT_PULSERATE},
    {"preamble", DEFAULT_PREAMBLE},
    {"pac", DEFAULT_PAC},
    {"nssfd", DEFAULT_NSSFD},
    {"pan_id", DEFAULT_PAN_ID},
};

#define LONGEST_SETTING_NAME_LEN SETTINGS_MAX_NAME_LEN
#define BELUGA_LEN               6
#define SEPARATOR_LEN            1
#define MAX_NAME_LENGTH                                                        \
    (LONGEST_SETTING_NAME_LEN + SEPARATOR_LEN + BELUGA_LEN + 1)

static int beluga_handle_get(const char *name, char *val, int val_len_max) {
    ARG_UNUSED(val_len_max);
    const char *next;

    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        if (settings_name_steq(name, settingValues[i].key, &next) && !next) {
            memcpy(val, &settingValues[i].value,
                   sizeof(settingValues[0].value));
            return sizeof(settingValues[0].value);
        }
    }

    return -ENOENT;
}

static int beluga_handle_set(const char *name, size_t len,
                             settings_read_cb read_cb, void *cb_arg) {
    ARG_UNUSED(len);
    const char *next;
    size_t name_len;
    int rc = -ENOENT;

    name_len = settings_name_next(name, &next);

    for (size_t i = 0; (i < ARRAY_SIZE(settingValues)) && !next; i++) {
        if (strncmp(name, settingValues[i].key, name_len) == 0) {
            rc = read_cb(cb_arg, &settingValues[i].value,
                         sizeof(settingValues[0].value));
            if (rc > 0) {
                LOG_INF("<beluga/%s> = %d", settingValues[i].key,
                        settingValues[i].value);
            }
            break;
        }
    }

    return rc;
}

static int beluga_handle_commit(void) {
    LOG_INF("Loading all settings under <beluga> handler is done");
    return 0;
}

static int beluga_handle_export(int (*cb)(const char *name, const void *value,
                                          size_t val_len)) {
    char name[2 * MAX_NAME_LENGTH];
    LOG_INF("export keys under <beluga> handler\n");

    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        snprintf(name, MAX_NAME_LENGTH, "beluga/%s", settingValues[i].key);
        (void)cb(name, &settingValues[i].value, sizeof(settingValues[0].value));
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(BelugaSettings, "beluga", beluga_handle_get,
                               beluga_handle_set, beluga_handle_commit,
                               beluga_handle_export);

void updateSetting(enum beluga_setting setting, int32_t value) {
    char name[2 * MAX_NAME_LENGTH];
    int rc;

    if (setting >= BELUGA_RESERVED) {
        return;
    }
    snprintf(name, MAX_NAME_LENGTH, "beluga/%s", settingValues[setting].key);

    rc = settings_save_one(name, (const void *)&value, sizeof(value));

    if (rc) {
        LOG_ERR("Error saving %s (%d)", name, rc);
    } else {
        settingValues[setting].value = value;
    }
}

int32_t retrieveSetting(enum beluga_setting setting) {
    int32_t retVal =
        (setting == BELUGA_ID) ? DEFAULT_ID_SETTING : DEFAULT_SETTING;

    if (setting >= BELUGA_RESERVED) {
        return retVal;
    }

    // Settings were loaded at initialization, saved whenever written
    retVal = settingValues[setting].value;

    return retVal;
}

void resetBelugaSettings(void) {
    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        settingValues[i].value = default_settings[i];
    }

    LOG_INF("Reset all settings");
    settings_save();
}

int initBelugaSettings(void) {
    int rc;

    __ASSERT(ARRAY_SIZE(default_settings) == ARRAY_SIZE(settingValues),
             "Setting arrays mismatch! Default settings: %" PRId32
             " != Setting values: %" PRId32,
             ARRAY_SIZE(default_settings), ARRAY_SIZE(settingValues));

#if defined(CONFIG_SETTINGS_FILE)
    FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);

    /* mounting info */
    static struct fs_mount_t littlefs_mnt = {.type = FS_LITTLEFS,
                                             .fs_data = &cstorage,
                                             .storage_dev =
                                                 (void *)STORAGE_PARTITION_ID,
                                             .mnt_point = "/ff"};

    rc = fs_mount(&littlefs_mnt);
    if (rc != 0) {
        LOG_ERR("mounting littlefs error: [%d]", rc);
        return rc;
    } else {

        rc = fs_unlink(CONFIG_SETTINGS_FILE_PATH);
        if ((rc != 0) && (rc != -ENOENT)) {
            LOG_ERR("can't delete config file%d", rc);
            return rc;
        } else {
            LOG_INF("FS initialized: OK\n");
        }
    }
#endif

    rc = settings_subsys_init();

    if (rc) {
        LOG_ERR("settings subsys initialization: fail (err %d)\n", rc);
        return rc;
    }

    settings_load();

    LOG_INF("settings subsys initialization: OK.\n");

    // Init any dynamic settings here

    return rc;
}
