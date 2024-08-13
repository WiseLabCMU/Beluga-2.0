//
// Created by tom on 7/11/24.
//

#include <settings.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

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

static struct beluga_settings_dict settingValues[] = {
    {"id", DEFAULT_ID_SETTING},       {"boot_mode", DEFAULT_SETTING},
    {"poll_rate", DEFAULT_SETTING},   {"uwb_channel", DEFAULT_SETTING},
    {"ble_timeout", DEFAULT_SETTING}, {"tx_power", DEFAULT_SETTING},
    {"stream_mode", DEFAULT_SETTING}, {"twr", DEFAULT_SETTING},
    {"led_mode", DEFAULT_SETTING},
    {"fem_programmed", DEFAULT_SETTING}};

static struct beluga_settings_dict staticSettingValues[] = {
        {"fem_programmed", DEFAULT_SETTING}
};

#define LONGEST_SETTING_NAME_LEN 11
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
                printk("<beluga/%s> = %d\n", settingValues[i].key,
                       settingValues[i].value);
            }
            break;
        }
    }

    return rc;
}

static int beluga_handle_commit(void) {
    printk("Loading all settings under <beluga> handler is done\n");
    return 0;
}

static int beluga_handle_export(int (*cb)(const char *name, const void *value,
                                          size_t val_len)) {
    char name[2 * MAX_NAME_LENGTH];
    printk("export keys under <beluga> handler\n");

    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        snprintf(name, MAX_NAME_LENGTH, "beluga/%s", settingValues[i].key);
        (void)cb(name, &settingValues[i].value, sizeof(settingValues[0].value));
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(BelugaSettings, "beluga", beluga_handle_get,
                               beluga_handle_set, beluga_handle_commit,
                               beluga_handle_export);

static int static_beluga_get(const char *name, char *val, int val_len_max) {
    ARG_UNUSED(val_len_max);
    const char *next;

    for(size_t i = 0; i < ARRAY_SIZE(staticSettingValues); i++) {
        if (settings_name_steq(name, staticSettingValues[i].key, &next) && !next) {
            memcpy(val, &staticSettingValues[i].value, sizeof(staticSettingValues[0].value));
            return sizeof(staticSettingValues[0].value);
        }
    }
    return -ENOENT;
}

static int static_beluga_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    ARG_UNUSED(len);
    const char *next;
    size_t name_len;
    int rc = -ENOENT;

    name_len = settings_name_next(name, &next);

    for (size_t i = 0; (i < ARRAY_SIZE(staticSettingValues)) && !next; i++) {
        if (strncmp(name, staticSettingValues[i].key, name_len) == 0) {
            rc = read_cb(cb_arg, &staticSettingValues[i].value, sizeof(staticSettingValues[0].value));

            if (rc > 0) {
                printk("static_beluga/%s> = %d\n", staticSettingValues[i].key, staticSettingValues[i].value);
            }
            break;
        }
    }
    return rc;
}

static int static_beluga_handle_commit(void) {
    printk("Loading all settings under <static_beluga> handler is done\n");
    return 0;
}

static int static_beluga_handle_export(int (*cb)(const char *name, const void *value,
                                          size_t val_len)) {
    char name[2 * MAX_NAME_LENGTH];
    printk("export keys under <static_beluga> handler\n");

    for (size_t i = 0; i < ARRAY_SIZE(staticSettingValues); i++) {
        snprintf(name, MAX_NAME_LENGTH, "static_beluga/%s", staticSettingValues[i].key);
        (void)cb(name, &staticSettingValues[i].value, sizeof(staticSettingValues[0].value));
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(StaticBelugaSettings, "static_beluga", static_beluga_get,
                               static_beluga_set, static_beluga_handle_commit,
                               static_beluga_handle_export);

void updateSetting(enum beluga_setting setting, int32_t value) {
    char name[2 * MAX_NAME_LENGTH];
    int rc;

    if (setting >= BELUGA_RESERVED) {
        return;
    }
    snprintf(name, MAX_NAME_LENGTH, "beluga/%s", settingValues[setting].key);

    rc = settings_save_one(name, (const void *)&value, sizeof(value));

    if (rc) {
        printk("Error saving %s (%d)\n", name, rc);
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

void updateStaticSetting(enum beluga_static_settings setting, int32_t value) {
    char name[2 *MAX_NAME_LENGTH];
    int rc;

    if (setting >= BELUGA_STATIC_RESERVED) {
        return;
    }
    snprintf(name, MAX_NAME_LENGTH, "static_beluga/%s", staticSettingValues[setting].key);

    rc = settings_save_one(name, (const void *)&value, sizeof(value));

    if (rc) {
        printk("Error saving %s (%d)\n", name, rc);
    }
}

int32_t retrieveStaticSetting(enum beluga_static_settings setting) {
    int32_t retVal = DEFAULT_SETTING;

    if (setting >= BELUGA_STATIC_RESERVED) {
        return retVal;
    }

    retVal = staticSettingValues[setting].value;

    return retVal;
}

void resetBelugaSettings(void) {
    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        settingValues[i].value = DEFAULT_SETTING;
    }
    settingValues[BELUGA_ID].value = DEFAULT_ID_SETTING;

    printk("Reset all settings\n");
    settings_save();
}

int initBelugaSettings(void) {
    int rc;

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
        printk("mounting littlefs error: [%d]\n", rc);
        return rc;
    } else {

        rc = fs_unlink(CONFIG_SETTINGS_FILE_PATH);
        if ((rc != 0) && (rc != -ENOENT)) {
            printk("can't delete config file%d\n", rc);
            return rc;
        } else {
            printk("FS initialized: OK\n");
        }
    }
#endif

    rc = settings_subsys_init();

    if (rc) {
        printk("settings subsys initialization: fail (err %d)\n", rc);
        return rc;
    }

    settings_load();

    printk("settings subsys initialization: OK.\n");

    // Init any dynamic settings here

    return rc;
}
