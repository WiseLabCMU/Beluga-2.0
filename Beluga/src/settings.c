/**
 * @file settings.c
 * @brief Beluga settings management module.
 *
 * Contains functions and structures related to managing settings
 * for the Beluga. It includes functionality for reading, writing,
 * resetting settings. The settings are stored as key-value
 * pairs and can be persisted using a storage backend. This module handles
 * both the initialization of settings from persistent storage and the
 * runtime handling of settings updates.
 *
 * @author Tom Schmitz
 * @date 7/11/24
 */

#include <settings.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <deca_regs.h>

/**
 * Logger for the settings
 */
LOG_MODULE_REGISTER(settings_logger, CONFIG_SETTINGS_MODULE_LOG_LEVEL);

#if defined(CONFIG_SETTINGS_FILE)
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#endif

/**
 * The name of the storage partition
 */
#define STORAGE_PARTITION storage_partition

/**
 * The storage partition device
 */
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

/**
 * @brief A structure to store a key-value pair for settings.
 *
 * This structure represents a dictionary entry where each entry consists of
 * a key (a string) and a corresponding value (an integer).
 */
struct beluga_settings_dict {
    const char *key; ///< The key associated with the setting
    int32_t value;   ///< The value of the setting
};

#define GENERATE_BELUGA_DEFAULTS(name_, value_) value_,

/**
 * Default values associated with each setting
 *
 * @note Each value is mapped to the value of the beluga settings enumerator
 */
static const int32_t default_settings[] = {
    FOREACH_BELUGA_SETTING(GENERATE_BELUGA_DEFAULTS)};

#undef GENERATE_BELUGA_DEFAULTS

#define GENERATE_BELUGA_SETTINGS(name_, value_) {#name_, value_},

/**
 * Runtime storage of the settings
 */
static struct beluga_settings_dict settingValues[] = {
    FOREACH_BELUGA_SETTING(GENERATE_BELUGA_SETTINGS)};

#undef GENERATE_BELUGA_SETTINGS

/**
 * Maximum length of a setting key
 */
#define MAX_NAME_LENGTH SETTINGS_MAX_NAME_LEN

/**]
 * @brief Settings handle to get a specific value from runtime construct
 * @param[in] name The setting key
 * @param[in] val The setting value
 * @param[in] val_len_max Maximum length of the setting value in bytes (unused)
 *
 * @return -ENOENT if key was not found in settings dictionary
 * @return The length of the value
 */
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

/**
 * @brief Settings handle for loading settings from persistent storage
 *
 * @param[in] name The key name
 * @param[in] len Unused parameter
 * @param[in] read_cb Callback for reading the setting from storage
 * @param[in] cb_arg Additional context for the storage
 *
 * @return The number of bytes read upon success
 * @return -ENOENT if entry was not found
 * @return 0 if the entry was deleted
 * @return negative error code otherwise
 */
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

/**
 * @brief Handler for indicating that settings have been loaded in full
 * @return 0
 */
static int beluga_handle_commit(void) {
    LOG_INF("Loading all settings under <beluga> handler is done");
    return 0;
}

/**
 * @brief Handler to write out all the current settings
 * @param cb The callback that saves the settings
 * @return 0
 */
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

/**
 * Static settings handlers for "beluga"
 */
SETTINGS_STATIC_HANDLER_DEFINE(BelugaSettings, "beluga", beluga_handle_get,
                               beluga_handle_set, beluga_handle_commit,
                               beluga_handle_export);

/**
 * @brief Write a new value for a beluga setting
 * @param[in] setting The beluga setting to update
 * @param[in] value The new value of the setting
 */
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

/**
 * @brief Gets a current beluga setting
 *
 * @param[in] setting The beluga setting to retrieve
 *
 * @return The value of the setting
 * @return -1 if setting is invalid
 */
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

/**
 * @brief Erases/resets all the beluga settings to their default values
 */
void resetBelugaSettings(void) {
    for (size_t i = 0; i < ARRAY_SIZE(settingValues); i++) {
        settingValues[i].value = default_settings[i];
    }

    LOG_INF("Reset all settings");
    settings_save();
}

/**
 * @brief Initializes the settings subsystem and loads all the settings from
 * persistent storage
 * @return 0 upon success
 * @return negative error code otherwise
 */
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

    rc = settings_load();

    LOG_INF("settings subsys initialization: OK.\n");

    // Init any dynamic settings here

    return rc;
}
