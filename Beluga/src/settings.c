//
// Created by tom on 7/11/24.
//

#include <settings.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include <errno.h>

#if defined(CONFIG_SETTINGS_FILE)
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#endif

#define STORAGE_PARTITION    storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

struct beluga_settings_vals {
    const char *name;
    int32_t value;
};

int beluga_handle_get(const char *name, char *val, int val_len_max) {
    return -ENOENT;
}

int beluga_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                      void *cb_arg) {
    return -ENOENT;
}

int beluga_handle_commit(void) {
    printk("Loading all settings under <beluga> handler is done\n");
    return 0;
}

int beluga_handle_export(int (*cb)(const char *name, const void *value,
                                   size_t val_len)) {
    printk("export keys under <beluga> handler\n");
    // TODO
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(BelugaSettings, "beluga", beluga_handle_get,
                               beluga_handle_set, beluga_handle_commit,
                               beluga_handle_export);

void updateSetting(enum beluga_setting setting, int32_t value) {
    // TODO
}

int32_t retrieveSetting(enum beluga_setting setting) {
    // TODO
    return DEFAULT_SETTING;
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

    printk("settings subsys initialization: OK.\n");

    // Init any dynamic settings here

    return rc;
}
