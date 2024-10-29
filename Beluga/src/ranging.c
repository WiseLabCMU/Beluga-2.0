//
// Created by tom on 7/9/24.
//

#include <app_leds.h>
#include <ble_app.h>
#include <deca_device_api.h>
#include <init_main.h>
#include <port_platform.h>
#include <random.h>
#include <ranging.h>
#include <resp_main.h>
#include <spi.h>
#include <stdbool.h>
#include <thread_priorities.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ranging_logger, CONFIG_RANGING_MODULE_LOG_LEVEL);

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100

/* Maximum transmission power register value */
#define TX_POWER_MAX 0x1F1F1F1F

#define SUSPEND_RESPONDER_TASK                                                 \
    do {                                                                       \
        k_sem_take(&k_sus_resp, K_NO_WAIT);                                    \
        k_sem_take(&k_sus_init, K_FOREVER);                                    \
        k_sleep(K_MSEC(2));                                                    \
    } while (0)

#define RESUME_RESPONDER_TASK                                                  \
    do {                                                                       \
        k_sem_give(&k_sus_init);                                               \
        k_sem_give(&k_sus_resp);                                               \
    } while (0)

static int32_t initiator_freq = 100;
static bool twr_mode = true;

// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
#define SFD_TO(PREAMBLE, SFD_LENGTH, PAC_SIZE)                                 \
    ((PREAMBLE) + 1 + (SFD_LENGTH) - (PAC_SIZE))

/* DW1000 config struct */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    10,              /* TX preamble code. Used in TX only. */
    10,              /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    SFD_TO(128, 8, 8)};

/* DW1000 TX config struct */
static dwt_txconfig_t config_tx = {TC_PGDELAY_CH5, TX_POWER_MAN_DEFAULT};

static volatile bool rangingStarted = false;
static struct task_wdt_attr watchdogAttr = {.period = 2000};

enum uwb_preamble_length setting_to_preamble_enum(int32_t setting) {
    enum uwb_preamble_length length;

    switch (setting) {
    case 0:
        length = UWB_PRL_64;
        break;
    case 1:
        length = UWB_PRL_128;
        break;
    case 2:
        length = UWB_PRL_256;
        break;
    case 3:
        length = UWB_PRL_512;
        break;
    case 4:
        length = UWB_PRL_1024;
        break;
    case 5:
        length = UWB_PRL_2048;
        break;
    case 6:
        length = UWB_PRL_4096;
        break;
    default:
        length = UWB_PRL_ERROR;
    }

    return length;
}

int32_t preamble_length_to_setting(enum uwb_preamble_length length) {
    int32_t setting;

    switch (length) {
    case UWB_PRL_64:
        setting = 0;
        break;
    case UWB_PRL_256:
        setting = 2;
        break;
    case UWB_PRL_512:
        setting = 3;
        break;
    case UWB_PRL_1024:
        setting = 4;
        break;
    case UWB_PRL_2048:
        setting = 5;
        break;
    case UWB_PRL_4096:
        setting = 6;
        break;
    case UWB_PRL_128:
    default:
        setting = 1;
        break;
    }

    return setting;
}

// Forces Preamble to acceptable value
bool set_uwb_data_rate(enum uwb_datarate rate,
                       enum uwb_preamble_length *new_preamble) {
    uint8 new_data_rate;

    if (new_preamble == NULL) {
        return false;
    }

    switch (rate) {
    case UWB_DR_6M8:
        new_data_rate = DWT_BR_6M8;
        *new_preamble = UWB_PRL_128;
        break;
    case UWB_DR_850K:
        new_data_rate = DWT_BR_850K;
        *new_preamble = UWB_PRL_512;
        break;
    case UWB_DR_110K:
        new_data_rate = DWT_BR_110K;
        *new_preamble = UWB_PRL_2048;
        break;
    default:
        return false;
    }

    config.dataRate = new_data_rate;
    config.txPreambLength = (uint8)*new_preamble;

    return set_uwb_preamble_length(*new_preamble);
}

bool set_uwb_preamble_length(enum uwb_preamble_length length) {
    uint8 newLength = UWB_PRL_ERROR, ns_sfd = 0;
    uint16 preamble_len = 0, pac_len, sfd_len;

    switch (config.dataRate) {
    case DWT_BR_6M8: {
        if (length == UWB_PRL_64 || length == UWB_PRL_128 ||
            length == UWB_PRL_256) {
            newLength = (uint8)length;
            sfd_len = 8;
        }
        break;
    }
    case DWT_BR_850K: {
        if (length == UWB_PRL_256 || length == UWB_PRL_512 ||
            length == UWB_PRL_1024) {
            newLength = (uint8)length;
            sfd_len = 16;
        }
    }
    case DWT_BR_110K: {
        if (length == UWB_PRL_2048 || length == UWB_PRL_4096) {
            newLength = (uint8)length;
            sfd_len = 64;
        }
        ns_sfd = 1;
        break;
    }
    default:
        return false;
    }

    switch (newLength) {
    case UWB_PRL_64:
        preamble_len = -64;
        // In fallthrough, preamble_len will get 128 added to it, thus
        // resulting in a preamble_len of 64 (which is the correct value)
    case UWB_PRL_128:
        preamble_len += 128;
        config.rxPAC = DWT_PAC8;
        pac_len = 8;
        break;
    case UWB_PRL_256:
        preamble_len = -256;
        // In fallthrough, preamble_len will get 256 added to it, thus
        // resulting in a preamble_len of 256 (which is the correct value)
    case UWB_PRL_512:
        preamble_len += 512;
        config.rxPAC = DWT_PAC16;
        pac_len = 16;
        break;
    case UWB_PRL_1024:
        preamble_len = 1024;
        config.rxPAC = DWT_PAC32;
        pac_len = 32;
        break;
    case UWB_PRL_2048:
        preamble_len = -2048;
        // In fallthrough, preamble_len will get 4096 added to it, thus
        // resulting in a preamble_len of 2048 (which is the correct value)
    case UWB_PRL_4096:
        preamble_len += 4096;
        config.rxPAC = DWT_PAC64;
        pac_len = 64;
        break;
    default:
        return false;
    }

    config.nsSFD = ns_sfd;
    config.sfdTO = SFD_TO(preamble_len, sfd_len, pac_len);
    config.txPreambLength = newLength;
    dwt_configure(&config);
    return true;
}

bool set_pulse_rate(enum uwb_pulse_rate rate) {
    switch (rate) {
    case UWB_PR_16M:
        config.prf = DWT_PRF_16M;
        break;
    case UWB_PR_64M:
        config.prf = DWT_PRF_64M;
        break;
    default:
        return false;
    }

    dwt_configure(&config);
    return true;
}

bool set_uwb_channel(uint32_t channel) {
    enum pgdelay_ch uwb_pgdelay;

    switch (channel) {
    case 1:
        uwb_pgdelay = ch1;
        break;
    case 2:
        uwb_pgdelay = ch2;
        break;
    case 3:
        uwb_pgdelay = ch3;
        break;
    case 4:
        uwb_pgdelay = ch4;
        break;
    case 5:
        uwb_pgdelay = ch5;
        break;
    case 7:
        uwb_pgdelay = ch7;
        break;
    default:
        return false;
    }

    config_tx.PGdly = uwb_pgdelay;
    config.chan = channel;
    dwt_configure(&config);
    dwt_configuretxrf(&config_tx);
    return true;
}

void set_tx_power(bool power_max) {
    if (power_max) {
        config_tx.power = TX_POWER_MAX;
    } else {
        config_tx.power = TX_POWER_MAN_DEFAULT;
    }
    dwt_configuretxrf(&config_tx);
}

void set_twr_mode(bool value) { twr_mode = value; }

bool get_twr_mode(void) { return twr_mode; }

void set_rate(uint32_t rate) {
    if (rate > (uint32_t)INT32_MAX) {
        return;
    }
    initiator_freq = (int32_t)rate;
}

uint32_t get_rate(void) { return initiator_freq; }

void init_uwb(void) {
    if (!IS_ENABLED(CONFIG_ENABLE_BELUGA_UWB)) {
        return;
    }
    setup_DW1000RSTnIRQ(0);
    toggle_cs_line(DW1000_SPI_CHANNEL, 400);

    reset_DW1000();

    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_ERR("Failed to load UWB code");
        while (true)
            ;
    }

    port_set_dw1000_fastrate();
    dwt_configure(&config);
    dwt_configuretxrf(&config_tx);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's timeout. (keep listening so timeout is 0) */
    dwt_setrxtimeout(0);

    LOG_INF("UWB initialized");
}

/**
 * @brief Reconfig UWB transmitter as an initiator
 */
static void init_reconfig() {
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(2000);
}

/**
 * @brief Reconfig UWB transmitter as a responder
 */
static void resp_reconfig() {
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
}

NO_RETURN void rangingTask(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    bool drop_flag = false;
    bool break_flag = false;
    static int curr_index = 0;

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        LOG_ERR("Unable to spawn ranging watchdog");
        while (1)
            ;
    }
    rangingStarted = true;

    while (true) {
        watchdog_red_rocket(&watchdogAttr);

        if (initiator_freq != 0) {
            k_msleep(initiator_freq);

            if (drop_flag) {
                uint16_t rand_small =
                    get_rand_num_exp_collision(initiator_freq);
                k_msleep(rand_small);
                drop_flag = false;
            }

            SUSPEND_RESPONDER_TASK;

            dwt_forcetrxoff();
            init_reconfig();

            int search_count = 0;
            double range1;

            while (seen_list[curr_index].UUID == 0) {
                curr_index++;

                if (curr_index >= MAX_ANCHOR_COUNT) {
                    curr_index = 0;
                }
                if (search_count >= (MAX_ANCHOR_COUNT - 1)) {
                    break_flag = true;
                    break;
                }
                search_count += 1;
            }

            if (!break_flag) {
                if (twr_mode) {
                    range1 = ds_init_run(seen_list[curr_index].UUID);
                } else {
                    range1 = ss_init_run(seen_list[curr_index].UUID);
                }

                if (range1 == -1) {
                    drop_flag = true;
                }

                int numThru = 1;

                if (range1 == -1) {
                    range1 = 0;
                    numThru -= 1;
                }

                float range = (range1) / numThru;

                if ((numThru != 0) && (range >= -5) && (range <= 100)) {
                    seen_list[curr_index].update_flag = 1;
                    seen_list[curr_index].range = range;
                    seen_list[curr_index].time_stamp = k_uptime_get();

                    // TODO: Update BLE value transfer to phone
                }

                curr_index += 1;

                if (curr_index >= MAX_ANCHOR_COUNT) {
                    curr_index = 0;
                }
            }

            break_flag = false;
            resp_reconfig();
            dwt_forcetrxoff();

            RESUME_RESPONDER_TASK;
        } else {
            k_msleep(1000);
        }

        int polling_count = 0;
        for (int x = 0; x < MAX_ANCHOR_COUNT; x++) {
            if (seen_list[x].UUID != 0 && seen_list[x].polling_flag != 0) {
                polling_count += 1;
            }
        }

        if (polling_count == 0) {
            SUSPEND_RESPONDER_TASK;
            dwt_forcetrxoff();
            resp_reconfig();
            dwt_forcetrxoff();
            RESUME_RESPONDER_TASK;
            k_sem_take(&k_sus_resp, K_NO_WAIT);
        } else {
            SUSPEND_RESPONDER_TASK;
            dwt_forcetrxoff();
            resp_reconfig();
            dwt_forcetrxoff();
            RESUME_RESPONDER_TASK;
            k_sem_give(&k_sus_resp);
        }
    }
}

/**
 * @brief SS TWR Initiator task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the
 * task.
 */
NO_RETURN static void responder_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (are_leds_on()) {
        dwt_setleds(DWT_LEDS_ENABLE);
    } else {
        dwt_setleds(DWT_LEDS_DISABLE);
    }

    while (!rangingStarted)
        ;

    while (true) {
        watchdog_red_rocket(&watchdogAttr);

        // Check if responding is suspended, return 0 means suspended
        unsigned int suspend_start = k_sem_count_get(&k_sus_resp);

        if (suspend_start != 0) {
            if (twr_mode) {
                ds_resp_run();
            } else {
                ss_resp_run();
            }
        }
    }
}

#if ENABLE_THREADS && ENABLE_RANGING
K_THREAD_STACK_DEFINE(ranging_stack, CONFIG_RANGING_STACK_SIZE);
static struct k_thread ranging_task_data;
static k_tid_t ranging_task_id;

void init_ranging_thread(void) {
    ranging_task_id =
        k_thread_create(&ranging_task_data, ranging_stack,
                        K_THREAD_STACK_SIZEOF(ranging_stack), rangingTask, NULL,
                        NULL, NULL, CONFIG_BELUGA_RANGING_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(ranging_task_id, "Ranging task");
    LOG_INF("Started ranging");
}
#else
void init_ranging_thread(void) { LOG_INF("Ranging disabled"); }
#endif

#if ENABLE_THREADS && ENABLE_RESPONDER
K_THREAD_STACK_DEFINE(responder_stack, CONFIG_RESPONDER_STACK_SIZE);
static struct k_thread responder_data;
static k_tid_t responder_task_id;

void init_responder_thread(void) {
    responder_task_id = k_thread_create(
        &responder_data, responder_stack,
        K_THREAD_STACK_SIZEOF(responder_stack), responder_task_function, NULL,
        NULL, NULL, CONFIG_BELUGA_RESPONDER_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(responder_task_id, "Responder task");
    LOG_INF("Started responder");
}
#else
void init_responder_thread(void) { LOG_INF("Responder disabled"); }
#endif
