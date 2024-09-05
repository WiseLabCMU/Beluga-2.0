//
// Created by tom on 7/9/24.
//

#include <ble_app.h>
#include <deca_device_api.h>
#include <init_main.h>
#include <port_platform.h>
#include <random.h>
#include <ranging.h>
#include <resp_main.h>
#include <stdbool.h>
#include <thread_priorities.h>
// #include <timestamp.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>

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
    (129 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size).
             Used in RX only. */
};

/* DW1000 TX config struct */
static dwt_txconfig_t config_tx = {TC_PGDELAY_CH5, TX_POWER_MAN_DEFAULT};

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

void set_twr_mode(bool value) {
    twr_mode = value;
}

bool get_twr_mode(void) {
    return twr_mode;
}

void set_rate(uint32_t rate) {
    if (rate > (uint32_t)INT32_MAX) {
        return;
    }
    initiator_freq = (int32_t)rate;
}

uint32_t get_rate(void) {
    return initiator_freq;
}

void init_uwb(void) {
    setup_DW1000RSTnIRQ(0);

    reset_DW1000();

    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        printk("Failed to load UWB code\r\n");
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

    printk("UWB initialized\n");
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

    while (true) {
        watchdog_red_rocket();

        if (initiator_freq != 0) {
            k_msleep(initiator_freq);

            if (drop_flag) {
                uint16_t rand_small = get_rand_num_exp_collision(initiator_freq);
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

                float range = (range1)/numThru;

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

#if ENABLE_THREADS && ENABLE_RANGING
K_THREAD_STACK_DEFINE(ranging_stack, CONFIG_RANGING_STACK_SIZE);
static struct k_thread ranging_task_data;
static k_tid_t ranging_task_id;

void init_ranging_thread(void) {
    ranging_task_id =
        k_thread_create(&ranging_task_data, ranging_stack,
                        K_THREAD_STACK_SIZEOF(ranging_stack), rangingTask, NULL,
                        NULL, NULL, CONFIG_BELUGA_RANGING_PRIO, 0, K_NO_WAIT);
    printk("Started ranging\n");
}
#else
void init_ranging_thread(void) { printk("Ranging disabled\n"); }
#endif
