//
// Created by tom on 7/9/24.
//

#include <app_leds.h>
#include <ble_app.h>
#include <deca_device_api.h>
#include <init_resp_common.h>
#include <initiator.h>
#include <port_platform.h>
#include <random.h>
#include <ranging.h>
#include <responder.h>
#include <spi.h>
#include <stdbool.h>
#include <stdio.h>
#include <thread_priorities.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logging module for the ranging module
 */
LOG_MODULE_REGISTER(ranging_logger, CONFIG_RANGING_MODULE_LOG_LEVEL);

/**
 * The delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature.
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100

#if !defined(CONFIG_UWB_INIT_RX_TIMEOUT)
#define UWB_INIT_TIMEOUT 2000
#else
#define UWB_INIT_TIMEOUT CONFIG_UWB_INIT_RX_TIMEOUT
#endif

#if !defined(CONFIG_UWB_RESP_RX_DELAY)
#define UWB_RESP_RX_DELAY 0
#else
#define UWB_RESP_RX_DELAY CONFIG_UWB_RESP_RX_DELAY
#endif

#if !defined(CONFIG_UWB_RESP_RX_TIMEOUT)
#define UWB_RESP_RX_TIMEOUT 0
#else
#define UWB_RESP_RX_TIMEOUT CONFIG_UWB_RESP_RX_TIMEOUT
#endif

#if defined(CONFIG_UWB_FILTER_RANGES)
#include <math.h>
#define LOWER_RANGE (double)CONFIG_UWB_RANGE_FILTER_LOWER_BOUND

#if CONFIG_UWB_RANGE_FILTER_UPPER_BOUND <= 0
#define UPPER_CONDITION(x) true
#else
#define UPPER_RANGE (double)CONFIG_UWB_RANGE_FILTER_UPPER_BOUND
#define UPPER_CONDITION(x) islessequal((x), UPPER_RANGE)
#endif

#define LOWER_CONDITION(x) isgreaterequal((x), LOWER_RANGE)
#define RANGE_CONDITION(x) (LOWER_CONDITION(x) && UPPER_CONDITION(x))

#else

#define RANGE_CONDITION(x) (true)

#endif

#define SUSPEND_RESPONDER_TASK()                                               \
    do {                                                                       \
        k_sem_take(&k_sus_resp, K_NO_WAIT);                                    \
        k_sem_take(&k_sus_init, K_FOREVER);                                    \
        k_sleep(K_MSEC(2));                                                    \
    } while (0)

#define RESUME_RESPONDER_TASK()                                                \
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

void print_tx_power(uint32_t tx_power) {
    printf("TX Power: 0x%08" PRIX32 " ", tx_power);
}

enum uwb_datarate print_uwb_datarate(enum uwb_datarate rate) {
    switch (rate) {
    case UWB_DR_850K:
        printf("Data Rate: 850 kHz ");
        break;
    case UWB_DR_110K:
        printf("Data Rate: 110 kHz ");
        break;
    case UWB_DR_6M8:
    default:
        printf("Data Rate: 6.8MHz ");
        rate = UWB_DR_6M8;
        break;
    }
    return rate;
}

enum uwb_pulse_rate print_pulse_rate(enum uwb_pulse_rate rate) {
    switch (rate) {
    case UWB_PR_16M:
        printf("Pulse Rate: 16MHz ");
        break;
    case UWB_PR_64M:
    default:
        printf("Pulse Rate: 64MHz ");
        rate = UWB_PR_64M;
        break;
    }
    return rate;
}

int32_t print_pac_size(int32_t pac) {
    switch ((enum uwb_pac)pac) {
    case UWB_PAC8:
        printf("PAC Size: 8 ");
        break;
    case UWB_PAC16:
        printf("PAC Size: 16 ");
        break;
    case UWB_PAC32:
        printf("PAC Size: 32 ");
        break;
    case UWB_PAC64:
        printf("PAC Size: 16 ");
        break;
    default:
        printf("PAC Size: 8 ");
        pac = (int32_t)UWB_PAC8;
        break;
    }
    return pac;
}

void print_pan_id(uint32_t pan_id) {
    printf("UWB PAN ID: 0x%04" PRIX16 " ", (uint16_t)pan_id);
}

int uwb_set_phr_mode(enum uwb_phr_mode mode) {
    CHECK_UWB_ACTIVE();

    switch (mode) {
    case UWB_PHR_MODE_STD:
        config.phrMode = DWT_PHRMODE_STD;
        break;
    case UWB_PWR_MODE_EXT:
        config.phrMode = DWT_PHRMODE_EXT;
        break;
    default:
        return -EINVAL;
    }

    dwt_configure(&config);
    return 0;
}

static uint16_t get_preamble_length(void) {
    switch (config.txPreambLength) {
    case DWT_PLEN_64:
        return 64;
    case DWT_PLEN_128:
        return 128;
    case DWT_PLEN_256:
        return 256;
    case DWT_PLEN_512:
        return 512;
    case DWT_PLEN_1024:
        return 1024;
    case DWT_PLEN_1536:
        return 1536;
    case DWT_PLEN_2048:
        return 2048;
    case DWT_PLEN_4096:
    default:
        return 4096;
    }
}

static uint16_t get_pac_size(void) {
    switch (config.rxPAC) {
    case DWT_PAC8:
        return 8;
    case DWT_PAC16:
        return 16;
    case DWT_PAC32:
        return 32;
    case DWT_PAC64:
    default:
        return 64;
    }
}

static uint16_t get_sfd_length(void) {
    switch (config.dataRate) {
    case DWT_BR_6M8:
        return DW_NS_SFD_LEN_6M8;
    case DWT_BR_850K:
        return (config.nsSFD) ? DW_NS_SFD_LEN_850K : DW_NS_SFD_LEN_6M8;
    case DWT_BR_110K:
    default:
        return DW_NS_SFD_LEN_110K;
    }
}

int uwb_set_datarate(enum uwb_datarate rate) {
    CHECK_UWB_ACTIVE();

    switch (rate) {
    case UWB_DR_6M8:
        config.dataRate = DWT_BR_6M8;
        break;
    case UWB_DR_850K:
        config.dataRate = DWT_BR_850K;
        break;
    case UWB_DR_110K:
        config.dataRate = DWT_BR_110K;
        break;
    default:
        return -EINVAL;
    }

    // Update the SFD length according to the data rate
    config.sfdTO =
        SFD_TO(get_preamble_length(), get_sfd_length(), get_pac_size());

    dwt_configure(&config);
    return 0;
}

int uwb_set_pulse_rate(enum uwb_pulse_rate rate) {
    CHECK_UWB_ACTIVE();

    switch (rate) {
    case UWB_PR_16M:
        config.prf = DWT_PRF_16M;
        break;
    case UWB_PR_64M:
        config.prf = DWT_PRF_64M;
        break;
    default:
        return -EINVAL;
    }

    dwt_configure(&config);
    return 0;
}

int uwb_set_preamble(enum uwb_preamble_length length) {
    CHECK_UWB_ACTIVE();

    switch (length) {
    case UWB_PRL_64:
        config.txPreambLength = DWT_PLEN_64;
        break;
    case UWB_PRL_128:
        config.txPreambLength = DWT_PLEN_128;
        break;
    case UWB_PRL_256:
        config.txPreambLength = DWT_PLEN_256;
        break;
    case UWB_PRL_512:
        config.txPreambLength = DWT_PLEN_512;
        break;
    case UWB_PRL_1024:
        config.txPreambLength = DWT_PLEN_1024;
        break;
    case UWB_PRL_1536:
        config.txPreambLength = DWT_PLEN_1536;
        break;
    case UWB_PRL_2048:
        config.txPreambLength = DWT_PLEN_2048;
        break;
    case UWB_PRL_4096:
        config.txPreambLength = DWT_PLEN_4096;
    default:
        return -EINVAL;
    }

    config.sfdTO =
        SFD_TO(get_preamble_length(), get_sfd_length(), get_pac_size());

    dwt_configure(&config);
    return 0;
}

int set_pac_size(enum uwb_pac pac) {
    CHECK_UWB_ACTIVE();

    switch (pac) {
    case UWB_PAC8:
        config.rxPAC = DWT_PAC8;
        break;
    case UWB_PAC16:
        config.rxPAC = DWT_PAC16;
        break;
    case UWB_PAC32:
        config.rxPAC = DWT_PAC32;
        break;
    case UWB_PAC64:
        config.rxPAC = DWT_PAC64;
        break;
    default:
        return -EINVAL;
    }

    config.sfdTO =
        SFD_TO(get_preamble_length(), get_sfd_length(), get_pac_size());

    dwt_configure(&config);
    return 0;
}

int set_sfd_mode(enum uwb_sfd mode) {
    CHECK_UWB_ACTIVE();

    switch (mode) {
    case UWB_STD_SFD:
        config.nsSFD = 0;
        break;
    case UWB_NSTD_SFD:
        config.nsSFD = 1;
        break;
    default:
        return -EINVAL;
    }

    config.sfdTO =
        SFD_TO(get_preamble_length(), get_sfd_length(), get_pac_size());

    dwt_configure(&config);
    return 0;
}

int set_uwb_channel(uint32_t channel) {
    enum pgdelay_ch delay;
    CHECK_UWB_ACTIVE();

    switch (channel) {
    case 1:
        delay = ch1;
        break;
    case 2:
        delay = ch2;
        break;
    case 3:
        delay = ch3;
        break;
    case 4:
        delay = ch4;
        break;
    case 5:
        delay = ch5;
        break;
    case 7:
        delay = ch7;
        break;
    default:
        return -EINVAL;
    }

    config_tx.PGdly = delay;
    config.chan = (uint8_t)channel;
    dwt_configure(&config);
    dwt_configuretxrf(&config_tx);
    return 0;
}

void set_tx_power(uint32_t tx_power) {
    config_tx.power = tx_power;
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
    dwt_setrxtimeout(UWB_INIT_TIMEOUT);
}

/**
 * @brief Reconfig UWB transmitter as a responder
 */
static void resp_reconfig() {
    dwt_setrxaftertxdelay(UWB_RESP_RX_DELAY);
    dwt_setrxtimeout(UWB_RESP_RX_TIMEOUT);
}

static void initiate_ranging(void) {
    static bool drop = false;
    bool search_broken = false;
    static size_t current_neighbor = 0;
    double range;
    uint32_t exchange;

    k_msleep(initiator_freq);

    if (drop) {
        uint16_t delay = get_rand_num_exp_collision(initiator_freq);
        k_msleep(delay);
        drop = false;
    }

    SUSPEND_RESPONDER_TASK();

    dwt_forcetrxoff();
    init_reconfig();

    for (size_t search_count = 0; seen_list[current_neighbor].UUID == 0;
         search_count++) {
        BOUND_INCREMENT(current_neighbor, MAX_ANCHOR_COUNT);

        if (search_count >= (MAX_ANCHOR_COUNT - 1)) {
            search_broken = true;
            break;
        }
    }

    if (!search_broken) {
        int err;

        if (twr_mode) {
            err = ds_init_run(seen_list[current_neighbor].UUID, &range,
                              &exchange);
            LOG_INF("Double sided ranging returned %d", err);
        } else {
            err = ss_init_run(seen_list[current_neighbor].UUID, &range,
                              &exchange);
            LOG_INF("Single sided ranging returned %d", err);
        }

        if (err != 0) {
            drop = true;
        }

        if (!drop && RANGE_CONDITION(range)) {
            seen_list[current_neighbor].update_flag = 1;
            seen_list[current_neighbor].range = (float)range;
            seen_list[current_neighbor].time_stamp = k_uptime_get();
#if defined(CONFIG_UWB_LOGIC_CLK)
            seen_list[current_neighbor].exchange_id = exchange;
#endif // defined(CONFIG_UWB_LOGIC_CLK)

            update_ble_service(seen_list[current_neighbor].UUID, range);
        }

        BOUND_INCREMENT(current_neighbor, MAX_ANCHOR_COUNT);
    }

    resp_reconfig();
    dwt_forcetrxoff();

    RESUME_RESPONDER_TASK();
}

void update_poll_count(void) {
    bool neighbors_polling = false;

    for (size_t x = 0; x < MAX_ANCHOR_COUNT; x++) {
        if (seen_list[x].UUID != 0 && seen_list[x].polling_flag != 0) {
            neighbors_polling = true;
            break;
        }
    }

    if (!neighbors_polling) {
        SUSPEND_RESPONDER_TASK();
        dwt_forcetrxoff();
        resp_reconfig();
        dwt_forcetrxoff();
        RESUME_RESPONDER_TASK();
        k_sem_take(&k_sus_resp, K_NO_WAIT);
    } else {
        SUSPEND_RESPONDER_TASK();
        dwt_forcetrxoff();
        resp_reconfig();
        dwt_forcetrxoff();
        RESUME_RESPONDER_TASK();
        k_sem_give(&k_sus_resp);
    }
}

NO_RETURN void rangingTask(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        LOG_ERR("Unable to spawn ranging watchdog");
        while (1)
            ;
    }
    rangingStarted = true;

    while (true) {
        watchdog_red_rocket(&watchdogAttr);

        if (initiator_freq != 0) {
            initiate_ranging();
        } else {
            k_msleep(1000);
        }

        update_poll_count();
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
                ds_resp_run(NULL, NULL);
            } else {
                ss_resp_run(NULL, NULL);
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
