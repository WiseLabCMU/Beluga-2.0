/**
 * @file ranging.c
 * @brief Ranging Module for UWB-based Distance Measurement System
 *
 * Implements the logic for the ranging, which supports both
 * single-sided and two-way ranging between nodes in a UWB (Ultra-Wideband)
 * network using the DW1000 chip. The module handles configuration,
 * initialization, and communication between nodes. It includes various
 * functions to configure and control the DW1000's parameters, such as data
 * rate, pulse rate, preamble length, and PAC size.
 *
 * Supports both the initiator and responder tasks, where the initiator
 * sends polling messages and the responder listens for them.
 *
 * @author WiSeLab CMU
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 * @date 7/9/24
 */

#include <app_leds.h>
#include <beluga_message.h>
#include <ble_app.h>
#include <deca_device_api.h>
#include <init_resp_common.h>
#include <initiator.h>
#include <port_platform.h>
#include <random.h>
#include <ranging.h>
#include <responder.h>
#include <serial/comms.h>
#include <spi.h>
#include <stdbool.h>
#include <stdio.h>
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
/**
 * The RX timeout when in initiator mode. This timeout is for complete reception
 * of a frame, i.e. timeout duration must take into account the length of the
 * expected frame.
 */
#define UWB_INIT_TIMEOUT 2000
#else
/**
 * The RX timeout when in initiator mode. This timeout is for complete reception
 * of a frame, i.e. timeout duration must take into account the length of the
 * expected frame.
 */
#define UWB_INIT_TIMEOUT CONFIG_UWB_INIT_RX_TIMEOUT
#endif

#if !defined(CONFIG_UWB_RESP_RX_DELAY)
/**
 * The delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature.
 */
#define UWB_RESP_RX_DELAY 0
#else
/**
 * The delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature.
 */
#define UWB_RESP_RX_DELAY CONFIG_UWB_RESP_RX_DELAY
#endif

#if !defined(CONFIG_UWB_RESP_RX_TIMEOUT)
/**
 * The RX timeout when in responder mode.
 */
#define UWB_RESP_RX_TIMEOUT 0
#else
/**
 * The RX timeout when in responder mode.
 */
#define UWB_RESP_RX_TIMEOUT CONFIG_UWB_RESP_RX_TIMEOUT
#endif

#if defined(CONFIG_UWB_FILTER_RANGES)
#include <math.h>

/**
 * The lower bound for filtering out nodes based on range
 */
#define LOWER_RANGE (double)CONFIG_UWB_RANGE_FILTER_LOWER_BOUND

#if CONFIG_UWB_RANGE_FILTER_UPPER_BOUND <= 0
/**
 * The upper bound condition for filtering
 */
#define UPPER_CONDITION(x) true
#else
/**
 * The upper bound for filtering out nodes based on range
 */
#define UPPER_RANGE        (double)CONFIG_UWB_RANGE_FILTER_UPPER_BOUND

/**
 * The upper bound condition for filtering
 */
#define UPPER_CONDITION(x) islessequal((x), UPPER_RANGE)
#endif

/**
 * The lower bound condition for filtering
 */
#define LOWER_CONDITION(x) isgreaterequal((x), LOWER_RANGE)

/**
 * The condition for filtering ranges
 */
#define RANGE_CONDITION(x) (LOWER_CONDITION(x) && UPPER_CONDITION(x))

#else

/**
 * The condition for filtering ranges
 */
#define RANGE_CONDITION(x) (true)
#endif

/**
 * Suspends the responder task when performing ranging to other nodes
 */
#define SUSPEND_RESPONDER_TASK()                                               \
    do {                                                                       \
        k_sem_take(&k_sus_resp, K_NO_WAIT);                                    \
        k_sem_take(&k_sus_init, K_FOREVER);                                    \
        k_sleep(K_MSEC(2));                                                    \
    } while (0)

/**
 * Resumes the responder task
 */
#define RESUME_RESPONDER_TASK()                                                \
    do {                                                                       \
        k_sem_give(&k_sus_init);                                               \
        k_sem_give(&k_sus_resp);                                               \
    } while (0)

/**
 * The number of milliseconds between initiator runs
 */
static int32_t initiator_freq = 100;

/**
 * Flag indicating whether to use two-way ranging or single-sided ranging
 */
static bool twr_mode = true;

/**
 * Calculates the SFD timeout (preamble length + 1 + SFD length - PAC size)
 *
 * @param[in] PREAMBLE The preamble length
 * @param[in] SFD_LENGTH The length of the SFD
 * @param[in] PAC_SIZE The PAC size
 */
#define SFD_TO(PREAMBLE, SFD_LENGTH, PAC_SIZE)                                 \
    ((PREAMBLE) + 1 + (SFD_LENGTH) - (PAC_SIZE))

/**
 * The DW1000 configurations
 */
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

/**
 * The DW1000 TX configurations
 */
static dwt_txconfig_t config_tx = {TC_PGDELAY_CH5, TX_POWER_MAN_DEFAULT};

/**
 * The watchdog timer instance for the ranging.
 */
static struct task_wdt_attr watchdogAttr = {.period =
                                                2 * CONFIG_MAX_POLLING_RATE};

/**
 * @brief Prints the TX power in a non-standard (human readable) format
 * @param[in] tx_power The current TX power
 */
void print_tx_power(const struct comms *comms, uint32_t tx_power) {
    struct beluga_msg msg = {
        .type = START_EVENT,
    };
    char s[32];
    snprintf(s, sizeof(s) - 1, "  TX Power: 0x%08" PRIX32, tx_power);
    msg.payload.node_version = s;
    comms_write_msg(comms, &msg);
}

/**
 * @brief Prints the UWB data rate in a non-standard (human readable) format
 * @param[in] rate The current data rate
 * @return The data rate that was just printed
 */
enum uwb_datarate print_uwb_datarate(const struct comms *comms,
                                     enum uwb_datarate rate) {
    struct beluga_msg msg = {
        .type = START_EVENT,
    };
    switch (rate) {
    case UWB_DR_850K:
        msg.payload.node_version = "  Data Rate: 850 kHz";
        break;
    case UWB_DR_110K:
        msg.payload.node_version = "  Data Rate: 110 kHz";
        break;
    case UWB_DR_6M8:
    default:
        msg.payload.node_version = "  Data Rate: 6.8MHz";
        rate = UWB_DR_6M8;
        break;
    }

    comms_write_msg(comms, &msg);

    return rate;
}

/**
 * @brief Prints the pulse rate in a non-standard (human readable) format
 * @param[in] rate The current pulse rate
 * @return The pulse rate
 */
enum uwb_pulse_rate print_pulse_rate(const struct comms *comms,
                                     enum uwb_pulse_rate rate) {
    struct beluga_msg msg = {
        .type = START_EVENT,
    };
    switch (rate) {
    case UWB_PR_16M:
        msg.payload.node_version = "  Pulse Rate: 16MHz";
        break;
    case UWB_PR_64M:
    default:
        msg.payload.node_version = "  Pulse Rate: 64MHz";
        rate = UWB_PR_64M;
        break;
    }

    comms_write_msg(comms, &msg);

    return rate;
}

/**
 * @brief Prints the PAC size in a non-standard (human readable) format
 * @param[in] pac The PAC size
 * @return The PAC size
 */
int32_t print_pac_size(const struct comms *comms, int32_t pac) {
    struct beluga_msg msg = {
        .type = START_EVENT,
    };
    switch ((enum uwb_pac)pac) {
    case UWB_PAC16:
        msg.payload.node_version = "  PAC Size: 16";
        break;
    case UWB_PAC32:
        msg.payload.node_version = "  PAC Size: 32";
        break;
    case UWB_PAC64:
        msg.payload.node_version = "  PAC Size: 16";
        break;
    case UWB_PAC8:
    default:
        msg.payload.node_version = "  PAC Size: 8";
        pac = (int32_t)UWB_PAC8;
        break;
    }

    comms_write_msg(comms, &msg);

    return pac;
}

/**
 * @brief Prints the current PAN ID in a non-standard (human readable) format
 * @param[in] pan_id The PAN ID to print
 */
void print_pan_id(const struct comms *comms, uint32_t pan_id) {
    struct beluga_msg msg = {
        .type = START_EVENT,
    };
    char s[64];
    snprintf(s, sizeof(s) - 1, "  UWB PAN ID: 0x%04" PRIX16 " ",
             (uint16_t)pan_id);
    msg.payload.node_version = s;
    comms_write_msg(comms, &msg);
}

/**
 * @brief Sets the PHR mode for the DW1000
 * @param[in] mode The PHR mode to update
 * @return 0 upon success
 * @return -EINVAL if PHR mode is not valid
 */
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

/**
 * @brief Retrieves the current preamble length being used by the DW1000
 * @return an integer representation of the preamble length
 */
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

/**
 * @brief Retrieves the current PAC size being used by the DW1000
 * @return an integer representing the PAC size
 */
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

/**
 * @brief Retrieves the SFD length based on the DW1000 data rate
 * @return The SFD length
 */
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

/**
 * @brief Sets the data rate of the DW1000
 * @param[in] rate The new data rate of the DW1000
 * @return 0 upon success
 * @return -EINVAL if data rate is an invalid value
 * @return -EBUSY if UWB is active
 */
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
    set_freq_offset_multiplier(rate == UWB_DR_110K);
    return 0;
}

/**
 * @brief Sets the DW1000 pulse rate
 * @param[in] rate The pulse rate to of the DW1000
 * @return 0 upon success
 * @return -EINVAL if rate is invalid
 * @return -EBUSY if UWB is active
 */
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

/**
 * @brief Sets the preamble length of the DW1000
 * @param[in] length The new preamble length of the DW1000
 * @return 0 upon success
 * @return -EINVAL if length is invalid
 * @return -EBUSY if UWB is active
 */
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

/**
 * @brief Sets the PAC size of the DW1000
 * @param[in] pac The new PAC size of the DW1000
 * @return 0 upon success
 * @return -EINVAL if PAC size is invalid
 * @return -EBUSY if UWB is active
 */
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

/**
 * @brief Sets the SFD mode of the DW1000
 * @param[in] mode The new SFD mode
 * @return 0 upon success
 * @return -EINVAL upon failure
 * @return -EBUSY if UWB is active
 */
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

/**
 * @brief Sets the UWB channel to use for the DW1000
 * @param[in] channel The new UWB channel
 * @return 0 upon success
 * @return -EINVAL if invalid channel'
 * @return -EBUSY if UWB is active
 */
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
    set_hertz_to_ppm_multiplier((uint8_t)channel);
    return 0;
}

/**
 * @brief Sets the transmit power of the DW1000
 * @param[in] tx_power The new transmit power of the DW1000
 */
void set_tx_power(uint32_t tx_power) {
    config_tx.power = tx_power;
    dwt_configuretxrf(&config_tx);
}

/**
 * Sets the ranging mode used by the initiator and responder
 * @param[in] value The ranging mode. If `true`, use two-way ranging, if
 * `false`, use single-sided ranging
 */
void set_twr_mode(bool value) { twr_mode = value; }

/**
 * @brief Set the rate the initiator sends polling messages
 * @param[in] rate The new rate the initiator sends polling messages. If 0, then
 * do not send polling messages.
 * @return 0 upon success
 * @return -EINVAL if rate is an invalid value
 */
int set_rate(uint32_t rate) {
    if (!IN_RANGE(rate, INT32_C(0), (int32_t)INT32_MAX)) {
        return -EINVAL;
    }
    initiator_freq = (int32_t)rate;
    return 0;
}

/**
 * @brief Initialize the DW1000 for ranging.
 *
 * Wakes the DW1000 (if coming out of deep sleep), resets the DW1000, and
 * initializes the DW1000 with default values
 */
void init_uwb(void) {
    if (!IS_ENABLED(CONFIG_ENABLE_BELUGA_UWB)) {
        LOG_INF("UWB disabled");
        return;
    }
    setup_DW1000RSTnIRQ(0);
    toggle_cs_line(400);

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

/**
 * @brief Find a node in the neighbors list and range to that node
 *
 * @param[in] comms Pointer to the comms object for drop reporting
 */
static void initiate_ranging(const struct comms *comms) {
    // Time left to sleep in ms
    static int32_t time_left = CONFIG_POLLING_REFRESH_PERIOD;
    // Flag to see if last ranging measurement was dropped
    static bool drop = false;
    // The neighbor currently being ranged to
    static size_t current_neighbor = 0;

    bool search_broken = false;
    double range;
    uint32_t exchange;
    int32_t sleep_for = (time_left < CONFIG_POLLING_REFRESH_PERIOD)
                            ? time_left
                            : CONFIG_POLLING_REFRESH_PERIOD;
    ARG_UNUSED(comms);

    k_sleep(K_MSEC(sleep_for));
    time_left -= sleep_for;

    if (time_left > 0) {
        return;
    }
    time_left = initiator_freq;

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
#if defined(CONFIG_REPORT_UWB_DROPS)
            struct dropped_packet_event event = {
                .id = seen_list[current_neighbor].UUID,
                .sequence = dropped_stage(),
            };
            struct beluga_msg msg = {
                .type = UWB_RANGING_DROP,
                .payload.drop_event = &event,
            };
            comms_write_msg(comms, &msg);
#endif // defined(CONFIG_REPORT_UWB_DROPS)
            drop = true;
        }

        if (!drop && RANGE_CONDITION(range)) {
            seen_list[current_neighbor].update_flag = true;
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

/**
 * @brief Checks how many neighboring nodes are polling. If none of them are
 * polling, then this will suspend responder.
 */
void update_poll_count(void) {
    bool neighbors_polling = false;

    for (size_t x = 0; x < MAX_ANCHOR_COUNT; x++) {
        if (seen_list[x].UUID != 0 && seen_list[x].polling_flag) {
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

/**
 * @brief Task that initiates ranging
 *
 * This task initiates ranging between nodes and updates the neighbor list with
 * new ranges
 *
 * @param p1 Additional context (unused)
 * @param p2 Additional context (unused)
 * @param p3 Additional context (unused)
 */
NO_RETURN void rangingTask(void *p1, void *p2, void *p3) {
    const struct comms *comms = comms_backend_uart_get_ptr();
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (spawn_task_watchdog(&watchdogAttr) < 0) {
        LOG_ERR("Unable to spawn ranging watchdog");
        while (1)
            ;
    }

    while (true) {
        watchdog_red_rocket(&watchdogAttr);

        if (initiator_freq > 0) {
            initiate_ranging(comms);
        } else {
            k_sleep(K_SECONDS(1));
        }

        update_poll_count();
    }
}

/**
 * @brief Responds to UWB polling requests
 *
 * This task will check for polling messages and respond to any ranging requests
 *
 * @param p1 Additional context (unused)
 * @param p2 Additional context (unused)
 * @param p3 Additional context (unused)
 */
NO_RETURN static void responder_task_function(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    uint16_t id;
    uint32_t exchange;
    int ret;
    const struct comms *comms = comms_backend_uart_get_ptr();
    struct beluga_msg msg = {.type = RANGING_EVENT};

    if (are_leds_on()) {
        dwt_setleds(DWT_LEDS_ENABLE);
    } else {
        dwt_setleds(DWT_LEDS_DISABLE);
    }

    while (true) {
        watchdog_red_rocket(&watchdogAttr);

        // Check if responding is suspended, return 0 means suspended
        unsigned int suspend_start = k_sem_count_get(&k_sus_resp);

        if (suspend_start != 0) {
            if (twr_mode) {
                ret = ds_resp_run(&id, &exchange);
            } else {
                ret = ss_resp_run(&id, &exchange);
            }
        } else {
            ret = -EBUSY;
        }

        if (ret == 0) {
            struct ranging_event event = {
                .exchange_id = exchange, .id = id, .timestamp = k_uptime_get()};
            msg.payload.event = &event;
            comms_write_msg(comms, &msg);
        }
    }
}

#if defined(CONFIG_ENABLE_BELUGA_THREADS) && defined(CONFIG_ENABLE_RANGING)
/**
 * Ranging task's stack allocation
 */
K_THREAD_STACK_DEFINE(ranging_stack, CONFIG_RANGING_STACK_SIZE);

/**
 * Ranging task's thread data
 */
static struct k_thread ranging_task_data;

/**
 * Thread ID of the ranging task
 */
static k_tid_t ranging_task_id;

/**
 * @brief Creates the ranging thread and initiates its data
 */
void init_ranging_thread(void) {
    ranging_task_id =
        k_thread_create(&ranging_task_data, ranging_stack,
                        K_THREAD_STACK_SIZEOF(ranging_stack), rangingTask, NULL,
                        NULL, NULL, CONFIG_BELUGA_RANGING_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(ranging_task_id, "Ranging task");
    LOG_INF("Started ranging");
}
#else
/**
 * @brief Creates the ranging thread and initiates its data
 */
void init_ranging_thread(void) { LOG_INF("Ranging disabled"); }
#endif // defined(CONFIG_ENABLE_BELUGA_THREADS) &&
       // defined(CONFIG_ENABLE_RANGING)

#if defined(CONFIG_ENABLE_BELUGA_THREADS) && defined(CONFIG_ENABLE_RESPONDER)
/**
 * Responder task's stack allocation
 */
K_THREAD_STACK_DEFINE(responder_stack, CONFIG_RESPONDER_STACK_SIZE);

/**
 * Responder task's thread data
 */
static struct k_thread responder_data;

/**
 * Thread ID of the responder task
 */
static k_tid_t responder_task_id;

/**
 * @brief Creates the responder thread and initiates its data
 */
void init_responder_thread(void) {
    responder_task_id = k_thread_create(
        &responder_data, responder_stack,
        K_THREAD_STACK_SIZEOF(responder_stack), responder_task_function, NULL,
        NULL, NULL, CONFIG_BELUGA_RESPONDER_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(responder_task_id, "Responder task");
    LOG_INF("Started responder");
}
#else
/**
 * @brief Creates the responder thread and initiates its data
 */
void init_responder_thread(void) { LOG_INF("Responder disabled"); }
#endif // defined(CONFIG_ENABLE_BELUGA_THREADS) &&
       // defined(CONFIG_ENABLE_RESPONDER)
