//
// Created by tom on 7/9/24.
//

#include <ble_app.h>
#include <deca_device_api.h>
#include <init_main.h>
#include <random.h>
#include <ranging.h>
#include <thread_priorities.h>
#include <timestamp.h>
#include <utils.h>
#include <watchdog.h>
#include <zephyr/kernel.h>

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100

// TODO: Figure out better place for these
///////////////////////////////////////////////////////////////////////////////
K_SEM_DEFINE(k_sus_resp, 0, 1);
K_SEM_DEFINE(k_sus_init, 0, 1);

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

static enum pgdelay_ch uwb_pgdelay;

/* DW1000 config struct */
static const dwt_config_t config = {
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
static const dwt_txconfig_t config_tx = {TC_PGDELAY_CH5, TX_POWER_MAN_DEFAULT};

///////////////////////////////////////////////////////////////////////////////
// TODO: END

K_MUTEX_DEFINE(twr_mutex);
K_MUTEX_DEFINE(rate_mutex);

static uint32_t initiator_freq = 100;
static bool twr_mode = false;

void set_twr_mode(bool value) {
    k_mutex_lock(&twr_mutex, K_FOREVER);
    twr_mode = value;
    k_mutex_unlock(&twr_mutex);
}

bool get_twr_mode(void) {
    bool retVal;
    k_mutex_lock(&twr_mutex, K_FOREVER);
    retVal = twr_mode;
    k_mutex_unlock(&twr_mutex);
    return retVal;
}

void set_rate(uint32_t rate) {
    k_mutex_lock(&rate_mutex, K_FOREVER);
    initiator_freq = rate;
    k_mutex_unlock(&rate_mutex);
}

uint32_t get_rate(void) {
    uint32_t retVal;
    k_mutex_lock(&rate_mutex, K_FOREVER);
    retVal = initiator_freq;
    k_mutex_unlock(&rate_mutex);
    return retVal;
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

static void poll_nodes(void) {
    static bool drop_flag = false;
    bool break_flag = false;
    int cur_index = 0;

    // Stop the init task based on the input polling frequency
    k_sleep(K_MSEC(get_rate()));

    // If previous polling drop, give a random exponential distribution
    // delay
    if (drop_flag) {
        uint16_t rand_small = get_rand_num_exp_collision(get_rate());
        k_sleep(K_MSEC(rand_small));
        drop_flag = false;
    }

    SUSPEND_RESPONDER_TASK;

    dwt_forcetrxoff();
    init_reconfig();

    /* Do separate ranging (that is, poll only one node in the neighbor
     * list, see developer documentation to see the scheme)*/

    int search_count = 0;
    float range1;

    while (seen_list[cur_index].UUID == 0) {
        cur_index += 1;

        // Back to the head of seen list
        if (cur_index == MAX_ANCHOR_COUNT) {
            cur_index = 0;
        }
        // Finish search for whole list, no found, then break
        if (search_count == MAX_ANCHOR_COUNT - 1) {
            break_flag = true;
            break;
        }
        search_count += 1;
    }

    // Found a node that we want to poll
    if (!break_flag) {

        // Init UWB ranging measurment
        if (get_twr_mode()) {
            range1 = ds_init_run(seen_list[cur_index].UUID);
        } else {
            range1 = ss_init_run(seen_list[cur_index].UUID);
        }

        // Set up drop flag if the ranging fail
        if (range1 == -1)
            drop_flag = true;

        int numThru = 1;
        if (range1 == -1) {
            range1 = 0;
            numThru -= 1;
        }

        float range = (range1) / numThru;

        if ((numThru != 0) && (range >= -5) && (range <= 100)) {
            seen_list[cur_index].update_flag = 1;
            seen_list[cur_index].range = range;
            seen_list[cur_index].time_stamp = get_timestamp();

            // Update BLE transfer value to phone
            // TODO:
            // update_char_value(seen_list[cur_index].UUID,
            //                  seen_list[cur_index].range);
        }

        cur_index += 1;
        // Back to the head of seen list
        if (cur_index == MAX_ANCHOR_COUNT) {
            cur_index = 0;
        }
    }

    resp_reconfig();
    dwt_forcetrxoff();

    RESUME_RESPONDER_TASK;
}

void rangingTask(void) {
    while (true) {
        watchdog_red_rocket();

        // Only the polling node (not passive node) will init a polling message
        if (initiator_freq != 0) {
            poll_nodes();
        } else {
            k_sleep(K_MSEC(1000));
        }

        // Check polling flag of each node (see whether there are active nodes
        // in neighbor list)
        int polling_count = 0;
        for (int x = 0; x < MAX_ANCHOR_COUNT; x++) {
            if (seen_list[x].UUID != 0 && seen_list[x].polling_flag != 0) {
                polling_count += 1;
            }
        }

        // If no polling nodes in the network, suspend UWB response (listening)
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

#if ENABLE_THREADS
K_THREAD_DEFINE(ranging_task_id, STACK_SIZE, rangingTask, NULL, NULL, NULL,
                RANGING_PRIO, 0, 0);
#endif