/*! ----------------------------------------------------------------------------
 *  @file    resp_main.c
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) responder
 * code
 *
 *
 *           Notes at the end of this file, to expand on the inline comments.
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_device_api.h"
#include "deca_regs.h"
#include "init_main.h"
#include "port_platform.h"
#include "random.h"
#include <ble_app.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

K_SEM_DEFINE(k_sus_resp, 0, 1);

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 250

/* Frames used in the ranging process. See NOTE 1,2 below. */

/* Frames used in the ranging process. See NOTE 2,3 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                              'A',  'V',  'E', 0x61, 0,    0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0,    0xCA, 0xDE, 'V', 'E',
                              'W',  'A',  0x50, 0,    0,    0,   0,
                              0,    0,    0,    0,    0,    0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V',
                               'E',  0x69, 0, 0,    0,    0,   0,   0,
                               0,    0,    0, 0,    0,    0,   0,   0};
static uint8 tx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W',
                                'A',  0xE3, 0, 0,    0,    0,   0,   0};

/* Length of the common part of the message (up to and including the function
 * code, see NOTE 1 below). */
/* Length of the common part of the message (up to and including the function
 * code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define RESP_MSG_POLL_RX_TS_IDX   10
#define RESP_MSG_RESP_TX_TS_IDX   14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define RESP_MSG_TS_LEN           4

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to
 * handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be
 * examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor. 1 uus = 512 / 499.2 ?s and 1 ?s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS 1500

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them.
 */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;

/* Declaration of static functions. */
// static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
// static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them.
 */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/*Transactions Counters */
static volatile int tx_count = 0; // Successful transmit counter
static volatile int rx_count = 0; // Successful receive counter

/* Hold copies of computed time of flight and distance here for reference so
 * that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* This is the delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 4500

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_resp_run()
 *
 * @brief perform double-sided responses to signal initiator
 *
 * @param  none
 *
 * @return int represent task complete or abort
 */
int ds_resp_run(void) {
    uint16_t NODE_UUID = get_NODE_UUID();
    unsigned int suspend_start =
        k_sem_count_get(&k_sus_resp); // Check if responding is suspended
    if (suspend_start == 1)
        return 1;

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
    while (
        !((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
          (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        unsigned int suspend = k_sem_count_get(&k_sus_resp);

        if (suspend == 1) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
            return 1;
        }
    }

#if 0 // Include to determine the type of timeout if required.
    int temp = 0;
    // (frame wait timeout and preamble detect timeout)
    if(status_reg & SYS_STATUS_RXRFTO )
    temp =1;
    else if(status_reg & SYS_STATUS_RXPTO )
    temp =2;
#endif

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /*  Check that the frame is the expected response from the companion
         * frame and extract ID from sender' message */
        int id = rx_buffer[ALL_MSG_SN_IDX];
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if ((memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) &&
            (id == NODE_UUID)) {

            uint32 resp_tx_time;
            int ret;

            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. See NOTE 7 below. */
            resp_tx_time =
                (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Write and send the response message. See NOTE 9 below. */
            tx_resp_msg[ALL_MSG_SN_IDX] = NODE_UUID;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg,
                            0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0,
                             1); /* Zero offset in TX buffer, ranging. */

            /* Send Response message */
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange
             * and proceed to the next one. */
            if (ret == DWT_SUCCESS) {

                /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
                 */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                }
                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            } else {
                /* If we end up in here then we have not succeded in
                transmitting the packet we sent up. POLL_RX_TO_RESP_TX_DLY_UUS
                is a critical value for porting to different processors. For
                slower platforms where the SPI is at a slower speed or the
                processor is operating at a lower frequency (Comparing to
                STM32F, SPI of 18MHz and Processor internal 72MHz)this value
                needs to be increased. Knowing the exact time when the responder
                is going to send its response is vital for time of flight
                calculation. The specification of the time of respnse must allow
                the processor enough time to do its calculations and put the
                packet in the Tx buffer. So more time is required for a slower
                system(processor).
                */

                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
                return 1;
            }

            /* Poll for reception of a frame or error/timeout. See NOTE 5 below.
             */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                     (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO |
                      SYS_STATUS_ALL_RX_ERR))) {
                unsigned int suspend = k_sem_count_get(&k_sus_resp);
                if (suspend == 1) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                    return 1;
                }
            };

#if 0 // Include to determine the type of timeout if required.
            int temp = 0;
        // (frame wait timeout and preamble detect timeout)
        if(status_reg & SYS_STATUS_RXRFTO )
        temp =1;
        else if(status_reg & SYS_STATUS_RXPTO )
        temp =2;
#endif

            if (status_reg & SYS_STATUS_RXFCG) {
                uint32 frame_len;

                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                /* A frame has been received, read it into the local buffer. */
                frame_len =
                    dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= RX_BUFFER_LEN) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                /* Check that the frame is a poll sent by "SS TWR initiator"
                 * example. As the sequence number field of the frame is not
                 * relevant, it is cleared to simplify the validation of the
                 * frame. */
                int id = rx_buffer[ALL_MSG_SN_IDX];
                rx_buffer[ALL_MSG_SN_IDX] = 0;

                //----- Receive third(final) message -----

                if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0 &&
                    (id == NODE_UUID)) {
                    // int ret;
                    uint32 resp_rx_ts, poll_tx_ts, final_tx_ts;
                    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    double roundA, replyA, roundB, replyB;
                    int64 tof_dtu;

                    /* Retrieve final reception timestamp. */
                    final_rx_ts = get_rx_timestamp_u64();
                    resp_tx_ts = get_tx_timestamp_u64();

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX],
                                    &poll_tx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX],
                                    &resp_rx_ts);
                    resp_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],
                                    &final_tx_ts);

                    // TOF parameters
                    poll_rx_ts_32 = (uint32)poll_rx_ts;
                    resp_tx_ts_32 = (uint32)resp_tx_ts;
                    final_rx_ts_32 = (uint32)final_rx_ts;
                    roundB = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    replyB = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    roundA = (double)(resp_rx_ts - poll_tx_ts);
                    replyA = (double)(final_tx_ts - resp_rx_ts);

                    if ((roundA * roundB - replyA * replyB) <= 0)
                        return 1;

                    /* Compute time of flight and distance, using clock offset
                     * ratio to correct for differing local and remote clock
                     * rates */
                    tof_dtu = (uint64)((roundA * roundB - replyA * replyB) /
                                       (roundA + roundB + replyA + replyB));
                    tof = tof_dtu * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    //----- Send report message

                    /* Write all timestamps in the report message. */
                    resp_msg_set_ts(&tx_report_msg[RESP_MSG_POLL_RX_TS_IDX],
                                    tof_dtu);

                    /* Write and send the report message. */
                    tx_report_msg[ALL_MSG_SN_IDX] = NODE_UUID;
                    dwt_writetxdata(
                        sizeof(tx_report_msg), tx_report_msg,
                        0); /* Zero offset in TX buffer. See Note 5 below.*/
                    dwt_writetxfctrl(
                        sizeof(tx_report_msg), 0,
                        1); /* Zero offset in TX buffer, ranging. */
                    int ret_report = dwt_starttx(DWT_START_TX_IMMEDIATE);

                    if (ret_report == DWT_SUCCESS) {
                        /* Poll DW1000 until TX frame sent event set. See NOTE 5
                         * below. */
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                                 SYS_STATUS_TXFRS)) {
                        };
                        /* Clear TXFRS event. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                    }

                    else {
                        /* If we end up in here then we have not succeded in
                        transmitting the packet we sent up.
                        POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for
                        porting to different processors. For slower platforms
                        where the SPI is at a slower speed or the processor is
                        operating at a lower frequency (Comparing to STM32F, SPI
                        of 18MHz and Processor internal 72MHz)this value needs
                        to be increased. Knowing the exact time when the
                        responder is going to send its response is vital for
                        time of flight calculation. The specification of the
                        time of respnse must allow the processor enough time to
                        do its calculations and put the packet in the Tx buffer.
                        So more time is required for a slower system(processor).
                        */
                        /* Reset RX to properly reinitialise LDE operation. */
                        dwt_rxreset();
                        return 1;
                    }
                }
            } else {
                /* Clear RX error/timeout events in the DW1000 status register.
                 */
                dwt_write32bitreg(SYS_STATUS_ID,
                                  SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
            }
        }
    } else {
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }

    return (1);
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_resp_run()
 *
 * @brief perform single-sided responses to signal initiator
 *
 * @param  none
 *
 * @return int represent task complete or abort
 */
int ss_resp_run(void) {
    uint16_t NODE_UUID = get_NODE_UUID();
    unsigned int suspend_start =
        k_sem_count_get(&k_sus_resp); // Check if responding is suspended
    if (suspend_start == 0)
        return 1;

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (
        !((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
          (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        unsigned int suspend = k_sem_count_get(&k_sus_resp);
        if (suspend == 1) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
            return 1;
        }
    }

#if 0 // Include to determine the type of timeout if required.
    int temp = 0;
    // (frame wait timeout and preamble detect timeout)
    if(status_reg & SYS_STATUS_RXRFTO )
    temp =1;
    else if(status_reg & SYS_STATUS_RXPTO )
    temp =2;
#endif

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;
        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is a poll sent by "SS TWR initiator" example.
         * As the sequence number field of the frame is not relevant, it is
         * cleared to simplify the validation of the frame. */
        int id = rx_buffer[ALL_MSG_SN_IDX];
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if ((memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) &&
            (id == NODE_UUID)) {
            uint32 resp_tx_time;
            int ret;

            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. See NOTE 7 below. */
            resp_tx_time =
                (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus
             * the antenna delay. */
            resp_tx_ts =
                (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message. See NOTE 8 below. */
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message. See NOTE 9 below. */
            tx_resp_msg[ALL_MSG_SN_IDX] = NODE_UUID;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg,
                            0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0,
                             1); /* Zero offset in TX buffer, ranging. */

            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange
             * and proceed to the next one. */
            if (ret == DWT_SUCCESS) {

                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                    unsigned int suspend = k_sem_count_get(&k_sus_resp);
                    if (suspend == 1) {
                        dwt_forcetrxoff();
                        return 1;
                    }
                }

                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* Increment frame sequence number after transmission of the
                 * poll message (modulo 256). */
                frame_seq_nb++;
            } else {
                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
            }
        } else {
            dwt_rxreset();
        }

    } else {
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }
    return (1);
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given
 * value. In the timestamp fields of the response message, the least significant
 * byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts) {
    int i;
    for (i = 0; i < RESP_MSG_TS_LEN; i++) {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the
 * timestamp fields of the response message, the least significant byte is at
 * the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. This is the task delay when using FreeRTOS. Task is delayed a given number
 *of ticks. Useful to be able to define this out to see the effect of the RTOS
 *    on timing.
 * 2. The frames used here are Decawave specific ranging frames, complying with
 *the IEEE 802.15.4 standard data frame encoding. The frames are the following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and
 *provide all information needed by the initiator to compute the time-of-flight
 *(distance) estimate. The first 10 bytes of those frame are common and are
 *composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit
 *addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is
 *in the ranging process). The remaining bytes are specific to each message as
 *follows: Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example
 *to keep it simple but for a real product every device should have a unique ID.
 *Here, 16-bit addressing is used to keep the messages as short as possible but,
 *in an actual application, this should be done only after an exchange of
 *specific messages used to define those short addresses for each device
 *participating to the ranging exchange.
 * 4. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 * 5. We use polled mode of operation here to keep the example as simple as
 *possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also
 *to be noted that STATUS register is 5 bytes long but, as the event we use are
 *all in the first bytes of the register, we can use the simple
 *dwt_read32bitreg() API call to access it instead of reading the whole 5 bytes.
 * 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different
 *processors. For slower platforms where the SPI is at a slower speed or the
 *processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz
 *and Processor internal 72MHz)this value needs to be increased. Knowing the
 *exact time when the responder is going to send its response is vital for time
 *of flight calculation. The specification of the time of respnse must allow the
 *processor enough time to do its calculations and put the packet in the Tx
 *buffer. So more time required for a slower system(processor).
 * 7. As we want to send final TX timestamp in the final message, we have to
 *compute it in advance instead of relying on the reading of DW1000 register.
 *Timestamps and delayed transmission time are both expressed in device time
 *units so we just have to add the desired response delay to response RX
 *timestamp to get final transmission time. The delayed transmission time
 *resolution is 512 device time units which means that the lower 9 bits of the
 *obtained value must be zeroed. This also allows to encode the 40-bit value in
 *a 32-bit words by shifting the all-zero lower 8 bits.
 * 8. In this operation, the high order byte of each 40-bit timestamps is
 *discarded. This is acceptable as those time-stamps are not separated by more
 *than 2**32 device time units (which is around 67 ms) which means that the
 *calculation of the round-trip delays (needed in the time-of-flight
 *computation) can be handled by a 32-bit subtraction.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 *10. The user is referred to DecaRanging ARM application (distributed with
 *EVK1000 product) for additional practical example of usage, and to the DW1000
 *API Guide for more details on the DW1000 driver functions.
 *
 ****************************************************************************************************************************************************/
