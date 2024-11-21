/*! ----------------------------------------------------------------------------
 *  @file    init_main.c
 *  @brief   Double-sided and Single-sided two-way ranging (DS/SS TWR) initiator
 * code
 *
 *
 *           Notes at the end of this file, expand on the inline comments.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "init_main.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "random.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(initializer_logger, LOG_LEVEL_INF);

K_SEM_DEFINE(k_sus_init, 0, 1);

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                              'A',  'V',  'E', 0x61, 0,    0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0,    0xCA, 0xDE, 'V', 'E',
                              'W',  'A',  0x50, 0,    0,    0,   0,
                              0,    0,    0,    0,    0,    0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V',
                               'E',  0x69, 0, 0,    0,    0,   0,   0,
                               0,    0,    0, 0,    0,    0,   0,   0};
static uint8 rx_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W',
                                'A',  0xE3, 0, 0,    0,    0,   0,   0};

/* Length of the common part of the message (up to and including the function
 * code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX            2
#define RESP_MSG_POLL_RX_TS_IDX   10
#define RESP_MSG_RESP_TX_TS_IDX   14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define RESP_MSG_TS_LEN           4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to
 * handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be
 * examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor. 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT             299702547

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* Hold copies of computed time of flight and distance here for reference so
 * that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
// #define POLL_TX_TO_RESP_RX_DLY_UUS 100
#define POLL_TX_TO_RESP_RX_DLY_UUS  300
#define RESP_RX_TIMEOUT_UUS         4000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS        4500

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ds_init_run()
 *
 * @brief Initiate UWB double-sided two way ranging
 *
 * @param  node ID
 *
 * @return distance between sending nodes and id node
 */
double ds_init_run(uint8 id) {

    /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = id;
    dwt_write32bitreg(SYS_STATUS_ID,
                      SYS_STATUS_TXFRS); /* Clear Transmit sent frame flag */
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg,
                    0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0,
                     1); /* Zero offset in TX buffer, ranging. */

    /* ----- Send Poll message ----- */
    int check_poll_msg =
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (check_poll_msg == DWT_SUCCESS) {

        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
        };

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    } else {
        LOG_INF("Failed to send poll message");
        return -1;
    }

    /*  Poll for reception of 1. a frame 2. error 3. timeout. See NOTE 4 below.
     */
    while (
        !((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
          (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    };

    /* Check the status register is a frame or not */
    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

        if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is the expected response from the companion
         * frame and extract ID from sender's message */
        int got = rx_buffer[ALL_MSG_SN_IDX];

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if ((got == id) &&
            memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {

            /* Retrieve poll transmission and response reception timestamps. See
             * NOTE 4 below. */
            uint64 poll_tx_ts, resp_rx_ts;
            uint64 ts_replyA_end;

            poll_tx_ts = get_tx_timestamp_u64(); /* Get tx poll message transmit
                                                    timestamp */
            resp_rx_ts =
                get_rx_timestamp_u64(); /* Get rx response message timestamp */

            uint32 resp_tx_time;
            resp_tx_time =
                (resp_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus
             * the antenna delay. */
            ts_replyA_end =
                (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* ------ Send Final (third) message ------ */

            /* Write all timestamps in the final message */
            resp_msg_set_ts(&tx_final_msg[RESP_MSG_POLL_RX_TS_IDX], poll_tx_ts);
            resp_msg_set_ts(&tx_final_msg[RESP_MSG_RESP_TX_TS_IDX], resp_rx_ts);
            resp_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX],
                            ts_replyA_end);

            /* Write and send the response message. */
            tx_final_msg[ALL_MSG_SN_IDX] = id;
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg,
                            0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl(sizeof(tx_final_msg), 0,
                             1); /* Zero offset in TX buffer, ranging. */

            /* Send Final message */
            int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange
             * and proceed to the next one. */
            if (ret == DWT_SUCCESS) {
                /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
                 */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                };
                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            } else {
                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
                return -1;
            }

            /* ------ Receive Report message ------ */

            /* Poll for reception of a frame or error/timeout. See NOTE 5 below.
             */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                     (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO |
                      SYS_STATUS_ALL_RX_ERR))) {
            };

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
                int got = rx_buffer[ALL_MSG_SN_IDX];
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if ((got == id) &&
                    memcmp(rx_buffer, rx_report_msg, ALL_MSG_COMMON_LEN) == 0) {
                    uint32 msg_tof_dtu;
                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX],
                                    &msg_tof_dtu);
                    /* Compute time of flight and distance, using clock offset
                     * ratio to correct for differing local and remote clock
                     * rates */
                    tof = msg_tof_dtu * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;
                    return distance;
                }
            } else {
                /* Clear RX error events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID,
                                  SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                dwt_rxreset();
            }
        }
    } else {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        //        LOG_WRN("Not able to receive response");
        //        if (status_reg & SYS_STATUS_RXPHE) {
        //            LOG_WRN("SYS_STATUS_RXPHE");
        //        }
        //        if (status_reg & SYS_STATUS_RXFCE) {
        //            LOG_WRN("SYS_STATUS_RXFCE");
        //        }
        //        if (status_reg & SYS_STATUS_RXRFSL) {
        //            LOG_WRN("SYS_STATUS_RXRFSL");
        //        }
        //        if (status_reg & SYS_STATUS_RXSFDTO) {
        //            LOG_WRN("SYS_STATUS_RXSFDTO");
        //        }
        //        if (status_reg & SYS_STATUS_AFFREJ) {
        //            LOG_WRN("SYS_STATUS_AFFREJ");
        //        }
        //        if (status_reg & SYS_STATUS_ALL_RX_TO) {
        //            LOG_WRN("SYS_STATUS_LDEERR");
        //        }
        //        if (status_reg & SYS_STATUS_ALL_RX_TO) {
        //            LOG_WRN("SYS_STATUS_ALL_RX_TO (timeout)");
        //        }
    }

    return -1;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn ss_init_run()
 *
 * @brief Initiate UWB single-sided two way ranging
 *
 * @param  node ID
 *
 * @return distance between sending nodes and id node
 */
double ss_init_run(uint8 id) {

    /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = id;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg,
                    0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0,
                     1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that
     * reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (
        !((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
          (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    };

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is the expected response from the companion "SS
         * TWR responder" example. As the sequence number field of the frame is
         * not relevant, it is cleared to simplify the validation of the frame.
         */
        int got = rx_buffer[ALL_MSG_SN_IDX];
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if ((got == id) &&
            memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {

            uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32 rtd_init, rtd_resp;
            float clockOffsetRatio;

            /* Retrieve poll transmission and response reception timestamps. See
             * NOTE 4 below. */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio.
             * See NOTE 6 below. */
            clockOffsetRatio = dwt_readcarrierintegrator() *
                               (FREQ_OFFSET_MULTIPLIER *
                                HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

            /* Get timestamps embedded in response message. */
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* Compute time of flight and distance, using clock offset ratio to
             * correct for differing local and remote clock rates */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) *
                  DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to
            // clear warning
            distance = tof * SPEED_OF_LIGHT;
            return distance;

        } else {
            dwt_rxreset();
        }

    } else {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
    }
    return -1;
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

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The frames used here are Decawave specific ranging frames, complying with
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
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9: function code (specific values to indicate which message it is
 *in the ranging process). The remaining bytes are specific to each message as
 *follows: Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 2. Source and destination addresses are hard coded constants in this example
 *to keep it simple but for a real product every device should have a unique ID.
 *Here, 16-bit addressing is used to keep the messages as short as possible but,
 *in an actual application, this should be done only after an exchange of
 *specific messages used to define those short addresses for each device
 *participating to the ranging exchange.
 * 3. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not work
 *anymore then as we would still have to indicate the full length of the frame
 *to dwt_writetxdata()).
 * 4. The high order byte of each 40-bit time-stamps is discarded here. This is
 *acceptable as, on each device, those time-stamps are not separated by more
 *than 2**32 device time units (which is around 67 ms) which means that the
 *calculation of the round-trip delays can be handled by a 32-bit subtraction.
 * 5. The user is referred to DecaRanging ARM application (distributed with
 *EVK1000 product) for additional practical example of usage, and to the DW1000
 *API Guide for more details on the DW1000 driver functions.
 * 6. The use of the carrier integrator value to correct the TOF calculation,
 *was added Feb 2017 for v1.3 of this example.  This significantly improves the
 *result of the SS-TWR where the remote responder unit's clock is a number of
 *PPM offset from the local inmitiator unit's clock. As stated in NOTE 2 a fixed
 *offset in range will be seen unless the antenna delsy is calibratred and set
 *correctly.
 *
 ****************************************************************************************************************************************************/
