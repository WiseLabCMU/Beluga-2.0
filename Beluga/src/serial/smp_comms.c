/**
 * @file smp_comms.c
 *
 * @brief
 *
 * @date 2/20/25
 *
 * @author tom
 */

/** @file
 * @brief Shell transport for the mcumgr SMP protocol.
 */

#include <serial/comms.h>
#include <serial/comms_uart.h>
#include <serial/smp_comms.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/serial.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/net_buf.h>
#include <zephyr/syscalls/uart.h>

#include <mgmt/mcumgr/transport/smp_internal.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(smp_shell);

BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_COMMS_MTU != 0,
             "CONFIG_MCUMGR_TRANSPORT_COMMSL_MTU must be > 0");

#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT
BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT_TIME != 0,
             "CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT_TIME must be > 0");
#endif

static struct smp_transport smp_comms_transport;

static struct mcumgr_serial_rx_ctxt smp_comms_rx_ctxt;

#ifdef CONFIG_SMP_CLIENT
static struct smp_client_transport_entry smp_client_transport;
#endif

/** SMP mcumgr frame fragments. */
enum smp_comms_esc_mcumgr {
    ESC_MCUMGR_PKT_1,
    ESC_MCUMGR_PKT_2,
    ESC_MCUMGR_FRAG_1,
    ESC_MCUMGR_FRAG_2,
};

/** These states indicate whether an mcumgr frame is being received. */
enum smp_comms_mcumgr_state {
    SMP_COMMS_MCUMGR_STATE_NONE,
    SMP_COMMS_MCUMGR_STATE_HEADER,
    SMP_COMMS_MCUMGR_STATE_PAYLOAD
};

#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT
static void smp_comms_input_timeout_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);
    struct smp_comms_data *const data = comms_uart_smp_comms_data_get_ptr();

    atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
    atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
    atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
    atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);

    if (data->buf) {
        net_buf_reset(data->buf);
        net_buf_unref(data->buf);
        data->buf = NULL;
    }
}

K_TIMER_DEFINE(smp_comms_input_timer, smp_comms_input_timeout_handler, NULL);
#endif

static int read_mcumgr_byte(struct smp_comms_data *data, uint8_t byte) {
    bool frag_1;
    bool frag_2;
    bool pkt_1;
    bool pkt_2;

    pkt_1 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
    pkt_2 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
    frag_1 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
    frag_2 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);

    if (pkt_2 || frag_2) {
        /* Already fully framed. */
        return SMP_COMMS_MCUMGR_STATE_PAYLOAD;
    }

    if (pkt_1) {
        if (byte == MCUMGR_SERIAL_HDR_PKT_2) {
            /* Final framing byte received. */
            atomic_set_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT
            k_timer_start(
                &smp_comms_input_timer,
                K_MSEC(CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT_TIME),
                K_NO_WAIT);
#endif
            return SMP_COMMS_MCUMGR_STATE_PAYLOAD;
        }
    } else if (frag_1) {
        if (byte == MCUMGR_SERIAL_HDR_FRAG_2) {
            /* Final framing byte received. */
            atomic_set_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);
#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT
            k_timer_start(
                &smp_comms_input_timer,
                K_MSEC(CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT_TIME),
                K_NO_WAIT);
#endif
            return SMP_COMMS_MCUMGR_STATE_PAYLOAD;
        }
    } else {
        if (byte == MCUMGR_SERIAL_HDR_PKT_1) {
            /* First framing byte received. */
            atomic_set_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
            return SMP_COMMS_MCUMGR_STATE_HEADER;
        } else if (byte == MCUMGR_SERIAL_HDR_FRAG_1) {
            /* First framing byte received. */
            atomic_set_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
            return SMP_COMMS_MCUMGR_STATE_HEADER;
        }
    }

    /* Non-mcumgr byte received. */
    return SMP_COMMS_MCUMGR_STATE_NONE;
}

size_t smp_comms_rx_bytes(struct smp_comms_data *data, const uint8_t *bytes,
                          size_t size) {
    size_t consumed = 0; /* Number of bytes consumed by SMP */

    /* Process all bytes that are accepted as SMP commands. */
    while (size != consumed) {
        uint8_t byte = bytes[consumed];
        int mcumgr_state = read_mcumgr_byte(data, byte);

        if (mcumgr_state == SMP_COMMS_MCUMGR_STATE_NONE) {
            break;
        } else if (mcumgr_state == SMP_COMMS_MCUMGR_STATE_HEADER &&
                   !data->buf) {
            data->buf = net_buf_alloc(data->buf_pool, K_NO_WAIT);
            if (!data->buf) {
                LOG_WRN("Failed to alloc SMP buf");
            }
        }

        if (data->buf && net_buf_tailroom(data->buf) > 0) {
            net_buf_add_u8(data->buf, byte);
        }

        /* Newline in payload means complete frame */
        if (mcumgr_state == SMP_COMMS_MCUMGR_STATE_PAYLOAD && byte == '\n') {
            if (data->buf) {
                k_fifo_put(&data->buf_ready, data->buf);
                data->buf = NULL;
            }
            atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
            atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
            atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
            atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);

#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS_INPUT_TIMEOUT
            k_timer_stop(&smp_comms_input_timer);
#endif
        }

        ++consumed;
    }

    return consumed;
}

void smp_comms_process(struct smp_comms_data *data) {
    struct net_buf *buf;
    struct net_buf *nb;

    while (true) {
        buf = k_fifo_get(&data->buf_ready, K_NO_WAIT);
        if (!buf) {
            break;
        }

        nb =
            mcumgr_serial_process_frag(&smp_comms_rx_ctxt, buf->data, buf->len);
        if (nb != NULL) {
            smp_rx_req(&smp_comms_transport, nb);
        }

        net_buf_unref(buf);
    }
}

static uint16_t smp_comms_get_mtu(const struct net_buf *nb) {
    return CONFIG_MCUMGR_TRANSPORT_COMMS_MTU;
}

static int smp_comms_tx_raw(const void *data, int len) {
    static const struct device *const sh_dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    const uint8_t *out = data;

    while ((out != NULL) && (len != 0)) {
        uart_poll_out(sh_dev, *out);
        ++out;
        --len;
    }

    return 0;
}

static int smp_comms_tx_pkt(struct net_buf *nb) {
    int rc;

    rc = mcumgr_serial_tx_pkt(nb->data, nb->len, smp_comms_tx_raw);
    smp_packet_free(nb);

    return rc;
}

int smp_comms_init(void) {
    int rc;

    smp_comms_transport.functions.output = smp_comms_tx_pkt;
    smp_comms_transport.functions.get_mtu = smp_comms_get_mtu;

    rc = smp_transport_init(&smp_comms_transport);
#ifdef CONFIG_SMP_CLIENT
    if (rc == 0) {
        smp_client_transport.smpt = &smp_comms_transport;
        smp_client_transport.smpt_type = SMP_USER_DEFINED_TRANSPORT;
        smp_client_transport_register(&smp_client_transport);
    }
#endif

    return rc;
}