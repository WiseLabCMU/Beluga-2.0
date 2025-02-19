/**
 * @file comms_uart.c
 *
 * @brief
 *
 * @date 2/17/25
 *
 * @author tom
 */

#include <serial/comms_uart.h>
#include <serial_led.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net_buf.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(comms_uart);

NET_BUF_POOL_DEFINE(smp_comms_rx_pool,
                    CONFIG_MCUMGR_TRANSPORT_COMMS_RX_BUF_COUNT,
                    SMP_COMMS_RX_BUF_SIZE, 0, NULL);

static void uart_rx_handle(const struct device *dev,
                           struct comms_uart_int_driven *uart) {
    uint8_t *data;
    uint32_t len;
    uint32_t rd_len;
    bool new_data = false;
    struct smp_comms_data *const smp = &uart->common.smp;

    do {
        len =
            ring_buf_put_claim(&uart->rx_ringbuf, &data, uart->rx_ringbuf.size);

        if (len > 0) {
            rd_len = uart_fifo_read(dev, data, len);

            if (rd_len > 0) {
                new_data = true;
            }

            size_t i = smp_comms_rx_bytes(smp, data, rd_len);

            rd_len -= i;

            if (rd_len) {
                for (uint32_t j = 0; j < rd_len; j++) {
                    data[j] = data[i + j];
                }
            }

            int err = ring_buf_put_finish(&uart->rx_ringbuf, rd_len);
            (void)err;
        } else {
            uint8_t dummy;

            LOG_WRN("RX Ring buffer is full.");

            rd_len = uart_fifo_read(dev, &dummy, 1);

            /* If successful in getting byte from the fifo, try
             * feeding it to SMP as a part of mcumgr frame.
             */
            if ((rd_len != 0) && (smp_comms_rx_bytes(smp, &dummy, 1) == 1)) {
                new_data = true;
            }
        }
    } while (rd_len && (rd_len == len));

    if (new_data) {
        uart->common.handler(COMMS_TRANSPORT_EVT_RX_RDY, uart->common.context);
    }
}

static bool uart_dtr_check(const struct device *dev) {
    BUILD_ASSERT(!IS_ENABLED(CONFIG_COMMS_BACKEND_SERIAL_CHECK_DTR) ||
                     IS_ENABLED(CONFIG_UART_LINE_CTRL),
                 "DTR check requires CONFIG_UART_LINE_CTRL");

#if IS_ENABLED(CONFIG_COMMS_BACKEND_SERIAL_CHECK_DTR)
    int dtr, err;

    err = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    if (err == -ENOSYS || err == -ENOTSUP) {
        return true;
    }

    return (bool)dtr;
#else
    return true;
#endif
}

static void dtr_timer_handler(struct k_timer *timer) {
    struct comms_uart_int_driven *uart = k_timer_user_data_get(timer);

    if (!uart_dtr_check(uart->common.dev)) {
        return;
    }

    k_timer_stop(timer);
    uart_irq_tx_enable(uart->common.dev);
}

static void uart_tx_handle(const struct device *dev,
                           struct comms_uart_int_driven *uart) {
    uint32_t len;
    const uint8_t *data;

    if (!uart_dtr_check(dev)) {
        uart_irq_tx_disable(dev);
        k_timer_start(&uart->dtr_timer, K_MSEC(100), K_MSEC(100));
        return;
    }

    len = ring_buf_get_claim(&uart->tx_ringbuf, (uint8_t **)&data,
                             uart->tx_ringbuf.size);

    if (len) {
        int err;

        len = uart_fifo_fill(dev, data, len);
        err = ring_buf_get_finish(&uart->tx_ringbuf, len);
        __ASSERT_NO_MSG(err == 0);
        ARG_UNUSED(err);
    } else {
        uart_irq_tx_disable(dev);
        uart->tx_busy = 0;
    }

    uart->common.handler(COMMS_TRANSPORT_EVT_TX_RDY, uart->common.context);
}

static void uart_callback(const struct device *dev, void *user_data) {
    struct comms_uart_int_driven *uart =
        (struct comms_uart_int_driven *)user_data;

    uart_irq_update(dev);

    if (uart_irq_rx_ready(dev)) {
        serial_leds_update_state(LED_START_RX);
        uart_rx_handle(dev, uart);
        serial_leds_update_state(LED_STOP_RX);
    }

    if (uart_irq_tx_ready(dev)) {
        serial_leds_update_state(LED_START_TX);
        uart_tx_handle(dev, uart);
        serial_leds_update_state(LED_STOP_TX);
    }
}

static void irq_init(struct comms_uart_int_driven *uart) {
    const struct device *dev = uart->common.dev;

    ring_buf_init(&uart->rx_ringbuf,
                  CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE,
                  uart->rx_buf);
    ring_buf_init(&uart->tx_ringbuf,
                  CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE,
                  uart->tx_buf);

    uart->tx_busy = false;
    uart_irq_callback_user_data_set(dev, uart_callback, (void *)uart);
    uart_irq_rx_enable(dev);

#if !IS_ENABLED(CONFIG_COMMS_BACKEND_SERIAL_CHECK_DTR)
    k_timer_init(&uart->dtr_timer, dtr_timer_handler, NULL);
    k_timer_user_data_set(&uart->dtr_timer, (void *)uart);
#endif
}

static int rx_enable(const struct device *dev, uint8_t *buf, size_t len) {
    return uart_rx_enable(dev, buf, len, 10000);
}

static int init(const struct comms_transport *transport, const void *config,
                comms_transport_handler_t evt_handler, void *context) {
    struct comms_uart_common *common =
        (struct comms_uart_common *)transport->ctx;

    common->dev = (const struct device *)config;
    common->handler = evt_handler;
    common->context = context;
    common->smp.buf_pool = &smp_comms_rx_pool;
    k_fifo_init(&common->smp.buf_ready);
    irq_init((struct comms_uart_int_driven *)transport->ctx);
    return 0;
}

static void irq_uninit(struct comms_uart_int_driven *uart) {
    const struct device *dev = uart->common.dev;

    k_timer_stop(&uart->dtr_timer);
    uart_irq_tx_disable(dev);
    uart_irq_rx_disable(dev);
}

static int uninit(const struct comms_transport *transport) {
    irq_uninit((struct comms_uart_int_driven *)transport->ctx);
    return 0;
}

static int enable(const struct comms_transport *transport, bool blocking_tx) {
    struct comms_uart_common *uart = (struct comms_uart_common *)transport->ctx;

    uart->blocking_tx =
        blocking_tx || IS_ENABLED(CONFIG_COMMS_BACKEND_FOREC_TX_BLOCKING_MODE);

    // TODO: int driven API stuff. See shell_uart.c
    if (blocking_tx) {
        uart_irq_tx_disable(uart->dev);
    }

    return 0;
}

static int irq_write(struct comms_uart_int_driven *uart, const void *data,
                     size_t length, size_t *cnt) {
    *cnt = ring_buf_put(&uart->tx_ringbuf, data, length);

    if (atomic_set(&uart->tx_busy, 1) == 0) {
        uart_irq_tx_enable(uart->common.dev);
    }

    return 0;
}

static int write_uart(const struct comms_transport *transport, const void *data,
                      size_t length, size_t *cnt) {
    struct comms_uart_common *uart = (struct comms_uart_common *)transport->ctx;
    return irq_write((struct comms_uart_int_driven *)transport->ctx, data,
                     length, cnt);
}

static int irq_read(struct comms_uart_int_driven *uart, void *data,
                    size_t length, size_t *cnt) {
    *cnt = ring_buf_get(&uart->rx_ringbuf, data, length);
    return 0;
}

static int read_uart(const struct comms_transport *transport, void *data,
                     size_t length, size_t *cnt) {
    return irq_read((struct comms_uart_int_driven *)transport->ctx, data,
                    length, cnt);
}

static void update(const struct comms_transport *transport) {
    /*
     * This is dependent on the fact that `struct comms_uart_common`
     * is always the first member, regardless of the UART configuration
     */
    struct comms_uart_common *uart = (struct comms_uart_common *)transport->ctx;

    smp_comms_process(&uart->smp);
}

const struct comms_transport_api comms_uart_transport_api = {.init = init,
                                                             .uninit = uninit,
                                                             .enable = enable,
                                                             .write =
                                                                 write_uart,
                                                             .read = read_uart,
                                                             .update = update};
COMMS_UART_DEFINE(comms_transport_uart);
COMMS_DEFINE(comms_uart, &comms_transport_uart);

static int enable_comms_uart(void) {
    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    smp_comms_init();
    comms_init(&comms_uart, dev);

    return 0;
}

SYS_INIT(enable_comms_uart, APPLICATION,
         UTIL_INC(CONFIG_APPLICATION_INIT_PRIORITY));

struct smp_comms_data *comms_uart_smp_comms_data_get_ptr(void) {
    struct comms_uart_common *common =
        (struct comms_uart_common *)comms_transport_uart.ctx;
    return &common->smp;
}

const struct comms *comms_backend_uart_get_ptr(void) { return &comms_uart; }
