//
// Created by tom on 7/8/24.
//

#include <at_commands.h>
#include <stdint.h>
#include <uart.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(uart_log, LOG_LEVEL_INF);

#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
#define USB_CONSOLE
#endif

#define RX_BUFFER_SIZE 256

struct uart_data {
    size_t len;
    uint8_t data[RX_BUFFER_SIZE];
};

K_FIFO_DEFINE(uart_rx_queue);

static const struct device *serial = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static struct uart_data rx_buf;

static void serial_callback(const struct device *dev, void *user_data) {
    uint8_t c;

    if (dev != serial) {
        LOG_DBG("Device does not match serial");
        return;
    }

    if (!uart_irq_update(dev)) {
        LOG_DBG("There was no update in serial");
        return;
    }

    if (!uart_irq_rx_ready(dev)) {
        LOG_ERR("RX irq is not ready in serial");
        return;
    }

    while (uart_fifo_read(dev, &c, 1) == 1) {
        LOG_DBG("Received %c", c);
        if ((c == '\n' || c == '\r' || c == '\0') && rx_buf.len > 0) {
            struct buffer *_buf;
            rx_buf.data[rx_buf.len] = '\0';
            LOG_INF("Command reception complete: %s", rx_buf.data);

            _buf = k_malloc(sizeof(*_buf));
            if (_buf == NULL) {
                LOG_ERR("Unable to allocate fifo item");
                break;
            }
            _buf->len = rx_buf.len + 1;
            _buf->buf = k_malloc(_buf->len);
            if (_buf->buf == NULL) {
                LOG_ERR("Unable to allocate buffer for fifo item");
                k_free(_buf);
                return;
            }
            memcpy(_buf->buf, rx_buf.data, rx_buf.len + 1);
            k_fifo_put(&uart_rx_queue, _buf);
            rx_buf.len = 0;
            LOG_INF("Placed item into fifo");
        } else if ((c != '\n' && c != '\r') &&
                   (rx_buf.len < (RX_BUFFER_SIZE - 1))) {
            rx_buf.data[rx_buf.len++] = c;
        }
    }
}

#if defined(USB_CONSOLE)
#define device_init() usb_enable(NULL)

#define WAIT_DTR                                                               \
    do {                                                                       \
        uint32_t dtr = 0;                                                      \
        LOG_INF("Waiting on DTR");                                             \
        while (!dtr) {                                                         \
            uart_line_ctrl_get(serial, UART_LINE_CTRL_DTR, &dtr);              \
            k_sleep(K_MSEC(100));                                              \
        }                                                                      \
    } while (0)
#else
#define device_init() !device_is_ready(serial)
#define WAIT_DTR      LOG_INF("UART ready")
#endif

int uart_init(void) {
    int err;

    if (device_init()) {
        printk("Device is not ready\n");
        return -1;
    }

    err = uart_irq_callback_user_data_set(serial, serial_callback, NULL);

    if (err < 0) {
        printk("Unable to set UART callback (err %d)\n", err);
        return err;
    }
    LOG_DBG("UART IRQ set");

    uart_irq_rx_enable(serial);
    LOG_DBG("UART RX IRQ enabled");

    WAIT_DTR;

    printk("UART initialized\n");

    return 0;
}
