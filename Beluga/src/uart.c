//
// Created by tom on 7/8/24.
//

#include <at_commands.h>
#include <stdint.h>
#include <uart.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define RX_BUFFER_SIZE 256

struct uart_data {
    size_t len;
    uint8_t data[RX_BUFFER_SIZE];
};

K_FIFO_DEFINE(uart_rx_queue);

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static struct uart_data rx_buf;

static void serial_callback(const struct device *dev, void *user_data) {
    uint8_t c;

    if (!uart_irq_update(uart)) {
        return;
    }

    if (!uart_irq_rx_ready(uart)) {
        return;
    }

    while (uart_fifo_read(uart, &c, 1) == 1) {
        if ((c == '\n' || c == '\r' || c == '\0') && rx_buf.len > 0) {
            struct buffer *_buf;
            rx_buf.data[rx_buf.len] = '\0';

            _buf = k_malloc(sizeof(*_buf));
            if (_buf == NULL) {
                printk("Unable to allocate fifo item\n");
                break;
            }
            _buf->len = rx_buf.len + 1;
            _buf->buf = k_malloc(_buf->len);
            if (_buf->buf == NULL) {
                printk("Unable to allocate buffer for fifo item\n");
                k_free(_buf);
                return;
            }
            memcpy(_buf->buf, rx_buf.data, rx_buf.len + 1);
            k_fifo_put(&uart_rx_queue, _buf);
            rx_buf.len = 0;
            printk("Placed item into fifo\n");
        } else if ((c != '\n' && c != '\r') &&
                   (rx_buf.len < (RX_BUFFER_SIZE - 1))) {
            rx_buf.data[rx_buf.len++] = c;
        }
    }
}

int uart_init(void) {
    int err;

    if (!device_is_ready(uart)) {
        printk("UART is not ready\n");
        return -1;
    }

    err = uart_irq_callback_user_data_set(uart, serial_callback, NULL);

    if (err < 0) {
        printk("Unable to set UART callback (err %d)\n", err);
        return err;
    }
    uart_irq_rx_enable(uart);

    return 0;
}
