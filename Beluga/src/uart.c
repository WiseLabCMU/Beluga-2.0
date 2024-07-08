//
// Created by tom on 7/8/24.
//

#include <uart.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <stdint.h>

#define RX_BUFFER_SIZE 256

struct uart_data {
    void * fifo_reserved;
    size_t len;
    uint8_t data[RX_BUFFER_SIZE];
};

K_FIFO_DEFINE(uart_rx_queue);

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;
static struct uart_data rxBuffer = {
        .len = 0
};

K_SEM_DEFINE(nus_write_sem, 0, 1);

static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
    ARG_UNUSED(dev);

    static size_t aborted_len;
    static struct uart_data *buf;
    static uint8_t *aborted_buf;
    static bool disable_req;

    switch(evt->type) {
        case UART_RX_RDY:
            break;
        case UART_RX_DISABLED:
            break;
        case UART_RX_BUF_REQUEST:
            break;
        case UART_RX_BUF_RELEASED:
            break;
        case UART_TX_DONE:
            break;
        case UART_TX_ABORTED:
            break;
        default:
            break;
    }
}

int uart_init(void) {
    int err;

    if (!device_is_ready(uart)) {
        printk("UART device was not ready\n");
        return -ENODEV;
    }

    err = uart_callback_set(uart, uart_callback, NULL);
    if (err != 0) {
        printk("UART init error (%d)\n", err);
        return err;
    }

    return uart_rx_enable(uart, rxBuffer.data, RX_BUFFER_SIZE, UART_RX_TIMEOUT);
}
