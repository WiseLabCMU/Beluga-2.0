/**
 * @file comms_uart.h
 *
 * @brief
 *
 * @date 2/17/25
 *
 * @author tom
 */

#ifndef BELUGA_DTS_COMMS_UART_H
#define BELUGA_DTS_COMMS_UART_H

#include <serial/comms.h>
#include <serial/smp_comms.h>
#include <zephyr/sys/ring_buffer.h>

#ifndef CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE
#define CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE 0
#endif

#ifndef CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE
#define CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE 0
#endif

struct comms_uart_common {
    const struct device *dev;
    comms_transport_handler_t handler;
    void *context;
    bool blocking_tx;
    struct smp_comms_data smp;
};

struct comms_uart_int_driven {
    struct comms_uart_common common;
    struct ring_buf tx_ringbuf;
    struct ring_buf rx_ringbuf;
    uint8_t tx_buf[CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE];
    uint8_t rx_buf[CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE];
    struct k_timer dtr_timer;
    atomic_t tx_busy;
};

#define COMMS_UART_DEFINE(_name)                                               \
    static struct comms_uart_int_driven _name##_comms_uart;                    \
    struct comms_transport _name = {                                           \
        .api = &comms_uart_transport_api,                                      \
        .ctx = (struct comms_telnet *)&_name##_comms_uart,                     \
    }

// const struct comms *comms_backend_uart_get_ptr(void);

struct smp_comms_data *comms_uart_smp_comms_data_get_ptr(void);

#endif // BELUGA_DTS_COMMS_UART_H
