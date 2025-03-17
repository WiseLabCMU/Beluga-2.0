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
#include <zephyr/drivers/serial/uart_async_rx.h>
#include <zephyr/sys/ring_buffer.h>

#if defined(__cplusplus)
extern "C" {
#endif

extern const struct comms_transport_api comms_uart_transport_api;

#ifndef CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE
#define CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE 0
#endif

#ifndef CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE
#define CONFIG_COMMS_BACKEND_SERIAL_TX_RING_BUFFER_SIZE 0
#endif

#ifndef CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_COUNT
#define CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_COUNT 0
#endif

#ifndef CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_SIZE
#define CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_SIZE 0
#endif

#define ASYNC_RX_BUF_SIZE                                                      \
    (CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_COUNT *                       \
     (CONFIG_COMMS_BACKEND_SERIAL_ASYNC_RX_BUFFER_SIZE +                       \
      UART_ASYNC_RX_BUF_OVERHEAD))

struct comms_uart_common {
    const struct device *dev;
    comms_transport_handler_t handler;
    void *context;
    bool blocking_tx;
#ifdef CONFIG_MCUMGR_TRANSPORT_COMMS
    struct smp_comms_data smp;
#endif // CONFIG_MCUMGR_TRANSPORT_COMMS
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

struct comms_uart_async {
    struct comms_uart_common common;
    struct k_sem tx_sem;
    struct uart_async_rx async_rx;
    struct uart_async_rx_config async_rx_config;
    atomic_t pending_rx_req;
    uint8_t rx_data[ASYNC_RX_BUF_SIZE];
};

struct comms_uart_polling {
    struct comms_uart_common common;
    struct ring_buf rx_ringbuf;
    uint8_t rx_buf[CONFIG_COMMS_BACKEND_SERIAL_RX_RING_BUFFER_SIZE];
    struct k_timer rx_timer;
};

#if defined(CONFIG_COMMS_BACKEND_SERIAL_API_POLLING)
#define COMMS_UART_STRUCT struct comms_uart_polling
#elif defined(CONFIG_COMMS_BACKEND_SERIAL_API_ASYNC)
#define COMMS_UART_STRUCT struct comms_uart_async
#else
#define COMMS_UART_STRUCT struct comms_uart_int_driven
#endif

/**
 * @brief Macro for creating comms UART transport instance named @p _name
 */
#define COMMS_UART_DEFINE(_name)                                               \
    static COMMS_UART_STRUCT _name##_comms_uart;                               \
    struct comms_transport _name = {                                           \
        .api = &comms_uart_transport_api,                                      \
        .ctx = (struct comms_telnet *)&_name##_comms_uart,                     \
    }

/**
 * @brief This function provides pointer to the comms UART backend instance.
 *
 * Function returns pointer to the comms UART instance. This instance can be
 * next used with comms_write_msg function in order to print data outside of
 * the comms thread.
 *
 * @returns Pointer to the shell instance.
 */
const struct comms *comms_backend_uart_get_ptr(void);

/**
 * @brief This function provides pointer to the smp comms data of the UART comms
 * transport.
 *
 * @returns Pointer to the smp comms data.
 */
struct smp_comms_data *comms_uart_smp_comms_data_get_ptr(void);

#if defined(__cplusplus)
}
#endif

#endif // BELUGA_DTS_COMMS_UART_H
