/**
 * @file uart.c
 * @brief Serial communication driver for handling UART data reception and
 * transmission.
 *
 * This file contains the implementation for initializing and managing UART
 * communication. It includes:
 * - UART interrupt handling
 * - Buffer management for received UART data
 * - Serial output hooks for custom console output behavior
 * - LED control integration during UART activity
 *
 * @date 7/8/2024
 * @author Tom Schmitz
 */

#include <at_commands.h>
#include <beluga_message.h>
#include <serial_led.h>
#include <stdint.h>
#include <stdio.h>
#include <uart.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#if defined(CONFIG_BELUGA_FRAMES) || defined(CONFIG_SERIAL_LEDS)
#include <zephyr/pm/device_runtime.h>
#endif // defined(CONFIG_BELUGA_FRAMES) || defined(CONFIG_SERIAL_LEDS)

/**
 * Logger for the serial
 */
LOG_MODULE_REGISTER(serial_log, CONFIG_SERIAL_MODULE_LOG_LEVEL);

/**
 * The receiver buffer size
 */
#define RX_BUFFER_SIZE 256

/**
 * @brief UART received data.
 *
 * Holds data received over UART along with the length of the data.
 */
struct uart_data {
    size_t len;                   ///< Number of bytes received
    uint8_t data[RX_BUFFER_SIZE]; ///< Buffer to store received data
};

/**
 * Fifo for received messages (terminated by \\r\\n)
 */
K_FIFO_DEFINE(uart_rx_queue);

/**
 * Serial device from the devicetree
 */
static const struct device *serial = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/**
 * Received data over serial
 */
static struct uart_data rx_buf;

/**
 * @brief Callback for servicing serial interrupts
 * @param[in] dev The serial device that the interrupt is for
 * @param[in] user_data Additional (unused) context
 */
static void serial_callback(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    uint8_t c;

    if (dev != serial) {
        LOG_DBG("Device does not match serial");
        return;
    }

    if (!uart_irq_update(dev)) {
        LOG_DBG("There was no update in serial");
        return;
    }

    if (uart_irq_rx_ready(dev)) {
#if defined(CONFIG_SERIAL_LEDS)
        serial_leds_update_state(LED_START_RX);
#endif // defined(CONFIG_SERIAL_LEDS)
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
#if defined(CONFIG_SERIAL_LEDS)
        serial_leds_update_state(LED_STOP_RX);
#endif // defined(CONFIG_SERIAL_LEDS)
    }
#if defined(CONFIG_SERIAL_LEDS)
    if (uart_irq_tx_complete(dev)) {
        serial_leds_update_state(LED_STOP_TX);
    }
#endif // defined(CONFIG_SERIAL_LEDS)
}

#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
/**
 * Initialization routine for the serial device
 */
#define device_init() usb_enable(NULL)

/**
 * Routine that waits until the DTR control line for serial is set
 */
#define WAIT_DTR()                                                             \
    do {                                                                       \
        uint32_t dtr = 0;                                                      \
        LOG_INF("Waiting on DTR");                                             \
        while (!dtr) {                                                         \
            uart_line_ctrl_get(serial, UART_LINE_CTRL_DTR, &dtr);              \
            k_sleep(K_MSEC(100));                                              \
        }                                                                      \
    } while (0)
#else
/**
 * Initialization routine for the serial device
 */
#define device_init() !device_is_ready(serial)

/**
 * Routine that waits until the DTR control line for serial is set
 */
#define WAIT_DTR()    LOG_INF("UART ready")
#endif // DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)

#if defined(CONFIG_SERIAL_LEDS) && !defined(CONFIG_BELUGA_FRAMES)

/**
 * @brief Serial output hook for printing serial data
 * @param[in] c The character to output
 * @return The character printed
 */
static int console_out(int c) {
    // Taken from Zephyr's uart console driver
#ifdef CONFIG_UART_CONSOLE_DEBUG_SERVER_HOOKS

    int handled_by_debug_server = HANDLE_DEBUG_HOOK_OUT(c);

    if (handled_by_debug_server) {
        return c;
    }

#endif /* CONFIG_UART_CONSOLE_DEBUG_SERVER_HOOKS */

    if (pm_device_runtime_is_enabled(serial)) {
        if (pm_device_runtime_get(serial) < 0) {
            /* Enabling the UART instance has failed but this
             * function MUST return the byte output.
             */
            return c;
        }
    }

    serial_leds_update_state(LED_START_TX);

    if ('\n' == c) {
        uart_poll_out(serial, '\r');
    }
    uart_poll_out(serial, c);

    if (pm_device_runtime_is_enabled(serial)) {
        /* As errors cannot be returned, ignore the return value */
        (void)pm_device_runtime_put(serial);
    }

    return c;
}

#if defined(CONFIG_STDOUT_CONSOLE)
extern void __stdout_hook_install(int (*hook)(int c));
#endif // defined(CONFIG_STDOUT_CONSOLE)

#if defined(CONFIG_PRINTK)
extern void __printk_hook_install(int (*fn)(int c));
#endif // defined(CONFIG_PRINTK)

/**
 * @brief Helper function to replace the original console output hooks
 */
static void install_uart_hooks(void) {
#if defined(CONFIG_STDOUT_CONSOLE)
    __stdout_hook_install(console_out);
#endif // defined(CONFIG_STDOUT_CONSOLE)
#if defined(CONFIG_PRINTK)
    __printk_hook_install(console_out);
#endif // defined(CONFIG_PRINTK)
}

/**
 * @brief Macro to enable the tx complete irq for serial
 */
#define enable_uart_tx_irq(x) uart_irq_tx_enable(x)
#elif defined(CONFIG_BELUGA_FRAMES)

/**
 * Prevents characters from being printed to console while frames are being
 * used.
 * @param c
 * @return
 */
static int dev_null(int c) { return c; }

#if defined(CONFIG_STDOUT_CONSOLE)
extern void __stdout_hook_install(int (*hook)(int c));
#endif // defined(CONFIG_STDOUT_CONSOLE)

/**
 * @brief Helper function to replace the original console output hooks
 */
static void install_uart_hooks(void) {
#if defined(CONFIG_STDOUT_CONSOLE)
    __stdout_hook_install(dev_null);
#endif // defined(CONFIG_STDOUT_CONSOLE)
}

#if defined(CONFIG_SERIAL_LEDS)
/**
 * @brief Macro to enable the tx complete irq for serial
 */
#define enable_uart_tx_irq(x) uart_irq_tx_enable(x)
#else
/**
 * @brief Macro to enable the tx complete irq for serial
 */
#define enable_uart_tx_irq(x) (void)0
#endif

#else
/**
 * @brief Helper function to replace the original console output hooks
 */
#define install_uart_hooks()  (void)0

/**
 * @brief Macro to enable the tx complete irq for serial
 */
#define enable_uart_tx_irq(x) (void)0
#endif // defined(CONFIG_SERIAL_LEDS)

/**
 * @brief Initializes the serial connection by enabling interrupts, waiting for
 * the peer to be ready, and installing the output hooks.
 * @return 0 upon success
 * @return -ENODEV if serial device is not ready
 * @return negative error code otherwise
 */
int uart_init(void) {
    int err;

    if (device_init()) {
        LOG_ERR("Device is not ready\n");
        return -ENODEV;
    }

    err = serial_leds_init();

    if (err != 0) {
        LOG_ERR("Cannot initialize serial LEDs");
        return err;
    }

    err = uart_irq_callback_user_data_set(serial, serial_callback, NULL);

    if (err < 0) {
        LOG_ERR("Unable to set UART callback (err %d)\n", err);
        return err;
    }
    LOG_DBG("UART IRQ set");

    install_uart_hooks();

    uart_irq_rx_enable(serial);
    enable_uart_tx_irq(serial);

    LOG_DBG("UART TRX IRQ enabled");

    WAIT_DTR();

    LOG_INF("UART initialized\n");

    return 0;
}

#if defined(CONFIG_BELUGA_FRAMES)

K_MUTEX_DEFINE(write_lock);

static void write_char(char c) {
    // Taken from Zephyr's uart console driver
#ifdef CONFIG_UART_CONSOLE_DEBUG_SERVER_HOOKS

    int handled_by_debug_server = HANDLE_DEBUG_HOOK_OUT(c);

    if (handled_by_debug_server) {
        return c;
    }

#endif /* CONFIG_UART_CONSOLE_DEBUG_SERVER_HOOKS */

    if (pm_device_runtime_is_enabled(serial)) {
        if (pm_device_runtime_get(serial) < 0) {
            /* Enabling the UART instance has failed. */
            return;
        }
    }

#if defined(CONFIG_SERIAL_LEDS)
    serial_leds_update_state(LED_START_TX);
#endif // defined(CONFIG_SERIAL_LEDS)

    uart_poll_out(serial, c);

    if (pm_device_runtime_is_enabled(serial)) {
        /* As errors cannot be returned, ignore the return value */
        (void)pm_device_runtime_put(serial);
    }
}

static int frame_write(uint8_t *buffer, size_t len) {
    k_mutex_lock(&write_lock, K_FOREVER);

    for (size_t i = 0; i < len; i++) {
        write_char(buffer[i]);
    }

    k_mutex_unlock(&write_lock);
    return 0;
}

int write_message_frame(const struct beluga_msg *msg) {
    int ret;

    if (msg == NULL) {
        return -EINVAL;
    }

    ssize_t msg_length = frame_length(msg);
    if (msg_length < 1) {
        return -EINVAL;
    }

    uint8_t *buffer = k_malloc(msg_length + 1);
    if (buffer == NULL) {
        return -ENOMEM;
    }

    msg_length = construct_frame(msg, buffer, msg_length + 1);

    if (msg_length < 0) {
        k_free(buffer);
        return msg_length;
    }

    ret = frame_write(buffer, msg_length);

    k_free(buffer);

    return ret;
}

#else
int write_message_frame(const struct beluga_msg *msg) {
    ARG_UNUSED(msg);
    return -ENOTSUP;
}
#endif
