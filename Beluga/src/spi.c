/**
 * @file spi.c
 * @brief SPI communication.
 *
 * This file provides an implementation for SPI communication with the DW1000
 * module It includes functionality for initializing the SPI interface,
 * configuring different SPI settings, and performing read/write operations. The
 * SPI communication is locked to ensure safe access to shared resources, and
 * chip select toggling is also supported.
 *
 * @note The SPI configuration includes both fast and slow speeds to accommodate
 * different use cases. The module also includes error handling for invalid
 * buffer sizes or SPI device readiness.
 *
 * SPI is used to interface with the DW1000 Ultra-Wideband (UWB) module, with
 * specific settings for:
 * - Slow SPI frequency (2 MHz)
 * - Fast SPI frequency (8 MHz)
 *
 * @author Tom Schmitz
 * @date 6/26/24
 */

#include <spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

/**
 * Logger for the SPI module
 */
LOG_MODULE_REGISTER(spi_logger, CONFIG_SPI_MODULE_LOG_LEVEL);

/**
 * The number of SPI configurations for the DW1000 (fast and slow)
 */
#define NUM_SPI_CONFIGS 2

/**
 * The maximum buffer size for SPI transactions
 */
#define SPI_BUF_SIZE 255

/**
 * Index for the slow SPI configuration
 */
#define DW1000_CONFIG_SLOW 0

/**
 * Index for the fast SPI configuration
 */
#define DW1000_CONFIG_FAST 1

/**
 * Slow SPI frequency
 */
#define DW1000_SLOW_FREQUENCY MHZ(2)

/**
 * Fast SPI frequency
 */
#define DW1000_FAST_FREQUENCY MHZ(8)

/**
 * Index for the receive buffer
 */
#define RX_BUF_IDX 1

/**
 * Index for the transmit buffer
 */
#define TX_BUF_IDX 0

/**
 * The SPI device from the device tree
 */
#define SPI1_NAME DEVICE_DT_GET(DT_NODELABEL(spi1))

/**
 * The SPI device
 */
static const struct device *spi_device = SPI1_NAME;

/**
 * The current SPI configuration for the DW1000
 */
static struct spi_config *dw1000SpiConfig = NULL;

/**
 * The different SPI configurations for the DW1000
 */
static struct spi_config spiConfigs[NUM_SPI_CONFIGS];

/**
 * The DW1000 transmit buffer
 */
static uint8_t dw1000_txBuf[SPI_BUF_SIZE];

/**
 * The DW1000 receive buffer
 */
static uint8_t dw1000_rxBuf[SPI_BUF_SIZE];

/**
 * The SPI buffers
 */
static struct spi_buf dw1000_spiBufs[2];

/**
 * SPI buffer set for transmission
 */
static struct spi_buf_set dw1000_tx;

/**
 * SPI buffer set for reception
 */
static struct spi_buf_set dw1000_rx;

/**
 * Chip select for the DW1000
 */
static const struct spi_cs_control dw1000_cs =
    SPI_CS_CONTROL_INIT(DT_NODELABEL(dw1000_spi), 0);

/**
 * Lock for SPI 1
 */
K_MUTEX_DEFINE(spi1_lock);

/**
 * @brief Initializes SPI 1 for communication with the DW1000
 * @return 0 upon success
 * @return -ENODEV if SPI device is not ready
 */
int init_spi1(void) {
    if (!device_is_ready(spi_device)) {
        LOG_ERR("Failed to bind SPI1");
        return -ENODEV;
    }

    spiConfigs[DW1000_CONFIG_SLOW].cs = dw1000_cs;
    spiConfigs[DW1000_CONFIG_FAST].cs = dw1000_cs;

    for (int i = 0; i < NUM_SPI_CONFIGS; i++) {
        spiConfigs[i].operation = SPI_WORD_SET(8);
    }

    spiConfigs[DW1000_CONFIG_SLOW].frequency = DW1000_SLOW_FREQUENCY;
    spiConfigs[DW1000_CONFIG_FAST].frequency = DW1000_FAST_FREQUENCY;

    memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
    memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);

    dw1000_spiBufs[TX_BUF_IDX].buf = dw1000_txBuf;
    dw1000_spiBufs[TX_BUF_IDX].len = SPI_BUF_SIZE;
    dw1000_spiBufs[RX_BUF_IDX].buf = dw1000_rxBuf;
    dw1000_spiBufs[RX_BUF_IDX].len = SPI_BUF_SIZE;

    dw1000_tx.buffers = &dw1000_spiBufs[TX_BUF_IDX];
    dw1000_tx.count = 1;
    dw1000_rx.buffers = &dw1000_spiBufs[RX_BUF_IDX];
    dw1000_rx.count = 1;

    dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_SLOW];

    LOG_INF("SPI 1 and SPI 2 initialized");

    return 0;
}

/**
 * @brief Selects the low speed configuration for the SPI
 */
void set_spi_slow(void) {
    dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_SLOW];
    memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
    memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
}

/**
 * @brief Selects the high speed configuration for the SPI
 */
void set_spi_fast(void) {
    dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_FAST];
    memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
    memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
}

/**
 * @brief Writes the bytes stored in buffer over SPI
 * @param[in] buffer The bytes to write over SPI
 * @param[in] bufLength The number of bytes to write
 * @return 0 upon success
 * @return -EINVAL if buffer is NULL
 * @return negative error code otherwise
 */
int write_spi(const uint8_t *buffer, size_t bufLength) {
    int err;

    if (buffer == NULL) {
        return -EINVAL;
    }

    // Lock here because we are using the shared resources now...
    k_mutex_lock(&spi1_lock, K_FOREVER);
    memcpy(dw1000_txBuf, buffer, bufLength);
    dw1000_spiBufs[TX_BUF_IDX].len = bufLength;
    dw1000_spiBufs[RX_BUF_IDX].len = bufLength;

    err = spi_transceive(spi_device, dw1000SpiConfig, &dw1000_tx, &dw1000_rx);
    k_mutex_unlock(&spi1_lock);

    return err;
}

/**
 * @brief Reads bytes over SPI given write date
 * @param[in] writeBuffer The data to write over SPI
 * @param[out] readBuf The data to read from SPI
 * @param[in] bufLength The number of bytes to read/write over SPI
 * @return 0 upon success
 * @return -EINVAL if invalid parameter
 * @return negative error code otherwise
 */
int read_spi(const uint8_t *writeBuffer, uint8_t *readBuf, size_t bufLength) {
    int err;

    if (writeBuffer == NULL) {
        return -EINVAL;
    }

    if (readBuf == NULL) {
        return -EINVAL;
    }

    // Lock here because we are using the shared resources now
    k_mutex_lock(&spi1_lock, K_FOREVER);
    memcpy(dw1000_txBuf, writeBuffer, bufLength);
    dw1000_spiBufs[TX_BUF_IDX].len = bufLength;
    dw1000_spiBufs[RX_BUF_IDX].len = bufLength;

    err = spi_transceive(spi_device, dw1000SpiConfig, &dw1000_tx, &dw1000_rx);

    if (err == 0) {
        memcpy(readBuf, dw1000_rxBuf, bufLength);
    }
    k_mutex_unlock(&spi1_lock);

    return err;
}

/**
 * @brief Shuts down the SPI device
 */
void shutdown_spi(void) {
    int rc = pm_device_action_run(spi_device, PM_DEVICE_ACTION_TURN_OFF);
    if (rc < 0) {
        LOG_ERR("Unable to turn off SPI 1 (%d)\n", rc);
    }
}

/**
 * @brief Toggles the chip select line for a specified amount of time.
 * @param[in] us The number of microseconds to toggle the chip select for
 */
void toggle_cs_line(int32_t us) {
    gpio_pin_toggle_dt(&dw1000SpiConfig->cs.gpio);
    k_sleep(K_USEC(us));
    gpio_pin_toggle_dt(&dw1000SpiConfig->cs.gpio);
    k_sleep(K_USEC(us));
}
