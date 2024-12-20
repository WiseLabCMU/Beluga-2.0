//
// Created by tom on 6/26/24.
//

#include <spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(spi_logger, CONFIG_SPI_MODULE_LOG_LEVEL);

#if defined(CONFIG_BELUGA_USE_SPI2)
#define NUM_SPI_CONFIGS 4
#else
#define NUM_SPI_CONFIGS 2
#endif

#define SPI_BUF_SIZE       255

#define DW1000_CONFIG_SLOW 0
#define DW1000_CONFIG_FAST 1
#if defined(CONFIG_BELUGA_USE_SPI2)
#define NRF21540_CONFIG_SLOW 2
#define NRF21540_CONFIG_FAST 3
#endif

#define DW1000_SLOW_FREQUENCY 2000000
#define DW1000_FAST_FREQUENCY 8000000

#if defined(CONFIG_BELUGA_USE_SPI2)
#define NRF21540_SLOW_FREQUENCY 1000000
#define NRF21540_FAST_FREQUENCY 8000000
#endif

#define RX_BUF_IDX 1
#define TX_BUF_IDX 0

#define SPI1       0
#define SPI2       1

#define SPI1_NAME  DEVICE_DT_GET(DT_NODELABEL(spi1))
#if defined(CONFIG_BELUGA_USE_SPI2)
#define SPI2_NAME DEVICE_DT_GET(DT_NODELABEL(spi2))
#endif

#if defined(CONFIG_BELUGA_USE_SPI2)
static const struct device *spi_device[2];
#else
static const struct device *spi_device[1];
#endif
static struct spi_config *dw1000SpiConfig = NULL;
#if defined(CONFIG_BELUGA_USE_SPI2)
static struct spi_config *nrfSpiConfig = NULL;
#endif
static struct spi_config spiConfigs[NUM_SPI_CONFIGS];

static uint8_t dw1000_txBuf[SPI_BUF_SIZE];
static uint8_t dw1000_rxBuf[SPI_BUF_SIZE];
#if defined(CONFIG_BELUGA_USE_SPI2)
static uint8_t nrf21_txBuf[SPI_BUF_SIZE];
static uint8_t nrf21_rxBuf[SPI_BUF_SIZE];
#endif

static struct spi_buf dw1000_spiBufs[2];
#if defined(CONFIG_BELUGA_USE_SPI2)
static struct spi_buf nrf21_spiBufs[2];
#endif

static struct spi_buf_set dw1000_tx;
static struct spi_buf_set dw1000_rx;
#if defined(CONFIG_BELUGA_USE_SPI2)
static struct spi_buf_set nrf21_tx;
static struct spi_buf_set nrf21_rx;
#endif

#if defined(CONFIG_BELUGA_USE_SPI2)
static const struct spi_cs_control nrf_fem_cs =
    SPI_CS_CONTROL_INIT(DT_NODELABEL(nrf_radio_fem_spi), 0);
#endif
static const struct spi_cs_control dw1000_cs =
    SPI_CS_CONTROL_INIT(DT_NODELABEL(dw1000_spi), 0);

K_MUTEX_DEFINE(spi1_lock);
#if defined(CONFIG_BELUGA_USE_SPI2)
K_MUTEX_DEFINE(spi2_lock);
#endif

int init_spi1(void) {
    spi_device[SPI1] = SPI1_NAME;
    if (!device_is_ready(spi_device[SPI1])) {
        LOG_ERR("Failed to bind SPI1");
        return -1;
    }

#if defined(CONFIG_BELUGA_USE_SPI2)
    spi_device[SPI2] = SPI2_NAME;
    if (!device_is_ready(spi_device[SPI2])) {
        LOG_ERR("Failed to bind SPI2");
        return -1;
    }
#endif

    spiConfigs[DW1000_CONFIG_SLOW].cs = dw1000_cs;
    spiConfigs[DW1000_CONFIG_FAST].cs = dw1000_cs;
#if defined(CONFIG_BELUGA_USE_SPI2)
    spiConfigs[NRF21540_CONFIG_SLOW].cs = nrf_fem_cs;
    spiConfigs[NRF21540_CONFIG_FAST].cs = nrf_fem_cs;
#endif

    for (int i = 0; i < NUM_SPI_CONFIGS; i++) {
        spiConfigs[i].operation = SPI_WORD_SET(8);
    }

    spiConfigs[DW1000_CONFIG_SLOW].frequency = DW1000_SLOW_FREQUENCY;
    spiConfigs[DW1000_CONFIG_FAST].frequency = DW1000_FAST_FREQUENCY;
#if defined(CONFIG_BELUGA_USE_SPI2)
    spiConfigs[NRF21540_CONFIG_SLOW].frequency = NRF21540_SLOW_FREQUENCY;
    spiConfigs[NRF21540_CONFIG_FAST].frequency = NRF21540_FAST_FREQUENCY;
#endif

    memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
    memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
#if defined(CONFIG_BELUGA_USE_SPI2)
    memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
    memset(nrf21_txBuf, 0, SPI_BUF_SIZE);
#endif

    dw1000_spiBufs[TX_BUF_IDX].buf = dw1000_txBuf;
    dw1000_spiBufs[TX_BUF_IDX].len = SPI_BUF_SIZE;
    dw1000_spiBufs[RX_BUF_IDX].buf = dw1000_rxBuf;
    dw1000_spiBufs[RX_BUF_IDX].len = SPI_BUF_SIZE;

#if defined(CONFIG_BELUGA_USE_SPI2)
    nrf21_spiBufs[TX_BUF_IDX].buf = nrf21_txBuf;
    nrf21_spiBufs[TX_BUF_IDX].len = SPI_BUF_SIZE;
    nrf21_spiBufs[RX_BUF_IDX].buf = nrf21_rxBuf;
    nrf21_spiBufs[RX_BUF_IDX].len = SPI_BUF_SIZE;
#endif

    dw1000_tx.buffers = &dw1000_spiBufs[TX_BUF_IDX];
    dw1000_tx.count = 1;
    dw1000_rx.buffers = &dw1000_spiBufs[RX_BUF_IDX];
    dw1000_rx.count = 1;

#if defined(CONFIG_BELUGA_USE_SPI2)
    nrf21_tx.buffers = &nrf21_spiBufs[TX_BUF_IDX];
    nrf21_tx.count = 1;
    nrf21_rx.buffers = &nrf21_spiBufs[RX_BUF_IDX];
    nrf21_rx.count = 1;
#endif

    dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_SLOW];

#if defined(CONFIG_BELUGA_USE_SPI2)
    nrfSpiConfig = &spiConfigs[NRF21540_CONFIG_SLOW];
#endif

    LOG_INF("SPI 1 and SPI 2 initialized");

    return 0;
}

void set_spi_slow(beluga_spi_channel_t channel) {
    switch (channel) {
    case DW1000_SPI_CHANNEL: {
        dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_SLOW];
        memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
        memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
        break;
    }
#if defined(CONFIG_BELUGA_USE_SPI2)
    case NRF21_SPI_CHANNEL: {
        nrfSpiConfig = &spiConfigs[NRF21540_CONFIG_SLOW];
        memset(nrf21_txBuf, 0, SPI_BUF_SIZE);
        memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
        break;
    }
#endif
    default: {
        assert_print("Invalid SPI channel!");
        break;
    }
    }
}

void set_spi_fast(beluga_spi_channel_t channel) {
    switch (channel) {
    case DW1000_SPI_CHANNEL: {
        dw1000SpiConfig = &spiConfigs[DW1000_CONFIG_FAST];
        memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
        memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
        break;
    }
#if defined(CONFIG_BELUGA_USE_SPI2)
    case NRF21_SPI_CHANNEL: {
        nrfSpiConfig = &spiConfigs[NRF21540_CONFIG_FAST];
        memset(nrf21_txBuf, 0, SPI_BUF_SIZE);
        memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
        break;
    }
#endif
    default: {
        assert_print("Invalid SPI channel!");
        break;
    }
    }
}

int write_spi(beluga_spi_channel_t channel, const uint8_t *buffer,
              size_t bufLength) {
    struct spi_config *_spiConfig;
    struct spi_buf *spiBuf;
    struct spi_buf_set *tx, *rx;
    uint8_t *txBuf, spi;
    int err;
    struct k_mutex *lock;

    switch (channel) {
    case DW1000_SPI_CHANNEL: {
        _spiConfig = dw1000SpiConfig;
        txBuf = dw1000_txBuf;
        spiBuf = dw1000_spiBufs;
        tx = &dw1000_tx;
        rx = &dw1000_rx;
        spi = SPI1;
        lock = &spi1_lock;
        break;
    }
#if defined(CONFIG_BELUGA_USE_SPI2)
    case NRF21_SPI_CHANNEL: {
        _spiConfig = nrfSpiConfig;
        txBuf = nrf21_txBuf;
        spiBuf = nrf21_spiBufs;
        tx = &nrf21_tx;
        rx = &nrf21_rx;
        spi = SPI2;
        lock = &spi2_lock;
        break;
    }
#endif
    default: {
        LOG_ERR("Invalid SPI channel!");
        return -1;
    }
    }

    // Lock here because we are using the shared resources now...
    k_mutex_lock(lock, K_FOREVER);
    memcpy(txBuf, buffer, bufLength);
    spiBuf[TX_BUF_IDX].len = bufLength;
    spiBuf[RX_BUF_IDX].len = bufLength;

    err = spi_transceive(spi_device[spi], _spiConfig, tx, rx);
    k_mutex_unlock(lock);

    return err;
}

int read_spi(beluga_spi_channel_t channel, const uint8_t *writeBuffer,
             uint8_t *readBuf, size_t bufLength) {
    struct spi_config *_spiConfig;
    struct spi_buf *spiBuf;
    struct spi_buf_set *tx, *rx;
    uint8_t *txBuf, *rxBuf, spi;
    int err;
    struct k_mutex *lock;

    switch (channel) {
    case DW1000_SPI_CHANNEL: {
        _spiConfig = dw1000SpiConfig;
        txBuf = dw1000_txBuf;
        rxBuf = dw1000_rxBuf;
        spiBuf = dw1000_spiBufs;
        tx = &dw1000_tx;
        rx = &dw1000_rx;
        spi = SPI1;
        lock = &spi1_lock;
        break;
    }
#if defined(CONFIG_BELUGA_USE_SPI2)
    case NRF21_SPI_CHANNEL: {
        _spiConfig = nrfSpiConfig;
        txBuf = nrf21_txBuf;
        rxBuf = nrf21_rxBuf;
        spiBuf = nrf21_spiBufs;
        tx = &nrf21_tx;
        rx = &nrf21_rx;
        spi = SPI2;
        lock = &spi2_lock;
        break;
    }
#endif
    default: {
        LOG_ERR("Invalid SPI channel!");
        return -1;
    }
    }

    // Lock here because we are using the shared resources now
    k_mutex_lock(lock, K_FOREVER);
    memcpy(txBuf, writeBuffer, bufLength);
    spiBuf[TX_BUF_IDX].len = bufLength;
    spiBuf[RX_BUF_IDX].len = bufLength;

    err = spi_transceive(spi_device[spi], _spiConfig, tx, rx);

    if (err == 0) {
        memcpy(readBuf, rxBuf, bufLength);
    }
    k_mutex_unlock(lock);

    return err;
}

void shutdown_spi(void) {
    int rc = pm_device_action_run(spi_device[SPI1], PM_DEVICE_ACTION_TURN_OFF);
    if (rc < 0) {
        LOG_ERR("Unable to turn off SPI 1 (%d)\n", rc);
    }

#if defined(CONFIG_BELUGA_USE_SPI2)
    rc = pm_device_action_run(spi_device[SPI2], PM_DEVICE_ACTION_TURN_OFF);
    if (rc < 0) {
        LOG_ERR("Unable to turn off SPI 2 (%d)", rc);
    }
#endif
}

void toggle_cs_line(beluga_spi_channel_t channel, int32_t us) {
    struct spi_config *_spiConfig;
    switch (channel) {
    case DW1000_SPI_CHANNEL:
        _spiConfig = dw1000SpiConfig;
        break;
#if defined(CONFIG_BELUGA_USE_SPI2)
    case NRF21_SPI_CHANNEL:
        _spiConfig = nrfSpiConfig;
        break;
#endif
    default:
        LOG_ERR("Invalid SPI channel\n");
        return;
    }

    gpio_pin_toggle_dt(&_spiConfig->cs.gpio);
    k_sleep(K_USEC(us));
    gpio_pin_toggle_dt(&_spiConfig->cs.gpio);
    k_sleep(K_USEC(us));
}
