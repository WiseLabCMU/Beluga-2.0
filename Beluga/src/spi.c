//
// Created by tom on 6/26/24.
//

#include <spi.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define NUM_SPI_CONFIGS 4
#define SPI_BUF_SIZE 255

#define DW1000_CONFIG_SLOW 0
#define DW1000_CONFIG_FAST 1
#define NRF21540_CONFIG_SLOW 2
#define NRF21540_CONFIG_FAST 3

#define DW1000_SLOW_FREQUENCY 2000000
#define DW1000_FAST_FREQUENCY 8000000

#define NRF21540_SLOW_FREQUENCY 1000000
#define NRF21540_FAST_FREQUENCY 8000000

#define RX_BUF_IDX 1
#define TX_BUF_IDX 0

#define SPI_NAME DT_NODE_FULL_NAME(DT_PHANDLE_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0))

static const struct device *spi_device;
static struct spi_config *dw1000SpiConfig = NULL;
static struct spi_config *nrfSpiConfig = NULL;
static struct spi_config spiConfigs[NUM_SPI_CONFIGS];

static uint8_t dw1000_txBuf[SPI_BUF_SIZE];
static uint8_t dw1000_rxBuf[SPI_BUF_SIZE];
static uint8_t nrf21_txBuf[SPI_BUF_SIZE];
static uint8_t nrf21_rxBuf[SPI_BUF_SIZE];

static struct spi_buf dw1000_spiBufs[2];
static struct spi_buf nrf21_spiBufs[2];

static struct spi_buf_set dw1000_tx;
static struct spi_buf_set dw1000_rx;
static struct spi_buf_set nrf21_tx;
static struct spi_buf_set nrf21_rx;

static const struct spi_cs_control nrf_fem_cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(nrf21540-fem-spi@0), 2);
static const struct spi_cs_control dw1000_cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(dw1000@1), 2);

int init_spi1(void) {
    spiConfigs[DW1000_CONFIG_SLOW].cs = dw1000_cs;
    spiConfigs[DW1000_CONFIG_FAST].cs = dw1000_cs;
    spiConfigs[NRF21540_CONFIG_SLOW].cs = nrf_fem_cs;
    spiConfigs[NRF21540_CONFIG_FAST].cs = nrf_fem_cs;

    spi_device = device_get_binding(SPI_NAME);
    if (spi_device == NULL) {
        printk("Failed to bind SPI1\n");
        return -1;
    }

    for (int i = 0; i < NUM_SPI_CONFIGS; i++) {
        spiConfigs[i].operation = SPI_WORD_SET(8);
    }

    spiConfigs[DW1000_CONFIG_SLOW].frequency = DW1000_SLOW_FREQUENCY;
    spiConfigs[DW1000_CONFIG_FAST].frequency = DW1000_FAST_FREQUENCY;
    spiConfigs[NRF21540_CONFIG_SLOW].frequency = NRF21540_SLOW_FREQUENCY;
    spiConfigs[NRF21540_CONFIG_FAST].frequency = NRF21540_FAST_FREQUENCY;

    memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
    memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
    memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
    memset(nrf21_txBuf, 0, SPI_BUF_SIZE);

    dw1000_spiBufs[TX_BUF_IDX].buf = dw1000_txBuf;
    dw1000_spiBufs[TX_BUF_IDX].len = SPI_BUF_SIZE;
    dw1000_spiBufs[RX_BUF_IDX].buf = dw1000_rxBuf;
    dw1000_spiBufs[RX_BUF_IDX].len = SPI_BUF_SIZE;

    nrf21_spiBufs[TX_BUF_IDX].buf = nrf21_txBuf;
    nrf21_spiBufs[TX_BUF_IDX].len = SPI_BUF_SIZE;
    nrf21_spiBufs[RX_BUF_IDX].buf = nrf21_rxBuf;
    nrf21_spiBufs[RX_BUF_IDX].len = SPI_BUF_SIZE;

    dw1000_tx.buffers = &dw1000_spiBufs[TX_BUF_IDX];
    dw1000_tx.count = 1;
    dw1000_rx.buffers = &dw1000_spiBufs[RX_BUF_IDX];
    dw1000_rx.count = 1;

    nrf21_tx.buffers = &nrf21_spiBufs[TX_BUF_IDX];
    nrf21_tx.count = 1;
    nrf21_rx.buffers = &nrf21_spiBufs[RX_BUF_IDX];
    nrf21_rx.count = 1;

    dw1000SpiConfig = &spiConfigs[DW1000_SLOW_FREQUENCY];
    nrfSpiConfig = &spiConfigs[NRF21540_SLOW_FREQUENCY];

    return 0;
}

void set_spi_slow(beluga_spi_channel_t channel) {
    switch(channel) {
        case DW1000_SPI_CHANNEL: {
            dw1000SpiConfig = &spiConfigs[DW1000_SLOW_FREQUENCY];
            memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
            memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
            break;
        }
        case NRF21_SPI_CHANNEL: {
            nrfSpiConfig = &spiConfigs[NRF21540_SLOW_FREQUENCY];
            memset(nrf21_txBuf, 0, SPI_BUF_SIZE);
            memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
            break;
        }
        default: {
            assert_print("Invalid SPI channel!");
            break;
        }
    }
}

void set_spi_fast(beluga_spi_channel_t channel) {
    switch(channel) {
        case DW1000_SPI_CHANNEL: {
            dw1000SpiConfig = &spiConfigs[DW1000_FAST_FREQUENCY];
            memset(dw1000_rxBuf, 0, SPI_BUF_SIZE);
            memset(dw1000_txBuf, 0, SPI_BUF_SIZE);
            break;
        }
        case NRF21_SPI_CHANNEL: {
            nrfSpiConfig = &spiConfigs[NRF21540_FAST_FREQUENCY];
            memset(nrf21_txBuf, 0, SPI_BUF_SIZE);
            memset(nrf21_rxBuf, 0, SPI_BUF_SIZE);
            break;
        }
        default: {
            assert_print("Invalid SPI channel!");
            break;
        }
    }
}
