//
// Created by tom on 6/26/24.
//

#ifndef BELUGA_SPI_H
#define BELUGA_SPI_H

#include <stdint.h>
#include <zephyr/types.h>

typedef enum {
    DW1000_SPI_CHANNEL,
    NRF21_SPI_CHANNEL,
    INVALID_SPI_CHANNEL
} beluga_spi_channel_t;

int init_spi1(void);
void set_spi_slow(beluga_spi_channel_t channel);
void set_spi_fast(beluga_spi_channel_t channel);
int write_spi(beluga_spi_channel_t channel, const uint8_t *buffer,
              size_t bufLength);
int read_spi(beluga_spi_channel_t channel, const uint8_t *writeBuffer,
             uint8_t *readBuf, size_t bufLength);

#endif // BELUGA_SPI_H
