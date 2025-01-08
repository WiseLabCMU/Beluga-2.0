/**
 * @file spi.h
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

#ifndef BELUGA_SPI_H
#define BELUGA_SPI_H

#include <stdint.h>
#include <zephyr/types.h>

/**
 * @brief Initializes SPI 1 for communication with the DW1000
 * @return 0 upon success
 * @return -ENODEV if SPI device is not ready
 */
int init_spi1(void);

/**
 * @brief Selects the low speed configuration for the SPI
 */
void set_spi_slow(void);

/**
 * @brief Selects the high speed configuration for the SPI
 */
void set_spi_fast(void);

/**
 * @brief Writes the bytes stored in buffer over SPI
 * @param[in] buffer The bytes to write over SPI
 * @param[in] bufLength The number of bytes to write
 * @return 0 upon success
 * @return -EINVAL if buffer is NULL
 * @return negative error code otherwise
 */
int write_spi(const uint8_t *buffer, size_t bufLength);

/**
 * @brief Reads bytes over SPI given write date
 * @param[in] writeBuffer The data to write over SPI
 * @param[out] readBuf The data to read from SPI
 * @param[in] bufLength The number of bytes to read/write over SPI
 * @return 0 upon success
 * @return -EINVAL if invalid parameter
 * @return negative error code otherwise
 */
int read_spi(const uint8_t *writeBuffer, uint8_t *readBuf, size_t bufLength);

/**
 * @brief Shuts down the SPI device
 */
void shutdown_spi(void);

/**
 * @brief Toggles the chip select line for a specified amount of time.
 * @param[in] us The number of microseconds to toggle the chip select for
 */
void toggle_cs_line(int32_t us);

#endif // BELUGA_SPI_H
