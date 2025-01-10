/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port_platform.h"
#include "deca_device_api.h"
#include <spi.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_ENABLE_BELUGA_UWB)

#define DW1000_MAXBUF 128

/****************************************************************************
 *
 *                              APP global variables
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                  Port private variables and function prototypes
 *
 ****************************************************************************/

#if DT_NODE_HAS_PROP(DT_NODELABEL(dw1000_spi), reset_gpios)
static const struct gpio_dt_spec dw1000_reset_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dw1000_spi), reset_gpios);
#else
#error "Unable to get DW1000 reset GPIO"
#endif

#if DT_NODE_HAS_PROP(DT_NODELABEL(dw1000_spi), int_gpios)
static const struct gpio_dt_spec dw1000_irq_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dw1000_spi), int_gpios);
#else
#error "Unable to get DW1000 IRQ GPIO"
#endif

/****************************************************************************
 *
 *                              Time section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                              END OF Time section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                              Configuration section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                          End of configuration section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                          DW1000 port section
 *
 ****************************************************************************/
// YB : STM HAL based function have to be updated using NRF drivers

/**
 * @fn      setup_DW1000RSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable) {
    ARG_UNUSED(enable);

    gpio_pin_configure_dt(&dw1000_irq_pin, GPIO_DISCONNECTED);
}

/**
 * @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 */
void port_wakeup_dw1000(void) {}

/**
 * @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup
 * time
 */
void port_wakeup_dw1000_fast(void) {}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be
 * found, or returns 0
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer,
                uint32 readlength, uint8 *readBuffer) {

    uint8 txBuf[DW1000_MAXBUF];
    uint8 rxBuf[DW1000_MAXBUF];

    if ((headerLength + readlength) > DW1000_MAXBUF) {
        // Buffer overflow prevention
        return -1;
    }

    memset(txBuf, 0, headerLength + readlength);
    memcpy(txBuf, headerBuffer, headerLength);

    // TODO: Disable UW IRQ
    int err = read_spi(txBuf, rxBuf, readlength + headerLength);
    // TODO: Restore UW IRQ

    if (err != 0) {
        printk("SPI read returned an error (err: %d)\n", err);
        return 1;
    }

    memcpy(readBuffer, rxBuf + headerLength, readlength);

    return 0;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer,
               uint32 bodylength, const uint8 *bodyBuffer) {

    uint8 txBuf[DW1000_MAXBUF];

    if ((headerLength + bodylength) > DW1000_MAXBUF) {
        // Buffer overflow prevention
        return -1;
    }

    memcpy(txBuf, headerBuffer, headerLength);
    memcpy(txBuf + headerLength, bodyBuffer, bodylength);

    // TODO: Disable UW IRQ
    int err = write_spi(txBuf, headerLength + bodylength);
    // TODO: Restore UW IRQ

    if (err != 0) {
        printk("SPI write returned an error (err: %d)\n", err);
        return 1;
    }

    return 0;
}

//------------------------------other---------------------------

/**
 * @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the
 * digital part of DW1000 by driving this pin low. Note, the DW_RESET pin should
 * not be driven high externally.
 */
void reset_DW1000(void) {

    gpio_pin_configure_dt(&dw1000_reset_pin, GPIO_OUTPUT);
    gpio_pin_set_dt(&dw1000_reset_pin, 0);
    k_msleep(2);
    gpio_pin_configure_dt(&dw1000_reset_pin, GPIO_DISCONNECTED);
    k_msleep(2);
}

/**
 * @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          n
 */
void port_set_dw1000_slowrate(void) {
    set_spi_slow();
    k_msleep(2);
}

/**
 * @fn      port_set_dw1000_fastrate
 * @brief   set 8MHz
 *
 */
void port_set_dw1000_fastrate(void) {
    set_spi_fast();
    k_msleep(2);
}

void deca_sleep(unsigned int time_ms) { k_msleep(time_ms); }

// currently do nothing
decaIrqStatus_t decamutexon(void) { return 0; }

// currently do nothing
void decamutexoff(decaIrqStatus_t s) {}

/****************************************************************************
 *
 *                          End APP port section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                              IRQ section
 *
 ****************************************************************************/

/****************************************************************************
 *
 *                              END OF IRQ section
 *
 ****************************************************************************/

/****************************************************************************
 *
 ****************************************************************************/

#endif // defined(CONFIG_ENABLE_BELUGA_UWB)
