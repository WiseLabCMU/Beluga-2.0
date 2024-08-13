//
// Created by tom on 8/13/24.
//

#include <assert.h>
#include <ble_app.h>
#include <nrf21540.h>
#include <settings.h>
#include <spi.h>
#include <stdbool.h>
#include <stdint.h>
#include <voltage_regulator.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_BELUGA_RANGE_EXTENSION)

#define NRF21_MAXBUF 2
#define NRF21_READ (0b10 << 6)
#define NRF21_WRITE (0b11 << 6)

#define NRF21540_NODE DT_NODELABEL(nrf_radio_fem)

#if DT_NODE_HAS_PROP(NRF21540_NODE, ant_sel_gpios)
#define ANTENNA_GPIO 1
#else
#define ANTENNA_GPIO 0
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, mode_gpios)
#error "nrf21540 must have mode gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, tx_en_gpios)
#error "nrf21540 must have TX gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, rx_en_gpios)
#error "nrf21540 must have RX gpio connected"
#endif

#if !DT_NODE_HAS_PROP(NRF21540_NODE, pdn_gpios)
#error "nrf21540 must have PDN gpio connected"
#endif

static struct nrf21540_gpios {
#if ANTENNA_GPIO
    const struct gpio_dt_spec ant_sel;
#endif
    const struct gpio_dt_spec power_mode;
    const struct gpio_dt_spec tx_gpio;
    const struct gpio_dt_spec rx_gpio;
    const struct gpio_dt_spec pdn_gpio;
} nrf21_gpios = {
#if ANTENNA_GPIO
    .ant_sel = GPIO_DT_SPEC_GET(NRF21540_NODE, ant_sel_gpios),
#endif
    .power_mode = GPIO_DT_SPEC_GET(NRF21540_NODE, mode_gpios),
    .tx_gpio = GPIO_DT_SPEC_GET(NRF21540_NODE, tx_en_gpios),
    .rx_gpio = GPIO_DT_SPEC_GET(NRF21540_NODE, rx_en_gpios),
    .pdn_gpio = GPIO_DT_SPEC_GET(NRF21540_NODE, pdn_gpios)};

static bool areBoundsSet(void) {
    int32_t poutA = retrieveStaticSetting(BELUGA_FEM_POUTA);
    int32_t poutB = retrieveStaticSetting(BELUGA_FEM_POUTB);

    return (poutA == CONFIG_BELUGA_POUTA) && (poutB == CONFIG_BELUGA_POUTB);
}

static bool configure_nrf21_gpios(void) {
    int err;

#if ANTENNA_GPIO
    if (!device_is_ready(nrf21_gpios.ant_sel.port)) {
        printk("Antenna device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.ant_sel, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Antenna GPIO configure (%d)\n", err);
        return false;
    }
#endif

    if (!device_is_ready(nrf21_gpios.power_mode.port)) {
        printk("Power mode device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.power_mode, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("Power mode GPIO configure (%d)\n", err);
        return false;
    }

    if (!device_is_ready(nrf21_gpios.tx_gpio.port)) {
        printk("TX GPIO device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.tx_gpio, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("TX GPIO configure (%d)\n", err);
        return false;
    }

    if (!device_is_ready(nrf21_gpios.rx_gpio.port)) {
        printk("RX GPIO device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.rx_gpio, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("RX GPIO configure (%d)\n", err);
        return false;
    }

    if (!device_is_ready(nrf21_gpios.pdn_gpio.port)) {
        printk("PDN GPIO device was not ready\n");
        return false;
    }

    err = gpio_pin_configure_dt(&nrf21_gpios.pdn_gpio, GPIO_OUTPUT_INACTIVE);

    if (err) {
        printk("PDN GPIO configure (%d)\n", err);
        return false;
    }

    return true;
}

static void power_on_nrf21(void) {
    int err;

    err = gpio_pin_set_dt(&nrf21_gpios.pdn_gpio, 1);

    if (err) {
        printk("Unable to power on nrf21 (%d)", err);
    }
}

static void power_down_nrf21(void) {
    int err;

    err = gpio_pin_set_dt(&nrf21_gpios.pdn_gpio, 0);

    if (err) {
        printk("Unable to power down nrf21 (%d)", err);
    }
}

static void enable_uicr_mode(void) {
    uint8_t command[NRF21_MAXBUF] = {
            0x1 | NRF21_WRITE, 0x4
    };

    write_spi(NRF21_SPI_CHANNEL, command, NRF21_MAXBUF);
}

static void enter_program_mode(void) {
    uint8_t command[NRF21_MAXBUF] = {
            0x1 | NRF21_WRITE, 0xF0
    };

    write_spi(NRF21_SPI_CHANNEL, command, NRF21_MAXBUF);
}

static void write_poutb(void) {
    uint8_t command[NRF21_MAXBUF] = {
            0x3 | NRF21_WRITE, (1 << 5) | CONFIG_BELUGA_POUTB
    };

    write_spi(NRF21_SPI_CHANNEL, command, NRF21_MAXBUF);
}

static void write_pouta(void) {
    uint8_t command[NRF21_MAXBUF] = {
            0x2 | NRF21_WRITE, (3 << 5) | CONFIG_BELUGA_POUTA
    };

    write_spi(NRF21_SPI_CHANNEL, command, NRF21_MAXBUF);
}

static void read_nrf21(uint8_t *pouta, uint8_t *poutb) {
    uint8_t command[NRF21_MAXBUF];
    uint8_t read_data[NRF21_MAXBUF];
    enable_uicr_mode();

    command[0] = 0x3 | NRF21_READ;

    read_spi(NRF21_SPI_CHANNEL, command, read_data, NRF21_MAXBUF);
    *poutb = read_data[1];

    command[0] = 0x2 | NRF21_READ;
    read_spi(NRF21_SPI_CHANNEL, command, read_data, NRF21_MAXBUF);
    *pouta = read_data[1];
}

bool init_nrf21540(void) {
    int err;

    disable_bluetooth();

    if (!configure_nrf21_gpios()) {
        return false;
    }

    if (!areBoundsSet()) {
        uint8_t pouta, poutb;
        update_voltage_level(VR_3V5);
        assert(get_current_voltage() == VR_3V5);
        power_on_nrf21();
        set_spi_slow(NRF21_SPI_CHANNEL);
        enable_uicr_mode();
        enter_program_mode();
        write_poutb();
        write_pouta();
        k_sleep(K_MSEC(2));
        power_down_nrf21();
        k_sleep(K_MSEC(2));
        update_voltage_level(VR_3V3);
        power_on_nrf21();
        read_nrf21(&pouta, &poutb);
        power_down_nrf21();
        if (pouta == CONFIG_BELUGA_POUTA && poutb == CONFIG_BELUGA_POUTB) {
            updateStaticSetting(BELUGA_FEM_POUTA, CONFIG_BELUGA_POUTA);
            updateStaticSetting(BELUGA_FEM_POUTB, CONFIG_BELUGA_POUTB);
        } else {
            printk("Configuration mismatches\n");
            return false;
        }
    }

    printk("Front end module configured\n");

    return true;
}

#endif
