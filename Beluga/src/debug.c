/**
 * @file debug.c
 *
 * @brief Contains various functions that examine why the code is crashing and
 * logs the crash reasons
 *
 * @date 1/9/25
 * @author Tom Schmitz
 */

#include <debug.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/**
 * Logger for the debug module
 */
LOG_MODULE_REGISTER(debug_module, CONFIG_BELUGA_DEBUG_LOG_LEVEL);

#if defined(CONFIG_DEBUG_BELUGA_CLOCK)
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

/**
 * @brief Retrieves the clock manager for the specified clock
 * @param[in] clk_subsys The clock to retrieve the manager for
 * @param[out] clk_mgr The retrieved clock manager
 * @return 0 upon success
 * @return -EINVAL for invalid clock manager
 * @return -EFAULT if unable to retrieve clock manager
 */
static int get_clock_manager(enum clk_cntrl clk_subsys,
                             struct onoff_manager **clk_mgr) {
    switch (clk_subsys) {
    case HIGH_FREQ:
        *clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
        break;
    case LOW_FREQ:
        *clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);
        break;
    default:
        return -EINVAL;
    }

    if (*clk_mgr == NULL) {
        LOG_ERR("Unable to get the Clock manager");
        return -EFAULT;
    }

    return 0;
}

/**
 * @brief Initializes a clock subsystem and checks if it runs correctly
 * @param[in] clk_subsys The clock subsystem to initialize
 * @return `true` if subsystem initialized correctly
 * @return `false` if the subsystem failed to initialize
 */
bool clock_init(enum clk_cntrl clk_subsys) {
    int err;
    int res;
    int retries = 15;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    err = get_clock_manager(clk_subsys, &clk_mgr);
    if (err != 0) {
        return false;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return false;
    }

    do {
        WAIT_FOR(true, 10000, k_busy_wait(1000));
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (err == 0 && res != 0) {
            LOG_ERR("Clock could not be started: %d", res);
            return false;
        } else if (err != 0) {
            LOG_ERR("sys_notify_fetch_result(): %d", err);
        }
        retries--;
    } while ((err != 0) && (retries != 0));

    if (err == 0) {
        LOG_INF("Clock has started\n");
    }
    return err == 0;
}
#endif // defined(CONFIG_DEBUG_BELUGA_CLOCK)

#if defined(CONFIG_BELUGA_RESET_REASON)
#include <zephyr/drivers/hwinfo.h>

/**
 * @brief Retrieve the reason why the hardware reset and indicate it to the
 * logger. Additionally, clears the reset reason for the next run.
 */
void get_reset_cause(void) {
    uint32_t reason;
    hwinfo_get_reset_cause(&reason);
    hwinfo_clear_reset_cause();

    if (reason & RESET_PIN) {
        LOG_INF("External pin reset");
    }
    if (reason & RESET_SOFTWARE) {
        LOG_INF("Software reset\n");
    }
    if (reason & RESET_BROWNOUT) {
        LOG_INF("Brownout\n");
    }
    if (reason & RESET_POR) {
        LOG_INF("Power-on reset\n");
    }
    if (reason & RESET_WATCHDOG) {
        LOG_INF("Watchdog timer expiration\n");
    }
    if (reason & RESET_DEBUG) {
        LOG_INF("Debug event\n");
    }
    if (reason & RESET_SECURITY) {
        LOG_INF("Security violation\n");
    }
    if (reason & RESET_LOW_POWER_WAKE) {
        LOG_INF("Waking up from low power mode\n");
    }
    if (reason & RESET_CPU_LOCKUP) {
        LOG_INF("CPU lock-up detected\n");
    }
    if (reason & RESET_PARITY) {
        LOG_INF("Parity error\n");
    }
    if (reason & RESET_PLL) {
        LOG_INF("PLL error\n");
    }
    if (reason & RESET_CLOCK) {
        LOG_INF("Clock error\n");
    }
    if (reason & RESET_HARDWARE) {
        LOG_INF("Hardware reset\n");
    }
    if (reason & RESET_USER) {
        LOG_INF("User reset\n");
    }
    if (reason & RESET_TEMPERATURE) {
        LOG_INF("Temperature reset\n");
    }
}
#endif // defined(CONFIG_BELUGA_RESET_REASON)
