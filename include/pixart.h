#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_SETTINGS_LENGTH 16
#define SETTINGS_PREFIX "cpi_cycle"

    /* device data structure */
    struct pixart_data
    {
    const struct device          *dev;
    bool                         sw_smart_flag; // for paw3395 smart algorithm

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;
    int                          init_retry_count; // current retry count
    int                          init_retry_attempts; // remaining retry attempts

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init

    bool                         data_ready;
    uint8_t                      data_index;

    int64_t last_smp_time, last_rpt_time;
    int64_t dx, dy;

    int32_t report_interval_ms;

#if defined(CONFIG_PAW3395_RATE_CYCLE_GPIO)
    struct gpio_callback rate_cycle_gpio_cb;
    struct k_work_delayable rate_cycle_work;
    uint8_t rate_cycle_idx;
#endif
};

// device config data structure
struct pixart_config {
    uint8_t id;
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    uint8_t init_retry_count;
    uint16_t init_retry_interval;
    uint8_t power_mode;

#if defined(CONFIG_PAW3395_OUTPUT_RATE_NOTIFY)
    int32_t usb_rate_ms; /* report_interval_ms when USB is active */
    int32_t ble_rate_ms; /* report_interval_ms when BLE is active */
#endif

#if defined(CONFIG_PAW3395_RATE_CYCLE_GPIO)
    struct gpio_dt_spec rate_cycle_gpio;
    const int32_t *rate_cycle_rates_ms;
    size_t rate_cycle_rates_count;
#endif
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
