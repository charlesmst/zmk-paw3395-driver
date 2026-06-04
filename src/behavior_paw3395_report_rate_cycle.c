/*
 * Copyright (c) 2025 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_paw3395_report_rate_cycle

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <paw3395.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(paw3395_rrc, CONFIG_PAW3395_LOG_LEVEL);

struct paw3395_rrc_config {
    const struct device *sensor;
    const uint32_t *rates_ms;
    size_t num_rates;
};

struct paw3395_rrc_data {
    uint8_t current_idx;
};

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct paw3395_rrc_data *data = dev->data;
    const struct paw3395_rrc_config *cfg = dev->config;

    data->current_idx = (data->current_idx + 1) % cfg->num_rates;
    uint32_t interval = cfg->rates_ms[data->current_idx];

    LOG_INF("cycling report rate to %u ms (%u Hz)", interval, interval ? 1000 / interval : 0);

    struct sensor_value val = {.val1 = (int32_t)interval};
    int err = sensor_attr_set(cfg->sensor, SENSOR_CHAN_ALL,
                              (enum sensor_attribute)PAW3395_ATTR_REPORT_INTERVAL_MS, &val);
    if (err) {
        LOG_ERR("sensor_attr_set failed: %d", err);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api paw3395_rrc_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define PAW3395_RRC_INST(n)                                                                        \
    static const uint32_t paw3395_rrc_rates_##n[] = DT_PROP(DT_DRV_INST(n), rates_ms);           \
    static struct paw3395_rrc_data paw3395_rrc_data_##n = {.current_idx = 0};                     \
    static const struct paw3395_rrc_config paw3395_rrc_cfg_##n = {                                \
        .sensor = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), sensor)),                              \
        .rates_ms = paw3395_rrc_rates_##n,                                                         \
        .num_rates = ARRAY_SIZE(paw3395_rrc_rates_##n),                                            \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &paw3395_rrc_data_##n, &paw3395_rrc_cfg_##n,           \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &paw3395_rrc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAW3395_RRC_INST)
