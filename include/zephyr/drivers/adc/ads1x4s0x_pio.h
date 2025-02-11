/*
 * Copyright (c) 2023 SILA Embedded Solutions GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_ADS1X4S0X_PIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_ADS1X4S0X_PIO_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

int ads1x4s0x_pio_gpio_set_output(const struct device *dev, uint8_t pin, bool initial_value);

int ads1x4s0x_pio_gpio_set_input(const struct device *dev, uint8_t pin);

int ads1x4s0x_pio_gpio_deconfigure(const struct device *dev, uint8_t pin);

int ads1x4s0x_pio_gpio_set_pin_value(const struct device *dev, uint8_t pin,
				bool value);

int ads1x4s0x_pio_gpio_get_pin_value(const struct device *dev, uint8_t pin,
				bool *value);

int ads1x4s0x_pio_gpio_port_get_raw(const struct device *dev,
			       gpio_port_value_t *value);

int ads1x4s0x_pio_gpio_port_set_masked_raw(const struct device *dev,
				      gpio_port_pins_t mask,
				      gpio_port_value_t value);

int ads1x4s0x_pio_gpio_port_toggle_bits(const struct device *dev,
				   gpio_port_pins_t pins);

/* Bulk ADC read functions */

struct ads1x4s0x_pio_bulk_read_config {
    const struct adc_channel_cfg *chan_cfgs;
    uint8_t chan_count;

    size_t sample_count;
    size_t repeat_count;
    uint8_t *sample_buf_1;
    uint8_t *sample_buf_2;
};

int ads1x4s0x_pio_bulk_read_setup(const struct device *dev,
        struct ads1x4s0x_pio_bulk_read_config *pio_cfg);

int ads1x4s0x_pio_bulk_read_start(const struct device *dev);

int ads1x4s0x_pio_bulk_read_stop(const struct device *dev);

size_t ads1x4s0x_pio_bulk_read_get_samples_blocking(
        const struct device *dev, uint8_t *buf_ind);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_ADS1X4S0X_PIO_H_ */
