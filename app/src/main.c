/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_dt_sample, LOG_LEVEL_DBG);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct gpio_dt_spec ads_gpio0 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), ads_gpios);

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

int main(void)
{
	int err;
	uint32_t count = 0;
	uint32_t buf;
    char logbuf[256] = {0};
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    if (!gpio_is_ready_dt(&ads_gpio0)) {
        return 0;
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
        return 0;
    }

    if (gpio_pin_configure_dt(&ads_gpio0, GPIO_OUTPUT_ACTIVE) < 0) {
        return 0;
    }

    gpio_pin_set_dt(&led, 1);
    gpio_pin_set_dt(&ads_gpio0, 1);

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}
	}

    struct adc_channel_cfg channel_cfg;

#ifndef CONFIG_COVERAGE
	while (1) {
#else
	for (int k = 0; k < 10; k++) {
#endif
		LOG_INF("ADC reading[%u]:", count++);
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int64_t val_mv;
            memcpy(&channel_cfg, &adc_channels[i].channel_cfg, sizeof(struct adc_channel_cfg));
            channel_cfg.channel_id = 0;
            channel_cfg.differential = true;

            //err = adc_channel_setup_dt(&adc_channel);
            err = adc_channel_setup(adc_channels[i].dev, &channel_cfg);
            if (err < 0) {
                LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
                return 0;
            }

            memset(logbuf, 0, sizeof(logbuf));

			sprintf(logbuf + strlen(logbuf), "- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id);

			//(void)adc_sequence_init_dt(&adc_channels[i], &sequence);
            sequence.channels = BIT(0);
            sequence.resolution = adc_channels[i].resolution;
            sequence.oversampling = adc_channels[i].oversampling;

			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				LOG_ERR("Could not read (%d)\n", err);
				continue;
			}

            val_mv = (int32_t)buf;
			sprintf(logbuf + strlen(logbuf), "%08X", val_mv);
            /* manual conversion */
            float my_fval_mv = 2500.0 * val_mv / (1 << 23);

            int64_t my_val_mv = (int64_t)val_mv * 2500 >> 23;

			//err = adc_raw_to_microvolts_dt(&adc_channels[i], &val_mv);

			///* conversion to mV may not be supported, skip if not */
			//if (err < 0) {
			//	sprintf(logbuf + strlen(logbuf), " (value in mV not available)");
			//} else {
			//	sprintf(logbuf + strlen(logbuf), " = %"PRId64" uV", val_mv);
			//}
            //LOG_INF("%s", logbuf);

            LOG_INF("Read %06x, My conversion: %lld mV, %.3f mV", val_mv, my_val_mv, my_fval_mv);
		}

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
