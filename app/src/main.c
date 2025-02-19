#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>

#include <zephyr/drivers/adc/ads1x4s0x_pio.h>

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
const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
            DT_SPEC_AND_COMMA)
};

struct adc_channel_cfg channel_cfgs[ARRAY_SIZE(adc_channels)];

#define ADS1X4S0X_RESOLUTION 24
#define REPEAT_CNT 5

uint32_t sample_buf1[ARRAY_SIZE(adc_channels)*REPEAT_CNT];
uint32_t sample_buf2[ARRAY_SIZE(adc_channels)*REPEAT_CNT];

struct interleaving_channel_buf {
    uint32_t channel_sample[ARRAY_SIZE(adc_channels)];
};

const struct device *dev = adc_channels[0].dev;

struct ads1x4s0x_pio_bulk_read_config br_cfg = {
    .chan_cfgs = channel_cfgs,
    .chan_count = ARRAY_SIZE(adc_channels),
    .sample_count = 1,
    .repeat_count = 5,
    .sample_buf_1 = (uint8_t *)sample_buf1,
    .sample_buf_2 = (uint8_t *)sample_buf2,
};

static void print_buffer(const struct interleaving_channel_buf *buf, size_t len, bool verbose) {
    int32_t sample;
    uint8_t crc;
    float my_fval_mv;

    for (size_t j = 0; j < len; ++j) {
        if (!verbose) {
            LOG_INF("#%2d", j);
        }

        for (int i = 0; i < ARRAY_SIZE(adc_channels); ++i) {
            crc = (buf[j].channel_sample[i] >> ADS1X4S0X_RESOLUTION) & 0xFF;
            sample = (int32_t)sys_get_be32((uint8_t *)&(buf[j].channel_sample[i])) >> (32 - ADS1X4S0X_RESOLUTION);

            if (verbose) {
                my_fval_mv = 2500.0f * sample / (1 << 23);
                LOG_INF("[%2d] %d: Read Raw 0x%08x (crc %02x) = 0x%06x = %d = %.3f mV",
                        j, i, buf[j].channel_sample[i], crc, sample, sample, my_fval_mv);
            } else {
                LOG_INF("%d", sample);
            }
        }
    }
}

int main(void)
{
    uint8_t buf_ind;
    uint32_t *buf;

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

    for (uint8_t i = 0; i < ARRAY_SIZE(adc_channels); i++) {
        memcpy(&channel_cfgs[i], &adc_channels[i].channel_cfg, sizeof(struct adc_channel_cfg));
    }

    /*ads1x4s0x_pio_bulk_read_setup(dev, &br_cfg);*/
    /*ads1x4s0x_pio_bulk_read_start(dev);*/

    while (ads1x4s0x_pio_bulk_read_get_samples_blocking(dev, &buf_ind)) {
        LOG_INF("DMA returned, buffer %d is ready", buf_ind);
        buf = buf_ind == 1 ? sample_buf1 : sample_buf2;
        gpio_pin_set_dt(&led, buf_ind % 2);
        print_buffer((struct interleaving_channel_buf *)buf, REPEAT_CNT, true);
    }

    return 0;
}

static int cmd_acq_start(const struct shell *sh, size_t argc,
			      char **argv, uint32_t period)
{
	ARG_UNUSED(argv);

    ads1x4s0x_pio_bulk_read_setup(dev, &br_cfg);
    ads1x4s0x_pio_bulk_read_start(dev);

	return 0;
}


static int cmd_acq_stop(const struct shell *sh, size_t argc,
			     char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

    ads1x4s0x_pio_bulk_read_stop(dev);

    LOG_INF("Acquisition stopped.");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_acq_cmd,
	SHELL_CMD_ARG(start, NULL, "Start acquisition", cmd_acq_start, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop acquisition.", cmd_acq_stop, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(acq, &sub_acq_cmd, "Acquisition control", NULL);
