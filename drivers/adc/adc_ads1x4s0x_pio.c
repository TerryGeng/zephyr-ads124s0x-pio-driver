/*
 * Copyright (c) 2023 SILA Embedded Solutions GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/ads1x4s0x_pio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/adc/ads1x4s0x_adc_pio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#define ADS1X4S0X_HAS_16_BIT_DEV                                                                   \
    (DT_HAS_COMPAT_STATUS_OKAY(ti_ads114s06_pio) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads114s08_pio))
#define ADS1X4S0X_HAS_24_BIT_DEV                                                                   \
    (DT_HAS_COMPAT_STATUS_OKAY(ti_ads124s06_pio) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads124s08_pio))

#define ADC_CONTEXT_USES_KERNEL_TIMER 1
#define ADC_CONTEXT_WAIT_FOR_COMPLETION_TIMEOUT                                                    \
    K_MSEC(CONFIG_ADC_ADS1X4S0X_PIO_WAIT_FOR_COMPLETION_TIMEOUT_MS)
#include "../../zephyr/drivers/adc/adc_context.h"

LOG_MODULE_REGISTER(ads1x4s0x_pio, CONFIG_ADC_LOG_LEVEL);

struct pio_spi_odm_raw_program {
    uint8_t *raw_inst;
    size_t raw_inst_max_len;
    size_t tx_cnt;
    size_t rx_cnt;
    size_t iptr;
    bool half;
};

struct pio_spi_odm_dma_chan_config {
    uint8_t tx;
    uint8_t rx1;
    uint8_t rx2;
    uint8_t tx_ctrl1;
    uint8_t tx_ctrl2;
};

#define SPI_ODM_INST_BUF_LEN 200

struct ads1x4s0x_pio_bulk_read_data {
    uint8_t sm_insts[SPI_ODM_INST_BUF_LEN];
    struct pio_spi_odm_raw_program sm_raw_pgm;
    struct ads1x4s0x_pio_bulk_read_config *current_pio_cfg;
    struct k_sem bulk_data_ready_sem;
    uint8_t data_ready_buf_ind;
    size_t rx_buf_len;
};

struct ads1x4s0x_pio_config {
#if CONFIG_ADC_ASYNC
    k_thread_stack_t *stack;
#endif
    const struct device *piodev;
    const struct device *dma_dev;
    const struct pinctrl_dev_config *pin_cfg;
    const struct pio_spi_odm_dma_chan_config dma_chan_cfg;
    struct gpio_dt_spec gpio_clk;
    struct gpio_dt_spec gpio_mosi;
    struct gpio_dt_spec gpio_miso;
    struct gpio_dt_spec gpio_cs;
    const struct device *clk_dev;
    clock_control_subsys_t clk_id;
    const uint32_t spi_freq;
    const struct gpio_dt_spec gpio_reset;
    const struct gpio_dt_spec gpio_data_ready;
    const struct gpio_dt_spec gpio_start_sync;
    int idac_current;
    uint8_t vbias_level;
    uint8_t channels;
    uint8_t resolution;
};

struct ads1x4s0x_pio_data {
    struct adc_context ctx;
#if CONFIG_ADC_ASYNC
    struct k_thread thread;
#endif /* CONFIG_ADC_ASYNC */
    PIO pio;
    size_t pio_sm;
    bool sm_configured;
    bool bulk_read_sm_running;

    struct gpio_callback callback_data_ready;
    struct k_sem data_ready_signal;
    struct k_sem acquire_signal;
    void *buffer;
    void *buffer_ptr;
#if CONFIG_ADC_ADS1X4S0X_PIO_GPIO
    struct k_mutex gpio_lock;
    uint8_t gpio_enabled;   /* one bit per GPIO, 1 = enabled */
    uint8_t gpio_direction; /* one bit per GPIO, 1 = input */
    uint8_t gpio_value;     /* one bit per GPIO, 1 = high */
#endif                          /* CONFIG_ADC_ADS1X4S0X_GPIO */
    struct ads1x4s0x_pio_bulk_read_data bulk_read_data;
};

/*############################################################*/
/*                        SPI via PIO                         */
/*############################################################*/

#define SPI_ODM_MODE_0_1_WRAP_TARGET      2
#define SPI_ODM_MODE_0_1_WRAP             12
#define SPI_ODM_MODE_0_1_CYCLES           12
#define SPI_ODM_MODE_0_1_OFFSET_WAIT_INST 1
#define SPI_ODM_MODE_0_1_ENTRY_POINT      2

/* ------------------------- */
/* SPI on-demand pio program */
/* ------------------------- */

RPI_PICO_PIO_DEFINE_PROGRAM(spi_odm_mode_0_1, SPI_ODM_MODE_0_1_WRAP_TARGET, SPI_ODM_MODE_0_1_WRAP,
        0x6021, /*  0: out    x, 1            side 0 */
        0x2006, /*  1: wait   0 gpio, 6       side 0 */
        /*     .wrap_target */
        0x6021, /*  2: out    x, 1            side 0 */
        0x0026, /*  3: jmp    !x, 6           side 0 */
        0x6023, /*  4: out    x, 3            side 0 */
        0x0002, /*  5: jmp    2               side 0 */
        0x6021, /*  6: out    x, 1            side 0 */
        0x6041, /*  7: out    y, 1            side 0 */
        0x0020, /*  8: jmp    !x, 0           side 0 */
        0x7401, /*  9: out    pins, 1         side 1 [4] */
        0x106c, /* 10: jmp    !y, 12          side 1 */
        0x0002, /* 11: jmp    2               side 0 */
        0x4001, /* 12: in     pins, 1         side 0 */
        /*     .wrap */
        );

static float spi_pico_pio_clock_divisor(uint32_t clock_freq, uint8_t cycles, uint32_t spi_frequency)
{
    return (float)clock_freq / (float)(cycles * spi_frequency);
}

static uint32_t spi_pico_pio_maximum_clock_frequency(uint32_t clock_freq, uint8_t cycles)
{
    return clock_freq / cycles;
}

static uint32_t spi_pico_pio_minimum_clock_frequency(uint32_t clock_freq, uint8_t cycles)
{
    return clock_freq / (cycles * 65536);
}

static inline void spi_pico_pio_sm_put8(PIO pio, uint sm, uint8_t data)
{
    /* Do 8 bit accesses on FIFO, so that write data is byte-replicated. This */
    /* gets us the left-justification for free (for MSB-first shift-out) */
    io_rw_8 *txfifo = (io_rw_8 *)&pio->txf[sm];

    *txfifo = data;
}

static inline uint8_t spi_pico_pio_sm_get8(PIO pio, uint sm)
{
    /* Do 8 bit accesses on FIFO, so that write data is byte-replicated. This */
    /* gets us the left-justification for free (for MSB-first shift-out) */
    io_rw_8 *rxfifo = (io_rw_8 *)&pio->rxf[sm];

    return *rxfifo;
}

static int spi_pico_pio_configure(const struct ads1x4s0x_pio_config *dev_cfg,
        struct ads1x4s0x_pio_data *data)
{
    const struct gpio_dt_spec *miso;
    const struct gpio_dt_spec *mosi;
    const struct gpio_dt_spec *clk;
    const struct gpio_dt_spec *drdy;
    pio_sm_config sm_config;
    uint32_t offset;
    uint32_t wrap_target;
    uint32_t wrap;
    uint32_t entry_point;
    uint32_t clock_freq;
    const pio_program_t *program;
    int rc;

    if (data->sm_configured) {
        return 0;
    }

    rc = clock_control_on(dev_cfg->clk_dev, dev_cfg->clk_id);
    if (rc < 0) {
        LOG_ERR("Failed to enable the clock");
        return rc;
    }

    rc = clock_control_get_rate(dev_cfg->clk_dev, dev_cfg->clk_id, &clock_freq);
    if (rc < 0) {
        LOG_ERR("Failed to get clock frequency");
        return rc;
    }

    if ((dev_cfg->spi_freq < spi_pico_pio_minimum_clock_frequency(clock_freq, SPI_ODM_MODE_0_1_CYCLES)) ||
            (dev_cfg->spi_freq > spi_pico_pio_maximum_clock_frequency(clock_freq, SPI_ODM_MODE_0_1_CYCLES))) {
        LOG_ERR("clock-frequency out of range");
        return -EINVAL;
    }

    float clock_div = spi_pico_pio_clock_divisor(clock_freq, SPI_ODM_MODE_0_1_CYCLES, dev_cfg->spi_freq);

    mosi = &dev_cfg->gpio_mosi;
    miso = &dev_cfg->gpio_miso;
    clk = &dev_cfg->gpio_clk;
    drdy = &dev_cfg->gpio_data_ready;
    data->pio = pio_rpi_pico_get_pio(dev_cfg->piodev);
    rc = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &data->pio_sm);
    if (rc < 0) {
        return rc;
    }

    program = RPI_PICO_PIO_GET_PROGRAM(spi_odm_mode_0_1);
    wrap_target = RPI_PICO_PIO_GET_WRAP_TARGET(spi_odm_mode_0_1);
    wrap = RPI_PICO_PIO_GET_WRAP(spi_odm_mode_0_1);
    entry_point = SPI_ODM_MODE_0_1_ENTRY_POINT;

    if (!pio_can_add_program(data->pio, program)) {
        return -EBUSY;
    }

    offset = pio_add_program(data->pio, program);
    data->pio->instr_mem[offset + SPI_ODM_MODE_0_1_OFFSET_WAIT_INST] = \
                                                                       0x2000 | drdy->pin;   /* wait for data ready to be set high */

    sm_config = pio_get_default_sm_config();

    sm_config_set_clkdiv(&sm_config, clock_div);
    sm_config_set_in_pins(&sm_config, miso->pin);
    sm_config_set_in_shift(&sm_config, false, true, 8);
    sm_config_set_out_pins(&sm_config, mosi->pin, 1);
    sm_config_set_out_shift(&sm_config, false, true, 8);
    sm_config_set_sideset_pins(&sm_config, clk->pin);
    sm_config_set_sideset(&sm_config, 1, false, false);
    sm_config_set_wrap(&sm_config, offset + wrap_target, offset + wrap);

    pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm, miso->pin, 1, false);
    pio_sm_set_pindirs_with_mask(data->pio, data->pio_sm, (BIT(clk->pin) | BIT(mosi->pin)),
            (BIT(clk->pin) | BIT(mosi->pin)));
    pio_sm_set_pins_with_mask(data->pio, data->pio_sm, 0,
            BIT(clk->pin) | BIT(mosi->pin));
    pio_gpio_init(data->pio, mosi->pin);
    pio_gpio_init(data->pio, miso->pin);
    pio_gpio_init(data->pio, clk->pin);
    pio_gpio_init(data->pio, drdy->pin);

    pio_sm_init(data->pio, data->pio_sm, offset + entry_point, &sm_config);
    pio_sm_set_enabled(data->pio, data->pio_sm, true);

    data->sm_configured = true;

    return 0;
}

static int spi_pico_pio_transceive(const struct device *dev,
        const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
{
    const struct ads1x4s0x_pio_config *dev_cfg = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;
    uint8_t inst = 0;
    uint8_t txrx;
    bool half = false;
    size_t tx_len = len;
    size_t rx_len = len;
    int rc = 0;

    rc = spi_pico_pio_configure(dev_cfg, data);
    if (rc < 0) {
        return rc;
    }

    gpio_pin_set_dt(&dev_cfg->gpio_cs, GPIO_OUTPUT_ACTIVE);

    pio_sm_clear_fifos(data->pio, data->pio_sm);

    while (tx_len || rx_len) {
        if (tx_len) {
            if (txbuf) {
                txrx = *txbuf;
            }

            for (int j = 8; j > 0; j--) {
                while (pio_sm_is_tx_fifo_full(data->pio, data->pio_sm)) {
                    ;
                }

                inst |= (1 << 2) | ((txrx >> (j-1)) & 1);

                if (!half) {
                    inst <<= 4;
                    half = true;
                } else {
                    half = false;
                    spi_pico_pio_sm_put8(data->pio, data->pio_sm, inst);
                    inst = 0;
                }
            }
            tx_len--;
            txbuf++;
        }

        if (rx_len) {
            if (!pio_sm_is_rx_fifo_empty(data->pio, data->pio_sm)) {
                txrx = spi_pico_pio_sm_get8(data->pio, data->pio_sm);
                if (rxbuf) {
                    *rxbuf++ = txrx;
                }
                rx_len--;
            }
        }
    }

    gpio_pin_set_dt(&dev_cfg->gpio_cs, GPIO_OUTPUT_INACTIVE);

    return rc;
}

static int spi_pico_pio_init(const struct device *dev)
{
    const struct ads1x4s0x_pio_config *dev_cfg = dev->config;
    int rc;

    rc = pinctrl_apply_state(dev_cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
    if (rc) {
        LOG_ERR("Failed to apply pinctrl state");
        return rc;
    }

    rc = gpio_pin_configure_dt(&dev_cfg->gpio_cs, GPIO_OUTPUT_INACTIVE);
    if (rc) {
        LOG_ERR("Failed to initialize cs pin");
        return rc;
    }

    return 0;
}

/* ------------- */
/* PIO bluk read */
/* ------------- */

static void pio_spi_odm_inst_inst(struct pio_spi_odm_raw_program *pgm,
        uint8_t *sm_insts, size_t max_inst_len) {
    memset(sm_insts, 0, max_inst_len);
    pgm->raw_inst = sm_insts;
    pgm->raw_inst_max_len = max_inst_len;
    pgm->tx_cnt = 0;
    pgm->rx_cnt = 0;
    pgm->iptr = 0;
    pgm->half = false;
}

static void pio_spi_odm_inst_do_tx_rx(struct pio_spi_odm_raw_program *pgm,
        uint8_t tx_byte, bool do_rx) {
    uint8_t raw_pio_inst = 0;

    pgm->rx_cnt += do_rx;

    for (int j = 8; j > 0; j--) {
        raw_pio_inst = (1 << 2) | (((!do_rx) & 1) << 1) | ((tx_byte >> (j-1)) & 1);

        if (!pgm->half) {
            pgm->raw_inst[pgm->iptr] = raw_pio_inst << 4;
            pgm->half = true;

        } else {
            pgm->raw_inst[pgm->iptr] |= raw_pio_inst;
            pgm->half = false;

            ++pgm->iptr;
        }
    }
}

static void pio_spi_odm_inst_do_tx(struct pio_spi_odm_raw_program *pgm, uint8_t tx_byte) {
    pio_spi_odm_inst_do_tx_rx(pgm, tx_byte, false);
}

static void pio_spi_odm_inst_do_wait(struct pio_spi_odm_raw_program *pgm) {
    uint8_t raw_pio_inst = 0;

    if (!pgm->half) {
        pgm->raw_inst[pgm->iptr] = raw_pio_inst << 4;
        pgm->half = true;

    } else {
        pgm->raw_inst[pgm->iptr] |= raw_pio_inst;
        pgm->half = false;

        ++pgm->iptr;
    }
}

static void pio_spi_odm_inst_finalize(struct pio_spi_odm_raw_program *pgm) {
    if (pgm->half) {
        pgm->tx_cnt = pgm->iptr + 1;
        pgm->raw_inst[pgm->iptr] |= (1 << 3);  /* write nop bit */
    } else {
        pgm->tx_cnt = pgm->iptr;
    }
}

/*############################################################*/
/*                     End of SPI via PIO                     */
/*############################################################*/

#define ADS1X4S0X_CLK_FREQ_IN_KHZ                           4096
#define ADS1X4S0X_RESET_LOW_TIME_IN_CLOCK_CYCLES            4
#define ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_CLOCK_CYCLES 4
#define ADS1X4S0X_SETUP_TIME_IN_CLOCK_CYCLES                32
#define ADS1X4S0X_INPUT_SELECTION_AINCOM                    12
#define ADS1X4S0X_REF_INTERNAL                              2500
#define ADS1X4S0X_GPIO_MAX                                  3
#define ADS1X4S0X_POWER_ON_RESET_TIME_IN_US                 2200
#define ADS1X4S0X_VBIAS_PIN_MAX                             7
#define ADS1X4S0X_VBIAS_PIN_MIN                             0

/* Not mentioned in the datasheet, but instead determined experimentally. */
#define ADS1X4S0X_RESET_DELAY_TIME_SAFETY_MARGIN_IN_US 1000
#define ADS1X4S0X_RESET_DELAY_TIME_IN_US                                                           \
    (4096 * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ + ADS1X4S0X_RESET_DELAY_TIME_SAFETY_MARGIN_IN_US)

#define ADS1X4S0X_RESET_LOW_TIME_IN_US                                                             \
    (ADS1X4S0X_RESET_LOW_TIME_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)
#define ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_US                                                  \
    (ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)
#define ADS1X4S0X_SETUP_TIME_IN_US                                                                 \
    (ADS1X4S0X_SETUP_TIME_IN_CLOCK_CYCLES * 1000 / ADS1X4S0X_CLK_FREQ_IN_KHZ)

enum ads1x4s0x_pio_command {
    ADS1X4S0X_COMMAND_NOP = 0x00,
    ADS1X4S0X_COMMAND_WAKEUP = 0x02,
    ADS1X4S0X_COMMAND_POWERDOWN = 0x04,
    ADS1X4S0X_COMMAND_RESET = 0x06,
    ADS1X4S0X_COMMAND_START = 0x08,
    ADS1X4S0X_COMMAND_STOP = 0x0A,
    ADS1X4S0X_COMMAND_SYOCAL = 0x16,
    ADS1X4S0X_COMMAND_SYGCAL = 0x17,
    ADS1X4S0X_COMMAND_SFOCAL = 0x19,
    ADS1X4S0X_COMMAND_RDATA = 0x12,
    ADS1X4S0X_COMMAND_RREG = 0x20,
    ADS1X4S0X_COMMAND_WREG = 0x40,
};

enum ads1x4s0x_pio_register {
    ADS1X4S0X_REGISTER_ID = 0x00,
    ADS1X4S0X_REGISTER_STATUS = 0x01,
    ADS1X4S0X_REGISTER_INPMUX = 0x02,
    ADS1X4S0X_REGISTER_PGA = 0x03,
    ADS1X4S0X_REGISTER_DATARATE = 0x04,
    ADS1X4S0X_REGISTER_REF = 0x05,
    ADS1X4S0X_REGISTER_IDACMAG = 0x06,
    ADS1X4S0X_REGISTER_IDACMUX = 0x07,
    ADS1X4S0X_REGISTER_VBIAS = 0x08,
    ADS1X4S0X_REGISTER_SYS = 0x09,
    ADS1X4S0X_REGISTER_GPIODAT = 0x10,
    ADS1X4S0X_REGISTER_GPIOCON = 0x11,
    ADS114S0X_REGISTER_OFCAL0 = 0x0B,
    ADS114S0X_REGISTER_OFCAL1 = 0x0C,
    ADS114S0X_REGISTER_FSCAL0 = 0x0E,
    ADS114S0X_REGISTER_FSCAL1 = 0x0F,
    ADS124S0X_REGISTER_OFCAL0 = 0x0A,
    ADS124S0X_REGISTER_OFCAL1 = 0x0B,
    ADS124S0X_REGISTER_OFCAL2 = 0x0C,
    ADS124S0X_REGISTER_FSCAL0 = 0x0E,
    ADS124S0X_REGISTER_FSCAL1 = 0x0F,
    ADS124S0X_REGISTER_FSCAL2 = 0x0F,
};

#define ADS1X4S0X_REGISTER_GET_VALUE(value, pos, length)                                           \
    FIELD_GET(GENMASK(pos + length - 1, pos), value)
#define ADS1X4S0X_REGISTER_SET_VALUE(target, value, pos, length)                                   \
    target &= ~GENMASK(pos + length - 1, pos);                                                 \
    target |= FIELD_PREP(GENMASK(pos + length - 1, pos), value)

#define ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH 3
#define ADS1X4S0X_REGISTER_ID_DEV_ID_POS    0
#define ADS1X4S0X_REGISTER_ID_DEV_ID_GET(value)                                                    \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_ID_DEV_ID_POS,                      \
            ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH)
#define ADS1X4S0X_REGISTER_ID_DEV_ID_SET(target, value)                                            \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_ID_DEV_ID_POS,              \
            ADS1X4S0X_REGISTER_ID_DEV_ID_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_POS    7
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_GET(value)                                                \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_POR_POS,                  \
            ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_POR_SET(target, value)                                        \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_POR_POS,          \
            ADS1X4S0X_REGISTER_STATUS_FL_POR_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS    6
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_GET(value)                                               \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS,                 \
            ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_NOT_RDY_SET(target, value)                                       \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_NOT_RDY_POS,         \
            ADS1X4S0X_REGISTER_STATUS_NOT_RDY_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS    5
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_GET(value)                                            \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS,              \
            ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_SET(target, value)                                    \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_POS,      \
            ADS1X4S0X_REGISTER_STATUS_FL_P_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS    4
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_GET(value)                                            \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS,              \
            ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_SET(target, value)                                    \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_POS,      \
            ADS1X4S0X_REGISTER_STATUS_FL_P_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS    3
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_GET(value)                                            \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS,              \
            ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_SET(target, value)                                    \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_POS,      \
            ADS1X4S0X_REGISTER_STATUS_FL_N_RAILP_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS    2
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_GET(value)                                            \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS,              \
            ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_SET(target, value)                                    \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_POS,      \
            ADS1X4S0X_REGISTER_STATUS_FL_N_RAILN_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS    1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_GET(value)                                             \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS,               \
            ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_SET(target, value)                                     \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_POS,       \
            ADS1X4S0X_REGISTER_STATUS_FL_REF_L1_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH 1
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS    0
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_GET(value)                                             \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS,               \
            ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH)
#define ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_SET(target, value)                                     \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_POS,       \
            ADS1X4S0X_REGISTER_STATUS_FL_REF_L0_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH 4
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_POS    4
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_INPMUX_MUXP_POS,                    \
            ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_INPMUX_MUXP_POS,            \
            ADS1X4S0X_REGISTER_INPMUX_MUXP_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH 4
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_POS    0
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_INPMUX_MUXN_POS,                    \
            ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH)
#define ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_INPMUX_MUXN_POS,            \
            ADS1X4S0X_REGISTER_INPMUX_MUXN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH 3
#define ADS1X4S0X_REGISTER_PGA_DELAY_POS    5
#define ADS1X4S0X_REGISTER_PGA_DELAY_GET(value)                                                    \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_DELAY_POS,                      \
            ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_DELAY_SET(target, value)                                            \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_DELAY_POS,              \
            ADS1X4S0X_REGISTER_PGA_DELAY_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH 2
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_POS    3
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_GET(value)                                                   \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_PGA_EN_POS,                     \
            ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_PGA_EN_SET(target, value)                                           \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_PGA_EN_POS,             \
            ADS1X4S0X_REGISTER_PGA_PGA_EN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH 3
#define ADS1X4S0X_REGISTER_PGA_GAIN_POS    0
#define ADS1X4S0X_REGISTER_PGA_GAIN_GET(value)                                                     \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_PGA_GAIN_POS,                       \
            ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH)
#define ADS1X4S0X_REGISTER_PGA_GAIN_SET(target, value)                                             \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_PGA_GAIN_POS,               \
            ADS1X4S0X_REGISTER_PGA_GAIN_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS    7
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_GET(value)                                              \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS,                \
            ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_G_CHOP_SET(target, value)                                      \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_G_CHOP_POS,        \
            ADS1X4S0X_REGISTER_DATARATE_G_CHOP_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_CLK_POS    6
#define ADS1X4S0X_REGISTER_DATARATE_CLK_GET(value)                                                 \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_CLK_POS,                   \
            ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_CLK_SET(target, value)                                         \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_CLK_POS,           \
            ADS1X4S0X_REGISTER_DATARATE_CLK_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_MODE_POS    5
#define ADS1X4S0X_REGISTER_DATARATE_MODE_GET(value)                                                \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_MODE_POS,                  \
            ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_MODE_SET(target, value)                                        \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_MODE_POS,          \
            ADS1X4S0X_REGISTER_DATARATE_MODE_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH 1
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_POS    4
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_GET(value)                                              \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_FILTER_POS,                \
            ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_FILTER_SET(target, value)                                      \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_FILTER_POS,        \
            ADS1X4S0X_REGISTER_DATARATE_FILTER_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH 4
#define ADS1X4S0X_REGISTER_DATARATE_DR_POS    0
#define ADS1X4S0X_REGISTER_DATARATE_DR_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_DATARATE_DR_POS,                    \
            ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH)
#define ADS1X4S0X_REGISTER_DATARATE_DR_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_DATARATE_DR_POS,            \
            ADS1X4S0X_REGISTER_DATARATE_DR_LENGTH)
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS    6
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_GET(value)                                                \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS,                  \
            ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH)
#define ADS1X4S0X_REGISTER_REF_FL_REF_EN_SET(target, value)                                        \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_FL_REF_EN_POS,          \
            ADS1X4S0X_REGISTER_REF_FL_REF_EN_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH 1
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS    5
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_GET(value)                                             \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS,               \
            ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(target, value)                                     \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_POS,       \
            ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH 1
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS    4
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_GET(value)                                             \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS,               \
            ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(target, value)                                     \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_POS,       \
            ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_REFSEL_POS    2
#define ADS1X4S0X_REGISTER_REF_REFSEL_GET(value)                                                   \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_REFSEL_POS,                     \
            ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFSEL_SET(target, value)                                           \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_REFSEL_POS,             \
            ADS1X4S0X_REGISTER_REF_REFSEL_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFCON_LENGTH 2
#define ADS1X4S0X_REGISTER_REF_REFCON_POS    0
#define ADS1X4S0X_REGISTER_REF_REFCON_GET(value)                                                   \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_REF_REFCON_POS,                     \
            ADS1X4S0X_REGISTER_REF_REFCON_LENGTH)
#define ADS1X4S0X_REGISTER_REF_REFCON_SET(target, value)                                           \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_REF_REFCON_POS,             \
            ADS1X4S0X_REGISTER_REF_REFCON_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH 1
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS    7
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_GET(value)                                           \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS,             \
            ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_SET(target, value)                                   \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_POS,     \
            ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH 1
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_POS    6
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_PSW_POS,                    \
            ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_PSW_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_PSW_POS,            \
            ADS1X4S0X_REGISTER_IDACMAG_PSW_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS    0
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_GET(value)                                                 \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS,                   \
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(target, value)                                         \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMAG_IMAG_POS,           \
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS    4
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_GET(value)                                                \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS,                  \
            ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I2MUX_SET(target, value)                                        \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMUX_I2MUX_POS,          \
            ADS1X4S0X_REGISTER_IDACMUX_I2MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH 4
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS    0
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_GET(value)                                                \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS,                  \
            ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH)
#define ADS1X4S0X_REGISTER_IDACMUX_I1MUX_SET(target, value)                                        \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_IDACMUX_I1MUX_POS,          \
            ADS1X4S0X_REGISTER_IDACMUX_I1MUX_LENGTH)
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH 1
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS    7
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_GET(value)                                               \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS,                 \
            ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH)
#define ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_SET(target, value)                                       \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_POS,         \
            ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_POS    4
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIODAT_DIR_POS,                    \
            ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DIR_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIODAT_DIR_POS,            \
            ADS1X4S0X_REGISTER_GPIODAT_DIR_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_POS    0
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIODAT_DAT_POS,                    \
            ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH)
#define ADS1X4S0X_REGISTER_GPIODAT_DAT_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIODAT_DAT_POS,            \
            ADS1X4S0X_REGISTER_GPIODAT_DAT_LENGTH)
#define ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH 4
#define ADS1X4S0X_REGISTER_GPIOCON_CON_POS    0
#define ADS1X4S0X_REGISTER_GPIOCON_CON_GET(value)                                                  \
    ADS1X4S0X_REGISTER_GET_VALUE(value, ADS1X4S0X_REGISTER_GPIOCON_CON_POS,                    \
            ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH)
#define ADS1X4S0X_REGISTER_GPIOCON_CON_SET(target, value)                                          \
    ADS1X4S0X_REGISTER_SET_VALUE(target, value, ADS1X4S0X_REGISTER_GPIOCON_CON_POS,            \
            ADS1X4S0X_REGISTER_GPIOCON_CON_LENGTH)

/*
 * - AIN0 as positive input
 * - AIN1 as negative input
 */
#define ADS1X4S0X_REGISTER_INPMUX_SET_DEFAULTS(target)                                             \
    ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(target, 0b0000);                                        \
    ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(target, 0b0001)
/*
 * - disable reference monitor
 * - enable positive reference buffer
 * - disable negative reference buffer
 * - use internal reference
 * - enable internal voltage reference
 */
#define ADS1X4S0X_REGISTER_REF_SET_DEFAULTS(target)                                                \
    ADS1X4S0X_REGISTER_REF_FL_REF_EN_SET(target, 0b00);                                        \
    ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(target, 0b0);                                      \
    ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(target, 0b1);                                      \
    ADS1X4S0X_REGISTER_REF_REFSEL_SET(target, 0b10);                                           \
    ADS1X4S0X_REGISTER_REF_REFCON_SET(target, 0b01)
/*
 * - disable global chop
 * - use internal oscillator
 * - single shot conversion mode
 * - low latency filter
 * - 20 samples per second
 */
#define ADS1X4S0X_REGISTER_DATARATE_SET_DEFAULTS(target)                                           \
    ADS1X4S0X_REGISTER_DATARATE_G_CHOP_SET(target, 0b0);                                       \
    ADS1X4S0X_REGISTER_DATARATE_CLK_SET(target, 0b0);                                          \
    ADS1X4S0X_REGISTER_DATARATE_MODE_SET(target, 0b1);                                         \
    ADS1X4S0X_REGISTER_DATARATE_FILTER_SET(target, 0b1);                                       \
    ADS1X4S0X_REGISTER_DATARATE_DR_SET(target, 0b0100)
/*
 * - delay of 14*t_mod
 * - disable gain
 * - gain 1
 */
#define ADS1X4S0X_REGISTER_PGA_SET_DEFAULTS(target)                                                \
    ADS1X4S0X_REGISTER_PGA_DELAY_SET(target, 0b000);                                           \
    ADS1X4S0X_REGISTER_PGA_PGA_EN_SET(target, 0b00);                                           \
    ADS1X4S0X_REGISTER_PGA_GAIN_SET(target, 0b000)
/*
 * - disable PGA output rail flag
 * - low-side power switch
 * - IDAC off
 */
#define ADS1X4S0X_REGISTER_IDACMAG_SET_DEFAULTS(target)                                            \
    ADS1X4S0X_REGISTER_IDACMAG_FL_RAIL_EN_SET(target, 0b0);                                    \
    ADS1X4S0X_REGISTER_IDACMAG_PSW_SET(target, 0b0);                                           \
    ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(target, 0b0000)
/*
 * - disconnect IDAC1
 * - disconnect IDAC2
 */
#define ADS1X4S0X_REGISTER_IDACMUX_SET_DEFAULTS(target)                                            \
    ADS1X4S0X_REGISTER_IDACMUX_I1MUX_SET(target, 0b1111);                                      \
    ADS1X4S0X_REGISTER_IDACMUX_I2MUX_SET(target, 0b1111)

static void ads1x4s0x_pio_data_ready_handler(const struct device *dev, struct gpio_callback *gpio_cb,
        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);

    struct ads1x4s0x_pio_data *data =
        CONTAINER_OF(gpio_cb, struct ads1x4s0x_pio_data, callback_data_ready);

    k_sem_give(&data->data_ready_signal);
}

static int ads1x4s0x_pio_read_register(const struct device *dev,
        enum ads1x4s0x_pio_register register_address, uint8_t *value)
{
    uint8_t buffer_tx[3];
    uint8_t buffer_rx[3];

    buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_RREG) | ((uint8_t)register_address);
    /* read one register */
    buffer_tx[1] = 0x00;

    int result = spi_pico_pio_transceive(dev, buffer_tx, buffer_rx, 3);

    if (result != 0) {
        LOG_ERR("%s: spi_pico_pio_transceive failed with error %i", dev->name, result);
        return result;
    }

    *value = buffer_rx[2];
    LOG_DBG("%s: read from register 0x%02X value 0x%02X", dev->name, register_address, *value);

    return 0;
}

static int ads1x4s0x_pio_write_register(const struct device *dev,
        enum ads1x4s0x_pio_register register_address, uint8_t value)
{
    uint8_t buffer_tx[3];

    buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_address);
    /* write one register */
    buffer_tx[1] = 0x00;
    buffer_tx[2] = value;

    LOG_DBG("%s: writing to register 0x%02X value 0x%02X", dev->name, register_address, value);
    int result = spi_pico_pio_transceive(dev, buffer_tx, NULL, 3);

    if (result != 0) {
        LOG_ERR("%s: spi_write failed with error %i", dev->name, result);
        return result;
    }

    return 0;
}

static int ads1x4s0x_pio_write_multiple_registers(const struct device *dev,
        enum ads1x4s0x_pio_register *register_addresses,
        uint8_t *values, size_t count)
{
    uint8_t buffer_tx[32];
    int result;

    if (count == 0) {
        LOG_WRN("%s: ignoring the command to write 0 registers", dev->name);
        return -EINVAL;
    }

    buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_addresses[0]);
    buffer_tx[1] = count - 1;

    memcpy(buffer_tx + 2, values, count);

    LOG_HEXDUMP_DBG(register_addresses, count, "writing to registers");
    LOG_HEXDUMP_DBG(values, count, "values");

    /* ensure that the register addresses are in the correct order */
    for (size_t i = 1; i < count; ++i) {
        __ASSERT(register_addresses[i - 1] + 1 == register_addresses[i],
                "register addresses are not consecutive");
    }

    result = spi_pico_pio_transceive(dev, buffer_tx, NULL, count + 2);

    if (result != 0) {
        LOG_ERR("%s: spi_write failed with error %i", dev->name, result);
        return result;
    }

    return 0;
}

static int ads1x4s0x_pio_send_command(const struct device *dev, enum ads1x4s0x_pio_command command)
{
    uint8_t buffer_tx[1];

    buffer_tx[0] = (uint8_t)command;

    LOG_DBG("%s: sending command 0x%02X", dev->name, command);
    int result = spi_pico_pio_transceive(dev, buffer_tx, NULL, 1);

    if (result != 0) {
        LOG_ERR("%s: spi_write failed with error %i", dev->name, result);
        return result;
    }

    return 0;
}

static int ads1x4s0x_pio_make_channel_setup(const struct device *dev,
        const struct adc_channel_cfg *channel_cfg,
        enum ads1x4s0x_pio_register *register_addresses,  // length is 7
        uint8_t *values)   // length is 7
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    uint8_t input_mux = 0;
    uint8_t reference_control = 0;
    uint8_t data_rate = 0;
    uint8_t gain = 0;
    uint8_t idac_magnitude = 0;
    uint8_t idac_mux = 0;
    uint8_t pin_selections[4];
    uint8_t vbias = 0;
    size_t pin_selections_size;
    uint16_t acquisition_time_value = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
    uint16_t acquisition_time_unit = ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time);

    ADS1X4S0X_REGISTER_INPMUX_SET_DEFAULTS(gain);
    ADS1X4S0X_REGISTER_REF_SET_DEFAULTS(reference_control);
    ADS1X4S0X_REGISTER_DATARATE_SET_DEFAULTS(data_rate);
    ADS1X4S0X_REGISTER_PGA_SET_DEFAULTS(gain);
    ADS1X4S0X_REGISTER_IDACMAG_SET_DEFAULTS(idac_magnitude);
    ADS1X4S0X_REGISTER_IDACMUX_SET_DEFAULTS(idac_mux);

    /* The ADS114 uses samples per seconds units with the lowest being 2.5SPS
     * and with acquisition_time only having 14b for time, this will not fit
     * within here for microsecond units. Use Tick units and allow the user to
     * specify the ODR directly.
     */
    if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT &&
            acquisition_time_unit != ADC_ACQ_TIME_TICKS) {
        LOG_ERR("%s: invalid acquisition time %i", dev->name,
                channel_cfg->acquisition_time);
        return -EINVAL;
    }

    if (channel_cfg->acquisition_time == ADC_ACQ_TIME_DEFAULT) {
        ADS1X4S0X_REGISTER_DATARATE_DR_SET(data_rate, ADS1X4S0X_CONFIG_DR_20);
    } else {
        ADS1X4S0X_REGISTER_DATARATE_DR_SET(data_rate, acquisition_time_value);
    }

    switch (channel_cfg->reference) {
        case ADC_REF_INTERNAL:
            /* disable negative reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(reference_control, 0b1);
            /* disable positive reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(reference_control, 0b1);
            /* use internal reference */
            ADS1X4S0X_REGISTER_REF_REFSEL_SET(reference_control, 0b10);
            break;
        case ADC_REF_EXTERNAL0:
            /* enable negative reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(reference_control, 0b0);
            /* enable positive reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(reference_control, 0b0);
            /* use external reference 0*/
            ADS1X4S0X_REGISTER_REF_REFSEL_SET(reference_control, 0b00);
            break;
        case ADC_REF_EXTERNAL1:
            /* enable negative reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFN_BUF_SET(reference_control, 0b0);
            /* enable positive reference buffer */
            ADS1X4S0X_REGISTER_REF_NOT_REFP_BUF_SET(reference_control, 0b0);
            /* use external reference 0*/
            ADS1X4S0X_REGISTER_REF_REFSEL_SET(reference_control, 0b01);
            break;
        default:
            LOG_ERR("%s: reference %i is not supported", dev->name, channel_cfg->reference);
            return -EINVAL;
    }

    if (channel_cfg->differential) {
        LOG_DBG("%s: configuring channel for a differential measurement from the pins (p, "
                "n) (%i, %i)",
                dev->name, channel_cfg->input_positive, channel_cfg->input_negative);
        if (channel_cfg->input_positive >= config->channels &&
                channel_cfg->input_positive != ADS1X4S0X_INPUT_SELECTION_AINCOM) {
            LOG_ERR("%s: positive channel input %i is invalid", dev->name,
                    channel_cfg->input_positive);
            return -EINVAL;
        }

        if (channel_cfg->input_negative >= config->channels &&
                channel_cfg->input_negative != ADS1X4S0X_INPUT_SELECTION_AINCOM) {
            LOG_ERR("%s: negative channel input %i is invalid", dev->name,
                    channel_cfg->input_negative);
            return -EINVAL;
        }

        if (channel_cfg->input_positive == channel_cfg->input_negative) {
            LOG_ERR("%s: negative and positive channel inputs must be different",
                    dev->name);
            return -EINVAL;
        }

        ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(input_mux, channel_cfg->input_positive);
        ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(input_mux, channel_cfg->input_negative);
        pin_selections[0] = channel_cfg->input_positive;
        pin_selections[1] = channel_cfg->input_negative;
    } else {
        LOG_DBG("%s: configuring channel for single ended measurement from input %i",
                dev->name, channel_cfg->input_positive);
        if (channel_cfg->input_positive >= config->channels &&
                channel_cfg->input_positive != ADS1X4S0X_INPUT_SELECTION_AINCOM) {
            LOG_ERR("%s: channel input %i is invalid", dev->name,
                    channel_cfg->input_positive);
            return -EINVAL;
        }

        ADS1X4S0X_REGISTER_INPMUX_MUXP_SET(input_mux, channel_cfg->input_positive);
        ADS1X4S0X_REGISTER_INPMUX_MUXN_SET(input_mux, ADS1X4S0X_INPUT_SELECTION_AINCOM);
        pin_selections[0] = channel_cfg->input_positive;
        pin_selections[1] = ADS1X4S0X_INPUT_SELECTION_AINCOM;
    }

    switch (channel_cfg->gain) {
        case ADC_GAIN_1:
            /* set gain value */
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b000);
            break;
        case ADC_GAIN_2:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b001);
            break;
        case ADC_GAIN_4:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b010);
            break;
        case ADC_GAIN_8:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b011);
            break;
        case ADC_GAIN_16:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b100);
            break;
        case ADC_GAIN_32:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b101);
            break;
        case ADC_GAIN_64:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b110);
            break;
        case ADC_GAIN_128:
            ADS1X4S0X_REGISTER_PGA_GAIN_SET(gain, 0b111);
            break;
        default:
            LOG_ERR("%s: gain value %i not supported", dev->name, channel_cfg->gain);
            return -EINVAL;
    }

    if (channel_cfg->gain != ADC_GAIN_1) {
        /* enable gain */
        ADS1X4S0X_REGISTER_PGA_PGA_EN_SET(gain, 0b01);
    }

    switch (config->idac_current) {
        case 0:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0000);
            break;
        case 10:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0001);
            break;
        case 50:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0010);
            break;
        case 100:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0011);
            break;
        case 250:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0100);
            break;
        case 500:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0101);
            break;
        case 750:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0110);
            break;
        case 1000:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b0111);
            break;
        case 1500:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b1000);
            break;
        case 2000:
            ADS1X4S0X_REGISTER_IDACMAG_IMAG_SET(idac_magnitude, 0b1001);
            break;
        default:
            LOG_ERR("%s: IDAC magnitude %i not supported", dev->name, config->idac_current);
            return -EINVAL;
    }

    if (channel_cfg->current_source_pin_set) {
        LOG_DBG("%s: current source pin set to %i and %i", dev->name,
                channel_cfg->current_source_pin[0], channel_cfg->current_source_pin[1]);
        if (channel_cfg->current_source_pin[0] > 0b1111) {
            LOG_ERR("%s: invalid selection %i for I1MUX", dev->name,
                    channel_cfg->current_source_pin[0]);
            return -EINVAL;
        }

        if (channel_cfg->current_source_pin[1] > 0b1111) {
            LOG_ERR("%s: invalid selection %i for I2MUX", dev->name,
                    channel_cfg->current_source_pin[1]);
            return -EINVAL;
        }

        ADS1X4S0X_REGISTER_IDACMUX_I1MUX_SET(idac_mux, channel_cfg->current_source_pin[0]);
        ADS1X4S0X_REGISTER_IDACMUX_I2MUX_SET(idac_mux, channel_cfg->current_source_pin[1]);
        pin_selections[2] = channel_cfg->current_source_pin[0];
        pin_selections[3] = channel_cfg->current_source_pin[1];
        pin_selections_size = 4;
    } else {
        LOG_DBG("%s: current source pins not set", dev->name);
        pin_selections_size = 2;
    }

    for (size_t i = 0; i < pin_selections_size; ++i) {
        if (pin_selections[i] > ADS1X4S0X_INPUT_SELECTION_AINCOM) {
            continue;
        }

        for (size_t j = i + 1; j < pin_selections_size; ++j) {
            if (pin_selections[j] > ADS1X4S0X_INPUT_SELECTION_AINCOM) {
                continue;
            }

            if (pin_selections[i] == pin_selections[j]) {
                LOG_ERR("%s: pins for inputs and current sources must be different",
                        dev->name);
                return -EINVAL;
            }
        }
    }

    ADS1X4S0X_REGISTER_VBIAS_VB_LEVEL_SET(vbias, config->vbias_level);

    if ((channel_cfg->vbias_pins &
                ~GENMASK(ADS1X4S0X_VBIAS_PIN_MAX, ADS1X4S0X_VBIAS_PIN_MIN)) != 0) {
        LOG_ERR("%s: invalid VBIAS pin selection 0x%08X", dev->name,
                channel_cfg->vbias_pins);
        return -EINVAL;
    }

    vbias |= channel_cfg->vbias_pins;

    register_addresses[0] = ADS1X4S0X_REGISTER_INPMUX;
    register_addresses[1] = ADS1X4S0X_REGISTER_PGA;
    register_addresses[2] = ADS1X4S0X_REGISTER_DATARATE;
    register_addresses[3] = ADS1X4S0X_REGISTER_REF;
    register_addresses[4] = ADS1X4S0X_REGISTER_IDACMAG;
    register_addresses[5] = ADS1X4S0X_REGISTER_IDACMUX;
    register_addresses[6] = ADS1X4S0X_REGISTER_VBIAS;
    values[0] = input_mux;
    values[1] = gain;
    values[2] = data_rate;
    values[3] = reference_control;
    values[4] = idac_magnitude;
    values[5] = idac_mux;
    values[6] = vbias;

    return 0;
}

static int ads1x4s0x_pio_channel_setup(const struct device *dev,
        const struct adc_channel_cfg *channel_cfg)
{
    int result;
    enum ads1x4s0x_pio_register register_addresses[7];
    uint8_t values[ARRAY_SIZE(register_addresses)];

    if (channel_cfg->channel_id != 0) {
        LOG_ERR("%s: only one channel is supported", dev->name);
        return -EINVAL;
    }

    result = ads1x4s0x_pio_make_channel_setup(dev, channel_cfg, register_addresses, values);

    if (result != 0) {
        return result;
    }

    result = ads1x4s0x_pio_write_multiple_registers(dev, register_addresses, values,
            ARRAY_SIZE(values));

    if (result != 0) {
        LOG_ERR("%s: unable to configure registers", dev->name);
        return result;
    }

    return 0;
}

static int ads1x4s0x_pio_validate_buffer_size(const struct device *dev,
        const struct adc_sequence *sequence)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    size_t needed;

    needed = (config->resolution > 16) ? sizeof(int32_t) : sizeof(int16_t);

    if (sequence->options) {
        needed *= (1 + sequence->options->extra_samplings);
    }

    if (sequence->buffer_size < needed) {
        return -ENOMEM;
    }

    return 0;
}

static int ads1x4s0x_pio_validate_sequence(const struct device *dev,
        const struct adc_sequence *sequence)
{
    const struct ads1x4s0x_pio_config *config = dev->config;

    if (sequence->resolution != config->resolution) {
        LOG_ERR("%s: invalid resolution", dev->name);
        return -EINVAL;
    }

    if (sequence->channels != BIT(0)) {
        LOG_ERR("%s: invalid channel", dev->name);
        return -EINVAL;
    }

    if (sequence->oversampling) {
        LOG_ERR("%s: oversampling is not supported", dev->name);
        return -EINVAL;
    }

    return ads1x4s0x_pio_validate_buffer_size(dev, sequence);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
    struct ads1x4s0x_pio_data *data = CONTAINER_OF(ctx, struct ads1x4s0x_pio_data, ctx);

    if (repeat_sampling) {
        data->buffer = data->buffer_ptr;
    }
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct ads1x4s0x_pio_data *data = CONTAINER_OF(ctx, struct ads1x4s0x_pio_data, ctx);

    data->buffer_ptr = data->buffer;
    k_sem_give(&data->acquire_signal);
}

static int ads1x4s0x_pio_adc_start_read(const struct device *dev, const struct adc_sequence *sequence,
        bool wait)
{
    int result;
    struct ads1x4s0x_pio_data *data = dev->data;

    result = ads1x4s0x_pio_validate_sequence(dev, sequence);

    if (result != 0) {
        LOG_ERR("%s: sequence validation failed", dev->name);
        return result;
    }

    data->buffer = sequence->buffer;

    adc_context_start_read(&data->ctx, sequence);

    if (wait) {
        result = adc_context_wait_for_completion(&data->ctx);
    }

    return result;
}

static int ads1x4s0x_pio_send_start_read(const struct device *dev)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    int result;

    if (config->gpio_start_sync.port == 0) {
        result = ads1x4s0x_pio_send_command(dev, ADS1X4S0X_COMMAND_START);
        if (result != 0) {
            LOG_ERR("%s: unable to send START/SYNC command", dev->name);
            return result;
        }
    } else {
        result = gpio_pin_set_dt(&config->gpio_start_sync, 1);

        if (result != 0) {
            LOG_ERR("%s: unable to start ADC operation", dev->name);
            return result;
        }

        k_sleep(K_USEC(ADS1X4S0X_START_SYNC_PULSE_DURATION_IN_US +
                    ADS1X4S0X_SETUP_TIME_IN_US));

        result = gpio_pin_set_dt(&config->gpio_start_sync, 0);

        if (result != 0) {
            LOG_ERR("%s: unable to start ADC operation", dev->name);
            return result;
        }
    }

    return 0;
}

static int ads1x4s0x_pio_wait_data_ready(const struct device *dev)
{
    struct ads1x4s0x_pio_data *data = dev->data;

    return k_sem_take(&data->data_ready_signal, ADC_CONTEXT_WAIT_FOR_COMPLETION_TIMEOUT);
}

#if ADS1X4S0X_HAS_16_BIT_DEV
static int ads1x4s0x_pio_read_sample_16(const struct device *dev, int16_t *buffer)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    uint8_t buffer_tx[3];
    uint8_t buffer_rx[3];

    buffer_tx[0] = (uint8_t)ADS1X4S0X_COMMAND_RDATA;

    int result = spi_pico_pio_transceive(dev, buffer_tx, buffer_rx, 3);

    if (result != 0) {
        LOG_ERR("%s: spi_pico_pio_transceive failed with error %i", dev->name, result);
        return result;
    }

    *buffer = sys_get_be16(buffer_rx + 1);
    LOG_DBG("%s: read ADC sample %i", dev->name, *buffer);

    return 0;
}
#endif

#if ADS1X4S0X_HAS_24_BIT_DEV
static int ads1x4s0x_pio_read_sample_24(const struct device *dev, int32_t *buffer)
{
    uint8_t buffer_tx[4] = {0};
    uint8_t buffer_rx[5] = {0};

    buffer_tx[0] = (uint8_t)ADS1X4S0X_COMMAND_RDATA;

    int result = spi_pico_pio_transceive(dev, buffer_tx, buffer_rx, 4);

    if (result != 0) {
        LOG_ERR("%s: spi_pico_pio_transceive failed with error %i", dev->name, result);
        return result;
    }

    *buffer = (int32_t)sys_get_be32(buffer_rx + 1) >> 8;

    LOG_DBG("%s: read ADC sample 0x%02X%02X%02X", dev->name, *(buffer_rx + 1), *(buffer_rx + 2),
            *(buffer_rx + 3));

    return 0;
}
#endif

static int ads1x4s0x_pio_adc_perform_read(const struct device *dev)
{
    int result;
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;
    void *buffer = data->buffer;

    k_sem_take(&data->acquire_signal, K_FOREVER);
    k_sem_reset(&data->data_ready_signal);

    result = ads1x4s0x_pio_send_start_read(dev);
    if (result != 0) {
        LOG_ERR("%s: unable to start ADC conversion", dev->name);
        adc_context_complete(&data->ctx, result);
        return result;
    }

    result = ads1x4s0x_pio_wait_data_ready(dev);
    if (result != 0) {
        LOG_ERR("%s: waiting for data to be ready failed", dev->name);
        adc_context_complete(&data->ctx, result);
        return result;
    }

#if ADS1X4S0X_HAS_24_BIT_DEV
    if (config->resolution == 24) {
        result = ads1x4s0x_pio_read_sample_24(dev, (int32_t *)data->buffer);

        if (result == 0) {
            buffer = (int32_t *)buffer + 1;
            adc_context_on_sampling_done(&data->ctx, dev);
            return 0;
        }

    }
#endif

#if ADS1X4S0X_HAS_16_BIT_DEV
    if (config->resolution == 16) {
        result = ads1x4s0x_pio_read_sample_16(dev, (int16_t *)data->buffer);

        if (result == 0) {
            buffer = (int16_t *)buffer + 1;
            adc_context_on_sampling_done(&data->ctx, dev);
            return 0;
        }

    }
#endif

    LOG_ERR("%s: reading sample failed", dev->name);
    adc_context_complete(&data->ctx, result);
    return result;
}

#if CONFIG_ADC_ASYNC
static int ads1x4s0x_pio_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
        struct k_poll_signal *async)
{
    int result;
    struct ads1x4s0x_pio_data *data = dev->data;

    adc_context_lock(&data->ctx, true, async);
    result = ads1x4s0x_pio_adc_start_read(dev, sequence, true);
    adc_context_release(&data->ctx, result);

    return result;
}

static int ads1x4s0x_pio_read(const struct device *dev, const struct adc_sequence *sequence)
{
    int result;
    struct ads1x4s0x_pio_data *data = dev->data;

    adc_context_lock(&data->ctx, false, NULL);
    result = ads1x4s0x_pio_adc_start_read(dev, sequence, true);
    adc_context_release(&data->ctx, result);

    return result;
}

#else
static int ads1x4s0x_pio_read(const struct device *dev, const struct adc_sequence *sequence)
{
    int result;
    struct ads1x4s0x_pio_data *data = dev->data;

    adc_context_lock(&data->ctx, false, NULL);
    result = ads1x4s0x_pio_adc_start_read(dev, sequence, false);

    while (result == 0 && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
        result = ads1x4s0x_pio_adc_perform_read(dev);
    }

    adc_context_release(&data->ctx, result);
    return result;
}
#endif

#if CONFIG_ADC_ASYNC
static void ads1x4s0x_pio_acquisition_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *dev = p1;

    while (true) {
        ads1x4s0x_pio_adc_perform_read(dev);
    }
}
#endif

static int ads1x4s0x_pio_append_write_multiple_registers_insts(
        struct pio_spi_odm_raw_program *sm_raw_pgm,
        enum ads1x4s0x_pio_register *register_addresses,
        uint8_t *values, size_t count)
{
    uint8_t buffer_tx[32];

    if (count == 0) {
        LOG_WRN("ignoring the command to write 0 registers");
        return -EINVAL;
    }

    buffer_tx[0] = ((uint8_t)ADS1X4S0X_COMMAND_WREG) | ((uint8_t)register_addresses[0]);
    buffer_tx[1] = count - 1;

    memcpy(buffer_tx + 2, values, count);

    /* ensure that the register addresses are in the correct order */
    for (size_t i = 1; i < count; ++i) {
        __ASSERT(register_addresses[i - 1] + 1 == register_addresses[i],
                "register addresses are not consecutive");
    }

    for (size_t i = 0; i < count + 2; ++i) {
        pio_spi_odm_inst_do_tx(sm_raw_pgm, buffer_tx[i]);
    }

    return 0;
}

static int ads1x4s0x_pio_prepare_bulk_read_sm_insts(
        const struct device *dev,
        struct pio_spi_odm_raw_program *sm_raw_pgm,
        struct ads1x4s0x_pio_bulk_read_config *pio_cfg)
{
    int result;
    enum ads1x4s0x_pio_register register_addresses[7];
    uint8_t values[ARRAY_SIZE(register_addresses)];
    uint8_t prev_values[ARRAY_SIZE(register_addresses)];
    uint8_t multi_write_start_ind;
    uint8_t multi_write_cnt;

    result = ads1x4s0x_pio_make_channel_setup(dev,
            &pio_cfg->chan_cfgs[pio_cfg->chan_count-1], register_addresses, prev_values);

    if (result != 0) {
        return result;
    }

    for (int i = 0; i < pio_cfg->chan_count; ++i) {
        multi_write_cnt = 0;

        result = ads1x4s0x_pio_make_channel_setup(dev,
                &pio_cfg->chan_cfgs[i], register_addresses, values);

        if (result != 0) {
            return result;
        }


        for (int j = 0; j < ARRAY_SIZE(register_addresses); ++j) {
            if (values[j] == prev_values[j]) {
                if (multi_write_cnt == 0) {
                    continue;
                }

                ads1x4s0x_pio_append_write_multiple_registers_insts(sm_raw_pgm,
                        register_addresses + multi_write_start_ind,
                        values + multi_write_start_ind,
                        multi_write_cnt);
                multi_write_cnt = 0;
            } else {
                if (multi_write_cnt == 0) {
                    multi_write_start_ind = j;
                }
                multi_write_cnt++;
            }
        }

        if (multi_write_cnt) {
            ads1x4s0x_pio_append_write_multiple_registers_insts(sm_raw_pgm,
                    register_addresses + multi_write_start_ind,
                    values + multi_write_start_ind,
                    multi_write_cnt);
        }

        for (int r = 0; r < pio_cfg->sample_count; ++r) {
            pio_spi_odm_inst_do_tx(sm_raw_pgm, ADS1X4S0X_COMMAND_START);
            pio_spi_odm_inst_do_wait(sm_raw_pgm);

            /*pio_spi_odm_inst_do_tx_rx(pgm, ADS1X4S0X_COMMAND_RDATA, false);*/
            pio_spi_odm_inst_do_tx_rx(sm_raw_pgm, 0x00, true);
            pio_spi_odm_inst_do_tx_rx(sm_raw_pgm, 0x00, true);
            pio_spi_odm_inst_do_tx_rx(sm_raw_pgm, 0x00, true);
            pio_spi_odm_inst_do_tx_rx(sm_raw_pgm, 0x00, true);
        }

        memcpy(prev_values, values, ARRAY_SIZE(register_addresses));
    }

    pio_spi_odm_inst_finalize(sm_raw_pgm);

    return 0;
}

static void ads1x4s0x_pio_bulk_read_callback(const struct device *dma_dev, void *arg, uint32_t channel,
        int status)
{
    const struct device *dev = (const struct device *)arg;
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;
    struct ads1x4s0x_pio_bulk_read_data *bdata = &data->bulk_read_data;

    if (channel == config->dma_chan_cfg.rx1) {
        bdata->data_ready_buf_ind = 1;
        dma_hw->ch[config->dma_chan_cfg.rx1].write_addr = (uint32_t)bdata->current_pio_cfg->sample_buf_1;
        dma_irqn_set_channel_enabled(config->dma_chan_cfg.rx1 % 2, config->dma_chan_cfg.rx1, true);
    } else if (channel == config->dma_chan_cfg.rx2) {
        bdata->data_ready_buf_ind = 2;
        dma_hw->ch[config->dma_chan_cfg.rx2].write_addr = (uint32_t)bdata->current_pio_cfg->sample_buf_2;
        dma_irqn_set_channel_enabled(config->dma_chan_cfg.rx2 % 2, config->dma_chan_cfg.rx2, true);
    } else {
        __ASSERT(false, "Unknown DMA channel");
    }

    k_sem_give(&bdata->bulk_data_ready_sem);
}

size_t ads1x4s0x_pio_bulk_read_get_samples_blocking(
        const struct device *dev, uint8_t *buf_ind) {
    int ret;
    struct ads1x4s0x_pio_data *data = dev->data;
    struct ads1x4s0x_pio_bulk_read_data *bdata = &data->bulk_read_data;

    ret = k_sem_take(&bdata->bulk_data_ready_sem, K_FOREVER);

    if (ret != 0) {
        return 0;
    }

    *buf_ind = bdata->data_ready_buf_ind;

    return bdata->rx_buf_len;
}

int ads1x4s0x_pio_bulk_read_setup(const struct device *dev,
        struct ads1x4s0x_pio_bulk_read_config *pio_cfg)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;
    struct ads1x4s0x_pio_bulk_read_data *bdata = &data->bulk_read_data;
    int result;
    dma_channel_config txcc1;
    dma_channel_config txcc2;
    dma_channel_config txc;
    dma_channel_config rxc1;
    dma_channel_config rxc2;
    struct dma_config zephyr_rxc;  /* just for getting irq */
    struct dma_block_config zephyr_rxc_block_cfg;

    result = spi_pico_pio_configure(config, data);
    if (result < 0) {
        return result;
    }

    pio_sm_clear_fifos(data->pio, data->pio_sm);

    bdata->rx_buf_len = pio_cfg->sample_count * pio_cfg->repeat_count * pio_cfg->chan_count * 4;

    struct pio_spi_odm_raw_program *pgm = &bdata->sm_raw_pgm;

    bdata->current_pio_cfg = pio_cfg;

    memset(&zephyr_rxc, 0, sizeof(struct dma_config));
    memset(&zephyr_rxc_block_cfg, 0, sizeof(struct dma_block_config));

    pio_spi_odm_inst_inst(pgm, bdata->sm_insts, SPI_ODM_INST_BUF_LEN);

    result = ads1x4s0x_pio_prepare_bulk_read_sm_insts(dev, pgm, pio_cfg);

    txcc1 = dma_channel_get_default_config(config->dma_chan_cfg.tx_ctrl1);
    txcc2 = dma_channel_get_default_config(config->dma_chan_cfg.tx_ctrl2);
    txc = dma_channel_get_default_config(config->dma_chan_cfg.tx);
    rxc1 = dma_channel_get_default_config(config->dma_chan_cfg.rx1);
    rxc2 = dma_channel_get_default_config(config->dma_chan_cfg.rx2);

    /* Config tx */

    channel_config_set_transfer_data_size(&txcc1, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc1, false);
    channel_config_set_write_increment(&txcc1, false);
    channel_config_set_chain_to(&txcc1, config->dma_chan_cfg.tx_ctrl2);

    dma_channel_configure(
            config->dma_chan_cfg.tx_ctrl1,
            &txcc1,
            &dma_hw->ch[config->dma_chan_cfg.tx].al3_transfer_count,
            &pgm->tx_cnt,
            1,
            false
            );

    channel_config_set_transfer_data_size(&txcc2, DMA_SIZE_32);
    channel_config_set_read_increment(&txcc2, false);
    channel_config_set_write_increment(&txcc2, false);

    dma_channel_configure(
            config->dma_chan_cfg.tx_ctrl2,
            &txcc2,
            &dma_hw->ch[config->dma_chan_cfg.tx].al3_read_addr_trig,
            &pgm->raw_inst,
            1,
            false
            );

    channel_config_set_read_increment(&txc, true);
    channel_config_set_write_increment(&txc, false);
    channel_config_set_dreq(&txc, pio_get_dreq(data->pio, data->pio_sm, true));
    channel_config_set_transfer_data_size(&txc, DMA_SIZE_8);
    channel_config_set_chain_to(&txc, config->dma_chan_cfg.tx_ctrl1);

    dma_channel_configure(config->dma_chan_cfg.tx, &txc, &data->pio->txf[data->pio_sm], NULL, 0, false);

    /* Config rx */

    zephyr_rxc.source_burst_length = 1;
    zephyr_rxc.dest_burst_length = 1;
    zephyr_rxc.user_data = (void *)dev;
    zephyr_rxc.block_count = 1U;
    zephyr_rxc.head_block = &zephyr_rxc_block_cfg;
    zephyr_rxc.dma_slot = 0x3b; /* Placeholder. Shall be overridden by actual DREQ */
    zephyr_rxc.channel_direction = PERIPHERAL_TO_MEMORY;
    zephyr_rxc.source_data_size = 1;
    zephyr_rxc.dest_data_size = 1;
    zephyr_rxc.dma_callback = ads1x4s0x_pio_bulk_read_callback;
    zephyr_rxc_block_cfg.block_size = bdata->rx_buf_len;
    zephyr_rxc_block_cfg.source_address = (uint32_t)&data->pio->rxf[data->pio_sm];
    zephyr_rxc_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

    /* just for setting up irq */
    result = dma_config(config->dma_dev, config->dma_chan_cfg.rx1, &zephyr_rxc);
    if (result != 0) {
        return result;
    }

    result = dma_config(config->dma_dev, config->dma_chan_cfg.rx2, &zephyr_rxc);
    if (result != 0) {
        return result;
    }

    channel_config_set_read_increment(&rxc1, false);
    channel_config_set_write_increment(&rxc1, true);
    channel_config_set_dreq(&rxc1, pio_get_dreq(data->pio, data->pio_sm, false));
    channel_config_set_transfer_data_size(&rxc1, DMA_SIZE_8);
    channel_config_set_chain_to(&rxc1, config->dma_chan_cfg.rx2);

    dma_channel_configure(config->dma_chan_cfg.rx1,
            &rxc1, pio_cfg->sample_buf_1, &data->pio->rxf[data->pio_sm], bdata->rx_buf_len, false);

    channel_config_set_read_increment(&rxc2, false);
    channel_config_set_write_increment(&rxc2, true);
    channel_config_set_dreq(&rxc2, pio_get_dreq(data->pio, data->pio_sm, false));
    channel_config_set_transfer_data_size(&rxc2, DMA_SIZE_8);
    channel_config_set_chain_to(&rxc2, config->dma_chan_cfg.rx1);

    dma_channel_configure(config->dma_chan_cfg.rx2,
            &rxc2, pio_cfg->sample_buf_2, &data->pio->rxf[data->pio_sm], bdata->rx_buf_len, false);

    dma_irqn_set_channel_enabled(config->dma_chan_cfg.rx1 % 2, config->dma_chan_cfg.rx1, true);
    dma_irqn_set_channel_enabled(config->dma_chan_cfg.rx2 % 2, config->dma_chan_cfg.rx2, true);

    return 0;
}

int ads1x4s0x_pio_bulk_read_start(const struct device *dev)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;
    struct ads1x4s0x_pio_bulk_read_data *bdata = &data->bulk_read_data;
    enum ads1x4s0x_pio_register register_addresses[7];
    uint8_t values[ARRAY_SIZE(register_addresses)];
    int result;

    if (bdata->current_pio_cfg == NULL) {
        LOG_ERR("%s: bulk read hasn't been configured", dev->name);
        return -EINVAL;
    }

    if (data->bulk_read_sm_running) {
        LOG_ERR("%s: bulk read is running", dev->name);
        return -EINVAL;
    }
    gpio_pin_set_dt(&config->gpio_cs, GPIO_OUTPUT_ACTIVE);

    result = ads1x4s0x_pio_make_channel_setup(dev, &bdata->current_pio_cfg->chan_cfgs[0],
            register_addresses, values);

    if (result != 0) {
        return result;
    }

    result = ads1x4s0x_pio_write_multiple_registers(dev, register_addresses, values,
            ARRAY_SIZE(values));

    if (result != 0) {
        LOG_ERR("%s: unable to configure registers", dev->name);
        return result;
    }

    data->bulk_read_sm_running = true;

    dma_start_channel_mask((1u << config->dma_chan_cfg.rx1) | (1u << config->dma_chan_cfg.tx_ctrl1));

    return 0;
}

int ads1x4s0x_pio_bulk_read_stop(const struct device *dev)
{
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;

    if (!data->bulk_read_sm_running) {
        LOG_ERR("%s: bulk read is not running", dev->name);
        return -EINVAL;
    }

    hw_clear_bits(&dma_hw->ch[config->dma_chan_cfg.tx_ctrl1].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);

    while (dma_hw->ch[config->dma_chan_cfg.tx].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) {
        k_sleep(K_MSEC(1));
    }

    hw_clear_bits(&dma_hw->ch[config->dma_chan_cfg.rx1].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[config->dma_chan_cfg.rx2].ctrl_trig, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_set_bits(&dma_hw->abort, (1u << config->dma_chan_cfg.rx1) | (1u << config->dma_chan_cfg.rx2));

    gpio_pin_set_dt(&config->gpio_cs, GPIO_OUTPUT_INACTIVE);

    data->bulk_read_sm_running = false;

    return 0;
}

#ifdef CONFIG_ADC_ADS1X4S0X_PIO_GPIO
static int ads1x4s0x_pio_gpio_write_config(const struct device *dev)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    enum ads1x4s0x_pio_register register_addresses[2];
    uint8_t register_values[ARRAY_SIZE(register_addresses)];
    uint8_t gpio_dat = 0;
    uint8_t gpio_con = 0;

    ADS1X4S0X_REGISTER_GPIOCON_CON_SET(gpio_con, data->gpio_enabled);
    ADS1X4S0X_REGISTER_GPIODAT_DAT_SET(gpio_dat, data->gpio_value);
    ADS1X4S0X_REGISTER_GPIODAT_DIR_SET(gpio_dat, data->gpio_direction);

    register_values[0] = gpio_dat;
    register_values[1] = gpio_con;
    register_addresses[0] = ADS1X4S0X_REGISTER_GPIODAT;
    register_addresses[1] = ADS1X4S0X_REGISTER_GPIOCON;
    return ads1x4s0x_pio_write_multiple_registers(dev, register_addresses, register_values,
            ARRAY_SIZE(register_values));
}

static int ads1x4s0x_pio_gpio_write_value(const struct device *dev)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    uint8_t gpio_dat = 0;

    ADS1X4S0X_REGISTER_GPIODAT_DAT_SET(gpio_dat, data->gpio_value);
    ADS1X4S0X_REGISTER_GPIODAT_DIR_SET(gpio_dat, data->gpio_direction);

    return ads1x4s0x_pio_write_register(dev, ADS1X4S0X_REGISTER_GPIODAT, gpio_dat);
}

int ads1x4s0x_pio_gpio_set_output(const struct device *dev, uint8_t pin, bool initial_value)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    if (pin > ADS1X4S0X_GPIO_MAX) {
        LOG_ERR("%s: invalid pin %i", dev->name, pin);
        return -EINVAL;
    }

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    data->gpio_enabled |= BIT(pin);
    data->gpio_direction &= ~BIT(pin);

    if (initial_value) {
        data->gpio_value |= BIT(pin);
    } else {
        data->gpio_value &= ~BIT(pin);
    }

    result = ads1x4s0x_pio_gpio_write_config(dev);

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_set_input(const struct device *dev, uint8_t pin)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    if (pin > ADS1X4S0X_GPIO_MAX) {
        LOG_ERR("%s: invalid pin %i", dev->name, pin);
        return -EINVAL;
    }

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    data->gpio_enabled |= BIT(pin);
    data->gpio_direction |= BIT(pin);
    data->gpio_value &= ~BIT(pin);

    result = ads1x4s0x_pio_gpio_write_config(dev);

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_deconfigure(const struct device *dev, uint8_t pin)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    if (pin > ADS1X4S0X_GPIO_MAX) {
        LOG_ERR("%s: invalid pin %i", dev->name, pin);
        return -EINVAL;
    }

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    data->gpio_enabled &= ~BIT(pin);
    data->gpio_direction |= BIT(pin);
    data->gpio_value &= ~BIT(pin);

    result = ads1x4s0x_pio_gpio_write_config(dev);

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_set_pin_value(const struct device *dev, uint8_t pin, bool value)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    if (pin > ADS1X4S0X_GPIO_MAX) {
        LOG_ERR("%s: invalid pin %i", dev->name, pin);
        return -EINVAL;
    }

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    if ((BIT(pin) & data->gpio_enabled) == 0) {
        LOG_ERR("%s: gpio pin %i not configured", dev->name, pin);
        result = -EINVAL;
    } else if ((BIT(pin) & data->gpio_direction) != 0) {
        LOG_ERR("%s: gpio pin %i not configured as output", dev->name, pin);
        result = -EINVAL;
    } else {
        data->gpio_value |= BIT(pin);

        result = ads1x4s0x_pio_gpio_write_value(dev);
    }

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_get_pin_value(const struct device *dev, uint8_t pin, bool *value)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;
    uint8_t gpio_dat;

    if (pin > ADS1X4S0X_GPIO_MAX) {
        LOG_ERR("%s: invalid pin %i", dev->name, pin);
        return -EINVAL;
    }

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    if ((BIT(pin) & data->gpio_enabled) == 0) {
        LOG_ERR("%s: gpio pin %i not configured", dev->name, pin);
        result = -EINVAL;
    } else if ((BIT(pin) & data->gpio_direction) == 0) {
        LOG_ERR("%s: gpio pin %i not configured as input", dev->name, pin);
        result = -EINVAL;
    } else {
        result = ads1x4s0x_pio_read_register(dev, ADS1X4S0X_REGISTER_GPIODAT, &gpio_dat);
        data->gpio_value = ADS1X4S0X_REGISTER_GPIODAT_DAT_GET(gpio_dat);
        *value = (BIT(pin) & data->gpio_value) != 0;
    }

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;
    uint8_t gpio_dat;

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    result = ads1x4s0x_pio_read_register(dev, ADS1X4S0X_REGISTER_GPIODAT, &gpio_dat);
    data->gpio_value = ADS1X4S0X_REGISTER_GPIODAT_DAT_GET(gpio_dat);
    *value = data->gpio_value;

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
        gpio_port_value_t value)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    data->gpio_value = ((data->gpio_value & ~mask) | (mask & value)) & data->gpio_enabled &
        ~data->gpio_direction;
    result = ads1x4s0x_pio_gpio_write_value(dev);

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

int ads1x4s0x_pio_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
    struct ads1x4s0x_pio_data *data = dev->data;
    int result = 0;

    k_mutex_lock(&data->gpio_lock, K_FOREVER);

    data->gpio_value = (data->gpio_value ^ pins) & data->gpio_enabled & ~data->gpio_direction;
    result = ads1x4s0x_pio_gpio_write_value(dev);

    k_mutex_unlock(&data->gpio_lock);

    return result;
}

#endif /* CONFIG_ADC_ADS1X4S0X_GPIO */

static int ads1x4s0x_pio_init(const struct device *dev)
{
    uint8_t status = 0;
    uint8_t reference_control = 0;
    uint8_t reference_control_read;
    int result;
    const struct ads1x4s0x_pio_config *config = dev->config;
    struct ads1x4s0x_pio_data *data = dev->data;

    adc_context_init(&data->ctx);

    k_sem_init(&data->data_ready_signal, 0, 1);
    k_sem_init(&data->acquire_signal, 0, 1);

#ifdef CONFIG_ADC_ADS1X4S0X_GPIO
    k_mutex_init(&data->gpio_lock);
#endif /* CONFIG_ADC_ADS1X4S0X_GPIO */

    if (spi_pico_pio_init(dev) != 0) {
        LOG_ERR("%s: Failed to initialize SPI vio PIO", dev->name);
        return -ENODEV;
    }

    if (config->gpio_reset.port != NULL) {
        result = gpio_pin_configure_dt(&config->gpio_reset, GPIO_OUTPUT_ACTIVE);
        if (result != 0) {
            LOG_ERR("%s: failed to initialize GPIO for reset", dev->name);
            return result;
        }
    }

    if (config->gpio_start_sync.port != NULL) {
        result = gpio_pin_configure_dt(&config->gpio_start_sync, GPIO_OUTPUT_INACTIVE);
        if (result != 0) {
            LOG_ERR("%s: failed to initialize GPIO for start/sync", dev->name);
            return result;
        }
    }

    result = gpio_pin_configure_dt(&config->gpio_data_ready, GPIO_INPUT);
    if (result != 0) {
        LOG_ERR("%s: failed to initialize GPIO for data ready", dev->name);
        return result;
    }

    result = gpio_pin_interrupt_configure_dt(&config->gpio_data_ready, GPIO_INT_EDGE_TO_ACTIVE);
    if (result != 0) {
        LOG_ERR("%s: failed to configure data ready interrupt", dev->name);
        return -EIO;
    }

    gpio_init_callback(&data->callback_data_ready, ads1x4s0x_pio_data_ready_handler,
            BIT(config->gpio_data_ready.pin));
    result = gpio_add_callback(config->gpio_data_ready.port, &data->callback_data_ready);
    if (result != 0) {
        LOG_ERR("%s: failed to add data ready callback", dev->name);
        return -EIO;
    }

#if CONFIG_ADC_ASYNC
    k_tid_t tid = k_thread_create(&data->thread, config->stack,
            CONFIG_ADC_ADS1X4S0X_PIO_ACQUISITION_THREAD_STACK_SIZE,
            ads1x4s0x_pio_acquisition_thread, (void *)dev, NULL, NULL,
            CONFIG_ADC_ADS1X4S0X_PIO_ASYNC_THREAD_INIT_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(tid, "adc_ads1x4s0x_pio");
#endif

    k_busy_wait(ADS1X4S0X_POWER_ON_RESET_TIME_IN_US);

    if (config->gpio_reset.port == NULL) {
        result = ads1x4s0x_pio_send_command(dev, ADS1X4S0X_COMMAND_RESET);
        if (result != 0) {
            LOG_ERR("%s: unable to send RESET command", dev->name);
            return result;
        }
    } else {
        k_busy_wait(ADS1X4S0X_RESET_LOW_TIME_IN_US);
        gpio_pin_set_dt(&config->gpio_reset, 0);
    }

    k_busy_wait(ADS1X4S0X_RESET_DELAY_TIME_IN_US);

    result = ads1x4s0x_pio_read_register(dev, ADS1X4S0X_REGISTER_STATUS, &status);
    if (result != 0) {
        LOG_ERR("%s: unable to read status register", dev->name);
        return result;
    }

    if (ADS1X4S0X_REGISTER_STATUS_NOT_RDY_GET(status) == 0x01) {
        LOG_ERR("%s: ADS114 is not yet ready", dev->name);
        return -EBUSY;
    }

    /*
     * Activate internal voltage reference during initialization to
     * avoid the necessary setup time for it to settle later on.
     */
    ADS1X4S0X_REGISTER_REF_SET_DEFAULTS(reference_control);

    result = ads1x4s0x_pio_write_register(dev, ADS1X4S0X_REGISTER_REF, reference_control);
    if (result != 0) {
        LOG_ERR("%s: unable to set default reference control values", dev->name);
        return result;
    }

    /*
     * Ensure that the internal voltage reference is active.
     */
    result = ads1x4s0x_pio_read_register(dev, ADS1X4S0X_REGISTER_REF, &reference_control_read);
    if (result != 0) {
        LOG_ERR("%s: unable to read reference control values", dev->name);
        return result;
    }

    if (reference_control != reference_control_read) {
        LOG_ERR("%s: reference control register is incorrect: 0x%02X", dev->name,
                reference_control_read);
        return -EIO;
    }

#ifdef CONFIG_ADC_ADS1X4S0X_GPIO
    data->gpio_enabled = 0x00;
    data->gpio_direction = 0x0F;
    data->gpio_value = 0x00;

    result = ads1x4s0x_pio_gpio_write_config(dev);

    if (result != 0) {
        LOG_ERR("%s: unable to configure defaults for GPIOs", dev->name);
        return result;
    }
#endif

    k_sem_init(&data->bulk_read_data.bulk_data_ready_sem, 0, 1);

    adc_context_unlock_unconditionally(&data->ctx);

    return result;
}

static DEVICE_API(adc, api) = {
    .channel_setup = ads1x4s0x_pio_channel_setup,
    .read = ads1x4s0x_pio_read,
    .ref_internal = ADS1X4S0X_REF_INTERNAL,
#ifdef CONFIG_ADC_ASYNC
    .read_async = ads1x4s0x_pio_adc_read_async,
#endif
};
BUILD_ASSERT(CONFIG_ADC_INIT_PRIORITY > CONFIG_SPI_INIT_PRIORITY,
        "CONFIG_ADC_INIT_PRIORITY must be higher than CONFIG_SPI_INIT_PRIORITY");

#define ADC_ADS1X4S0X_INST_DEFINE(n, name, ch, res)                                              \
    IF_ENABLED(                                                                                  \
        CONFIG_ADC_ASYNC,                                                                        \
        (static K_KERNEL_STACK_DEFINE(                                                           \
            thread_stack_##name##_##n,                                                           \
            CONFIG_ADC_ADS1X4S0X_ACQUISITION_THREAD_STACK_SIZE);)                                \
    )                                                                                            \
    PINCTRL_DT_INST_DEFINE(n);                                                                   \
    static const struct ads1x4s0x_pio_config config_##name##_##n = {                             \
        IF_ENABLED(CONFIG_ADC_ASYNC, (.stack = thread_stack_##n,))                               \
        .piodev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                              \
        .pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                            \
        .gpio_clk = GPIO_DT_SPEC_INST_GET(n, clk_gpios),                                         \
        .gpio_mosi = GPIO_DT_SPEC_INST_GET_OR(n, mosi_gpios, {0}),                               \
        .gpio_miso = GPIO_DT_SPEC_INST_GET_OR(n, miso_gpios, {0}),                               \
        .gpio_cs = GPIO_DT_SPEC_INST_GET_OR(n, cs_gpios, {0}),                                   \
        .clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                        \
        .clk_id = (clock_control_subsys_t)DT_INST_PHA_BY_IDX(n, clocks, 0, clk_id),              \
        .spi_freq = DT_INST_PROP_OR(n, spi_frequency, MHZ(1)),                                   \
        .gpio_reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                             \
        .gpio_data_ready = GPIO_DT_SPEC_INST_GET(n, drdy_gpios),                                 \
        .gpio_start_sync = GPIO_DT_SPEC_INST_GET_OR(n, start_sync_gpios, {0}),                   \
        .idac_current = DT_INST_PROP(n, idac_current),                                           \
        .vbias_level = DT_INST_PROP(n, vbias_level),                                             \
        .vbias_level = DT_INST_PROP(n, vbias_level),                                             \
        .resolution = res,                                                                       \
        .channels = ch,                                                                          \
        .dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                              \
        .dma_chan_cfg = {                                                                        \
            .tx = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                                     \
            .tx_ctrl1 = DT_INST_DMAS_CELL_BY_NAME(n, txc1, channel),                             \
            .tx_ctrl2 = DT_INST_DMAS_CELL_BY_NAME(n, txc2, channel),                             \
            .rx1 = DT_INST_DMAS_CELL_BY_NAME(n, rx1, channel),                                   \
            .rx2 = DT_INST_DMAS_CELL_BY_NAME(n, rx2, channel),                                   \
        }                                                                                        \
    };                                                                                           \
    static struct ads1x4s0x_pio_data data_##name##_##n;                                          \
    DEVICE_DT_INST_DEFINE(n, ads1x4s0x_pio_init, NULL, &data_##name##_##n, &config_##name##_##n, \
                    POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &api);                                \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, clk_gpios));                                           \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, mosi_gpios));                                          \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, miso_gpios));

/*
 * ADS114S06: 16 bit, 6 channels
 */
#define DT_DRV_COMPAT ti_ads114s06_pio
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define ADC_ADS114S06_INST_DEFINE(n) ADC_ADS1X4S0X_INST_DEFINE(n, ti_ads114s06_pio, 6, 16)
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS114S06_INST_DEFINE);
#endif

/*
 * ADS114S08: 16 bit, 12 channels
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads114s08_pio
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define ADC_ADS114S08_INST_DEFINE(n) ADC_ADS1X4S0X_INST_DEFINE(n, ti_ads114s08_pio, 12, 16)
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS114S08_INST_DEFINE);
#endif

/*
 * ADS124S06: 24 bit, 6 channels
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads124s06_pio
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define ADC_ADS124S06_INST_DEFINE(n) ADC_ADS1X4S0X_INST_DEFINE(n, ti_ads124s06_pio, 6, 24)
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS124S06_INST_DEFINE);
#endif

/*
 * ADS124S08: 24 bit, 12 channels
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads124s08_pio
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define ADC_ADS124S08_INST_DEFINE(n) ADC_ADS1X4S0X_INST_DEFINE(n, ti_ads124s08_pio, 12, 24)
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS124S08_INST_DEFINE);
#endif
