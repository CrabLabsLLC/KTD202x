/*
 * Copyright (c) 2025 Makani Science
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * KTD2026/KTD2027 RGB LED Driver
 *
 * Features:
 *   - 3/4 channel constant current LED driver
 *   - 0-24mA per channel (0.125mA steps)
 *   - Hardware PWM blinking with configurable period/duty
 *   - Programmable rise/fall times for breathing effects
 *   - S-curve or linear ramp profiles
 */

#define DT_DRV_COMPAT kinetic_ktd202x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ktd202x.h"

LOG_MODULE_REGISTER(ktd202x, CONFIG_LED_LOG_LEVEL);

/* ============================================================================
 * Register Definitions
 * ============================================================================ */

#define KTD202X_REG_EN_RST       0x00
#define KTD202X_REG_FLASH_PERIOD 0x01
#define KTD202X_REG_PWM1_TIMER   0x02
#define KTD202X_REG_PWM2_TIMER   0x03
#define KTD202X_REG_LED_EN       0x04
#define KTD202X_REG_TRISE_TFALL  0x05
#define KTD202X_REG_LED1         0x06
#define KTD202X_REG_LED2         0x07
#define KTD202X_REG_LED3         0x08
#define KTD202X_REG_LED4         0x09

/* EN_RST Register (0x00) bits */
#define KTD202X_EN_RST_RESET           0x07  /* Write to reset chip */
#define KTD202X_EN_RST_ENABLE_MASK     0x03  /* Bits [1:0]: Enable control */
#define KTD202X_EN_RST_ENABLE_OFF      0x00  /*   00 = Chip off */
#define KTD202X_EN_RST_ENABLE_NIGHT    0x01  /*   01 = Night mode (lower current) */
#define KTD202X_EN_RST_ENABLE_NORMAL   0x02  /*   10 = Normal operation */
#define KTD202X_EN_RST_TSLOT_MASK      0x1C  /* Bits [4:2]: Timer slot */
#define KTD202X_EN_RST_TSLOT_SHIFT     2
#define KTD202X_EN_RST_TSCALE_MASK     0x60  /* Bits [6:5]: Time scale for rise/fall */
#define KTD202X_EN_RST_TSCALE_SHIFT    5
#define KTD202X_EN_RST_RAMP_LOG        0x00  /* Bit 7 = 0: S-curve (logarithmic) */
#define KTD202X_EN_RST_RAMP_LINEAR     0x80  /* Bit 7 = 1: Linear ramp */

/* FLASH_PERIOD Register (0x01) bits */
#define KTD202X_FLASH_PERIOD_MASK      0x7F  /* Bits [6:0]: Period value */
#define KTD202X_FLASH_RAMP_TYPE_BIT    0x80  /* Bit 7: Ramp type (also in Reg0) */

/* LED Mode bits in LED_EN register (0x04) */
#define KTD202X_LED_MODE_OFF           0x00
#define KTD202X_LED_MODE_ON            0x01
#define KTD202X_LED_MODE_PWM1          0x02
#define KTD202X_LED_MODE_PWM2          0x03
#define KTD202X_LED_MODE_MASK          0x03

/* Current limits */
#define KTD202X_MAX_CURRENT_REG        0xBF  /* 191 = 24mA (0.125mA * 192) */
#define KTD202X_MAX_CHANNELS           4
#define KTD202X_DEFAULT_NUM_CHANNELS   3

/* Timing constants */
#define KTD202X_RESET_DELAY_US         200
#define KTD202X_INIT_RETRY_COUNT       3
#define KTD202X_RETRY_DELAY_MS         10

/* Rise/fall time: value * 96ms, 0 = 1.5ms minimum */
#define KTD202X_RAMP_TIME_STEP_MS      96
#define KTD202X_RAMP_TIME_MIN_MS       2     /* ~1.5ms when trise=0 */
#define KTD202X_RAMP_TIME_MAX_VALUE    15    /* 4-bit field max */

/* Flash period: 256ms + (value * 128ms), range 256ms to 16.4s */
#define KTD202X_FLASH_PERIOD_BASE_MS   256
#define KTD202X_FLASH_PERIOD_STEP_MS   128
#define KTD202X_FLASH_PERIOD_MIN_MS    128

/* ============================================================================
 * Kconfig-derived constants
 * ============================================================================ */

/* Max current from Kconfig (mA) -> register value */
/* Current = (reg + 1) * 0.125mA, so reg = (mA / 0.125) - 1 = mA * 8 - 1 */
#define KTD202X_MAX_BRIGHTNESS \
	MIN(((CONFIG_KTD202X_MAX_CURRENT_MA * 8) - 1), KTD202X_MAX_CURRENT_REG)

/* Default rise/fall time from Kconfig -> register value */
#define KTD202X_DEFAULT_TRISE \
	MIN((CONFIG_KTD202X_DEFAULT_RISE_TIME_MS / KTD202X_RAMP_TIME_STEP_MS), KTD202X_RAMP_TIME_MAX_VALUE)

#define KTD202X_DEFAULT_TFALL \
	MIN((CONFIG_KTD202X_DEFAULT_FALL_TIME_MS / KTD202X_RAMP_TIME_STEP_MS), KTD202X_RAMP_TIME_MAX_VALUE)

/* Ramp type from Kconfig */
#if IS_ENABLED(CONFIG_KTD202X_RAMP_LINEAR)
#define KTD202X_RAMP_TYPE KTD202X_EN_RST_RAMP_LINEAR
#else
#define KTD202X_RAMP_TYPE KTD202X_EN_RST_RAMP_LOG
#endif

/* ============================================================================
 * Data Structures
 * ============================================================================ */

struct ktd202x_config
{
	struct i2c_dt_spec i2c;
	const struct device *vin_supply;
	uint8_t num_leds;
	const struct led_info *leds_info;
	const uint8_t *channel_map;
	uint8_t num_channels;
};

struct ktd202x_data
{
	uint8_t led_enable_register;  /* Cached LED_EN register value */
	uint8_t en_rst_register;      /* Cached EN_RST register value */
	uint8_t trise_tfall_register; /* Cached TRISE_TFALL register value */
};

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

static const struct led_info *ktd202x_led_to_info(const struct ktd202x_config *const config,
						  const uint32_t led_index)
{
	if (led_index < config->num_leds)
	{
		return &config->leds_info[led_index];
	}
	return NULL;
}

static inline uint8_t ktd202x_led_mode_shift(const uint32_t channel)
{
	return (channel * 2);
}

static inline uint8_t ktd202x_map_channel(const struct ktd202x_config *const config,
					  const uint8_t color_index)
{
	uint8_t channel = color_index;

	if (config->channel_map != NULL && color_index < config->num_channels)
	{
		channel = config->channel_map[color_index];
	}

	if (channel >= KTD202X_MAX_CHANNELS)
	{
		LOG_WRN("Channel %u out of range, clamping to %u", channel, KTD202X_MAX_CHANNELS - 1);
		channel = KTD202X_MAX_CHANNELS - 1;
	}

	return channel;
}

/* Convert 0-100 brightness percentage to register value, capped by max current */
static inline uint8_t ktd202x_brightness_to_current(const uint8_t brightness)
{
	return (brightness * KTD202X_MAX_BRIGHTNESS) / 100U;
}

/* Convert 0-255 color value to register value, capped by max current */
static inline uint8_t ktd202x_color_to_current(const uint8_t color)
{
	return (color * KTD202X_MAX_BRIGHTNESS) / 255U;
}

/* Calculate flash period register value from period in ms */
static int ktd202x_calc_flash_period(const uint32_t period_ms, uint8_t *const register_value)
{
	if (period_ms < KTD202X_FLASH_PERIOD_MIN_MS)
	{
		return -EINVAL;
	}
	if (period_ms < KTD202X_FLASH_PERIOD_BASE_MS)
	{
		*register_value = 0;
		return 0;
	}
	const uint32_t calculated_value = (period_ms - KTD202X_FLASH_PERIOD_BASE_MS) / KTD202X_FLASH_PERIOD_STEP_MS;
	*register_value = (uint8_t)MIN(calculated_value, KTD202X_FLASH_PERIOD_MASK);
	return 0;
}

/* Calculate PWM duty cycle register value */
static uint8_t ktd202x_calc_pwm_duty(const uint32_t delay_on, const uint32_t period)
{
	if (period == 0)
	{
		return 0;
	}
	return (uint8_t)((delay_on * 256U) / period);
}

/* Convert time in ms to rise/fall register value (0-15) */
static uint8_t ktd202x_time_to_ramp_value(uint32_t time_ms)
{
	if (time_ms < KTD202X_RAMP_TIME_MIN_MS)
	{
		return 0;
	}
	uint32_t value = time_ms / KTD202X_RAMP_TIME_STEP_MS;
	return (uint8_t)MIN(value, KTD202X_RAMP_TIME_MAX_VALUE);
}

/* ============================================================================
 * LED API Implementation
 * ============================================================================ */

static int ktd202x_get_info(const struct device *const dev,
			    const uint32_t led_index,
			    const struct led_info **info)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -EINVAL;
	}

	*info = led_info;
	return 0;
}

static int ktd202x_set_brightness(const struct device *const dev,
				  const uint32_t led_index,
				  const uint8_t brightness)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL || led_info->num_colors != 1)
	{
		return -ENOTSUP;
	}

	struct ktd202x_data *const data = dev->data;
	const uint8_t current_value = ktd202x_brightness_to_current(brightness);
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, current_value);
	if (ret < 0)
	{
		return ret;
	}

	const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
	const uint8_t mode = (brightness > 0) ? KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

	data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	data->led_enable_register |= (mode << shift);

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

static int ktd202x_set_color(const struct device *const dev,
			     const uint32_t led_index,
			     const uint8_t num_colors,
			     const uint8_t *const color)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL || color == NULL || led_info->num_colors != num_colors)
	{
		return -EINVAL;
	}

	struct ktd202x_data *const data = dev->data;
	uint8_t led_enable_value = data->led_enable_register;

	for (uint8_t color_index = 0; color_index < num_colors; color_index++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + color_index);
		const uint8_t led_current = ktd202x_color_to_current(color[color_index]);

		int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, led_current);
		if (ret < 0)
		{
			return ret;
		}

		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		const uint8_t mode = (color[color_index] > 0) ? KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

		led_enable_value &= ~(KTD202X_LED_MODE_MASK << shift);
		led_enable_value |= (mode << shift);
	}

	data->led_enable_register = led_enable_value;
	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, led_enable_value);
}

static int ktd202x_on(const struct device *const dev, const uint32_t led_index)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;

	for (uint8_t i = 0; i < led_info->num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, KTD202X_MAX_BRIGHTNESS);
		if (ret < 0)
		{
			return ret;
		}
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		data->led_enable_register |= (KTD202X_LED_MODE_ON << shift);
	}

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

static int ktd202x_off(const struct device *const dev, const uint32_t led_index)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;

	for (uint8_t i = 0; i < led_info->num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	}

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

static int ktd202x_blink(const struct device *const dev,
			 const uint32_t led_index,
			 const uint32_t delay_on,
			 const uint32_t delay_off)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;
	const uint32_t period = delay_on + delay_off;
	uint8_t flash_period_register;

	if (ktd202x_calc_flash_period(period, &flash_period_register) < 0)
	{
		return -EINVAL;
	}

	if (i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_FLASH_PERIOD, flash_period_register) < 0)
	{
		return -EIO;
	}

	const uint8_t pwm_duty_cycle = ktd202x_calc_pwm_duty(delay_on, period);
	if (i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, pwm_duty_cycle) < 0)
	{
		return -EIO;
	}

	for (uint8_t i = 0; i < led_info->num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		data->led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);
	}

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

/* ============================================================================
 * Extended API (Breathing/Fade Effects)
 * ============================================================================ */

int ktd202x_set_fade_time(const struct device *dev, uint32_t rise_ms, uint32_t fall_ms)
{
	if (!dev)
	{
		return -EINVAL;
	}

	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;

	uint8_t trise = ktd202x_time_to_ramp_value(rise_ms);
	uint8_t tfall = ktd202x_time_to_ramp_value(fall_ms);

	data->trise_tfall_register = (tfall << 4) | trise;

	LOG_DBG("Set fade time: rise=%ums (reg=%u), fall=%ums (reg=%u)",
		rise_ms, trise, fall_ms, tfall);

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_TRISE_TFALL, data->trise_tfall_register);
}

int ktd202x_set_time_scale(const struct device *dev, ktd202x_time_scale_t scale)
{
	if (!dev || scale > KTD202X_TSCALE_8X)
	{
		return -EINVAL;
	}

	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;

	data->en_rst_register &= ~KTD202X_EN_RST_TSCALE_MASK;
	data->en_rst_register |= ((uint8_t)scale << KTD202X_EN_RST_TSCALE_SHIFT);

	LOG_DBG("Set time scale: %ux", 1 << scale);

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, data->en_rst_register);
}

int ktd202x_set_ramp_linear(const struct device *dev, bool linear)
{
	if (!dev)
	{
		return -EINVAL;
	}

	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;

	if (linear)
	{
		data->en_rst_register |= KTD202X_EN_RST_RAMP_LINEAR;
	}
	else
	{
		data->en_rst_register &= ~KTD202X_EN_RST_RAMP_LINEAR;
	}

	LOG_DBG("Set ramp type: %s", linear ? "linear" : "S-curve");

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, data->en_rst_register);
}

int ktd202x_breathe(const struct device *dev, uint32_t led_index,
		    uint32_t period_ms, uint8_t brightness)
{
	if (!dev)
	{
		return -EINVAL;
	}

	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;
	int ret;

	/* Set brightness for all channels in this LED */
	const uint8_t current_value = ktd202x_brightness_to_current(brightness);
	for (uint8_t i = 0; i < led_info->num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, current_value);
		if (ret < 0)
		{
			return ret;
		}
	}

	/* Configure flash period */
	uint8_t flash_period_reg;
	if (ktd202x_calc_flash_period(period_ms, &flash_period_reg) < 0)
	{
		return -EINVAL;
	}
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_FLASH_PERIOD, flash_period_reg);
	if (ret < 0)
	{
		return ret;
	}

	/* Set 50% duty cycle for breathing (equal on/off time) */
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, 128);
	if (ret < 0)
	{
		return ret;
	}

	/* Enable PWM mode for all channels in this LED */
	for (uint8_t i = 0; i < led_info->num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		data->led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);
	}

	LOG_DBG("Breathe LED %u: period=%ums, brightness=%u%%", led_index, period_ms, brightness);

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

int ktd202x_breathe_color(const struct device *dev, uint32_t led_index,
			  uint32_t period_ms, const uint8_t *color, uint8_t num_colors)
{
	if (!dev || !color)
	{
		return -EINVAL;
	}

	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL || led_info->num_colors != num_colors)
	{
		return -EINVAL;
	}

	struct ktd202x_data *const data = dev->data;
	int ret;

	/* Set color for each channel */
	for (uint8_t i = 0; i < num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t current_value = ktd202x_color_to_current(color[i]);
		ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, current_value);
		if (ret < 0)
		{
			return ret;
		}
	}

	/* Configure flash period */
	uint8_t flash_period_reg;
	if (ktd202x_calc_flash_period(period_ms, &flash_period_reg) < 0)
	{
		return -EINVAL;
	}
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_FLASH_PERIOD, flash_period_reg);
	if (ret < 0)
	{
		return ret;
	}

	/* Set 50% duty cycle for breathing */
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, 128);
	if (ret < 0)
	{
		return ret;
	}

	/* Enable PWM mode for all channels */
	for (uint8_t i = 0; i < num_colors; i++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		data->led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);
	}

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

uint8_t ktd202x_get_max_brightness(void)
{
	return KTD202X_MAX_BRIGHTNESS;
}

uint8_t ktd202x_get_max_current_ma(void)
{
	return CONFIG_KTD202X_MAX_CURRENT_MA;
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

static int ktd202x_init(const struct device *const dev)
{
	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;
	int ret;

	LOG_INF(">>> KTD202x init starting (max current: %umA)", CONFIG_KTD202X_MAX_CURRENT_MA);

	/* Check I2C bus is ready */
	if (!i2c_is_ready_dt(&config->i2c))
	{
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}
	LOG_INF(">>> I2C bus ready");

	/* Enable VIN supply if configured */
	if (config->vin_supply)
	{
		LOG_INF(">>> Checking VIN supply...");
		if (!device_is_ready(config->vin_supply))
		{
			LOG_ERR("VIN regulator not ready");
			return -ENODEV;
		}
		LOG_INF(">>> Enabling VIN supply...");
		ret = regulator_enable(config->vin_supply);
		if (ret < 0)
		{
			LOG_ERR("Failed to enable VIN regulator: %d", ret);
			return ret;
		}
		/* Increased from 5ms to 100ms for power stabilization */
		LOG_INF(">>> VIN enabled, waiting 100ms for stabilization...");
		k_msleep(100);
	}
	else
	{
		LOG_WRN(">>> No VIN supply configured");
	}

	/* Reset the device - retry a few times if I2C fails initially */
	LOG_INF(">>> Attempting device reset via I2C...");
	for (int attempt = 0; attempt < KTD202X_INIT_RETRY_COUNT; attempt++)
	{
		LOG_INF(">>> Reset attempt %d/%d", attempt + 1, KTD202X_INIT_RETRY_COUNT);
		ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, KTD202X_EN_RST_RESET);
		if (ret == 0)
		{
			LOG_INF(">>> Reset succeeded on attempt %d", attempt + 1);
			break;
		}
		LOG_WRN(">>> Reset attempt %d failed: %d", attempt + 1, ret);
		k_msleep(KTD202X_RETRY_DELAY_MS);
	}

	if (ret < 0)
	{
		LOG_ERR("Failed to reset device after %d attempts: %d", KTD202X_INIT_RETRY_COUNT, ret);
		return ret;
	}

	LOG_INF(">>> Waiting %uus after reset...", KTD202X_RESET_DELAY_US);
	k_usleep(KTD202X_RESET_DELAY_US);

	/* Configure EN_RST: normal mode, default time scale, ramp type from Kconfig */
	LOG_INF(">>> Configuring EN_RST register...");
	data->en_rst_register = KTD202X_EN_RST_ENABLE_NORMAL |
				(0x02 << KTD202X_EN_RST_TSLOT_SHIFT) | /* 64ms timer slot */
				KTD202X_RAMP_TYPE;

	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, data->en_rst_register);
	if (ret < 0)
	{
		LOG_ERR("Failed to enable chip: %d", ret);
		return ret;
	}
	LOG_INF(">>> EN_RST configured (0x%02X)", data->en_rst_register);

	/* Set default rise/fall times from Kconfig */
	LOG_INF(">>> Setting rise/fall times...");
	data->trise_tfall_register = (KTD202X_DEFAULT_TFALL << 4) | KTD202X_DEFAULT_TRISE;
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_TRISE_TFALL, data->trise_tfall_register);
	if (ret < 0)
	{
		LOG_ERR("Failed to set rise/fall times: %d", ret);
		return ret;
	}
	LOG_INF(">>> Rise/fall times set (0x%02X)", data->trise_tfall_register);

	/* All LEDs off initially */
	LOG_INF(">>> Disabling all LEDs...");
	data->led_enable_register = 0x00;
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, 0x00);
	if (ret < 0)
	{
		LOG_ERR("Failed to disable LEDs: %d", ret);
		return ret;
	}

	LOG_INF(">>> KTD202x init complete! (max %umA, rise=%ums, fall=%ums, %s ramp)",
		CONFIG_KTD202X_MAX_CURRENT_MA,
		CONFIG_KTD202X_DEFAULT_RISE_TIME_MS,
		CONFIG_KTD202X_DEFAULT_FALL_TIME_MS,
		IS_ENABLED(CONFIG_KTD202X_RAMP_LINEAR) ? "linear" : "S-curve");

	return 0;
}

/* ============================================================================
 * Device Tree Instantiation
 * ============================================================================ */

static DEVICE_API(led, ktd202x_led_api) = {
	.on = ktd202x_on,
	.off = ktd202x_off,
	.set_brightness = ktd202x_set_brightness,
	.set_color = ktd202x_set_color,
	.blink = ktd202x_blink,
	.get_info = ktd202x_get_info,
};

#define COLOR_MAPPING(led_node_id) \
	static const uint8_t DT_CAT(color_mapping_, led_node_id)[] = \
		DT_PROP(led_node_id, color_mapping);

#define LED_INFO(led_node_id) \
	{ \
		.label = DT_PROP(led_node_id, label), \
		.index = DT_PROP(led_node_id, reg), \
		.num_colors = DT_PROP_LEN(led_node_id, color_mapping), \
		.color_mapping = DT_CAT(color_mapping_, led_node_id), \
	},

#define KTD202X_DEFINE(inst) \
	DT_INST_FOREACH_CHILD(inst, COLOR_MAPPING) \
	static const struct led_info DT_CAT(ktd202x_leds_, inst)[] = { \
		DT_INST_FOREACH_CHILD(inst, LED_INFO) \
	}; \
	static struct ktd202x_data DT_CAT(ktd202x_data_, inst); \
	static const struct ktd202x_config DT_CAT(ktd202x_config_, inst) = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.vin_supply = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, vin_supply), \
			(DEVICE_DT_GET(DT_INST_PHANDLE(inst, vin_supply))), (NULL)), \
		.num_leds = ARRAY_SIZE(DT_CAT(ktd202x_leds_, inst)), \
		.leds_info = DT_CAT(ktd202x_leds_, inst), \
		.channel_map = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, channel_map), \
			(DT_INST_PROP(inst, channel_map)), (NULL)), \
		.num_channels = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, channel_map), \
			(DT_INST_PROP_LEN(inst, channel_map)), (KTD202X_DEFAULT_NUM_CHANNELS)), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, ktd202x_init, NULL, \
			      &DT_CAT(ktd202x_data_, inst), \
			      &DT_CAT(ktd202x_config_, inst), \
			      POST_KERNEL, CONFIG_LED_INIT_PRIORITY, \
			      &ktd202x_led_api);

DT_INST_FOREACH_STATUS_OKAY(KTD202X_DEFINE)
