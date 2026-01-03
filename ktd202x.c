/*
 * Copyright (c) 2025 Makani Science
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kinetic_ktd202x

/**
 * @file
 * @brief KTD202x RGB LED driver
 *
 * The KTD2026/KTD2027 are 3/4-channel constant current RGB/RGBW LED drivers
 * with I2C control interface. They support individual LED brightness control,
 * hardware PWM blinking, and programmable rise/fall times.
 *
 * Datasheet: https://www.kinet-ic.com/uploads/KTD2026-7-04h.pdf
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ktd202x, CONFIG_LED_LOG_LEVEL);

/* KTD202x Register Addresses */
#define KTD202X_REG_EN_RST       0x00  /* Enable/Reset control */
#define KTD202X_REG_FLASH_PERIOD 0x01  /* Flash period */
#define KTD202X_REG_PWM1_TIMER   0x02  /* PWM1 duty cycle (on-time percentage) */
#define KTD202X_REG_PWM2_TIMER   0x03  /* PWM2 duty cycle (on-time percentage) */
#define KTD202X_REG_LED_EN       0x04  /* LED channel enable/mode */
#define KTD202X_REG_TRISE_TFALL  0x05  /* Rise/fall time configuration */
#define KTD202X_REG_LED1         0x06  /* LED1 current (brightness) */
#define KTD202X_REG_LED2         0x07  /* LED2 current (brightness) */
#define KTD202X_REG_LED3         0x08  /* LED3 current (brightness) */
#define KTD202X_REG_LED4         0x09  /* LED4 current (KTD2027 only) */

/* EN_RST Register (0x00) bits */
#define KTD202X_EN_RST_RESET     0x07  /* Write to reset all registers */
#define KTD202X_EN_RST_TCTRL_MASK   0x07  /* Timer slot control mask */
#define KTD202X_EN_RST_ENABLE_MASK  0x18  /* Enable control mask */
#define KTD202X_EN_RST_ENABLE_SHIFT 3

/* Enable control modes */
#define KTD202X_ENABLE_OFF          0x00  /* Chip disabled (shutdown) */
#define KTD202X_ENABLE_NORMAL       0x01  /* Normal operation (SCL high, SDA toggle wake) */
#define KTD202X_ENABLE_ALWAYS_ON    0x02  /* Always on (no wake needed) */

/* LED_EN Register (0x04) - LED channel mode selection */
#define KTD202X_LED_MODE_OFF     0x00  /* LED off */
#define KTD202X_LED_MODE_ON      0x01  /* LED always on */
#define KTD202X_LED_MODE_PWM1    0x02  /* LED controlled by PWM1 */
#define KTD202X_LED_MODE_PWM2    0x03  /* LED controlled by PWM2 */

#define KTD202X_LED_MODE_MASK    0x03

/* TRISE_TFALL Register (0x05) */
#define KTD202X_TRISE_MASK       0x0F
#define KTD202X_TFALL_MASK       0xF0
#define KTD202X_TFALL_SHIFT      4
#define KTD202X_LOG_SCALE_BIT    0x80  /* In FLASH_PERIOD reg: 0=log, 1=linear */

/* LED current range: 0-191 maps to 0-24mA (0.125mA steps) */
#define KTD202X_MAX_BRIGHTNESS   191   /* 24mA */
#define KTD202X_MIN_BRIGHTNESS   0

/* Flash period: 0=128ms, else (N * 128ms) + 256ms, max ~16s */
#define KTD202X_FLASH_PERIOD_MIN_MS   128
#define KTD202X_FLASH_PERIOD_STEP_MS  128
#define KTD202X_FLASH_PERIOD_BASE_MS  256
#define KTD202X_FLASH_PERIOD_MAX      127  /* Register max value */

/* Maximum number of LED channels */
#define KTD202X_MAX_CHANNELS     4  /* KTD2027 has 4, KTD2026 has 3 */
#define KTD202X_DEFAULT_NUM_CHANNELS 3  /* Default for KTD2026 */

struct ktd202x_config
{
	struct i2c_dt_spec i2c;
	uint8_t num_leds;
	const struct led_info *leds_info;
	/* Channel mapping: maps color index to hardware channel */
	/* e.g., for RBG wiring: channel_map = {0, 2, 1} means R->LED1, B->LED2, G->LED3 */
	const uint8_t *channel_map;
	uint8_t num_channels;
};

struct ktd202x_data
{
	uint8_t led_enable_register;  /* Shadow of LED_EN register */
};

/*
 * Helper: Get LED info by index
 */
static const struct led_info *ktd202x_led_to_info(const struct ktd202x_config *const config,
						  const uint32_t led_index)
{
	if (led_index < config->num_leds)
	{
		return &config->leds_info[led_index];
	}
	return NULL;
}

/*
 * Helper: Get shift amount for LED mode in LED_EN register
 */
static inline uint8_t ktd202x_led_mode_shift(const uint32_t channel)
{
	return (channel * 2);
}

/*
 * Helper: Map color index to hardware channel
 */
static inline uint8_t ktd202x_map_channel(const struct ktd202x_config *const config,
					  const uint8_t color_index)
{
	if (config->channel_map != NULL && color_index < config->num_channels)
	{
		return config->channel_map[color_index];
	}
	/* Default: direct mapping (RGB -> LED1, LED2, LED3) */
	return color_index;
}

/*
 * Helper: Convert Zephyr brightness (0-100) to KTD202x current (0-191)
 */
static inline uint8_t ktd202x_brightness_to_current(const uint8_t brightness)
{
	return (brightness * KTD202X_MAX_BRIGHTNESS) / LED_BRIGTHNESS_MAX;
}

/*
 * Helper: Calculate flash period register value from delay_on + delay_off (ms)
 *
 * Flash period formula from datasheet:
 *   reg_val = 0: period = 128ms
 *   reg_val > 0: period = (reg_val * 128) + 256 ms
 */
static int ktd202x_calc_flash_period(const uint32_t period_ms, uint8_t *const register_value)
{
	if (period_ms < KTD202X_FLASH_PERIOD_MIN_MS)
	{
		return -EINVAL;
	}

	/* Special case: 128ms minimum period */
	if (period_ms < KTD202X_FLASH_PERIOD_BASE_MS)
	{
		*register_value = 0;
		return 0;
	}

	/* period = (reg_val * 128) + 256 ms, solve for reg_val */
	const uint32_t calculated_value =
		(period_ms - KTD202X_FLASH_PERIOD_BASE_MS) / KTD202X_FLASH_PERIOD_STEP_MS;
	const uint32_t clamped_value = MIN(calculated_value, KTD202X_FLASH_PERIOD_MAX);

	*register_value = (uint8_t)clamped_value;
	return 0;
}

/*
 * Helper: Calculate PWM duty cycle register value
 * PWM duty = on_time / period = reg_val / 256
 */
static uint8_t ktd202x_calc_pwm_duty(const uint32_t delay_on, const uint32_t period)
{
	if (period == 0)
	{
		return 0;
	}
	return (uint8_t)((delay_on * 256U) / period);
}

/*
 * API: Get LED information
 */
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

/*
 * API: Set LED brightness (single channel)
 */
static int ktd202x_set_brightness(const struct device *const dev,
				  const uint32_t led_index,
				  const uint8_t brightness)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	/* For multi-color LEDs, this controls overall intensity - use set_color for RGB */
	if (led_info->num_colors != 1)
	{
		return -ENOTSUP;
	}

	struct ktd202x_data *const data = dev->data;
	const uint8_t current_value = ktd202x_brightness_to_current(brightness);
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	/* Set LED current register */
	const int write_result = i2c_reg_write_byte_dt(&config->i2c,
						       KTD202X_REG_LED1 + hardware_channel,
						       current_value);
	if (write_result < 0)
	{
		LOG_ERR("Failed to write LED%d current: %d", hardware_channel + 1, write_result);
		return write_result;
	}

	/* Update LED mode to ON (constant current) if brightness > 0, else OFF */
	const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
	const uint8_t mode = (brightness > 0) ? KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

	data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	data->led_enable_register |= (mode << shift);

	const int enable_result = i2c_reg_write_byte_dt(&config->i2c,
							KTD202X_REG_LED_EN,
							data->led_enable_register);
	if (enable_result < 0)
	{
		LOG_ERR("Failed to write LED_EN: %d", enable_result);
		return enable_result;
	}

	return 0;
}

/*
 * API: Set RGB color (for multi-color LED)
 *
 * The color array follows the order specified in color-mapping property.
 * The channel_map property controls which hardware channel each color goes to.
 *
 * Example: If color-mapping = <RED, GREEN, BLUE> and channel-map = <0, 2, 1>
 *          Then: Red->LED1 (ch0), Green->LED3 (ch2), Blue->LED2 (ch1)
 */
static int ktd202x_set_color(const struct device *const dev,
			     const uint32_t led_index,
			     const uint8_t num_colors,
			     const uint8_t *const color)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	if (color == NULL)
	{
		return -EINVAL;
	}

	if (led_info->num_colors != num_colors)
	{
		LOG_ERR("Color count mismatch: expected %d, got %d",
			led_info->num_colors, num_colors);
		return -EINVAL;
	}

	if (num_colors > KTD202X_MAX_CHANNELS)
	{
		return -EINVAL;
	}

	struct ktd202x_data *const data = dev->data;
	uint8_t led_enable_value = data->led_enable_register;

	/* Write current values for each color channel, respecting channel mapping */
	for (uint8_t color_index = 0; color_index < num_colors; color_index++)
	{
		const uint8_t hardware_channel = ktd202x_map_channel(config, color_index);

		/* Convert 0-255 color to 0-191 current */
		const uint8_t led_current = (color[color_index] * KTD202X_MAX_BRIGHTNESS) / 255U;

		const int write_result = i2c_reg_write_byte_dt(&config->i2c,
							       KTD202X_REG_LED1 + hardware_channel,
							       led_current);
		if (write_result < 0)
		{
			LOG_ERR("Failed to write LED%d current: %d",
				hardware_channel + 1, write_result);
			return write_result;
		}

		/* Update mode for this channel */
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		const uint8_t mode = (color[color_index] > 0) ?
			KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

		led_enable_value &= ~(KTD202X_LED_MODE_MASK << shift);
		led_enable_value |= (mode << shift);
	}

	/* Update LED enable register */
	data->led_enable_register = led_enable_value;
	const int enable_result = i2c_reg_write_byte_dt(&config->i2c,
							KTD202X_REG_LED_EN,
							led_enable_value);
	if (enable_result < 0)
	{
		LOG_ERR("Failed to write LED_EN: %d", enable_result);
		return enable_result;
	}

	return 0;
}

/*
 * API: Turn LED on (full brightness)
 */
static int ktd202x_on(const struct device *const dev, const uint32_t led_index)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	/* Set full current */
	const int current_result = i2c_reg_write_byte_dt(&config->i2c,
							 KTD202X_REG_LED1 + hardware_channel,
							 KTD202X_MAX_BRIGHTNESS);
	if (current_result < 0)
	{
		LOG_ERR("Failed to write LED%d current: %d", hardware_channel + 1, current_result);
		return current_result;
	}

	/* Set mode to ON */
	const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
	data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	data->led_enable_register |= (KTD202X_LED_MODE_ON << shift);

	const int enable_result = i2c_reg_write_byte_dt(&config->i2c,
							KTD202X_REG_LED_EN,
							data->led_enable_register);
	if (enable_result < 0)
	{
		LOG_ERR("Failed to write LED_EN: %d", enable_result);
		return enable_result;
	}

	return 0;
}

/*
 * API: Turn LED off
 */
static int ktd202x_off(const struct device *const dev, const uint32_t led_index)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL)
	{
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	/* Set mode to OFF */
	const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
	data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	/* Mode OFF is 0, so no need to OR anything */

	const int write_result = i2c_reg_write_byte_dt(&config->i2c,
						       KTD202X_REG_LED_EN,
						       data->led_enable_register);
	if (write_result < 0)
	{
		LOG_ERR("Failed to write LED_EN: %d", write_result);
		return write_result;
	}

	return 0;
}

/*
 * API: Blink LED using hardware PWM
 */
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
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	/* Calculate and set flash period */
	uint8_t flash_period_register;
	const int period_result = ktd202x_calc_flash_period(period, &flash_period_register);
	if (period_result < 0)
	{
		LOG_ERR("Invalid blink period: %u ms", period);
		return period_result;
	}

	int write_result = i2c_reg_write_byte_dt(&config->i2c,
						 KTD202X_REG_FLASH_PERIOD,
						 flash_period_register);
	if (write_result < 0)
	{
		LOG_ERR("Failed to write FLASH_PERIOD: %d", write_result);
		return write_result;
	}

	/* Calculate and set PWM1 duty cycle */
	const uint8_t pwm_duty_cycle = ktd202x_calc_pwm_duty(delay_on, period);
	write_result = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, pwm_duty_cycle);
	if (write_result < 0)
	{
		LOG_ERR("Failed to write PWM1_TIMER: %d", write_result);
		return write_result;
	}

	/* Set LED mode to PWM1 */
	const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
	data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	data->led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);

	write_result = i2c_reg_write_byte_dt(&config->i2c,
					     KTD202X_REG_LED_EN,
					     data->led_enable_register);
	if (write_result < 0)
	{
		LOG_ERR("Failed to write LED_EN: %d", write_result);
		return write_result;
	}

	return 0;
}

/*
 * Device initialization
 */
static int ktd202x_init(const struct device *const dev)
{
	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;

	if (!i2c_is_ready_dt(&config->i2c))
	{
		LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	/* Reset the device */
	int write_result = i2c_reg_write_byte_dt(&config->i2c,
						 KTD202X_REG_EN_RST,
						 KTD202X_EN_RST_RESET);
	if (write_result < 0)
	{
		LOG_ERR("Failed to reset device: %d", write_result);
		return write_result;
	}

	/* Small delay after reset */
	k_usleep(100);

	/* Enable the device in normal operation mode */
	write_result = i2c_reg_write_byte_dt(&config->i2c,
					     KTD202X_REG_EN_RST,
					     KTD202X_ENABLE_ALWAYS_ON << KTD202X_EN_RST_ENABLE_SHIFT);
	if (write_result < 0)
	{
		LOG_ERR("Failed to enable device: %d", write_result);
		return write_result;
	}

	/* Initialize all LEDs to off */
	data->led_enable_register = 0x00;
	write_result = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, 0x00);
	if (write_result < 0)
	{
		LOG_ERR("Failed to disable LEDs: %d", write_result);
		return write_result;
	}

	/* Set default rise/fall time (instant) */
	write_result = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_TRISE_TFALL, 0x00);
	if (write_result < 0)
	{
		LOG_ERR("Failed to set rise/fall time: %d", write_result);
		return write_result;
	}

	LOG_DBG("KTD202x initialized with %d LEDs, %d channels",
		config->num_leds, config->num_channels);
	return 0;
}

/* LED driver API */
static DEVICE_API(led, ktd202x_led_api) =
{
	.on = ktd202x_on,
	.off = ktd202x_off,
	.set_brightness = ktd202x_set_brightness,
	.set_color = ktd202x_set_color,
	.blink = ktd202x_blink,
	.get_info = ktd202x_get_info,
};

/* Device tree instantiation macros */
#define COLOR_MAPPING(led_node_id)                                             \
	static const uint8_t color_mapping_##led_node_id[] =                   \
		DT_PROP(led_node_id, color_mapping);

#define LED_INFO(led_node_id)                                                  \
	{                                                                      \
		.label = DT_PROP(led_node_id, label),                          \
		.index = DT_PROP(led_node_id, index),                          \
		.num_colors = DT_PROP_LEN(led_node_id, color_mapping),         \
		.color_mapping = color_mapping_##led_node_id,                  \
	},

/* Channel map: allows remapping color indices to hardware channels */
#define KTD202X_CHANNEL_MAP(inst)                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, channel_map),                  \
		(static const uint8_t ktd202x_channel_map_##inst[] =           \
			DT_INST_PROP(inst, channel_map);),                     \
		())

#define KTD202X_CHANNEL_MAP_PTR(inst)                                          \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, channel_map),                  \
		(ktd202x_channel_map_##inst),                                  \
		(NULL))

#define KTD202X_GET_NUM_CHANNELS(inst)                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, channel_map),                  \
		(DT_INST_PROP_LEN(inst, channel_map)),                         \
		(KTD202X_DEFAULT_NUM_CHANNELS))

#define KTD202X_DEFINE(inst)                                                   \
	DT_INST_FOREACH_CHILD(inst, COLOR_MAPPING)                             \
                                                                               \
	static const struct led_info ktd202x_leds_##inst[] = {                 \
		DT_INST_FOREACH_CHILD(inst, LED_INFO)                          \
	};                                                                     \
                                                                               \
	KTD202X_CHANNEL_MAP(inst)                                              \
                                                                               \
	static struct ktd202x_data ktd202x_data_##inst;                        \
                                                                               \
	static const struct ktd202x_config ktd202x_config_##inst = {           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                             \
		.num_leds = ARRAY_SIZE(ktd202x_leds_##inst),                   \
		.leds_info = ktd202x_leds_##inst,                              \
		.channel_map = KTD202X_CHANNEL_MAP_PTR(inst),                  \
		.num_channels = KTD202X_GET_NUM_CHANNELS(inst),                \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      ktd202x_init,                                    \
			      NULL,                                            \
			      &ktd202x_data_##inst,                            \
			      &ktd202x_config_##inst,                          \
			      POST_KERNEL,                                     \
			      CONFIG_LED_INIT_PRIORITY,                        \
			      &ktd202x_led_api);

DT_INST_FOREACH_STATUS_OKAY(KTD202X_DEFINE)
