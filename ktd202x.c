/*
 * Copyright (c) 2025 Makani Science
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kinetic_ktd202x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ktd202x, CONFIG_LED_LOG_LEVEL);

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

#define KTD202X_EN_RST_RESET     0x07
#define KTD202X_ENABLE_ALWAYS_ON    0x02
#define KTD202X_EN_RST_ENABLE_SHIFT 3

#define KTD202X_LED_MODE_OFF     0x00
#define KTD202X_LED_MODE_ON      0x01
#define KTD202X_LED_MODE_PWM1    0x02
#define KTD202X_LED_MODE_PWM2    0x03
#define KTD202X_LED_MODE_MASK    0x03

#define KTD202X_MAX_BRIGHTNESS   191
#define KTD202X_MAX_CHANNELS     4
#define KTD202X_DEFAULT_NUM_CHANNELS 3

struct ktd202x_config {
	struct i2c_dt_spec i2c;
	const struct device *vin_supply;
	uint8_t num_leds;
	const struct led_info *leds_info;
	const uint8_t *channel_map;
	uint8_t num_channels;
};

struct ktd202x_data {
	uint8_t led_enable_register;
};

static const struct led_info *ktd202x_led_to_info(const struct ktd202x_config *const config,
					  const uint32_t led_index)
{
	if (led_index < config->num_leds) {
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
	if (config->channel_map != NULL && color_index < config->num_channels) {
		return config->channel_map[color_index];
	}
	return color_index;
}

static inline uint8_t ktd202x_brightness_to_current(const uint8_t brightness)
{
	return (brightness * KTD202X_MAX_BRIGHTNESS) / 100U;
}

static int ktd202x_calc_flash_period(const uint32_t period_ms, uint8_t *const register_value)
{
	if (period_ms < 128) {
		return -EINVAL;
	}
	if (period_ms < 256) {
		*register_value = 0;
		return 0;
	}
	const uint32_t calculated_value = (period_ms - 256) / 128;
	*register_value = (uint8_t)MIN(calculated_value, 127);
	return 0;
}

static uint8_t ktd202x_calc_pwm_duty(const uint32_t delay_on, const uint32_t period)
{
	if (period == 0) {
		return 0;
	}
	return (uint8_t)((delay_on * 256U) / period);
}

static int ktd202x_get_info(const struct device *const dev,
			    const uint32_t led_index,
			    const struct led_info **info)
{
	const struct ktd202x_config *const config = dev->config;
	const struct led_info *const led_info = ktd202x_led_to_info(config, led_index);

	if (led_info == NULL) {
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

	if (led_info == NULL || led_info->num_colors != 1) {
		return -ENOTSUP;
	}

	struct ktd202x_data *const data = dev->data;
	const uint8_t current_value = ktd202x_brightness_to_current(brightness);
	const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index);

	int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, current_value);
	if (ret < 0) {
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

	if (led_info == NULL || color == NULL || led_info->num_colors != num_colors) {
		return -EINVAL;
	}

	struct ktd202x_data *const data = dev->data;
	uint8_t led_enable_value = data->led_enable_register;

	for (uint8_t color_index = 0; color_index < num_colors; color_index++) {
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + color_index);
		const uint8_t led_current = (color[color_index] * KTD202X_MAX_BRIGHTNESS) / 255U;

		int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, led_current);
		if (ret < 0) {
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

	if (led_info == NULL) {
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;

	for (uint8_t i = 0; i < led_info->num_colors; i++) {
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		int ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED1 + hardware_channel, KTD202X_MAX_BRIGHTNESS);
		if (ret < 0) {
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

	if (led_info == NULL) {
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;

	for (uint8_t i = 0; i < led_info->num_colors; i++) {
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

	if (led_info == NULL) {
		return -ENODEV;
	}

	struct ktd202x_data *const data = dev->data;
	const uint32_t period = delay_on + delay_off;
	uint8_t flash_period_register;

	if (ktd202x_calc_flash_period(period, &flash_period_register) < 0) {
		return -EINVAL;
	}

	if (i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_FLASH_PERIOD, flash_period_register) < 0) {
		return -EIO;
	}

	const uint8_t pwm_duty_cycle = ktd202x_calc_pwm_duty(delay_on, period);
	if (i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, pwm_duty_cycle) < 0) {
		return -EIO;
	}

	for (uint8_t i = 0; i < led_info->num_colors; i++) {
		const uint8_t hardware_channel = ktd202x_map_channel(config, led_info->index + i);
		const uint8_t shift = ktd202x_led_mode_shift(hardware_channel);
		data->led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		data->led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);
	}

	return i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, data->led_enable_register);
}

static int ktd202x_init(const struct device *const dev)
{
	const struct ktd202x_config *const config = dev->config;
	struct ktd202x_data *const data = dev->data;

	LOG_INF("KTD202x driver initializing...");

	if (config->vin_supply) {
		if (!device_is_ready(config->vin_supply)) {
			LOG_ERR("Regulator not ready");
			return -ENODEV;
		}
		if (regulator_enable(config->vin_supply) < 0) {
			LOG_ERR("Failed to enable regulator");
			return -EIO;
		}
		k_msleep(100);
	}

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	int ret = -1;
	for (int i = 0; i < 3; i++) {
		ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, KTD202X_EN_RST_RESET);
		if (ret == 0) break;
		k_msleep(10);
	}

	if (ret < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	k_usleep(100);

	if (i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, 0x12) < 0) {
		return -EIO;
	}

	data->led_enable_register = 0x00;
	i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_LED_EN, 0x00);
	i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_TRISE_TFALL, 0x00);

	LOG_INF("KTD202x initialized successfully");
	return 0;
}

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
	{\
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