/**
 * @file ktd202x.c
 * @brief KTD2026/KTD2027 RGB LED driver -- Zephyr LED API + breathing
 *
 * 3/4 channel constant-current LED driver over I2C.
 * 0-24mA per channel (0.125mA steps), hardware PWM blinking,
 * programmable rise/fall times, S-curve or linear ramp profiles.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer Daravuthy Ly <daravuthy@crablabs.io>
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT kinetic_ktd202x

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ktd202x.h"

LOG_MODULE_REGISTER(ktd202x, CONFIG_KTD202X_LOG_LEVEL);

#define KTD202X_REG_EN_RST 0x00
#define KTD202X_REG_FLASH_PERIOD 0x01
#define KTD202X_REG_PWM1_TIMER 0x02
#define KTD202X_REG_PWM2_TIMER 0x03
#define KTD202X_REG_LED_EN 0x04
#define KTD202X_REG_TRISE_TFALL 0x05
#define KTD202X_REG_LED1 0x06
#define KTD202X_REG_LED2 0x07
#define KTD202X_REG_LED3 0x08
#define KTD202X_REG_LED4 0x09

// EN_RST Register (0x00) bits -- per datasheet Table 1, page 13
// Reg0[2:0] Timer Slot Control / Reset
#define KTD202X_EN_RST_TCTRL_MASK 0x07 // Bits [2:0]: Timer slot / reset
#define KTD202X_EN_RST_TCTRL_TSLOT1 0x00 // 000 = Timer slot 1
#define KTD202X_EN_RST_TCTRL_TSLOT2 0x01 // 001 = Timer slot 2
#define KTD202X_EN_RST_TCTRL_TSLOT3 0x02 // 010 = Timer slot 3
#define KTD202X_EN_RST_TCTRL_TSLOT4 0x03 // 011 = Timer slot 4
#define KTD202X_EN_RST_TCTRL_NOP 0x04 // 100 = Do nothing (bit cleared)
#define KTD202X_EN_RST_RESET_CHIP 0x07 // 111 = Reset complete chip
// Reg0[4:3] Enable Control
#define KTD202X_EN_RST_EN_CTRL_MASK 0x18 // Bits [4:3]: Enable control
#define KTD202X_EN_RST_EN_CTRL_SHIFT 3
#define KTD202X_EN_RST_EN_SCL_SDA 0x00 // 00 = ON when SCL+SDA high
#define KTD202X_EN_RST_EN_SCL_TOG 0x08 // 01 = ON when SCL high, SDA toggling
#define KTD202X_EN_RST_EN_SCL 0x10 // 10 = ON when SCL high
#define KTD202X_EN_RST_EN_ALWAYS 0x18 // 11 = Always ON
// Reg0[6:5] Rise/Fall Time Scaling
#define KTD202X_EN_RST_TSCALE_MASK 0x60 // Bits [6:5]: Time scale for rise/fall
#define KTD202X_EN_RST_TSCALE_SHIFT 5
// Reg0[7] Reserved -- must be 0 (factory test)

// FLASH_PERIOD Register (0x01) bits
#define KTD202X_FLASH_PERIOD_MASK 0x7F // Bits [6:0]: Period value
#define KTD202X_FLASH_RAMP_LINEAR_BIT 0x80 // Reg1[7]: 1=linear ramp, 0=S-curve

// LED Mode bits in LED_EN register (0x04)
#define KTD202X_LED_MODE_OFF 0x00
#define KTD202X_LED_MODE_ON 0x01
#define KTD202X_LED_MODE_PWM1 0x02
#define KTD202X_LED_MODE_PWM2 0x03
#define KTD202X_LED_MODE_MASK 0x03

// Current limits
#define KTD202X_MAX_CURRENT_REG 0xBF // 191 = 24mA (0.125mA * 192)
#define KTD202X_MAX_CHANNELS 4

// Timing constants
#define KTD202X_RESET_DELAY_US 200
#define KTD202X_INIT_RETRY_COUNT 3
#define KTD202X_RETRY_DELAY_MS 20
#define KTD202X_LOCK_TIMEOUT K_MSEC(100)
#define KTD202X_WAKE_LOW_US 50
#define KTD202X_WAKE_DELAY_US 600
#define KTD202X_BREATHE_DUTY_CYCLE 128 // 50% duty for breathing

/* Rise/fall time: value * 128ms (1x scale), 0 = 2ms minimum
 * Per datasheet page 15: Reg5=1 at 1x -> 128ms, Reg5=4 at 1x -> 512ms */
#define KTD202X_RAMP_TIME_STEP_MS 128
#define KTD202X_RAMP_TIME_MIN_MS 2 // ~2ms when trise/tfall=0
#define KTD202X_RAMP_TIME_MAX_VALUE 15 // 4-bit field max

/* Flash period: value=0 -> 128ms, value>=1 -> 256 + value*128 ms
 * Range: 128ms to 16.4s (value 0-126), value 127 = one-shot */
#define KTD202X_FLASH_PERIOD_MIN_MS 128
#define KTD202X_FLASH_PERIOD_MAX_VALUE 126 // 127 = one-shot

// Max current from Kconfig (mA) -> register value
// Current = (reg + 1) * 0.125mA, so reg = (mA / 0.125) - 1 = mA * 8 - 1
#define KTD202X_MAX_BRIGHTNESS MIN(((CONFIG_KTD202X_MAX_CURRENT_MA * 8) - 1), KTD202X_MAX_CURRENT_REG)

// Default rise/fall time from Kconfig -> register value
#define KTD202X_DEFAULT_TRISE MIN((CONFIG_KTD202X_DEFAULT_RISE_TIME_MS / KTD202X_RAMP_TIME_STEP_MS), KTD202X_RAMP_TIME_MAX_VALUE)

#define KTD202X_DEFAULT_TFALL MIN((CONFIG_KTD202X_DEFAULT_FALL_TIME_MS / KTD202X_RAMP_TIME_STEP_MS), KTD202X_RAMP_TIME_MAX_VALUE)

// Ramp type from Kconfig -- stored in Reg1[7], NOT Reg0
#if IS_ENABLED(CONFIG_KTD202X_RAMP_LINEAR)
#define KTD202X_RAMP_TYPE_INIT true
#else
#define KTD202X_RAMP_TYPE_INIT false
#endif

BUILD_ASSERT(KTD202X_MAX_BRIGHTNESS <= KTD202X_MAX_CURRENT_REG, "MAX_CURRENT_MA produces out-of-range register value");
BUILD_ASSERT(KTD202X_DEFAULT_TRISE <= KTD202X_RAMP_TIME_MAX_VALUE, "Default rise time exceeds 4-bit field maximum");
BUILD_ASSERT(KTD202X_DEFAULT_TFALL <= KTD202X_RAMP_TIME_MAX_VALUE, "Default fall time exceeds 4-bit field maximum");

struct ktd202x_config
{
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec wake_gpio;
	const struct device* vin_supply;
	uint8_t num_leds;
	const struct led_info* leds_info;
	const uint8_t* channel_map;
	uint8_t num_channels;
};

struct ktd202x_data
{
	struct k_mutex lock;
	uint8_t led_enable_register; ///< Cached LED_EN register value
	uint8_t en_rst_register; ///< Cached EN_RST register value
	uint8_t flash_period_register; ///< Cached FLASH_PERIOD (Reg1) - includes ramp bit
	uint8_t trise_tfall_register; ///< Cached TRISE_TFALL register value
	uint8_t channel_current_register[KTD202X_MAX_CHANNELS]; ///< Cached per-channel LEDx current values
};

// Helpers
static int ktd202xLock(struct ktd202x_data* const data);
static int ktd202xValidateConfig(const struct ktd202x_config* const config);
static int ktd202xWriteCachedRegister(const struct ktd202x_config* const config, const uint8_t register_address, const uint8_t value, uint8_t* const cache);
static int ktd202xWriteChannelCurrent(const struct ktd202x_config* const config, struct ktd202x_data* const data, const uint8_t hardware_channel, const uint8_t current_value);
static void ktd202xResetCache(struct ktd202x_data* const data);
static const struct led_info* ktd202xLedToInfo(const struct ktd202x_config* const config, const uint32_t led_index);
static inline uint8_t ktd202xLedModeShift(const uint32_t hardware_channel);
static inline uint8_t ktd202xMapChannel(const struct ktd202x_config* const config, const uint8_t color_index);
static inline uint8_t ktd202xBrightnessToCurrent(const uint8_t brightness);
static inline uint8_t ktd202xColorToCurrent(const uint8_t color);
static int ktd202xCalcFlashPeriod(const uint32_t period_ms, uint8_t* const register_value);
static uint8_t ktd202xCalcPWMDuty(const uint32_t delay_on, const uint32_t period);
static uint8_t ktd202xTimeToRampValue(const uint32_t time_ms);
static int ktd202xConfigureBreathe(const struct ktd202x_config* const config, struct ktd202x_data* const data, const struct led_info* const led_info, const uint32_t period_ms, const uint8_t* const currents);

// Zephyr LED API
static int ktd202xGetInfo(const struct device* const dev, const uint32_t led_index, const struct led_info** info);
static int ktd202xSetBrightness(const struct device* const dev, const uint32_t led_index, const uint8_t brightness);
static int ktd202xSetColor(const struct device* const dev, const uint32_t led_index, const uint8_t num_colors, const uint8_t* const color);
static int ktd202xOn(const struct device* const dev, const uint32_t led_index);
static int ktd202xOff(const struct device* const dev, const uint32_t led_index);
static int ktd202xBlink(const struct device* const dev, const uint32_t led_index, const uint32_t delay_on, const uint32_t delay_off);

static int ktd202xDeviceInit(const struct device* const dev);

static DEVICE_API(led, ktd202x_led_api) = {
	.on = ktd202xOn,
	.off = ktd202xOff,
	.set_brightness = ktd202xSetBrightness,
	.set_color = ktd202xSetColor,
	.blink = ktd202xBlink,
	.get_info = ktd202xGetInfo,
};

int ktd202xSetFadeTime(const struct device* const dev, const uint32_t rise_ms, const uint32_t fall_ms)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	const uint8_t rise_time_register_value = ktd202xTimeToRampValue(rise_ms);
	const uint8_t fall_time_register_value = ktd202xTimeToRampValue(fall_ms);
	const uint8_t combined_fade_register = (fall_time_register_value << 4) | rise_time_register_value;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_TRISE_TFALL, combined_fade_register, &data->trise_tfall_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set fade time: rise=%ums (reg=%u), fall=%ums (reg=%u)", rise_ms, rise_time_register_value, fall_ms, fall_time_register_value);

	return ret;
}

int ktd202xSetTimeScale(const struct device* const dev, const KTD202xTimeScale scale)
{
	if (dev == NULL || scale > KTD202X_TSCALE_8X)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_enable_reset_register = data->en_rst_register;
	new_enable_reset_register &= ~KTD202X_EN_RST_TSCALE_MASK;
	new_enable_reset_register |= ((uint8_t)scale << KTD202X_EN_RST_TSCALE_SHIFT);

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, new_enable_reset_register, &data->en_rst_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set time scale: %ux", 1 << scale);

	return ret;
}

int ktd202xSetRampLinear(const struct device* const dev, const bool is_linear)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	/* Ramp linear bit is Reg1[7] (FLASH_PERIOD register). We use the
	 * driver-cached value of Reg1 to compute the new value rather than
	 * reading it back: the KTD2026/2027 on the NAM board does not ACK
	 * reads reliably, so a read-modify-write via i2c_reg_read would fail. */
	uint8_t flash_period_register = data->flash_period_register;
	if (is_linear)
		flash_period_register |= KTD202X_FLASH_RAMP_LINEAR_BIT;
	else
		flash_period_register &= ~KTD202X_FLASH_RAMP_LINEAR_BIT;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, flash_period_register, &data->flash_period_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set ramp type: %s", is_linear ? "linear" : "S-curve");

	return ret;
}

int ktd202xBreathe(const struct device* const dev, const uint32_t led_index, const uint32_t period_ms, const uint8_t brightness)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;
	const uint8_t current_value = ktd202xBrightnessToCurrent(brightness);
	uint8_t currents[KTD202X_MAX_CHANNELS];

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	// Set brightness for all channels in this LED
	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, current_value);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}
		currents[color_channel_index] = current_value;
	}

	ret = ktd202xConfigureBreathe(config, data, led_info, period_ms, currents);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Breathe LED %u: period=%ums, brightness=%u%%", led_index, period_ms, brightness);

	return ret;
}

int ktd202xBreatheColor(const struct device* const dev, const uint32_t led_index, const uint32_t period_ms, const uint8_t* const color, const uint8_t num_colors)
{
	if (dev == NULL || color == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL || led_info->num_colors != num_colors)
		return -EINVAL;

	struct ktd202x_data* const data = dev->data;
	uint8_t currents[KTD202X_MAX_CHANNELS];

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	// Set color for each channel
	for (uint8_t color_channel_index = 0; color_channel_index < num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t current_value = ktd202xColorToCurrent(color[color_channel_index]);
		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, current_value);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}
		currents[color_channel_index] = current_value;
	}

	ret = ktd202xConfigureBreathe(config, data, led_info, period_ms, currents);

	k_mutex_unlock(&data->lock);
	return ret;
}

int ktd202xSetPWM2DutyCycle(const struct device* const dev, const uint8_t duty_cycle)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM2_TIMER, duty_cycle);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set PWM2 duty cycle: %u", duty_cycle);

	return ret;
}

int ktd202xReset(const struct device* const dev)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	/* Datasheet p17: the reset command intentionally NACKs the last byte.
	 * Zephyr returns -EIO in that case, which is the expected outcome
	 * here; only a different error code indicates a real failure. */
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_EN_RST, KTD202X_EN_RST_RESET_CHIP);
	if (ret != 0 && ret != -EIO)
	{
		k_mutex_unlock(&data->lock);
		LOG_ERR("Chip reset failed: %d", ret);
		return ret;
	}

	k_usleep(KTD202X_RESET_DELAY_US);

	ktd202xResetCache(data);
	const uint8_t enable_reset_register = KTD202X_EN_RST_EN_ALWAYS | KTD202X_EN_RST_TCTRL_TSLOT1;
	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, enable_reset_register, &data->en_rst_register);

	k_mutex_unlock(&data->lock);

	if (ret < 0)
		LOG_ERR("Post-reset enable failed: %d", ret);
	else
		LOG_DBG("Chip reset complete");

	return ret;
}

int ktd202xSetEnabled(const struct device* const dev, const bool is_enabled)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_enable_reset_register = data->en_rst_register;
	new_enable_reset_register &= ~KTD202X_EN_RST_EN_CTRL_MASK;

	if (is_enabled)
		new_enable_reset_register |= KTD202X_EN_RST_EN_ALWAYS;
	else
		new_enable_reset_register |= KTD202X_EN_RST_EN_SCL_TOG;
	/* EN_SCL_TOG (mode 01) shuts down once the I2C bus goes idle, reaching
	 * the datasheet's ~0.1-1uA shutdown current. EN_SCL_SDA (mode 00, all
	 * bits clear) stays powered because idle I2C lines sit high. */

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, new_enable_reset_register, &data->en_rst_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Chip %s", is_enabled ? "enabled" : "disabled (standby)");

	return ret;
}

int ktd202xFlashOnce(const struct device* const dev, const uint32_t led_index, const uint32_t on_time_ms, const uint8_t brightness)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;
	const uint8_t current_value = ktd202xBrightnessToCurrent(brightness);

	/* One-shot period is fixed at flash_period=127.
	 * Datasheet: value >= 1 means period = 256 + value*128 ms.
	 * So one-shot total period = 256 + 127*128 = 16512 ms.
	 * Duty cycle = on_time / 16512 * 256. */
	static const uint32_t oneshot_period_ms = 256 + (127 * 128);
	const uint32_t clamped_on_time_ms = MIN(on_time_ms, oneshot_period_ms);
	const uint8_t pwm_duty_cycle = ktd202xCalcPWMDuty(clamped_on_time_ms, oneshot_period_ms);

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	// Set brightness for all channels
	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, current_value);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}
	}

	// Set flash period to 127 (one-shot), preserve Reg1[7] ramp linear bit
	uint8_t flash_period_register = 127;
	flash_period_register |= data->flash_period_register & KTD202X_FLASH_RAMP_LINEAR_BIT;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, flash_period_register, &data->flash_period_register);
	if (ret < 0)
	{
		k_mutex_unlock(&data->lock);
		return ret;
	}

	// Set PWM1 duty cycle for on-time
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, pwm_duty_cycle);
	if (ret < 0)
	{
		k_mutex_unlock(&data->lock);
		return ret;
	}

	// Set channels to PWM1 mode
	uint8_t new_led_enable_register = data->led_enable_register;
	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (KTD202X_LED_MODE_PWM1 << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Flash once LED %u: on_time=%ums, brightness=%u%%", led_index, on_time_ms, brightness);

	return ret;
}

int ktd202xSetChannelPWM(const struct device* const dev, const uint32_t led_index, const KTD202xPWMChannel pwm_channel)
{
	if (dev == NULL)
		return -EINVAL;

	if (pwm_channel != KTD202X_PWM_CHANNEL_1 && pwm_channel != KTD202X_PWM_CHANNEL_2)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_led_enable_register = data->led_enable_register;

	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= ((uint8_t)pwm_channel << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set LED %u to PWM%u", led_index, (pwm_channel == KTD202X_PWM_CHANNEL_1) ? 1 : 2);

	return ret;
}

int ktd202xSetTimerSlot(const struct device* const dev, const KTD202xTimerSlot slot)
{
	if (dev == NULL || slot > KTD202X_TIMER_SLOT_4)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_enable_reset_register = data->en_rst_register;
	new_enable_reset_register &= ~KTD202X_EN_RST_TCTRL_MASK;
	new_enable_reset_register |= (uint8_t)slot;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, new_enable_reset_register, &data->en_rst_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set timer slot: %u", (uint32_t)slot + 1);

	return ret;
}

int ktd202xSetEnableMode(const struct device* const dev, const KTD202xEnableMode mode)
{
	if (dev == NULL || mode > KTD202X_EN_ALWAYS)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_enable_reset_register = data->en_rst_register;
	new_enable_reset_register &= ~KTD202X_EN_RST_EN_CTRL_MASK;
	new_enable_reset_register |= ((uint8_t)mode << KTD202X_EN_RST_EN_CTRL_SHIFT);

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, new_enable_reset_register, &data->en_rst_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set enable mode: %u", (uint32_t)mode);

	return ret;
}

int ktd202xGetLEDEnable(const struct device* const dev, uint8_t* const led_enable)
{
	if (dev == NULL || led_enable == NULL)
		return -EINVAL;

	struct ktd202x_data* const data = dev->data;

	const int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	*led_enable = data->led_enable_register;

	k_mutex_unlock(&data->lock);

	return 0;
}

int ktd202xSetFlashPeriod(const struct device* const dev, const uint32_t period_ms)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	uint8_t flash_period_register;
	if (ktd202xCalcFlashPeriod(period_ms, &flash_period_register) < 0)
		return -EINVAL;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	// Preserve Reg1[7] ramp linear bit
	flash_period_register |= data->flash_period_register & KTD202X_FLASH_RAMP_LINEAR_BIT;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, flash_period_register, &data->flash_period_register);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set flash period: %ums (reg=0x%02X)", period_ms, flash_period_register & KTD202X_FLASH_PERIOD_MASK);

	return ret;
}

int ktd202xGetFadeTime(const struct device* const dev, uint32_t* const rise_ms, uint32_t* const fall_ms)
{
	if (dev == NULL || rise_ms == NULL || fall_ms == NULL)
		return -EINVAL;

	struct ktd202x_data* const data = dev->data;

	const int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	const uint8_t rise_time_register_value = data->trise_tfall_register & 0x0F;
	const uint8_t fall_time_register_value = (data->trise_tfall_register >> 4) & 0x0F;

	k_mutex_unlock(&data->lock);

	// Convert register values to ms: 0 = ~2ms, otherwise value * 128ms
	*rise_ms = (rise_time_register_value == 0) ? KTD202X_RAMP_TIME_MIN_MS : (uint32_t)rise_time_register_value * KTD202X_RAMP_TIME_STEP_MS;
	*fall_ms = (fall_time_register_value == 0) ? KTD202X_RAMP_TIME_MIN_MS : (uint32_t)fall_time_register_value * KTD202X_RAMP_TIME_STEP_MS;

	return 0;
}

KTD202xTimeScale ktd202xGetTimeScale(const struct device* const dev)
{
	if (dev == NULL)
		return KTD202X_TSCALE_1X;

	struct ktd202x_data* const data = dev->data;

	const int ret = ktd202xLock(data);
	if (ret < 0)
		return KTD202X_TSCALE_1X;

	const KTD202xTimeScale scale = (KTD202xTimeScale)((data->en_rst_register & KTD202X_EN_RST_TSCALE_MASK) >> KTD202X_EN_RST_TSCALE_SHIFT);

	k_mutex_unlock(&data->lock);

	return scale;
}

bool ktd202xIsRampLinear(const struct device* const dev)
{
	if (dev == NULL)
		return false;

	struct ktd202x_data* const data = dev->data;

	const int ret = ktd202xLock(data);
	if (ret < 0)
		return false;

	const bool is_linear = (data->flash_period_register & KTD202X_FLASH_RAMP_LINEAR_BIT) != 0;

	k_mutex_unlock(&data->lock);

	return is_linear;
}

int ktd202xSetPWM1DutyCycle(const struct device* const dev, const uint8_t duty_cycle)
{
	if (dev == NULL)
		return -EINVAL;

	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, duty_cycle);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Set PWM1 duty cycle: %u", duty_cycle);

	return ret;
}

uint8_t ktd202xGetMaxBrightness(void)
{
	return KTD202X_MAX_BRIGHTNESS;
}

uint8_t ktd202xGetMaxCurrentMa(void)
{
	return CONFIG_KTD202X_MAX_CURRENT_MA;
}

/**
 * @brief Lock driver state for a public operation.
 *
 * @param[in] data Driver runtime state.
 * @return 0 on success; -ETIMEDOUT when the lock cannot be acquired.
 */
static int ktd202xLock(struct ktd202x_data* const data)
{
	const int ret = k_mutex_lock(&data->lock, KTD202X_LOCK_TIMEOUT);
	if (ret < 0)
	{
		LOG_ERR("Mutex lock failed: %d", ret);
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * @brief Validate immutable LED and hardware channel routing.
 *
 * @param[in] config Driver configuration generated from devicetree.
 * @return 0 when valid; -EINVAL for an invalid LED span or channel map.
 */
static int ktd202xValidateConfig(const struct ktd202x_config* const config)
{
	if (config->num_leds == 0 || config->leds_info == NULL)
	{
		LOG_ERR("No LED channels configured");
		return -EINVAL;
	}

	if (config->num_channels == 0 || config->num_channels > KTD202X_MAX_CHANNELS)
	{
		LOG_ERR("Invalid channel-map length: %u", config->num_channels);
		return -EINVAL;
	}

	uint8_t mapped_channels = 0;
	for (uint8_t index = 0; index < config->num_channels; index++)
	{
		const uint8_t hardware_channel = config->channel_map[index];
		if (hardware_channel >= KTD202X_MAX_CHANNELS)
		{
			LOG_ERR("channel-map[%u]=%u is out of range", index, hardware_channel);
			return -EINVAL;
		}

		const uint8_t channel_bit = BIT(hardware_channel);
		if ((mapped_channels & channel_bit) != 0)
		{
			LOG_ERR("channel-map repeats hardware channel %u", hardware_channel);
			return -EINVAL;
		}

		mapped_channels |= channel_bit;
	}

	for (uint8_t index = 0; index < config->num_leds; index++)
	{
		const struct led_info* const led = &config->leds_info[index];
		const uint32_t channel_end = led->index + led->num_colors;

		if (led->num_colors == 0 || channel_end > KTD202X_MAX_CHANNELS)
		{
			LOG_ERR("LED %u has invalid channel span [%u, %u)", index, led->index, channel_end);
			return -EINVAL;
		}

		if (channel_end > config->num_channels)
		{
			LOG_ERR("LED %u exceeds channel-map length %u", index, config->num_channels);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * @brief Write a register and update its cache only after successful I2C transfer.
 *
 * The caller must hold the driver lock.
 *
 * @param[in] config Driver configuration.
 * @param[in] register_address Register address.
 * @param[in] value New register value.
 * @param[out] cache Cached register value to commit.
 * @return 0 on success; negative errno on I2C failure.
 */
static int ktd202xWriteCachedRegister(const struct ktd202x_config* const config, const uint8_t register_address, const uint8_t value, uint8_t* const cache)
{
	const int ret = i2c_reg_write_byte_dt(&config->i2c, register_address, value);
	if (ret == 0)
		*cache = value;

	return ret;
}

/**
 * @brief Write one channel's current register and cache the value.
 *
 * @param[in] config            Driver configuration.
 * @param[in,out] data          Driver runtime state holding the cache.
 * @param[in] hardware_channel  Channel index on the part.
 * @param[in] current_value     Current register value to write.
 * @return 0 on success, or a negative errno from the I2C write.
 */
static int ktd202xWriteChannelCurrent(const struct ktd202x_config* const config, struct ktd202x_data* const data, const uint8_t hardware_channel, const uint8_t current_value)
{
	if (hardware_channel >= KTD202X_MAX_CHANNELS)
		return -EINVAL;

	return ktd202xWriteCachedRegister(config, KTD202X_REG_LED1 + hardware_channel, current_value, &data->channel_current_register[hardware_channel]);
}

/**
 * @brief Restore cached state to the hardware power-on reset values.
 *
 * @param[out] data Driver runtime state.
 */
static void ktd202xResetCache(struct ktd202x_data* const data)
{
	memset(data->channel_current_register, 0, sizeof(data->channel_current_register));
	data->en_rst_register = 0;
	data->flash_period_register = 0;
	data->led_enable_register = 0;
	data->trise_tfall_register = 0;
}

static const struct led_info* ktd202xLedToInfo(const struct ktd202x_config* const config, const uint32_t led_index)
{
	if (led_index < config->num_leds)
		return &config->leds_info[led_index];

	return NULL;
}

static inline uint8_t ktd202xLedModeShift(const uint32_t hardware_channel)
{
	return (hardware_channel * 2);
}

static inline uint8_t ktd202xMapChannel(const struct ktd202x_config* const config, const uint8_t color_index)
{
	return config->channel_map[color_index];
}

// Convert 0-100 brightness percentage to register value, capped by max current
static inline uint8_t ktd202xBrightnessToCurrent(const uint8_t brightness)
{
	const uint32_t clamped = MIN(brightness, 100U);
	return (uint8_t)((clamped * KTD202X_MAX_BRIGHTNESS) / 100U);
}

// Convert 0-255 color value to register value, capped by max current
static inline uint8_t ktd202xColorToCurrent(const uint8_t color)
{
	return (uint8_t)(((uint32_t)color * KTD202X_MAX_BRIGHTNESS) / 255U);
}

// Convert a flash period register code to its real period in ms.
// Datasheet: value=0 -> 128ms, value>=1 -> 256 + value*128 ms
static inline uint32_t ktd202xFlashPeriodCodeToMs(const uint8_t code)
{
	if (code == 0)
		return 128U;

	return 256U + ((uint32_t)code * 128U);
}

/* Calculate flash period register value from period in ms.
 * Datasheet: value=0 -> 128ms, value>=1 -> 256 + value*128 ms */
static int ktd202xCalcFlashPeriod(const uint32_t period_ms, uint8_t* const register_value)
{
	if (period_ms < KTD202X_FLASH_PERIOD_MIN_MS)
		return -EINVAL;

	uint8_t floor_code;

	// Value 0 = 128ms. For values >= 1: period = 256 + value*128
	if (period_ms < 384)
	{
		floor_code = 0;
	}
	else
	{
		const uint32_t calculated_register_value = (period_ms - 256) / 128;
		floor_code = (uint8_t)MIN(calculated_register_value, KTD202X_FLASH_PERIOD_MAX_VALUE);
	}

	const uint32_t floor_period_ms = ktd202xFlashPeriodCodeToMs(floor_code);

	// Pick whichever of floor/ceiling codes lands closer to the requested
	// period, instead of always flooring (Reg1 code 0=128ms jumps straight
	// to code 1=384ms, so a floor-only pick left [128,383]ms stuck at 128ms).
	if (floor_code < KTD202X_FLASH_PERIOD_MAX_VALUE)
	{
		const uint32_t ceil_period_ms = ktd202xFlashPeriodCodeToMs(floor_code + 1);
		if ((ceil_period_ms - period_ms) < (period_ms - floor_period_ms))
		{
			*register_value = floor_code + 1;
			return 0;
		}
	}

	*register_value = floor_code;
	return 0;
}

// Calculate PWM duty cycle register value
static uint8_t ktd202xCalcPWMDuty(const uint32_t delay_on, const uint32_t period)
{
	if (period == 0)
		return 0;

	return (uint8_t)((delay_on * 256U) / period);
}

// Convert time in ms to rise/fall register value (0-15)
static uint8_t ktd202xTimeToRampValue(const uint32_t time_ms)
{
	if (time_ms < KTD202X_RAMP_TIME_MIN_MS)
		return 0;

	const uint32_t value = time_ms / KTD202X_RAMP_TIME_STEP_MS;
	return (uint8_t)MIN(value, KTD202X_RAMP_TIME_MAX_VALUE);
}

/* Configure breathing: set flash period, 50% duty, PWM mode on all channels.
 * Caller must hold data->lock. Channel brightness must already be set. */
static int ktd202xConfigureBreathe(const struct ktd202x_config* const config, struct ktd202x_data* const data, const struct led_info* const led_info, const uint32_t period_ms, const uint8_t* const currents)
{
	uint8_t flash_period_register;
	if (ktd202xCalcFlashPeriod(period_ms, &flash_period_register) < 0)
		return -EINVAL;

	flash_period_register |= data->flash_period_register & KTD202X_FLASH_RAMP_LINEAR_BIT;

	int ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, flash_period_register, &data->flash_period_register);
	if (ret < 0)
		return ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, KTD202X_BREATHE_DUTY_CYCLE);
	if (ret < 0)
		return ret;

	// A zero-current channel still breathes visibly in PWM1 mode; force it off instead.
	uint8_t new_led_enable_register = data->led_enable_register;
	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		const uint8_t mode = (currents[color_channel_index] > 0) ? KTD202X_LED_MODE_PWM1 : KTD202X_LED_MODE_OFF;
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (mode << shift);
	}

	return ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);
}

static int ktd202xGetInfo(const struct device* const dev, const uint32_t led_index, const struct led_info** info)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -EINVAL;

	*info = led_info;
	return 0;
}

static int ktd202xSetBrightness(const struct device* const dev, const uint32_t led_index, const uint8_t brightness)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -EINVAL;

	struct ktd202x_data* const data = dev->data;
	const uint8_t current_value = ktd202xBrightnessToCurrent(brightness);

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	// Set same brightness on all channels of this LED
	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, current_value);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}
	}

	// Build new enable register locally, commit only on success
	uint8_t new_led_enable_register = data->led_enable_register;
	const uint8_t mode = (brightness > 0) ? KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (mode << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ktd202xSetColor(const struct device* const dev, const uint32_t led_index, const uint8_t num_colors, const uint8_t* const color)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL || color == NULL || led_info->num_colors != num_colors)
		return -EINVAL;

	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_led_enable_register = data->led_enable_register;

	for (uint8_t color_channel_index = 0; color_channel_index < num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t led_current = ktd202xColorToCurrent(color[color_channel_index]);

		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, led_current);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}

		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		const uint8_t mode = (color[color_channel_index] > 0) ? KTD202X_LED_MODE_ON : KTD202X_LED_MODE_OFF;

		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (mode << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ktd202xOn(const struct device* const dev, const uint32_t led_index)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_led_enable_register = data->led_enable_register;

	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		ret = ktd202xWriteChannelCurrent(config, data, hardware_channel, KTD202X_MAX_BRIGHTNESS);
		if (ret < 0)
		{
			k_mutex_unlock(&data->lock);
			return ret;
		}
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (KTD202X_LED_MODE_ON << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ktd202xOff(const struct device* const dev, const uint32_t led_index)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	uint8_t new_led_enable_register = data->led_enable_register;

	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ktd202xBlink(const struct device* const dev, const uint32_t led_index, const uint32_t delay_on, const uint32_t delay_off)
{
	const struct ktd202x_config* const config = dev->config;
	const struct led_info* const led_info = ktd202xLedToInfo(config, led_index);

	if (led_info == NULL)
		return -ENODEV;

	struct ktd202x_data* const data = dev->data;
	const uint32_t period = delay_on + delay_off;
	uint8_t flash_period_register;

	if (ktd202xCalcFlashPeriod(period, &flash_period_register) < 0)
		return -EINVAL;

	int ret = ktd202xLock(data);
	if (ret < 0)
		return ret;

	flash_period_register |= data->flash_period_register & KTD202X_FLASH_RAMP_LINEAR_BIT;

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, flash_period_register, &data->flash_period_register);
	if (ret < 0)
	{
		k_mutex_unlock(&data->lock);
		return ret;
	}

	const uint8_t pwm_duty_cycle = ktd202xCalcPWMDuty(delay_on, period);
	ret = i2c_reg_write_byte_dt(&config->i2c, KTD202X_REG_PWM1_TIMER, pwm_duty_cycle);
	if (ret < 0)
	{
		k_mutex_unlock(&data->lock);
		return ret;
	}

	uint8_t new_led_enable_register = data->led_enable_register;

	for (uint8_t color_channel_index = 0; color_channel_index < led_info->num_colors; color_channel_index++)
	{
		const uint8_t hardware_channel = ktd202xMapChannel(config, led_info->index + color_channel_index);
		const uint8_t shift = ktd202xLedModeShift(hardware_channel);

		const uint8_t mode = (data->channel_current_register[hardware_channel] > 0) ? KTD202X_LED_MODE_PWM1 : KTD202X_LED_MODE_OFF; // A zero-current channel still lights in PWM1, washing one colour out to white
		new_led_enable_register &= ~(KTD202X_LED_MODE_MASK << shift);
		new_led_enable_register |= (mode << shift);
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, new_led_enable_register, &data->led_enable_register);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ktd202xDeviceInit(const struct device* const dev)
{
	const struct ktd202x_config* const config = dev->config;
	struct ktd202x_data* const data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	ktd202xResetCache(data);

	ret = ktd202xValidateConfig(config);
	if (ret < 0)
		return ret;

	if (!i2c_is_ready_dt(&config->i2c))
	{
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (config->vin_supply != NULL)
	{
		if (!device_is_ready(config->vin_supply))
		{
			LOG_ERR("VIN regulator not ready");
			return -ENODEV;
		}
		ret = regulator_enable(config->vin_supply);
		if (ret < 0)
		{
			LOG_ERR("Failed to enable VIN regulator: %d", ret);
			return ret;
		}
		k_msleep(100); // Power stabilization
	}

	/* Datasheet p18 requires SDA low for at least 10 us followed by the
	 * controller wake delay before the first transaction. */
	if (config->wake_gpio.port != NULL)
	{
		if (!gpio_is_ready_dt(&config->wake_gpio))
		{
			LOG_ERR("Wake GPIO not ready");
			return -ENODEV;
		}

		const gpio_dt_flags_t required_flags = GPIO_ACTIVE_LOW | GPIO_OPEN_DRAIN;
		if ((config->wake_gpio.dt_flags & required_flags) != required_flags)
		{
			LOG_ERR("wake-gpios must be active-low and open-drain");
			return -EINVAL;
		}

		ret = gpio_pin_configure_dt(&config->wake_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0)
		{
			LOG_ERR("Failed to assert wake GPIO: %d", ret);
			return ret;
		}

		k_busy_wait(KTD202X_WAKE_LOW_US);
		ret = gpio_pin_configure_dt(&config->wake_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0)
		{
			LOG_ERR("Failed to release wake GPIO: %d", ret);
			return ret;
		}

		k_usleep(KTD202X_WAKE_DELAY_US);
	}

	const uint8_t enable_reset_register = KTD202X_EN_RST_EN_ALWAYS | KTD202X_EN_RST_TCTRL_TSLOT1;
	for (int attempt = 0; attempt < KTD202X_INIT_RETRY_COUNT; attempt++)
	{
		ret = ktd202xWriteCachedRegister(config, KTD202X_REG_EN_RST, enable_reset_register, &data->en_rst_register);
		if (ret == 0)
			break;
		k_msleep(KTD202X_RETRY_DELAY_MS);
	}

	if (ret < 0)
	{
		LOG_ERR("KTD202x not responding (chip absent or at wrong address): %d", ret);
		return ret;
	}

	k_msleep(2);

	if (KTD202X_RAMP_TYPE_INIT)
	{
		ret = ktd202xWriteCachedRegister(config, KTD202X_REG_FLASH_PERIOD, KTD202X_FLASH_RAMP_LINEAR_BIT, &data->flash_period_register);
		if (ret < 0)
		{
			LOG_ERR("Failed to set ramp type: %d", ret);
			return ret;
		}
	}

	const uint8_t trise_tfall_register = (KTD202X_DEFAULT_TFALL << 4) | KTD202X_DEFAULT_TRISE;
	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_TRISE_TFALL, trise_tfall_register, &data->trise_tfall_register);
	if (ret < 0)
	{
		LOG_ERR("Failed to set rise/fall times: %d", ret);
		return ret;
	}

	ret = ktd202xWriteCachedRegister(config, KTD202X_REG_LED_EN, 0, &data->led_enable_register);
	if (ret < 0)
	{
		LOG_ERR("Failed to disable LEDs: %d", ret);
		return ret;
	}

	LOG_INF("KTD202x initialized (max %umA, %s ramp)", CONFIG_KTD202X_MAX_CURRENT_MA, IS_ENABLED(CONFIG_KTD202X_RAMP_LINEAR) ? "linear" : "S-curve");

	return 0;
}

#define COLOR_MAPPING(led_node_id) static const uint8_t DT_CAT(color_mapping_, led_node_id)[] = DT_PROP(led_node_id, color_mapping);

#define LED_INFO(led_node_id)                                                                                                                                                                                                                                                                              \
	{                                                                                                                                                                                                                                                                                                      \
		.label = DT_PROP(led_node_id, label),                                                                                                                                                                                                                                                              \
		.index = DT_PROP(led_node_id, reg),                                                                                                                                                                                                                                                                \
		.num_colors = DT_PROP_LEN(led_node_id, color_mapping),                                                                                                                                                                                                                                             \
		.color_mapping = DT_CAT(color_mapping_, led_node_id),                                                                                                                                                                                                                                              \
	},

// Named per-colour properties rather than one array: devicetree discards any property
// whose name ends in "-map" (edtlib reserves that suffix for nexus mapping), so the former
// channel-map was silently ignored and every board got the default order.
#define CHANNEL_MAP(inst)                                        \
	static const uint8_t DT_CAT(ktd202x_channel_map_, inst)[] = { \
		DT_INST_PROP(inst, red_channel),                          \
		DT_INST_PROP(inst, green_channel),                        \
		DT_INST_PROP(inst, blue_channel),                         \
		3,                                                        \
	};

#define KTD202X_DEFINE(inst)                                                                                                                                                                                                                                                                               \
	DT_INST_FOREACH_CHILD(inst, COLOR_MAPPING)                                                                                                                                                                                                                                                             \
	CHANNEL_MAP(inst)                                                                                                                                                                                                                                                                                      \
	static const struct led_info DT_CAT(ktd202x_leds_, inst)[] = { DT_INST_FOREACH_CHILD(inst, LED_INFO) };                                                                                                                                                                                                \
	static struct ktd202x_data DT_CAT(ktd202x_data_, inst);                                                                                                                                                                                                                                                \
	static const struct ktd202x_config DT_CAT(ktd202x_config_, inst) = {                                                                                                                                                                                                                                   \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                                                                                                                                                                                                                                 \
		.wake_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, wake_gpios, { 0 }),                                                                                                                                                                                                                                    \
		.vin_supply = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, vin_supply), (DEVICE_DT_GET(DT_INST_PHANDLE(inst, vin_supply))), (NULL)),                                                                                                                                                                    \
		.num_leds = ARRAY_SIZE(DT_CAT(ktd202x_leds_, inst)),                                                                                                                                                                                                                                               \
		.leds_info = DT_CAT(ktd202x_leds_, inst),                                                                                                                                                                                                                                                          \
		.channel_map = DT_CAT(ktd202x_channel_map_, inst),                                                                                                                                                         \
		.num_channels = KTD202X_MAX_CHANNELS,                                                                                                                                                              \
	};                                                                                                                                                                                                                                                                                                     \
	DEVICE_DT_INST_DEFINE(inst, ktd202xDeviceInit, NULL, &DT_CAT(ktd202x_data_, inst), &DT_CAT(ktd202x_config_, inst), POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &ktd202x_led_api);

DT_INST_FOREACH_STATUS_OKAY(KTD202X_DEFINE)
