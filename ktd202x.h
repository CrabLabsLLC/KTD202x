/**
 * @file ktd202x.h
 * @brief KTD2026/KTD2027 RGB LED driver -- extended API
 *
 * Provides breathing effects, fade timing control, time scaling,
 * PWM2 timer, per-channel PWM assignment, power control, chip reset,
 * and one-shot flash beyond the standard Zephyr LED API.
 *
 * @author Orion Serup <orion@crablabs.io>
 * @author Claude <noreply@anthropic.com>
 *
 * @reviewer
 */

/* Copyright (c) 2025 Crab Labs LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#ifndef KTD202X_H
#define KTD202X_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Time scale multiplier for rise/fall times.
 *
 * Applies to both rise and fall times set via ktd202xSetFadeTime().
 */
typedef enum
{
	KTD202X_TSCALE_1X = 0,  ///< 1x normal
	KTD202X_TSCALE_2X = 1,  ///< 2x slower
	KTD202X_TSCALE_4X = 2,  ///< 4x slower
	KTD202X_TSCALE_8X = 3,  ///< 8x faster
} KTD202xTimeScale;

/**
 * @brief PWM timer channel assignment.
 *
 * Each LED channel can be independently assigned to either PWM1 or PWM2.
 * The two timers have separate duty cycle and period settings, allowing
 * different blink/breathe patterns on different LEDs simultaneously.
 */
typedef enum
{
	KTD202X_PWM_CHANNEL_1 = 0x02,  ///< Use PWM1 timer
	KTD202X_PWM_CHANNEL_2 = 0x03,  ///< Use PWM2 timer
} KTD202xPWMChannel;

/**
 * @brief Timer slot selection for blinking/breathing.
 *
 * The EN_RST register bits [2:0] select which timer slot controls
 * the flash period and duty cycle timing. Up to 4 independent
 * timer slots are available.
 */
typedef enum
{
	KTD202X_TIMER_SLOT_1 = 0,  ///< Timer slot 1
	KTD202X_TIMER_SLOT_2 = 1,  ///< Timer slot 2
	KTD202X_TIMER_SLOT_3 = 2,  ///< Timer slot 3
	KTD202X_TIMER_SLOT_4 = 3,  ///< Timer slot 4
} KTD202xTimerSlot;

/**
 * @brief Enable mode for the chip.
 *
 * Controls when the chip is active, based on I2C bus line state.
 * The EN_RST register bits [4:3] select the enable mode.
 */
typedef enum
{
	KTD202X_EN_STANDBY       = 0,  ///< Standby: on only when SCL+SDA high
	KTD202X_EN_SCL_SDA       = 1,  ///< On when SCL high and SDA toggling
	KTD202X_EN_SCL_ONLY      = 2,  ///< On when SCL high
	KTD202X_EN_ALWAYS        = 3,  ///< Always on (default)
} KTD202xEnableMode;

/**
 * @brief Set the rise and fall times for breathing/fade effects.
 *
 * Actual ramp time = (register_value * 128ms) * time_scale.
 * With 1x scale: 0=2ms, 128ms steps, max 1920ms per ramp.
 * With 8x faster: max 240ms per ramp.
 *
 * @param[in] dev      KTD202x device. Must not be NULL.
 * @param[in] rise_ms  Rise time in milliseconds (0-1920 with 1x scale).
 * @param[in] fall_ms  Fall time in milliseconds (0-1920 with 1x scale).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetFadeTime(const struct device* const dev, const uint32_t rise_ms, const uint32_t fall_ms);

/**
 * @brief Set the time scale multiplier for rise/fall times.
 *
 * Scales all rise/fall times by the specified factor.
 * Useful for very slow breathing effects.
 *
 * @param[in] dev    KTD202x device. Must not be NULL.
 * @param[in] scale  Time scale (1x, 2x, 4x, or 8x).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetTimeScale(const struct device* const dev, const KTD202xTimeScale scale);

/**
 * @brief Set the ramp profile type.
 *
 * The S-curve (default) provides a more natural-looking fade that
 * matches human brightness perception. Linear is a simple linear ramp.
 *
 * @param[in] dev        KTD202x device. Must not be NULL.
 * @param[in] is_linear  true for linear ramp, false for S-curve (logarithmic).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetRampLinear(const struct device* const dev, const bool is_linear);

/**
 * @brief Start a breathing effect on an LED.
 *
 * The LED will smoothly fade up and down at the specified period,
 * using the current rise/fall times and ramp profile.
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] led_index   LED index (from devicetree).
 * @param[in] period_ms   Total period for one breath cycle (128ms-16.4s).
 * @param[in] brightness  Peak brightness (0-100%).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xBreathe(const struct device* const dev, const uint32_t led_index, const uint32_t period_ms, const uint8_t brightness);

/**
 * @brief Start a breathing effect with a specific color.
 *
 * Like ktd202xBreathe() but for RGB LEDs with specific color values.
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] led_index   LED index (from devicetree).
 * @param[in] period_ms   Total period for one breath cycle (128ms-16.4s).
 * @param[in] color       Array of color values (0-255 per channel). Must not be NULL.
 * @param[in] num_colors  Number of color channels (must match LED definition).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xBreatheColor(
	const struct device* const dev,
	const uint32_t led_index,
	const uint32_t period_ms,
	const uint8_t* const color,
	const uint8_t num_colors);

/**
 * @brief Set the second PWM timer duty cycle.
 *
 * PWM2 is independent from PWM1 -- different channels can use different
 * timers. Assign channels to PWM2 via ktd202xSetChannelPWM() to use this
 * timer's duty cycle setting.
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] duty_cycle  Duty cycle register value (0-255). 0=always off,
 *                        255=always on, 128=50%.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetPWM2DutyCycle(const struct device* const dev, const uint8_t duty_cycle);

/**
 * @brief Reset the chip to power-on defaults.
 *
 * Writes the reset command to Reg0, waits 200us for the reset to complete,
 * then re-initializes the enable register to always-on mode. All cached
 * register state is reset to defaults.
 *
 * @param[in] dev  KTD202x device. Must not be NULL.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xReset(const struct device* const dev);

/**
 * @brief Enable or disable the chip (power down / standby).
 *
 * When disabled, the chip enters low-power mode where it is only active
 * when I2C lines (SCL and SDA) are both high. When enabled, the chip
 * is always active regardless of I2C bus state.
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] is_enabled  true to enable (always on), false to disable (standby).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetEnabled(const struct device* const dev, const bool is_enabled);

/**
 * @brief Single-shot flash (one blink then stop).
 *
 * Configures the LED for a one-shot flash: the LED turns on for
 * @p on_time_ms then turns off and does not repeat. Uses flash period
 * register value 127 (one-shot mode).
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] led_index   LED index (from devicetree).
 * @param[in] on_time_ms  On time in milliseconds (determines duty cycle
 *                        relative to one-shot period).
 * @param[in] brightness  Peak brightness (0-100%).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xFlashOnce(const struct device* const dev, const uint32_t led_index, const uint32_t on_time_ms, const uint8_t brightness);

/**
 * @brief Assign an LED channel to a specific PWM timer.
 *
 * Each channel in the LED_EN register can be set to OFF, ON, PWM1, or PWM2.
 * This function sets a single LED channel to use the specified PWM timer,
 * allowing independent blink/breathe timing per channel.
 *
 * @param[in] dev          KTD202x device. Must not be NULL.
 * @param[in] led_index    LED index (from devicetree).
 * @param[in] pwm_channel  PWM timer to assign (KTD202X_PWM_CHANNEL_1 or
 *                         KTD202X_PWM_CHANNEL_2).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetChannelPWM(const struct device* const dev, const uint32_t led_index, const KTD202xPWMChannel pwm_channel);

/**
 * @brief Select the active timer slot.
 *
 * The EN_RST register bits [2:0] control which timer slot is active.
 * Timer slots allow independent timing configurations.
 *
 * @param[in] dev   KTD202x device. Must not be NULL.
 * @param[in] slot  Timer slot to select (1-4).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetTimerSlot(const struct device* const dev, const KTD202xTimerSlot slot);

/**
 * @brief Set the enable mode for the chip.
 *
 * Controls when the chip is active based on I2C bus line state.
 * More granular than ktd202xSetEnabled(), which only toggles between
 * always-on and standby.
 *
 * @param[in] dev   KTD202x device. Must not be NULL.
 * @param[in] mode  Enable mode to set.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetEnableMode(const struct device* const dev, const KTD202xEnableMode mode);

/**
 * @brief Read the current LED enable register value.
 *
 * Returns the cached LED_EN register (0x04) value, which encodes
 * the mode (OFF/ON/PWM1/PWM2) for each of the 4 LED channels
 * in 2-bit fields.
 *
 * @param[in]  dev         KTD202x device. Must not be NULL.
 * @param[out] led_enable  Pointer to store the LED enable register value.
 *                         Must not be NULL.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xGetLEDEnable(const struct device* const dev, uint8_t* const led_enable);

/**
 * @brief Set the flash period directly.
 *
 * Programs the flash period register (0x01) with the period closest
 * to the requested value. The ramp linear bit (Reg1[7]) is preserved.
 *
 * Datasheet: value=0 -> 128ms, value>=1 -> 256 + value*128 ms.
 * Range: 128ms to 16.4s.
 *
 * @param[in] dev        KTD202x device. Must not be NULL.
 * @param[in] period_ms  Desired flash period in milliseconds (128-16384).
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetFlashPeriod(const struct device* const dev, const uint32_t period_ms);

/**
 * @brief Get the current fade (rise/fall) times.
 *
 * Reads the cached TRISE_TFALL register and converts the register
 * values back to milliseconds. Note: these are base times before
 * time scale multiplication.
 *
 * @param[in]  dev      KTD202x device. Must not be NULL.
 * @param[out] rise_ms  Pointer to store rise time in milliseconds.
 *                      Must not be NULL.
 * @param[out] fall_ms  Pointer to store fall time in milliseconds.
 *                      Must not be NULL.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xGetFadeTime(const struct device* const dev, uint32_t* const rise_ms, uint32_t* const fall_ms);

/**
 * @brief Get the current time scale setting.
 *
 * Returns the time scale multiplier from the cached EN_RST register.
 *
 * @param[in] dev  KTD202x device. Must not be NULL.
 * @return Current time scale (KTD202X_TSCALE_1X through KTD202X_TSCALE_8X).
 */
KTD202xTimeScale ktd202xGetTimeScale(const struct device* const dev);

/**
 * @brief Check if the ramp profile is linear.
 *
 * @param[in] dev  KTD202x device. Must not be NULL.
 * @return true if linear ramp, false if S-curve (logarithmic).
 */
bool ktd202xIsRampLinear(const struct device* const dev);

/**
 * @brief Set the PWM1 timer duty cycle.
 *
 * PWM1 is the default timer used by blink and breathe operations.
 * Writing directly allows custom duty cycles independent of
 * blink/breathe helpers.
 *
 * @param[in] dev         KTD202x device. Must not be NULL.
 * @param[in] duty_cycle  Duty cycle register value (0-255). 0=always off,
 *                        255=always on, 128=50%.
 * @return 0 on success; negative errno on failure.
 */
int ktd202xSetPWM1DutyCycle(const struct device* const dev, const uint8_t duty_cycle);

/**
 * @brief Get the maximum brightness register value.
 *
 * Determined by CONFIG_KTD202X_MAX_CURRENT_MA.
 *
 * @return Maximum brightness register value (0-191).
 */
uint8_t ktd202xGetMaxBrightness(void);

/**
 * @brief Get the configured maximum current in mA.
 *
 * @return Maximum current from Kconfig (1-24 mA).
 */
uint8_t ktd202xGetMaxCurrentMa(void);

#ifdef __cplusplus
}
#endif

#endif // KTD202X_H
