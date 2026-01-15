/*
 * Copyright (c) 2025 Makani Science
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * KTD2026/KTD2027 RGB LED Driver - Extended API
 *
 * This header provides extended functionality beyond the standard Zephyr LED API,
 * including breathing effects, fade timing control, and time scaling.
 */

#ifndef KTD202X_H
#define KTD202X_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Time scale multiplier for rise/fall times.
 * Applies to both rise and fall times set via ktd202x_set_fade_time().
 */
typedef enum {
	KTD202X_TSCALE_1X = 0,  /**< 1x (default) */
	KTD202X_TSCALE_2X = 1,  /**< 2x slower */
	KTD202X_TSCALE_4X = 2,  /**< 4x slower */
	KTD202X_TSCALE_8X = 3,  /**< 8x slower */
} ktd202x_time_scale_t;

/**
 * Set the rise and fall times for breathing/fade effects.
 *
 * The actual ramp time = (register_value * 96ms) * time_scale.
 * With 1x scale: 0=1.5ms, 96ms steps, max ~1.4s per ramp.
 * With 8x scale: max ~11s per ramp.
 *
 * @param dev      KTD202x device
 * @param rise_ms  Rise time in milliseconds (0-1440 with 1x scale)
 * @param fall_ms  Fall time in milliseconds (0-1440 with 1x scale)
 * @return 0 on success, negative errno on failure
 */
int ktd202x_set_fade_time(const struct device *dev, uint32_t rise_ms, uint32_t fall_ms);

/**
 * Set the time scale multiplier for rise/fall times.
 *
 * This scales all rise/fall times by the specified factor.
 * Useful for very slow breathing effects.
 *
 * @param dev    KTD202x device
 * @param scale  Time scale (1x, 2x, 4x, or 8x)
 * @return 0 on success, negative errno on failure
 */
int ktd202x_set_time_scale(const struct device *dev, ktd202x_time_scale_t scale);

/**
 * Set the ramp profile type.
 *
 * @param dev     KTD202x device
 * @param linear  true for linear ramp, false for S-curve (logarithmic)
 * @return 0 on success, negative errno on failure
 *
 * The S-curve (default) provides a more natural-looking fade that
 * matches human brightness perception. Linear is a simple linear ramp.
 */
int ktd202x_set_ramp_linear(const struct device *dev, bool linear);

/**
 * Start a breathing effect on an LED.
 *
 * The LED will smoothly fade up and down at the specified period,
 * using the current rise/fall times and ramp profile.
 *
 * @param dev         KTD202x device
 * @param led_index   LED index (from devicetree)
 * @param period_ms   Total period for one breath cycle (256ms-16.4s)
 * @param brightness  Peak brightness (0-100%)
 * @return 0 on success, negative errno on failure
 */
int ktd202x_breathe(const struct device *dev, uint32_t led_index,
		    uint32_t period_ms, uint8_t brightness);

/**
 * Start a breathing effect with a specific color.
 *
 * Like ktd202x_breathe() but for RGB LEDs with specific color values.
 *
 * @param dev         KTD202x device
 * @param led_index   LED index (from devicetree)
 * @param period_ms   Total period for one breath cycle (256ms-16.4s)
 * @param color       Array of color values (0-255 per channel)
 * @param num_colors  Number of color channels (must match LED definition)
 * @return 0 on success, negative errno on failure
 */
int ktd202x_breathe_color(const struct device *dev, uint32_t led_index,
			  uint32_t period_ms, const uint8_t *color, uint8_t num_colors);

/**
 * Get the maximum brightness register value.
 *
 * This is determined by CONFIG_KTD202X_MAX_CURRENT_MA.
 *
 * @return Maximum brightness register value (0-191)
 */
uint8_t ktd202x_get_max_brightness(void);

/**
 * Get the configured maximum current in mA.
 *
 * @return Maximum current from Kconfig (1-24 mA)
 */
uint8_t ktd202x_get_max_current_ma(void);

#ifdef __cplusplus
}
#endif

#endif /* KTD202X_H */
