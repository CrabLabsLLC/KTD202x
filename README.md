# KTD202x Zephyr LED Driver

Zephyr RTOS driver for the Kinetic Technologies KTD2026/KTD2027 I2C RGB/RGBW LED controllers.

## Features

- Full Zephyr LED API support (`led_on`, `led_off`, `led_set_brightness`, `led_set_color`, `led_blink`)
- Hardware PWM blinking with configurable period and duty cycle
- Individual LED current control (0-24mA in 0.125mA steps)
- Named per-colour channel properties for different LED wiring (RGB, RBG, GRB, etc.)
- Support for single RGB LED or multiple individual LEDs
- Low power modes

## Supported Devices

| Device | Channels | I2C Address |
|--------|----------|-------------|
| KTD2026  | 3 (RGB)  | 0x30 |
| KTD2026B | 3 (RGB)  | 0x31 |
| KTD2026C | 3 (RGB)  | 0x32 |
| KTD2027  | 4 (RGBW) | 0x30 |

## Installation

Add to your project's `west.yml` or include directly as a Zephyr module.

## Device Tree Configuration

### Single RGB LED (Standard Wiring)

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/dt-bindings/led/led.h>

&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";
        wake-gpios = <&gpio1 4 (GPIO_ACTIVE_LOW | GPIO_OPEN_DRAIN)>;

        led_rgb: led_rgb@0 {
            label = "Status LED";
            reg = <0>;
            color-mapping = <LED_COLOR_ID_RED>,
                            <LED_COLOR_ID_GREEN>,
                            <LED_COLOR_ID_BLUE>;
        };
    };
};
```

`wake-gpios` is optional and omitted on most boards. It is the GPIO view of the controller's SDA line, so declaring it hands that pin to this driver; only wire it where the controller is actually put into shutdown and the I2C bus is otherwise idle. Configure it as
active-low and open-drain when the controller can enter its bus-line shutdown
state. The driver uses it only for the datasheet wake pulse before the first
I2C transaction.

### With Channel Remapping (e.g., RBG Wiring)

If your LED is wired differently (e.g., Blue on LED2, Green on LED3), name each
colour's hardware channel explicitly:

```dts
&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";
        /* R->LED1(ch0), G->LED3(ch2), B->LED2(ch1) */
        red-channel = <0>;
        green-channel = <2>;
        blue-channel = <1>;

        led_rgb: led_rgb@0 {
            label = "Status LED";
            reg = <0>;
            color-mapping = <LED_COLOR_ID_RED>,
                            <LED_COLOR_ID_GREEN>,
                            <LED_COLOR_ID_BLUE>;
        };
    };
};
```

These three properties replace a former `channel-map` array. Devicetree silently
discards any property whose name ends in `-map`: edtlib reserves that suffix for
nexus specifier mapping (`gpio-map`, `interrupt-map`), so `channel-map` never
reached the build and every value behaved as the default. The failure was
invisible — two different mapping values produced byte-identical output on
hardware — which is why the properties are now named individually.

### Channel Reference

| Wiring | red-channel | green-channel | blue-channel |
|--------|-------------|---------------|--------------|
| RGB (default) | `0` | `1` | `2` |
| RBG | `0` | `2` | `1` |
| GRB | `1` | `0` | `2` |
| GBR | `2` | `0` | `1` |
| BRG | `1` | `2` | `0` |
| BGR | `2` | `1` | `0` |

Each value is the hardware channel driving that colour: LED1=0, LED2=1, LED3=2,
LED4=3. Read the table as "which channel drives this colour", not as a
positional permutation.

### Three Separate LEDs

```dts
&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";

        led_r: led_r@0 {
            label = "Red LED";
            reg = <0>;
            color-mapping = <LED_COLOR_ID_RED>;
        };

        led_g: led_g@1 {
            label = "Green LED";
            reg = <1>;
            color-mapping = <LED_COLOR_ID_GREEN>;
        };

        led_b: led_b@2 {
            label = "Blue LED";
            reg = <2>;
            color-mapping = <LED_COLOR_ID_BLUE>;
        };
    };
};
```

## Kconfig Options

```
CONFIG_LED=y
CONFIG_LED_KTD202X=y
```

## Usage Example

```c
#include <zephyr/drivers/led.h>

const struct device *led_dev = DEVICE_DT_GET(DT_NODELABEL(ktd2026));

void main(void)
{
    if (!device_is_ready(led_dev)) {
        printk("LED device not ready\n");
        return;
    }

    /* Set RGB color (Red=255, Green=0, Blue=128) */
    uint8_t color[3] = {255, 0, 128};
    led_set_color(led_dev, 0, 3, color);

    /* Blink: 500ms on, 500ms off */
    led_blink(led_dev, 0, 500, 500);

    /* Turn off */
    led_off(led_dev, 0);
}
```

## API Reference

### led_on(dev, led)
Turn on LED at full brightness.

### led_off(dev, led)
Turn off LED.

### led_set_brightness(dev, led, brightness)
Set brightness (0-100%). For single-color LED definitions only.

### led_set_color(dev, led, num_colors, color)
Set RGB color. `color` is an array of 3 bytes (R, G, B) in range 0-255.

### led_blink(dev, led, delay_on, delay_off)
Start hardware PWM blinking with specified on/off times in milliseconds.
- Minimum period: 128ms
- Maximum period: ~16s

### led_get_info(dev, led, info)
Get LED information (label, color mapping).

## Vendor API (`ktd202x.h`)

Beyond the Zephyr LED API, the driver exposes controller-specific entry points:

- `ktd202xBreathe()` / `ktd202xBreatheColor()` — hardware fade-in/fade-out over a
  full period, run by the controller's own timers with no CPU involvement.
- `ktd202xSetFadeTime()`, `ktd202xSetTimeScale()`, `ktd202xSetRampLinear()`,
  `ktd202xSetFlashPeriod()` — ramp and cadence control.
- `ktd202xSetEnabled()`, `ktd202xSetEnableMode()`, `ktd202xReset()` — power state
  and a return to the power-on-reset register set.
- `ktd202xFlashOnce()`, `ktd202xSetChannelPWM()`, `ktd202xSetTimerSlot()`,
  `ktd202xSetPWM1DutyCycle()`, `ktd202xSetPWM2DutyCycle()` — one-shot flash and
  per-channel PWM assignment.
- `ktd202xGetFadeTime()`, `ktd202xGetTimeScale()`, `ktd202xIsRampLinear()`,
  `ktd202xGetLEDEnable()`, `ktd202xGetMaxBrightness()`, `ktd202xGetMaxCurrentMa()`
  — readback.

## Hardware Notes

- LED current is programmable from 0-24mA in 0.125mA steps (192 levels)
- Flash period can be set from 128ms to ~16 seconds
- Hardware supports rise/fall time ramping (not currently exposed in driver)
- Chip has low-power sleep mode with register retention

## References

- [KTD2026/KTD2027 Datasheet](https://www.kinet-ic.com/uploads/KTD2026-7-04h.pdf)
- [Kinetic Technologies KTD2026 Product Page](https://www.kinet-ic.com/ktd2026/)

## License

SPDX-License-Identifier: MIT
