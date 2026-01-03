# KTD202x Zephyr LED Driver

Zephyr RTOS driver for the Kinetic Technologies KTD2026/KTD2027 I2C RGB/RGBW LED controllers.

## Features

- Full Zephyr LED API support (`led_on`, `led_off`, `led_set_brightness`, `led_set_color`, `led_blink`)
- Hardware PWM blinking with configurable period and duty cycle
- Individual LED current control (0-24mA in 0.125mA steps)
- Configurable channel mapping for different LED wiring (RGB, RBG, GRB, etc.)
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
#include <zephyr/dt-bindings/led/led.h>

&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";

        led_rgb {
            label = "Status LED";
            index = <0>;
            color-mapping = <LED_COLOR_ID_RED>,
                            <LED_COLOR_ID_GREEN>,
                            <LED_COLOR_ID_BLUE>;
        };
    };
};
```

### With Channel Remapping (e.g., RBG Wiring)

If your LED is wired differently (e.g., Blue on LED2, Green on LED3):

```dts
&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";
        /* Maps: R->LED1(ch0), G->LED3(ch2), B->LED2(ch1) */
        channel-map = <0 2 1>;

        led_rgb {
            label = "Status LED";
            index = <0>;
            color-mapping = <LED_COLOR_ID_RED>,
                            <LED_COLOR_ID_GREEN>,
                            <LED_COLOR_ID_BLUE>;
        };
    };
};
```

### Channel Map Reference

| Wiring | channel-map |
|--------|-------------|
| RGB (default) | `<0 1 2>` or omit |
| RBG | `<0 2 1>` |
| GRB | `<1 0 2>` |
| GBR | `<1 2 0>` |
| BRG | `<2 0 1>` |
| BGR | `<2 1 0>` |

### Three Separate LEDs

```dts
&i2c0 {
    ktd2026: ktd2026@30 {
        compatible = "kinetic,ktd202x";
        reg = <0x30>;
        status = "okay";

        led_r {
            label = "Red LED";
            index = <0>;
            color-mapping = <LED_COLOR_ID_RED>;
        };

        led_g {
            label = "Green LED";
            index = <1>;
            color-mapping = <LED_COLOR_ID_GREEN>;
        };

        led_b {
            label = "Blue LED";
            index = <2>;
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

## Hardware Notes

- LED current is programmable from 0-24mA in 0.125mA steps (192 levels)
- Flash period can be set from 128ms to ~16 seconds
- Hardware supports rise/fall time ramping (not currently exposed in driver)
- Chip has low-power sleep mode with register retention

## References

- [KTD2026/KTD2027 Datasheet](https://www.kinet-ic.com/uploads/KTD2026-7-04h.pdf)
- [Kinetic Technologies KTD2026 Product Page](https://www.kinet-ic.com/ktd2026/)

## License

SPDX-License-Identifier: Apache-2.0
