# Touch LED Lamp Controller

A battery-powered, touch-controlled LED lamp built with an Arduino (ATmega328P). Tap to toggle the lamp on/off, hold to smoothly dim. Brightness is saved to EEPROM and the microcontroller enters deep sleep when the lamp is off for minimal power consumption.

## Features

- **Touch on/off** - Quick tap toggles the lamp
- **Smooth dimming** - Hold to adjust brightness continuously
- **Direction memory** - Dimming direction alternates each hold (up then down, etc.)
- **Brightness persistence** - Last brightness level saved to EEPROM and restored on power-up
- **Deep sleep** - MCU enters `SLEEP_MODE_PWR_DOWN` when lamp is off, waking on touch via interrupt
- **Ultrasonic PWM** - Timer1 set to ~31 kHz to eliminate audible coil whine from the boost converter
- **Power optimization** - Unused peripherals (ADC, SPI, TWI, USART, Timer2) disabled to reduce current draw

## Hardware

### Components

| Component | Purpose |
|---|---|
| Arduino Nano / ATmega328P | Microcontroller |
| TTP223 | Capacitive touch sensor |
| TP4056 | Li-Ion battery charger module |
| Boost converter (set to 12 V) | Steps up battery voltage for LED strip |
| IRLZ44N N-channel MOSFET | Switches the 12 V LED strip via PWM |
| 10 k&Omega; resistor | Gate pull-down on IRLZ44N (ensures LED off when MCU is unpowered) |
| 100 &Omega; resistor | Gate series resistor (limits ringing / inrush) |
| 12 V LED strip | Light output |
| Li-Ion 18650 battery | Power source |

### TTP223 Configuration

| Jumper | Setting | Effect |
|---|---|---|
| A | OPEN | Active HIGH output |
| B | OPEN | Momentary mode |

### Wiring

```
TTP223 OUT ──> Arduino Pin 2  (INT0, wake interrupt)
Arduino Pin 9 ──> 100 Ohm ──> IRLZ44N Gate
                                  │
                                10k Ohm
                                  │
                                 GND

IRLZ44N Drain ──> LED Strip (−)
IRLZ44N Source ──> GND

Boost Converter 12 V OUT ──> LED Strip (+)
Battery (+) ──> TP4056 B+ ──> Boost Converter IN
```
<img width="1315" height="688" alt="image" src="https://github.com/user-attachments/assets/f239a7ad-be6d-4f71-8d00-a9bcc2e3af56" />




## How It Works

1. **Startup** - PWM frequency is set to ~31 kHz (ultrasonic). The saved brightness is loaded from EEPROM. The lamp starts in the OFF state and immediately enters deep sleep.

2. **Touch to wake** - A rising edge on Pin 2 fires an interrupt that wakes the MCU from sleep, toggles the lamp ON at the saved brightness.

3. **Tap (< 300 ms)** - Toggles the lamp on or off.

4. **Hold (>= 300 ms)** - While the lamp is on, brightness ramps up or down. On release, the direction reverses for the next hold. The new brightness is saved to EEPROM after a 2-second debounce delay.

5. **Sleep** - When the lamp is off and the touch pin is idle, the MCU enters `SLEEP_MODE_PWR_DOWN`, drawing minimal current until the next touch.

## Software Configuration

Key constants in the sketch:

| Constant | Default | Description |
|---|---|---|
| `HOLD_THRESHOLD` | 300 ms | Tap vs hold detection threshold |
| `DIM_STEP_INTERVAL` | 20 ms | Speed of brightness ramping |
| `DIM_STEP` | 3 | Brightness change per step (0-255 range) |
| `MIN_BRIGHTNESS` | 10 | Lowest dimming level |
| `MAX_BRIGHTNESS` | 255 | Full brightness |
| `DEFAULT_BRIGHTNESS` | 128 | Initial brightness on first boot |
| `SAVE_DELAY` | 2000 ms | Delay before writing brightness to EEPROM |

## Building & Uploading

1. Open `LedLamp-withTouchButton.ino` in the Arduino IDE
2. Select your board (e.g. **Arduino Nano**, **ATmega328P**)
3. Select the correct COM port
4. Click **Upload**

## License

This project is open source. Feel free to use and modify.
