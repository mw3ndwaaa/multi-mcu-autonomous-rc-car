# Wiring and Power Notes

## Common power architecture

The motor power path and logic/sensor power path are deliberately separated.

```text
12 V supply
   |
   +---------------------------> L298N motor Vs
   |
   +----> regulated 5 V buck ----+----> L298N logic 5 V
                                 +----> servo 5 V
                                 +----> 5 V peripherals where appropriate

All grounds share one common reference.
```

## L298N

The onboard regulator is not used.

- Remove `5V-EN`.
- 12 V motor supply -> L298N `Vs/+12V` terminal.
- Regulated 5.0 V -> L298N `5V` logic terminal.
- Common ground -> L298N GND.
- ENA/ENB jumpers are removed when PWM lines are connected from the MCU.

The module used during development produced about 7.8 V at its nominal 5 V terminal when the regulator jumper was installed, so the onboard regulator is permanently bypassed in this project.

## Servo

Power the pan servo from the regulated 5 V rail, not from an MCU GPIO.

```text
regulated 5 V -> servo V+
common GND    -> servo GND
MCU GPIO      -> servo signal
```

A local bulk capacitor near the servo power connector can help absorb current transients. Use a correctly rated electrolytic plus a small ceramic decoupling capacitor where appropriate.

## Arduino Mega 2560 logic levels

The Mega uses 5 V GPIO, so the normal 5 V HC-SR04 Echo signal can connect directly to its digital input.

The Mega-to-Bluetooth TX line may require a divider if the eventual Bluetooth module exposes a 3.3 V-only RX input. Verify the exact Bluetooth breakout before connecting it.

## ESP32 logic levels

ESP32 GPIO is 3.3 V only.

### HC-SR04 Echo

Reduce the 5 V Echo signal before GPIO19. A simple divider is suitable, for example:

```text
HC-SR04 ECHO ---- R1 ----+----> ESP32 GPIO19
                         |
                         R2
                         |
                        GND
```

Choose resistor values that produce approximately 3.3 V from a 5 V input.

### IR sensor outputs

If an IR obstacle module is powered at 5 V and its `OUT` line rises to 5 V, do not connect that output directly to ESP32 GPIO. Either:

- operate a compatible module at 3.3 V, or
- level-shift/divide the output.

### MPU6050

For the GY-521 style breakout in the ESP32 version, power the board from 3.3 V so any onboard I2C pull-ups also remain at a 3.3 V-safe level.

### Bluetooth

The reserved UART2 pins are GPIO17 TX and GPIO16 RX. A normal 3.3 V UART Bluetooth module can communicate directly at the logic level, but verify the module's VCC requirement separately from its UART logic level.

## STM32 Blue Pill logic levels

Treat all sensor inputs as 3.3 V signals even though some STM32F103 pins have limited 5 V tolerance in specific modes.

### HC-SR04 Echo

Use a divider/level shifter before PB1.

### IR outputs

Power compatible modules at 3.3 V or reduce 5 V outputs before the MCU.

### MPU6050

Power the GY-521 from 3.3 V in the Blue Pill implementation so SDA/SCL remain on a 3.3 V bus.

### Bluetooth

USART2 is reserved:

```text
PA2 TX -> Bluetooth RX
PA3 RX <- Bluetooth TX
```

Both MCU pins use 3.3 V logic.

## Grounding

Do not daisy-chain grounds through high-current motor paths. Use a common/star-style distribution point where practical:

```text
battery negative / common ground
   +---- L298N ground
   +---- buck ground
   +---- MCU ground
   +---- servo ground
   +---- sensor ground distribution
```

Keep motor-current wiring physically separate from low-level I2C and sensor wiring where practical.
