# Validation Plan

The Mega 2560 is the current validated reference implementation. Every other controller port should repeat the same subsystem tests before autonomous operation.

## Test order after an MCU swap

1. **Power rails only**
   - Verify motor supply voltage.
   - Verify regulated 5 V rail.
   - Verify the controller's 3.3 V rail where applicable.
   - Verify common ground.

2. **L298N control with wheels raised**
   - Left forward/reverse.
   - Right forward/reverse.
   - Both forward/reverse.
   - In-place left/right.
   - Determine minimum reliable PWM for the new MCU port.

3. **IR sensors**
   - Confirm each physical sensor maps to the correct software label.
   - Reconfirm active-low behavior.
   - Check that 3.3 V ports never receive a 5 V sensor output.

4. **HC-SR04**
   - Confirm distance at approximately 10, 20, 35 and 50 cm.
   - Confirm invalid/no-echo behavior.

5. **Pan servo**
   - Reconfirm mechanical center.
   - Reconfirm increasing command angle points in the expected physical direction.
   - Verify endpoints do not bind.

6. **Five-angle scan**
   - Place an obstacle asymmetrically.
   - Confirm the reported best direction matches the physical opening.

7. **MPU6050**
   - I2C device detected.
   - Yaw stable while stationary.
   - Left rotation sign and right rotation sign confirmed.
   - Approximate 90 degree manual rotation produces a similar integrated yaw change.

8. **Closed-loop turns**
   - Start at 20-40 degrees.
   - Test left and right repeatedly.
   - Record commanded angle, final angle and error.
   - Tune slow/medium/fast turn PWM values per MCU/platform.

9. **Manual command safety**
   - `S` always stops.
   - Front IR blocks manual forward.
   - Rear IR blocks manual reverse.

10. **Autonomous testing**
    - Low speed first.
    - Open controlled area.
    - Test single obstacle cases before cluttered environments.

## Quantitative turn table

Use a table like this for every MCU:

| MCU | Command | Trial | Final rotation | Absolute error |
|---|---:|---:|---:|---:|
| Mega | -40 deg | 1 |  |  |
| Mega | +40 deg | 1 |  |  |
| ESP32 | -40 deg | 1 |  |  |
| ESP32 | +40 deg | 1 |  |  |
| STM32 | -40 deg | 1 |  |  |
| STM32 | +40 deg | 1 |  |  |

Once enough trials are collected, compare mean absolute turn error, variance, loop/update rate, memory use, and response latency.

## Current physical measurements

IR trigger distances measured on the robot:

| Sensor | Trigger distance |
|---|---:|
| FL | 5 cm |
| FR | 11 cm |
| RL | 6 cm |
| RR | 8 cm |

Measured MPU yaw convention on the current mechanical build:

```text
left  -> negative yaw
right -> positive yaw
```
