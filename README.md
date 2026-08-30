# Multi-MCU Autonomous RC Car

## Prototype

![RC Car Prototype](images/prototype/rc-car-overview.jpg)

The first hardware revision was assembled using breadboards and jumper wiring
to support rapid subsystem development and firmware testing.

The next revision will replace the temporary wiring with soldered perfboard
and ultimately a custom PCB designed for the Mega 2560, ESP32, and STM32F103.A 4WD autonomous mobile-robot platform used to compare embedded control implementations across **Arduino Mega 2560**, **ESP32**, and **STM32F103C8T6 Blue Pill** hardware.

The same navigation architecture is retained across all three firmware ports: close-range IR protection, pan-mounted ultrasonic scanning, MPU6050 yaw feedback, L298N differential drive, manual/automatic operating modes, and a serial command interface.

> **Validation status:** the Arduino Mega 2560 is the current hardware-validated baseline. The ESP32 and STM32 Blue Pill ports are implementation-complete but should be bench-tested subsystem-by-subsystem before autonomous driving.

## Firmware implementations

| MCU | Firmware status | Debug interface | Bluetooth interface |
|---|---|---|---|
| Arduino Mega 2560 | Hardware baseline | USB `Serial` | `Serial1`, D18/D19 |
| ESP32 NodeMCU / WROOM-32 | Port complete, validation pending | USB `Serial` | UART2, GPIO17/GPIO16 |
| STM32F103C8T6 Blue Pill | Port complete, validation pending | `Serial` / USB CDC depending board setup | USART2, PA2/PA3 |

## Current capabilities

- Four TT motors controlled as left/right motor pairs through an L298N driver
- Four digital IR sensors for close-range obstacle protection
- HC-SR04 ultrasonic ranging sensor on a pan servo
- Five-direction ultrasonic environment scan
- MPU6050 yaw feedback for closed-loop in-place turning
- Finite-state-machine autonomous navigation
- Front and rear collision safeguards
- Turn progress and timeout recovery
- Non-blocking buzzer status patterns
- Safe startup: the robot remains stopped until commanded to begin autonomous mode
- USB/debug serial command interface
- Hardware UART reserved and enabled for a future external Bluetooth module on every MCU port

## Repository structure

```text
multi-mcu-autonomous-rc-car/
├── firmware/
│   ├── mega2560/
│   │   └── rc_car_mega2560/
│   │       └── rc_car_mega2560.ino
│   ├── esp32/
│   │   └── rc_car_esp32/
│   │       └── rc_car_esp32.ino
│   └── stm32_bluepill/
│       └── rc_car_stm32_bluepill/
│           └── rc_car_stm32_bluepill.ino
├── docs/
│   ├── pinout.md
│   ├── wiring.md
│   ├── testing.md
│   └── mcu-comparison.md
├── images/
│   └── README.md
├── .gitignore
└── README.md
```

## Control architecture

```text
                      +------------------+
                      |  IR Sensors      |
                      | FL FR RL RR      |
                      +--------+---------+
                               |
                               v
+-----------+        +------------------+        +------------------+
| HC-SR04   |------->| Navigation FSM   |------->| L298N + Motors   |
| + Pan     |        |                  |        |                  |
+-----------+        +--------+---------+        +------------------+
                              ^
                              |
                      +-------+--------+
                      | MPU6050 Yaw    |
                      | Turn Feedback  |
                      +----------------+
```

Autonomous state flow:

```text
FORWARD
   |
   | obstacle <= scan threshold
   v
SCANNING
   |
   | choose best valid direction
   v
TURNING
   |
   | target yaw reached
   v
FORWARD

Emergency / blocked path:
FORWARD or TURNING -> REVERSING -> SCANNING
```

## Shared physical sensor layout

```text
                    FRONT
                      ^

             FL  \         /  FR
                  \       /
                   +-----+
                   | CAR |
                   +-----+
                  /       \
             RL  /         \  RR

The pan-mounted HC-SR04 covers the forward center region.
```

Measured IR trigger distances on the current physical robot:

| Sensor | Approximate trigger distance |
|---|---:|
| Front-left | 5 cm |
| Front-right | 11 cm |
| Rear-left | 6 cm |
| Rear-right | 8 cm |

These sensors are used as close-range safety switches rather than precision ranging sensors.

## Power architecture

The L298N module's onboard 5 V regulator is deliberately bypassed. During development its nominal 5 V terminal produced an abnormal voltage, so the final architecture removes the `5V-EN` jumper and uses an external regulated rail.

```text
                         12 V SUPPLY
                              |
               +--------------+--------------+
               |                             |
               v                             v
          L298N motor Vs                regulated 5 V
               |                             |
               v                    +--------+----------+
            4 motors                |        |          |
                                    v        v          v
                                 L298N     servo    5 V devices
                                  logic

All grounds share one common reference.
```

The Mega is a 5 V-logic MCU. The ESP32 and Blue Pill are 3.3 V-logic MCUs and require additional signal-level precautions; see [`docs/wiring.md`](docs/wiring.md).

## Shared firmware calibration

Each firmware contains the same high-level calibration constants:

```cpp
const int SERVO_CENTER = 90;
const bool SERVO_POSITIVE_IS_LEFT = true;

const uint8_t DRIVE_SPEED = 180;
const uint8_t REVERSE_SPEED = 160;

const float DIST_SCAN_TRIGGER_CM = 35.0f;
const float DIST_SAFE_CM = 40.0f;
```

The physical robot's measured yaw convention is:

```text
Left rotation  -> negative yaw
Right rotation -> positive yaw
```

Re-check servo center, motor direction, PWM thresholds, and yaw sign after changing controller boards because wiring and peripherals can change even when the mechanical platform stays the same.

## Serial commands

The command set is identical across all three versions:

| Command | Action |
|---|---|
| `A` | Start autonomous mode |
| `S` | Stop |
| `F` | Manual forward |
| `B` | Manual reverse |
| `L` | Manual left |
| `R` | Manual right |
| `C` | Center pan servo |
| `H` or `?` | Print help |

The robot boots into `STOPPED` mode.

## Arduino Mega 2560

Firmware:

```text
firmware/mega2560/rc_car_mega2560/rc_car_mega2560.ino
```

Required library:

- `MPU6050_light` by rfetick

The standard `Wire` and `Servo` libraries are supplied by the Arduino AVR core.

## ESP32 NodeMCU / ESP32-WROOM-32

Firmware:

```text
firmware/esp32/rc_car_esp32/rc_car_esp32.ino
```

Required libraries:

- `MPU6050_light` by rfetick
- `ESP32Servo`

Target: classic ESP32 NodeMCU/WROOM-32 style development board. This pin map is intentionally not advertised as drop-in for every ESP32-S2/S3/C3 board.

Important: ESP32 GPIO is **3.3 V only**. HC-SR04 Echo must be reduced to 3.3 V, and any 5 V-powered IR sensor outputs must also be made 3.3 V-safe.

## STM32F103C8T6 Blue Pill

Firmware:

```text
firmware/stm32_bluepill/rc_car_stm32_bluepill/rc_car_stm32_bluepill.ino
```

Required software:

- STM32duino / `STM32 MCU based boards` Arduino core
- `MPU6050_light` by rfetick

The STM32duino `Servo` library is included with the core. The Blue Pill variant uses TIM2 for the Servo library, so the motor-enable PWM outputs are deliberately assigned to PA6/PA7 on TIM3.

Important: design Blue Pill sensor inputs as **3.3 V logic**. The HC-SR04 Echo line is level shifted/divided in the documented wiring.

## Development and validation workflow

The Mega baseline was developed by testing subsystems independently before integration:

1. L298N and all four motors
2. Four IR sensors
3. HC-SR04 distance measurement
4. Pan-servo mechanical center and sweep
5. Ultrasonic scanning
6. MPU6050 I2C communication and yaw direction
7. Permanent wiring harness and power distribution
8. Closed-loop turning
9. Autonomous state-machine tuning

Use the same sequence for the ESP32 and STM32 ports. Do not jump directly to autonomous testing after an MCU swap.

See [`docs/testing.md`](docs/testing.md).

## Design choices

### Closed-loop turning

Timed turns change with battery voltage, floor friction, wheel loading, and motor mismatch. The controller instead calculates a relative yaw target and uses MPU6050 feedback to stop near that target.

### No tilt servo

Horizontal free-space scanning is sufficient for the current navigation objective. Removing the tilt axis reduces wiring, current consumption, mechanical complexity, and failure points.

### External Bluetooth UART reserved on every MCU

Using the same external serial Bluetooth interface allows the communications layer to stay comparable between controllers. The ESP32 can later gain a separate native Bluetooth/BLE implementation without changing the baseline comparison.

## Known limitations

- HC-SR04 measurements still use `pulseIn()`, which can block for up to 25 ms per sample.
- MPU6050 yaw is gyro-integrated and drifts over long periods because the MPU6050 has no magnetometer.
- Forward-motion stall detection is intentionally not inferred from accelerometer vibration; wheel encoders are a better solution.
- L298N efficiency is lower than modern MOSFET motor drivers.
- ESP32 and STM32 ports require hardware validation and controller-specific PWM tuning.

## Planned improvements

- Wheel encoders and odometry
- Quantitative closed-loop turn testing
- Proportional/PID heading tuning
- Bluetooth remote control and telemetry
- Battery-voltage monitoring
- Modern MOSFET motor driver
- Native ESP32 Bluetooth/BLE implementation
- Data logging and cross-MCU timing/performance comparison
- CAD/electrical schematic images and wiring-harness documentation

## Project objective

This repository is intended to show more than a single obstacle-avoidance demo. The same robot is being used as a controlled platform for comparing embedded architectures, peripheral handling, real-time behavior, firmware portability, and closed-loop robotics control across three microcontroller families.
