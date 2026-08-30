# MCU Port Comparison

The goal of maintaining three firmware ports is to compare embedded-platform tradeoffs while holding the robot mechanics and navigation behavior approximately constant.

| Characteristic | Mega 2560 | ESP32 WROOM-32 | STM32F103C8T6 Blue Pill |
|---|---|---|---|
| Logic voltage | 5 V | 3.3 V | 3.3 V |
| Architecture | AVR 8-bit | Xtensa 32-bit | ARM Cortex-M3 32-bit |
| Hardware UART strategy | Serial1 for BT | UART2 for BT | USART2 for BT |
| I2C | D20/D21 | GPIO21/GPIO22 | PB7/PB6 |
| Motor PWM pins | D5/D6 | GPIO25/GPIO26 | PA6/PA7 TIM3 |
| Servo library | Arduino Servo | ESP32Servo | STM32duino Servo |
| HC-SR04 Echo level shifting | Not required for 5 V Mega input | Required | Required by project design |
| Current role | hardware baseline | port / future validation | port / future validation |

## Why keep the behavior equivalent?

Using the same finite-state machine, thresholds, scan positions, and command set makes later measurements more meaningful. Differences can then be attributed more clearly to controller implementation, timing, peripherals, PWM behavior, and resource usage rather than completely different navigation algorithms.

## Platform-specific differences that remain intentional

- GPIO assignments
- Servo implementation
- UART object and pins
- I2C pin initialization
- 3.3 V level-shifting requirements
- PWM/timer resource selection

## Future comparison metrics

- Flash/program size
- SRAM/RAM use
- Main-loop frequency
- MPU update rate
- Ultrasonic scan completion time
- Command-to-motor latency
- Closed-loop turn error
- Current draw of the controller electronics
- Ease of debugging and library support
- Bluetooth/telemetry capability
