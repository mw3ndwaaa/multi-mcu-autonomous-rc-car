# Pin Assignments

The three firmware ports keep the same functional architecture but use controller-specific pins.

## Arduino Mega 2560

| Function | Mega pin |
|---|---|
| L298N ENA | D5 PWM |
| L298N ENB | D6 PWM |
| L298N IN1 | D22 |
| L298N IN2 | D23 |
| L298N IN3 | D24 |
| L298N IN4 | D25 |
| Buzzer | D8 |
| Pan servo signal | D9 |
| Bluetooth TX from MCU | D18 / TX1 |
| Bluetooth RX into MCU | D19 / RX1 |
| MPU6050 SDA | D20 / SDA |
| MPU6050 SCL | D21 / SCL |
| IR front-left | D30 |
| IR front-right | D31 |
| IR rear-left | D32 |
| IR rear-right | D33 |
| HC-SR04 TRIG | D34 |
| HC-SR04 ECHO | D35 |

## ESP32 NodeMCU / ESP32-WROOM-32

| Function | ESP32 pin |
|---|---|
| L298N ENA | GPIO25 PWM |
| L298N ENB | GPIO26 PWM |
| L298N IN1 | GPIO27 |
| L298N IN2 | GPIO14 |
| L298N IN3 | GPIO32 |
| L298N IN4 | GPIO33 |
| Buzzer | GPIO13 |
| Pan servo signal | GPIO18 |
| Bluetooth TX from MCU | GPIO17 / UART2 TX |
| Bluetooth RX into MCU | GPIO16 / UART2 RX |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| IR front-left | GPIO34 |
| IR front-right | GPIO35 |
| IR rear-left | GPIO36 |
| IR rear-right | GPIO39 |
| HC-SR04 TRIG | GPIO23 |
| HC-SR04 ECHO | GPIO19 through level divider |

GPIO34/35/36/39 are input-only on the classic ESP32 and are deliberately used only for IR sensor outputs.

## STM32F103C8T6 Blue Pill

| Function | Blue Pill pin |
|---|---|
| L298N ENA | PA6 / TIM3 PWM |
| L298N ENB | PA7 / TIM3 PWM |
| L298N IN1 | PB12 |
| L298N IN2 | PB13 |
| L298N IN3 | PB14 |
| L298N IN4 | PB15 |
| Buzzer | PB8 |
| Pan servo signal | PA8 |
| Bluetooth TX from MCU | PA2 / USART2 TX |
| Bluetooth RX into MCU | PA3 / USART2 RX |
| MPU6050 SDA | PB7 |
| MPU6050 SCL | PB6 |
| IR front-left | PA4 |
| IR front-right | PA5 |
| IR rear-left | PB10 |
| IR rear-right | PB11 |
| HC-SR04 TRIG | PB0 |
| HC-SR04 ECHO | PB1 through level divider |

### Blue Pill timer choice

The STM32duino Blue Pill Servo library uses TIM2 as its timing resource. ENA and ENB are therefore assigned to PA6/PA7, which provide TIM3 PWM, avoiding the servo/PWM timer conflict.

### Blue Pill serial ports

- Default debug `Serial`: normally USART1 on PA9/PA10 unless USB CDC is selected/configured.
- External Bluetooth: separate USART2 on PA2/PA3.
