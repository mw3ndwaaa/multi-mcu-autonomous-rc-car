# Hardware Design Recommendations

## Prototype Wiring

Breadboards and jumper wires were used during early subsystem development because
they allowed rapid changes to sensor connections, pin assignments, and firmware.

However, loose jumper connections are unsuitable for a mobile robotic platform.
Vehicle vibration, repeated movement, and motor-induced electrical noise can cause
intermittent connections.

An intermittent MPU6050 connection was observed during development, causing I2C
communication loss and frozen yaw measurements.

## Recommended Implementation

For reliable vehicle operation, the control electronics should be transferred from
breadboard wiring to one of the following:

1. A neatly soldered perfboard for intermediate prototypes.
2. A purpose-designed PCB for the final implementation.

A custom PCB is preferred because it provides:

- secure electrical connections
- controlled power distribution
- common grounding
- local decoupling capacitors
- locking sensor connectors
- reduced wiring complexity
- improved vibration resistance
- easier assembly and troubleshooting

## Future PCB Development

Dedicated controller interfaces are planned for:

- Arduino Mega 2560
- ESP32
- STM32F103 Blue Pill

The long-term design will use a common vehicle interface for the motor driver,
IR sensors, ultrasonic scanner, MPU6050, servo, Bluetooth module, and power
distribution while adapting the logic interface for each MCU.