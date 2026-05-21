# ICM-20948 STM32 Library with DMP

STM32 HAL C port of the SparkFun ICM-20948 Arduino library, with onboard DMP (Digital Motion Processor) support. Built and tested on STM32F103C8 (Blue Pill).

## Why this exists

The ICM-20948 is a 9-axis IMU (accelerometer, gyroscope, magnetometer) with an onboard DMP that runs sensor fusion in hardware and outputs quaternions without spending host MCU cycles. The vendor and SparkFun support Arduino directly, but loading the DMP firmware blob and bringing it up on STM32 with the HAL stack is undocumented register-level work. This port does that.

## Features

- DMP firmware loading and configuration on STM32 HAL
- Quaternion output from the DMP (9-axis and game rotation vector)
- Raw accel, gyro, mag readings with configurable full-scale ranges
- FIFO-based DMP data readout
- Complete STM32CubeIDE project: `.ioc`, linker script, and HAL configuration included

## Hardware

- MCU: STM32F103C8 (Blue Pill). Portable to any STM32 with the same HAL peripherals.
- Sensor: TDK InvenSense ICM-20948 9-axis IMU.

## Quick start

1. Open the project in STM32CubeIDE.
2. Wire the IMU to the STM32 per the `.ioc` pin assignments.
3. Build, flash, observe quaternion output.

## Credits

Original library: SparkFun ICM-20948 Arduino Library (MIT).

## Author

Amir Hariri.
