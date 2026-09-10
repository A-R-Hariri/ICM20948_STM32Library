# ICM-20948 STM32 Library with DMP

STM32 HAL port of the SparkFun ICM-20948 library, with the InvenSense DMP firmware image included and loaded at startup. The repository is a complete STM32CubeIDE project targeting the STM32F103C8T6 (Blue Pill), with a working example that reads 9-axis quaternions, raw accelerometer, raw gyroscope and uncalibrated magnetometer data from the DMP FIFO over I2C.

## Overview

The upstream library is built on Arduino `TwoWire`. Here the transport is replaced by two HAL-based transfer functions, so the driver runs on bare STM32 HAL with no Arduino layer. Everything below the transport, including the DMP firmware loader, register bank switching, AK09916 auxiliary I2C setup and the FIFO parser, is kept intact so upstream behaviour is preserved.

- I2C transport through `HAL_I2C_Mem_Read` and `HAL_I2C_Mem_Write`
- DMP firmware loading, start address configuration and gyro scale factor setup
- DMP FIFO readout and header parsing (`readDMPdataFromFIFO`)
- Full register-level API of the C backbone (`ICM_20948_C.c`) and the C++ wrapper (`ICM_20948.cpp`)
- Complete CubeIDE project: `.ioc`, linker script, startup file and HAL configuration

## Repository layout

```
Core/Inc/ICM_20948.h              C++ class declarations (ICM_20948, ICM_20948_I2C)
Core/Inc/ICM_20948_C.h            C backbone, status codes, ICM_20948_USE_DMP define (line 29)
Core/Inc/ICM_20948_DMP.h          DMP memory map, ODR registers, FIFO data structures
Core/Inc/icm20948_img.dmp3a.h     DMP3a firmware image, loaded to 0x1000
Core/Inc/ICM_20948_REGISTERS.h    ICM-20948 register map
Core/Inc/AK09916_REGISTERS.h      AK09916 magnetometer register map
Core/Src/ICM_20948.cpp            C++ wrapper, initializeDMP, I2C serif functions
Core/Src/ICM_20948_C.c            C backbone
Core/Src/main.cpp                 Example application, I2C_write / I2C_read, CAN and GPIO setup
ICM20948_STM32Library.ioc         CubeMX configuration
STM32F103C8TX_FLASH.ld            Linker script
```

## Hardware configuration

Target: STM32F103C8T6, LQFP48. HSE with PLL x9 for a 72 MHz system clock, APB1 at 36 MHz, APB2 at 72 MHz, flash latency 2.

| Signal | Pin | Notes |
| --- | --- | --- |
| I2C2_SCL | PB10 | 100 kHz standard mode |
| I2C2_SDA | PB11 | |
| LED1 | PB8 | Push-pull output, status indicator |
| CAN_RX | PA11 | CAN1 peripheral, 1 Mbit/s, filtered on standard ID 0x446 |
| CAN_TX | PA12 | Transmit header prepared for standard ID 0x69, 8-byte payload |
| SWDIO / SWCLK | PA13 / PA14 | Debug |

The sensor is addressed at 0x68 in `ICM_20948_write_I2C` and `ICM_20948_read_I2C`, so AD0 on the breakout is tied low. For 0x69, change the two `I2C_write` / `I2C_read` calls at the bottom of `ICM_20948.cpp`.

## Transport layer

The library calls two functions declared in `main.h` and defined at the end of `main.cpp`:

```c
HAL_StatusTypeDef I2C_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
HAL_StatusTypeDef I2C_read (uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
```

Both wrap the blocking HAL memory access calls on `hi2c2` with a 10 ms timeout and a left-shifted 8-bit address. Moving to another STM32 family or another I2C instance means editing these two functions and regenerating the CubeMX project. Nothing else in the library touches the HAL.

## DMP configuration

DMP support is enabled by `#define ICM_20948_USE_DMP` at line 29 of `ICM_20948_C.h`. With that define active, `begin()` runs a minimal startup and `initializeDMP()` performs the rest of the sequence:

- Accelerometer full scale +/- 4 g, gyroscope full scale 2000 dps
- Gyroscope DLPF enabled
- Sample rate dividers set to 19, giving roughly 55 Hz on the gyroscope and 56.25 Hz on the accelerometer
- I2C_SLV0 configured to read 10 bytes from AK09916 register 0x03 each cycle, I2C_SLV1 configured to trigger single measurement mode
- DMP3a image loaded, start address set to 0x1000, gyro scale factor set for 55 Hz and 2000 dps

The example then enables the DMP sensors it needs and starts the FIFO:

```cpp
myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);

myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0);   // maximum rate
myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0);
myICM.setDMPODRrate(DMP_ODR_Reg_Gyro,  0);
myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0);

myICM.enableFIFO();
myICM.enableDMP();
myICM.resetDMP();
myICM.resetFIFO();
```

LED1 pulses after a successful `begin()` and stays on once the full DMP configuration returns `ICM_20948_Stat_Ok`.

## Reading data

The main loop calls `readDMPdataFromFIFO` and checks the packet header bitmap before reading any field. Quat9 carries three signed Q30 components; the scalar term is recovered from the unit norm constraint:

```cpp
q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;   // 2^30
q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
```

Roll, pitch and yaw are then computed in degrees, with the pitch argument clamped to [-1, 1] before `asin`. When the status returns `ICM_20948_Stat_FIFOMoreDataAvail` the loop reads again immediately; otherwise it waits 10 ms.

## Build and flash

1. Clone the repository and import it into STM32CubeIDE with File > Open Projects from File System.
2. Build the Debug configuration.
3. Connect an ST-Link to SWDIO, SWCLK, GND and 3V3, then flash.
4. Wire the IMU to PB10 and PB11 with pull-ups, power it at 3.3 V, and tie AD0 low.
5. Attach the debugger and watch `roll`, `pitch`, `yaw`, `ax`, `gx` and `mx` in the live expressions view.

## License

MIT. See `LICENSE`.

The original library is the SparkFun ICM-20948 Arduino Library, also MIT licensed. The DMP firmware image belongs to TDK InvenSense and is redistributed under the terms of the upstream library.

## Author

Amir Hariri