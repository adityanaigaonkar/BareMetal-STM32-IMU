# BareMetal-STM32-IMU

A bare-metal STM32 attitude estimation and telemetry system built using CMSIS and register-level programming. The project interfaces with an MPU6050 IMU over I2C, performs gyroscope and accelerometer calibration, estimates roll and pitch angles using a complementary filter, and transmits orientation data through UART for telemetry and monitoring applications.

## Features

- Bare-metal STM32 development using CMSIS
- MPU6050 interfacing via I2C
- Accelerometer and gyroscope data acquisition
- Gyroscope offset calibration
- Automatic zero-position calibration
- Roll and pitch angle estimation
- Complementary filter implementation
- Noise reduction and drift compensation
- UART communication
- ESP32 telemetry support

## Hardware Used

- STM32F4 Microcontroller
- MPU6050 6-Axis IMU
- ESP32 Development Board
- USB-to-UART Converter
- Breadboard and Jumper Wires

## Project Structure

```
BareMetal-STM32-IMU/
├── ESP32/
│   └── telemetry.ino
├── stm32/
│   └── main.c
├── LICENSE
└── README.md
```

## Working Principle

The MPU6050 provides raw accelerometer and gyroscope measurements. The STM32 acquires sensor data using the I2C protocol and performs calibration to remove sensor offsets. Accelerometer-based orientation estimates are combined with gyroscope measurements using a complementary filter to produce stable roll and pitch estimates. The resulting orientation data is transmitted over UART and can be forwarded through an ESP32 telemetry module for wireless monitoring.

## Technical Highlights

- Register-level peripheral configuration
- I2C driver implementation
- UART driver implementation
- Sensor calibration routines
- Complementary filter-based sensor fusion
- Real-time attitude estimation
- Embedded telemetry communication

## Future Improvements

- Kalman Filter implementation
- Magnetometer integration for yaw estimation
- Wireless dashboard visualization
- Data logging to SD card
- CubeSat ADCS integration
- Real-time graphical monitoring

## Author

Aditya Naigaonkar

Instrumentation Engineering, VIT Pune
