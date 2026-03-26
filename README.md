# VL53L0X Bare-Metal Driver for MSP432P401R

## Overview
This repository provides a bare-metal driver for the STMicroelectronics VL53L0X Time-of-Flight (ToF) distance sensor, specifically targeted for the Texas Instruments MSP432P401R board. 

The primary motivation to write a VL53L0X driver is to bypass the heavy, abstraction-layered ST API. It provides a ready-to-deploy solution that manages the otherwise undisclosed register initialization sequence required to operate the sensor.

## Features
* **Bare-Metal Implementation:** Strictly utilizes direct register access. No dependencies on TI DriverLib or SimpleLink SDK routines.
* **Single Sensor Support:** Optimized for single-device operation on the I2C bus (unlike the original repo, check it out if you have an array of sensors).
* **Lightweight Footprint:** Written with simplicity and efficiency in mind.

## Linked Resources
* [VL53L0X Sensor Product Page](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)
* [Partial Sensor Register Map Reference](https://github.com/GrimbiXcode/VL53L0X-Register-Map)

## Acknowledgements
This repository is a fork and readaptation of @artfulbytes's original project, refactored specifically for the MSP432P401R architecture and a single sensor setup.

## Contributions and Support
Contributions, bug reports, and architectural improvements are welcome. Please open an issue or submit a pull request if you wish to discuss modifications or report unexpected behavior.