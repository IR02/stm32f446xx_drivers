# Bare Metal Programming Practice Repository

This repository contains code and learning materials focused on **bare-metal programming** for embedded systems. The goal of this project is to practice writing drivers for various communication protocols and peripherals, such as **I2C**, **SPI**, **USART**, and **GPIO**, and to better understand how hardware interacts with software at a low level.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Technologies Used](#technologies-used)
3. [Folder Structure](#folder-structure)
4. [Drivers Implemented](#drivers-implemented)
   - [I2C](#i2c)
   - [SPI](#spi)
   - [USART](#usart)
   - [GPIO](#gpio)
5. [Setup](#setup)
6. [Contributing](#contributing)
7. [License](#license)

## Project Overview

The purpose of this repository is to dive deep into embedded systems programming by working at the **bare metal** level, i.e., writing code without the support of an operating system or large frameworks. The focus is on implementing low-level hardware drivers to understand how software controls hardware, including peripherals and communication protocols commonly used in microcontrollers.

## Technologies Used

- **Microcontroller**: STM32, ATmega, or other platforms
- **Languages**: C, Assembly (optional)
- **Communication Protocols**: I2C, SPI, USART
- **Peripherals**: GPIO, LEDs, etc.
- **Development Tools**: STM32CubeIDE, GCC, ARM Cortex toolchain, or AVR toolchain

## Folder Structure


## Drivers Implemented

### I2C

The I2C driver allows for communication between the microcontroller and various I2C-compatible devices, such as sensors and memory chips.

- **Features**:
  - Master mode communication
  - Support for multi-byte reads and writes

### SPI

The SPI driver is used to interact with external devices over the SPI protocol, such as EEPROMs, sensors, and displays.

- **Features**:
  - Full-duplex communication
  - Configurable clock speed and polarity

### USART

The USART driver handles serial communication between the microcontroller and a PC or other devices.

- **Features**:
  - Configurable baud rate
  - Support for interrupts and polling modes

### GPIO

The GPIO driver is responsible for controlling and monitoring the state of the general-purpose input/output pins.

- **Features**:
  - Configurable pin modes (input/output)
  - Pull-up and pull-down resistor control

## Setup

### Prerequisites

1. **Toolchain**: You will need an ARM Cortex toolchain or AVR toolchain, depending on your microcontroller. Install `gcc-arm-none-eabi` for ARM or `avr-gcc` for AVR.
2. **Hardware**: A development board (e.g., STM32 Nucleo, Arduino, etc.) and any necessary external peripherals.

### Build and Flash Instructions

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/bare-metal-practice.git
   cd bare-metal-practice
