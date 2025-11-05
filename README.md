# LIQUID LEVEL AND TEMPERATURE MEASUREMENT AND REGULATION SYSTEM

## Project Overview

This project presents the design and implementation of a liquid level and temperature measurement and regulation system using modern microcontroller-based sensing and actuation. The system continuously monitors liquid level and temperature, and automatically regulates temperature via a controlled heating mechanism.

The implementation utilizes an STM32F401 microcontroller, DS18B20 digital temperature sensor, HC-SR04 ultrasonic level sensor, boost converter module, and an electrical heating element. The system is designed for industrial relevance, with a focus on processes such as fertilizer chemical preparation where precise mixing, heating, and level control are required.

## Objectives

Implement accurate liquid level measurement using an ultrasonic sensor.

Measure and regulate liquid temperature using an embedded system.

Apply control logic on STM32 to drive heating and monitoring processes.

Relate the designed system to real industrial applications, specifically the fertilizer processing industry.

## Key Features

Real-time temperature measurement and control

Non-contact ultrasonic liquid level sensing

Microcontroller-driven heater regulation

Boost converter for high-power heating control

Expandable for automation and industrial SCADA integration

## System Components

## *Microcontroller — STM32F401RCT6*

The STM32F401 series MCU is used as the primary processing unit.

**Key Specifications**

ARM Cortex-M4 @ 84 MHz
512 KB Flash memory
96 KB SRAM
ADC, DMA, PWM, SPI, I2C, UART peripherals
High-speed timers for precise sensor timing and control
Suitable for real-time embedded control systems

This controller provides superior processing capability, accuracy, and interface flexibility compared to 8-bit alternatives, making it suitable for industrial-grade control systems.

## Sensors and Modules

## *DS18B20 — Digital Temperature Sensor*

One-Wire interface
−55°C to +125°C range
High resolution (up to 12-bit)
Digital output for noise-resistant measurement

## *HC-SR04 — Ultrasonic Liquid Level Sensor*

Non-contact distance measurement
Trigger-Echo timing method
Liquid level computed via ultrasound time-of-flight

Heating System + Boost Converter

thermistor for temperature regulation

DC-DC boost converter to supply required voltage to heater

Controlled by STM32 using PWM / GPIO switching
