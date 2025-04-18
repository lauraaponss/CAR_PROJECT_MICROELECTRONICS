# Ultrasonic Sensor Bluetooth Robot Car

This project implements an STM32-based robot car with ultrasonic sensor for obstacle detection, adaptive speed control, and Bluetooth remote operation capabilities.

## Features

- **Obstacle detection**: Uses HC-SR04 ultrasonic sensor to measure distances
- **Sound alerts**: Buzzer with variable intensity based on proximity to obstacles
- **Dual operation modes**:
  - **Automatic mode**: Autonomous navigation with collision avoidance
  - **Manual mode**: Remote control via Bluetooth
- **Voltage monitoring**: Protection system that stops motors when battery is low
- **Adaptive speed control**: Adjusts speed based on distance to obstacles

## Hardware

- Microcontroller: STM32L1 series
- HC-SR04 ultrasonic sensor
- Bluetooth module (serial communication at 9600 baud)
- Motor driver (configured for 2 motors control)
- PWM buzzer
- Voltage sensor (connected to ADC)

### Connections

- **Ultrasonic sensor**:
  - TRIG: PA1
  - ECHO: PA0

- **Buzzer**:
  - Signal: PB0 (PWM via TIM3_CH3)

- **Motors**:
  - Right motor forward: PA6 (PWM via TIM3_CH1)
  - Right motor backward: PA7
  - Left motor forward: PC7 (PWM via TIM3_CH2)
  - Left motor backward: PB4

- **Voltage Sensor**:
  - Connected to ADC1 Channel 5

## Software

The system operates in two modes:

### Automatic Mode
- The ultrasonic sensor continuously measures the distance to obstacles
- If distance > 30cm: Full speed forward, buzzer off
- If distance between 10-30cm: Reduced speed, medium buzzer
- If distance < 10cm: Stop motors, full buzzer alert
- System constantly monitors battery voltage and stops all operations if voltage drops below threshold

### Manual Mode
The car can be controlled via Bluetooth using these commands:

- `M`: Switch to Manual mode
- `A`: Switch to Automatic mode
- `F`: Move Forward
- `B`: Move Backward
- `L`: Turn Left
- `R`: Turn Right
- `S`: Stop motors

## Implementation Details

- Uses STM32 HAL library for hardware abstraction
- Timer-based distance measurement with HC-SR04
- PWM control for motor speed and buzzer intensity
- Interrupt-based Bluetooth communication
- ADC for battery voltage monitoring
- Timer-based periodic sensing and decision making

## Safety Features

- Motors automatically stop when:
  - Battery voltage drops below 2V
  - Obstacles detected too close in automatic mode
  - Timeout occurs during communications
