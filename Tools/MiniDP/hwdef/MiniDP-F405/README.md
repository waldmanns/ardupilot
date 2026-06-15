# MiniDP-F405 hwdef

This directory is reserved for the first vessel's board definition.

Do not create `hwdef.dat` from a generic STM32F405 template. The definition
must be derived from the actual board schematic and confirm at least:

- board ID and flash layout
- oscillator and USB configuration
- BMI088 SPI buses and chip-select pins
- external compass I2C bus
- GPS, MAVLink, and RC UART assignments
- six PWM-capable output pins
- LED pins
- battery voltage and current ADC pins
- SD or dataflash logging interface

Until those details are known, build the MiniDP app shell against an existing
registered ArduPilot board with compatible hardware.
