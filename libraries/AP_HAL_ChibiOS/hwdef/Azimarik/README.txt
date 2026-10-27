AZIMARIK ARDUPILOT HWDEF -- BRING-UP DRAFT
==========================================

Inputs: AZIMARIK.sch and AZIMARIK.brd (Eagle 7.2 XML).
The code is based on the netlist and checked against the board contactrefs.
Bootloader and Vektor2 compile in this checkout (2026-10-22).
Not tested on hardware and NOT flight-ready. Board ID 9999 remains provisional.

INSTALL
-------
Copy hwdef.dat and hwdef-bl.dat to:
  ardupilot/libraries/AP_HAL_ChibiOS/hwdef/Azimarik/

Reserve a non-conflicting ArduPilot APJ_BOARD_ID in Tools/AP_Bootloader/board_types.txt.
Replace the provisional 9999 in both files with that registered ID or a suitable
local-development ID not already in use.

From the ArduPilot repository:
  python3 Tools/scripts/build_bootloaders.py Azimarik
  ./waf configure --board Azimarik
  ./waf --targets bin/Vektor2

Check the current project's bootloader/flashing workflow before programming.
Do not flash without access to SWD recovery (PA13/PA14).

IMPORTANT BEFORE FIRST FLIGHT
-----------------------------
1) SPI2 MISO is connected to U2 pad 17, labeled PC2_C, not a bonded-out PC2
   digital pad. The H743 LQFP100 has an analog switch between PC2 and PC2_C.
   hwdef refers to PC2 logical SPI2_MISO function. Validate actual data
   transfers to both BMI088 halves, including under load and at temperature.
2) BMI088 ROTATION_NONE is a placeholder. The sensor physical axes vs the
   airframe forward direction must be measured before flight. Electrical
   connectivity alone does not establish that orientation.
3) DPS310 U6 SDO high -> I2C address 0x77. Its CSB pin is left open in the
   design; DPS310 includes an internal pull-up and explicitly permits this.
4) SENSE_U = PA7 ADC1 channel 7, SENSE_I = PA3 ADC1 channel 15. These
   originate at J2 pins 1/2 without on-board voltage dividers/shunts.
   Use appropriate external conditioning, protect 0..3.3 V ADC inputs, and
   set HAL_BATT_VOLT_SCALE / HAL_BATT_CURR_SCALE only after calibration.
5) CAN2 PB12/PB13 (J2 pins 9/10) are MCU logic lines only, NOT bus-level CAN.
   CAN1 PD0/PD1 includes SN65HVD232D and runs to X1/X8. CAN1 is classic
   CAN physical-layer transceiver, not CAN-FD-capable hardware.
6) Confirm SD card logging over SPI1, both BMI088 chip selects, GPS ports,
   UARTs, PWM output mapping, RPM inputs, I2C compass scanning, LEDs.
7) The board exposes no dedicated hardware RCIN pin in the netlist. Use a
   suitable serial RC receiver on a supported UART and configure its protocol.
8) Check real MCU flash size and device marking, HSE oscillation, rail voltage,
   regulator capacity, bootloader sector layout, CAN termination and board ID.

NETLIST MAPPING
---------------
SPI1 microSD:  PA5 SCK, PB4 MISO, PD7 MOSI, PA4 CS
SPI2 BMI088:  PA9 SCK, PC2_C MISO (logical PC2), PC1 MOSI,
              PA8 accel CS, PD13 gyro CS, PD14 gyro INT
I2C1 GPS:     PB6 SCL, PB7 SDA, X2/X4
I2C2 DPS310:  PB10 SCL, PB11 SDA, address 0x77
CAN1:         PD0 RX, PD1 TX, onboard U7 transceiver
CAN2:         PB12 RX, PB13 TX, header J2 only (no transceiver)
USB FS:       PA11 DM, PA12 DP
GPS1:         PB14 USART1 TX, PB15 USART1 RX (X2)
GPS2:         PD8 USART3 TX, PD9 USART3 RX (X4)
TELEM1:       PD5 USART2 TX, PD6 USART2 RX (J2)
TELEM2:       PC10 UART4 TX, PC11 UART4 RX (J2)
TELEM3:       PC12 UART5 TX, PD2 UART5 RX (J2)
TELEM4:       PE8 UART7 TX, PE7 UART7 RX (J2)
PWM1-4:       PE9, PE11, PE13, PE14 (TIM1 1..4)
PWM5-8:       PC6, PC7, PC8, PC9 (TIM8 1..4)
RPM1/RPM2:    PE2/PE3 from X3
AUX1/AUX2:    PE5/PE6 (solder pads)
LED:          PC4 yellow, PC5 green, active high

Reference examples: ArduPilot hwdef/MatekH743, hwdef/KakuteH7-Wing,
hwdef/omnibusf4pro, and hwdef/scripts/STM32H743xx.py.
