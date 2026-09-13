# Vektor Core Evo — Truth Base

**Purpose:** canonical working base for future Vektor Core Evo hardware, firmware, protocol, and configurator work  
**Generated from supplied project sources:** 2026-09-10  
**Firmware status last synchronized:** 2026-09-13\
**Status:** evidence-oriented consolidation, not a claim that the firmware or GUI already implements the design  
**Primary platforms:** Vektor Core Evo STM32H743 and Vektor Core Reduced STM32F405

---

## 0. How to read this document

This file deliberately separates facts that are visible in hardware or board-definition sources from architecture decisions, draft protocol ideas, legacy VRS design material, and engineering inference.

Status tags used throughout:

- **[VERIFIED-HW]** Directly supported by the current H743 KiCad schematic, H743 CubeMX `.ioc`, or reduced-board ArduPilot `hwdef`.
- **[DECIDED]** Explicit project direction from the current Vektor architecture notes, two-platform brief, protocol draft, or project discussion.
- **[IMPLEMENTED]** Supported by source code and proportionate build or test evidence in the current repository.
- **[DRAFT]** A proposed interface, rate, data structure, or behavior that is not proven by implementation in the supplied sources.
- **[LEGACY-VRS]** Carried from the earlier CLIM6/VRS design. It remains useful as VSP/VRS domain knowledge, but its old G0B1 board-specific hardware details are not Vektor Core Evo hardware facts.
- **[INFERENCE]** Engineering interpretation of verified source material. These items should be checked during implementation or bring-up.
- **[OPEN]** Not yet resolved, internally inconsistent, or not evidenced by the supplied source set.

### 0.1 Evidence precedence

For the **full H743 board**, the KiCad schematic is the strongest source for what is electrically present and connected. The `.ioc` is the strongest source for intended STM32 alternate-function assignment and current CubeMX peripheral configuration. Neither source silently overrides the other. A disagreement is an issue to fix.

For the **reduced F405 board**, `reduced-board-hwdef.txt` is the current board-definition source of truth.

For **firmware architecture and protocol**, the original supplied markdown files describe design intent. A Vektor firmware scaffold now exists under `Tools/Vektor`; Section 41 records which parts are implemented. `VEKTOR_SERIAL_PROTOCOL_TRUTH.md` is the canonical wire-level protocol specification and supersedes the historical draft summary in Part VII of this document.

For **VSP/VRS control behavior**, the CLIM6 brief is useful domain history. Its STM32G0B1 hardware sections are superseded for Vektor Core Evo.

The `vevo.png` render is useful visual corroboration only. It is not an electrical source of truth and cannot prove hidden copper, routing, or connectivity.

---

## 1. Source inventory and applicability

| Source | Role in this truth base | SHA-256 |
|---|---|---|
| `vektor-core-evo(2).kicad_sch` | Full H743 electrical hardware source | `517220ea271e4ea1d98debc9b02d4b7fb920a63f046a65b8c8ec3424721e44c2` |
| `vektor-core-evo.ioc` | Full H743 MCU pin and peripheral intent | `41272c6c210a4d06eb166ff280bacdfd58c3519afd2110aeace0c7d0474c5e05` |
| `reduced-board-hwdef.txt` | Reduced F405 board definition | `37c56c88f61decf3199d89a7622afe08c1d13415e373a48d19e7c6483bfb60e9` |
| `VEKTOR_CORE_EVO_ARCHITECTURE(1).md` | Component, routing, scheduling, parameter architecture | `9107f65ef1187d7e21adcca4933077dc2aaa18831f0b1a85f536331aaff58ff7` |
| `VEKTOR_PROTOCOL(1).md` | Native protocol draft | `4f97829a244d4b992e171398525c4de408ef54f9062293114cc0ed8f4569cf4f` |
| `VEKTOR_CORE_EVO_PROJECT_BRIEF(1).md` | Two-platform consolidation and capability-discovery direction | `107df4ca62cf34a1146b3880fea73d92dbd92c3c36b64d086da8b34d1801c86f` |
| `CLIM6_VRS_Project_Brief(1).md` | Legacy VSP/VRS control and commissioning design | `074e470bb75f3aebd8e6603788ce6f0eaeee39d55ad0b8226ac46679d927b986` |
| `vevo.png` | Current full-board visual render | `881d678c4669910771b021f639e0553d47e3c49017effc1bd933ee9f094a1b16` |

Project discussion after the written briefs also establishes several current directions, especially common application compilation, runtime hardware discovery, bidirectional DroneCAN use, mixed UART/CAN command sourcing, low-priority SD/logging, fast VSP/VRS execution, and the configurator UX direction. Those decisions are incorporated below and are marked **[DECIDED]** rather than **[VERIFIED-HW]**.

---

## 2. Project identity and invariants

**[DECIDED]** Vektor Core Evo is a configurable embedded machine-control platform. It is not intended to become a generic PLC and it is not intended to inherit the Rover/Plane vehicle model merely because ArduPilot infrastructure is reused.

**[DECIDED]** There is one Vektor application model compiled for multiple supported boards. The two current targets are:

1. **Vektor Core Evo**, based on STM32H743.
2. **Vektor Core Reduced**, based on STM32F405.

**[DECIDED]** The reduced board is not a separate product architecture. It is a lower-resource hardware target that advertises fewer endpoints and may use a different scheduling profile.

**[DECIDED]** The board layer answers what physical resources exist. The application layer answers what logical components exist. The Vektor Protocol tells the client about both at runtime.

**[DECIDED]** Normal application components remain board-agnostic. Raw GPIO names, timer channels, DMA streams, EXTI lines, ADC channel numbers, and bus pin mappings belong below the application layer.

**[DECIDED]** The machine is power-on operational. The current architecture does not adopt ArduPilot vehicle arming, missions, flight modes, fences, or centralized vehicle-style failsafe machinery as a default framework.

**[DECIDED]** Protection remains local and explicit. Components and endpoints are responsible for bounds, freshness, validity, saturation, safe fallback, watchdog-compatible behavior, and bounded actions.

**[DECIDED]** A normal user configuration should not require an arbitrary PLC chain. The intended UX limit is roughly two consecutive functional blocks between a source and destination. Repeated deeper patterns should become higher-level components.

---

# Part I — Full Vektor Core Evo hardware

## 3. MCU and current clock configuration

**[VERIFIED-HW]** The current full-board MCU is `STM32H743VIT6`, represented in the schematic as `STM32H743VITx`, in LQFP100.

**[VERIFIED-HW]** An 8 MHz resonator is present as `Y1 = CSTNE8M00G55A000R0` on the HSE pins PH0/PH1.

**[VERIFIED-HW]** The current `.ioc` does **not** describe a high-clocked H743 configuration. It reports:

```text
CpuClockFreq_Value = 64 MHz
CortexFreq_Value   = 64 MHz
SYSCLKFreq_VALUE   = 64 MHz
AHB/APB clocks     = 64 MHz class
PLL source         = HSI
```

The HSE pins are assigned, but the current CubeMX PLL source is HSI.

**[OPEN]** The present `.ioc` clock tree should be treated as provisional configuration data, not as the final Vektor H743 runtime clock policy. The architecture notes that discuss approximately 1 kHz-class H743 control rates are design targets, not measurements from this 64 MHz CubeMX configuration.

**[OPEN]** The `.ioc` reports `USBFreq_Value=129000000`, while USB FS requires a valid USB kernel clock arrangement. The eventual ChibiOS/ArduPilot board definition must explicitly establish and validate its own clock tree. Do not assume this CubeMX clock page is production-ready.

---

## 4. H743 pin and peripheral map

The following table reflects the current `.ioc` assignment. Physical exposure is addressed separately because an enabled MCU peripheral is not automatically an external board endpoint.

| MCU pin | Current function | Physical role in current schematic |
|---|---|---|
| PE2 | SPI4_SCK | ICM-20602 SCK |
| PE3 | GPIO `SPI4_INT` | ICM-20602 interrupt net, but direction is currently wrong in `.ioc` |
| PE4 | GPIO `SPI4_CS` | ICM-20602 chip select |
| PE5 | SPI4_MISO | ICM-20602 MISO |
| PE6 | SPI4_MOSI | ICM-20602 MOSI |
| PC14 | GPIO `BUZZER` | Buzzer/control output |
| PC2_C | GPIO `CONF1` | Configuration strap input |
| PC3_C | GPIO `CONF2` | Configuration strap input |
| PA0 | TIM2_CH1 | Primary output bank |
| PA1 | TIM2_CH2 | Primary output bank |
| PA2 | TIM5_CH3 | Flex timer channel |
| PA3 | TIM5_CH4 | Flex timer channel |
| PA6 | TIM3_CH1 | Flex timer channel |
| PA7 | TIM3_CH2 | Flex timer channel |
| PB0 | TIM3_CH3 | Flex timer channel |
| PB1 | TIM3_CH4 | Flex timer channel |
| PC4 | ADC1_INP4 | VCC sensing divider |
| PE7 | UART7_RX | Dedicated receiver/input row in Flex header bank |
| PE8 | UART7_TX | Configured in MCU, not exposed in current schematic |
| PB10 | I2C2_SCL | GNSS/expansion connector |
| PB11 | I2C2_SDA | GNSS/expansion connector |
| PB12 | UART5_RX | External UART5 connector |
| PB13 | UART5_TX | External UART5 connector |
| PB14 | USART1_TX | External USART1 connector |
| PB15 | USART1_RX | External USART1 connector |
| PD8 | USART3_TX | GNSS/expansion connector |
| PD9 | USART3_RX | GNSS/expansion connector |
| PD12 | TIM4_CH1 | Primary output bank |
| PD13 | TIM4_CH2 | Primary output bank |
| PD14 | TIM4_CH3 | Primary output bank |
| PD15 | TIM4_CH4 | Primary output bank |
| PC6 | TIM8_CH1 | Primary output bank |
| PC7 | TIM8_CH2 | Primary output bank |
| PC8 | TIM8_CH3 | Primary output bank |
| PC9 | TIM8_CH4 | Primary output bank |
| PA8 | TIM1_CH1 | Primary output bank |
| PA9 | TIM1_CH2 | Primary output bank |
| PA11 | USB_OTG_FS_DM | USB D- |
| PA12 | USB_OTG_FS_DP | USB D+ |
| PA13 | SWDIO | SWD net exists in schematic |
| PA14 | SWCLK | SWD net exists in schematic |
| PA15 | GPIO `SPI3_CS` | SPI3 chip select, no current storage device shown |
| PC10 | SPI3_SCK | SPI3, no current storage device shown |
| PC11 | SPI3_MISO | SPI3, no current storage device shown |
| PC12 | SPI3_MOSI | SPI3, no current storage device shown |
| PD0 | FDCAN1_RX | CAN1 transceiver |
| PD1 | FDCAN1_TX | CAN1 transceiver |
| PD3 | GPIO `LED_BLUE` | Status LED |
| PD4 | GPIO `LED_AMBER` | Status LED |
| PD5 | USART2_TX | Configured in MCU, not externally exposed in current schematic |
| PD6 | USART2_RX | Configured in MCU, not externally exposed in current schematic |
| PB5 | FDCAN2_RX | CAN2 transceiver |
| PB6 | FDCAN2_TX | CAN2 transceiver |
| PB7 | I2C1_SDA | MMC5983MA |
| PB8 | I2C1_SCL | MMC5983MA |
| PE0 | UART8_RX | External UART8 connector |
| PE1 | UART8_TX | External UART8 connector |

### 4.1 Critical `.ioc` direction error

**[VERIFIED-HW]** `PE3` is labeled `SPI4_INT` and is electrically associated with the ICM-20602 interrupt signal in the schematic.

**[VERIFIED-HW]** The `.ioc` currently configures PE3 as `GPIO_Output`.

**[OPEN]** This must be corrected to an input or interrupt-capable input in the real board definition. It should not be copied literally into a ChibiOS hwdef.

---

## 5. Main output bank

**[VERIFIED-HW]** The full-board schematic contains three parallel 12-row headers:

- `OUT_PWM1` — twelve signal rows
- `OUT_VCC1` — twelve VCC rows
- `OUT_GND1` — twelve ground rows

**[VERIFIED-HW]** The twelve signal nets are:

```text
TIM2_CH1
TIM2_CH2
TIM4_CH1
TIM4_CH2
TIM4_CH3
TIM4_CH4
TIM8_CH1
TIM8_CH2
TIM8_CH3
TIM8_CH4
TIM1_CH1
TIM1_CH2
```

**[VERIFIED-HW]** Each of these twelve signal lines passes through a 330 ohm series resistor in the current schematic (`R39` through `R50`).

**[VERIFIED-HW]** The companion output-power bank is tied to **VCC**, not to the regulated +5 V rail.

**[DECIDED]** Application components should normally output normalized logical values. The PWM endpoint owns pulse calibration such as minimum, center, maximum, reverse, trim, and optional slew behavior.

### 5.1 Physical timer groups

The main bank has four hardware timer groups:

```text
TIM2 : 2 channels
TIM4 : 4 channels
TIM8 : 4 channels
TIM1 : 2 channels
```

**[DECIDED]** Channels that share one timer should be presented as a frame-rate group in the configurator. The user should not need to know the STM32 timer name.

**[DRAFT]** Useful user-facing servo frame presets remain approximately 50, 100, 200, and 330 Hz, subject to endpoint validation.

---

## 6. Flex and dedicated receiver bank

A previous brief says **six Flex ports**, while the schematic visibly contains a **seven-row** signal header. Both statements can be true once the actual nets are separated.

**[VERIFIED-HW]** The current board has three parallel seven-row headers:

- `FLEX_PWM1`
- `FLEX_VCC1`
- `FLEX_GND1`

**[VERIFIED-HW]** One row is a dedicated `UART7_RX` signal.

**[VERIFIED-HW]** The remaining six rows are timer-capable Flex signals:

```text
TIM5_CH3
TIM5_CH4
TIM3_CH1
TIM3_CH2
TIM3_CH3
TIM3_CH4
```

Therefore the correct product description is:

> **Six Flex timer/I/O ports plus one dedicated UART7 receiver/input row, all physically arranged in the same seven-row three-rail header bank.**

**[VERIFIED-HW]** `UART7_RX` has a 330 ohm series resistor (`R51`).

**[VERIFIED-HW]** Each of the six timer Flex channels has a 330 ohm series resistor (`R52` through `R57`) and the schematic contains six associated 10 nF capacitors (`C37` through `C42`).

**[DECIDED]** The intended Flex abstraction, where electrically valid, supports modes such as:

- PWM input
- PWM output
- RPM or Hall capture
- ADC input when a specific Flex pin actually supports ADC in the final board definition
- end-switch or GPIO input

**[IMPORTANT]** The generic Flex software concept must not claim ADC capability for all six full-board Flex rows merely because the abstract architecture supports ADC. The current `.ioc` assigns these six physical rows to timer pins, while the only explicitly configured ADC input in the full board source is PC4. Per-port electrical capability must come from the final board capability table.

### 6.1 Flex timer groups

The six timer Flex channels form two timer groups:

```text
TIM5 : 2 channels
TIM3 : 4 channels
```

That grouping matters when PWM frame rate is configured.

---

## 7. Serial and I2C external interfaces

### 7.1 Full-duplex four-pin UART connectors

**[VERIFIED-HW]** The schematic exposes three four-pin JST-GH serial connectors:

- `USART1`
- `UART5`
- `UART8`

Each uses the regulated **+5 V** rail and ground, with TX and RX routed through 330 ohm series resistors.

MCU mapping:

```text
USART1_TX PB14
USART1_RX PB15
UART5_TX  PB13
UART5_RX  PB12
UART8_TX  PE1
UART8_RX  PE0
```

### 7.2 GNSS/expansion connector

**[VERIFIED-HW]** `USART_GNSS1` is a six-pin JST-GH connector carrying:

```text
+5 V
GND
USART3_TX
USART3_RX
I2C2_SCL
I2C2_SDA
```

The USART3 TX/RX lines pass through 330 ohm series resistors (`R37`, `R38`).

MCU mapping:

```text
USART3_TX PD8
USART3_RX PD9
I2C2_SCL  PB10
I2C2_SDA  PB11
```

### 7.3 Configured but not fully exposed serial peripherals

**[VERIFIED-HW]** USART2 is enabled in the `.ioc` on PD5/PD6, but the current schematic does not show a second external net endpoint or connector for `USART2_TX/RX`.

**[VERIFIED-HW]** UART7 is enabled in the `.ioc` on PE7/PE8, but only `UART7_RX` is brought into the external seven-row Flex/receiver bank. `UART7_TX` has no corresponding external endpoint in the current schematic.

**[DECIDED]** Runtime capability discovery should describe what the board physically exposes, not every peripheral that happens to be enabled in CubeMX.

---

## 8. Onboard sensors

### 8.1 ICM-20602

**[VERIFIED-HW]** `U2` is an ICM-20602 connected through SPI4.

Current signal mapping:

```text
PE2  SPI4_SCK
PE5  SPI4_MISO
PE6  SPI4_MOSI
PE4  SPI4_CS
PE3  SPI4_INT
```

**[VERIFIED-HW]** The current schematic includes the established local ICM capacitor set:

```text
C16 100 nF
C17 10 nF
C18 100 nF
C19 2.2 uF
```

The exact association of those capacitors with VDD, VDDIO, and REGOUT follows the earlier ICM design work and should be preserved when the schematic is revised.

**[DECIDED]** The IMU data-ready signal should drive event-oriented acquisition. The architecture may use DMA for clean timing, but DMA is not required for raw CPU capacity.

### 8.2 MMC5983MA

**[VERIFIED-HW]** `U3` is an onboard MMC5983MA magnetometer.

**[VERIFIED-HW]** It is connected to I2C1:

```text
PB8 I2C1_SCL
PB7 I2C1_SDA
```

No separate magnetometer interrupt net is evident in the current schematic.

**[DECIDED]** The presence of a magnetometer on the full board does not mean every application must use heading. In particular, the roll-only VRS function inherited from CLIM6 does not require a magnetometer.

---

## 9. ADC and VCC sensing

**[VERIFIED-HW]** PC4 is configured as `ADC1_INP4`.

**[VERIFIED-HW]** The schematic connects the ADC node to a VCC divider/filter made from:

```text
R8  56 kOhm from VCC toward sense node
R7  10 kOhm from sense node to GND
C35 100 nF from sense node to GND
```

**[INFERENCE]** The divider ratio is nominally 6.6:1. With a 3.3 V ADC reference, that corresponds to about 21.8 V at the divider input for full-scale ADC voltage, before tolerance and protection margins.

**[OPEN]** The `.ioc` currently sets ADC sampling time to 1.5 cycles. Because the divider source impedance is not negligible, final firmware should verify acquisition settling rather than copying that setting uncritically.

---

## 10. CAN hardware

**[VERIFIED-HW]** The full board has two independent MCU CAN controller connections:

```text
FDCAN1_RX PD0
FDCAN1_TX PD1
FDCAN2_RX PB5
FDCAN2_TX PB6
```

**[VERIFIED-HW]** Two `SN65HVD232` transceivers are present as `U7` and `U8`.

**[VERIFIED-HW]** Two four-pin JST-GH CAN connectors are present:

- `CAN-A2` is associated with the CAN1 H/L nets.
- `CAN-A1` is associated with the CAN2 H/L nets.

Both connector groups carry `CAN_POWER` and GND in addition to CAN H/L.

**[VERIFIED-HW]** `CAN_POWER` is shared as a board net and also enters the board power ORing network through a Schottky diode. The board can therefore receive raw supply power from the CAN power net in the current schematic topology.

### 10.1 Termination

**[VERIFIED-HW]** The schematic contains two 120 ohm resistors (`R3`, `R4`) and one open solder jumper `CAN_120R1`.

**[VERIFIED-HW]** The visible net placement indicates CAN1 has a direct 120 ohm termination network around `R4`, while CAN2 uses `R3` with `CAN_120R1` in the termination path.

**[OPEN]** Confirm the desired termination policy before PCB release. A reusable controller generally benefits from deliberate, documented termination behavior on both ports.

### 10.2 FDCAN versus physical CAN capability

**[VERIFIED-HW]** The MCU peripheral is FDCAN-capable, but the board uses SN65HVD232 transceivers, which were selected in the project as classic high-speed CAN-class transceivers.

**[DECIDED]** Current DroneCAN direction only requires classic CAN behavior. Do not advertise CAN FD data-phase capability merely because the H743 contains FDCAN controllers.

**[OPEN]** The current `.ioc` contains CubeMX-calculated nominal FDCAN values of approximately 2.6875 Mbit/s for both controllers. This is not accepted here as the intended Vektor bus bitrate. Final DroneCAN/CAN timing must be explicitly chosen and validated against the physical transceiver and network.

---

## 11. USB

**[VERIFIED-HW]** The board uses a USB-C receptacle `J3 = TYPE-C-31-M-12`.

**[VERIFIED-HW]** Native USB FS is connected to:

```text
PA11 USB_DN / USB_OTG_FS_DM
PA12 USB_DP / USB_OTG_FS_DP
```

**[VERIFIED-HW]** Two 5.1 kOhm CC resistors are present as `R20` and `R25`, consistent with a USB-C sink/device connection.

**[VERIFIED-HW]** USB VBUS enters the board power OR network through Schottky `D3`.

**[VERIFIED-HW]** No dedicated USB ESD/TVS component is obvious in the current schematic snapshot.

**[DECIDED]** Native USB is the primary human/service connection for the Vektor configurator and firmware service workflow.

**[OPEN]** Final USB clocking, ESD strategy, and update/bootloader mechanism remain implementation work.

---

## 12. Board power topology

The full H743 schematic is materially different from the old G0B1 CLIM6 board notes, so the current topology is stated explicitly here.

### 12.1 Raw-source ORing

**[VERIFIED-HW]** Three source nets feed a common raw regulator-input rail through Schottky diodes:

```text
VCC       -> D1 -> common raw rail
CAN_POWER -> D2 -> common raw rail
VBUS      -> D3 -> common raw rail
```

The common rail then supplies both onboard linear regulators.

### 12.2 Regulated rails

**[VERIFIED-HW]** `U4 = SPX3819M5-L-3-3` generates +3.3 V.

**[VERIFIED-HW]** `U5 = SPX3819M5-L-5-0` generates the board +5 V rail.

The schematic contains substantial local bulk and bypass capacitance around these rails, including multiple 22 uF and 100 nF capacitors.

### 12.3 External connector supply rails

**[VERIFIED-HW]** The 12-channel output VCC bank and the seven-row Flex/receiver VCC bank are tied to raw **VCC**.

**[VERIFIED-HW]** The external UART and GNSS connectors use the regulated **+5 V** rail.

**[VERIFIED-HW]** The CAN connectors use **CAN_POWER**, which is also a board power input through D2.

**[INFERENCE]** When the board is powered only from USB VBUS, the +5 V LDO input is already below ideal 5 V due to the Schottky path. Do not assume external +5 V connector power is exactly 5.00 V under USB-only supply without measurement. This does not affect the statement that the rail is schematically named +5 V.

---

## 13. Configuration straps, status, debug

**[VERIFIED-HW]** Two configuration inputs exist:

```text
CONF1 PC2_C
CONF2 PC3_C
```

**[VERIFIED-HW]** They are associated with the 2x2 header `J1`, two 10 kOhm resistors (`R5`, `R6`), +3.3 V, and GND. The topology is consistent with normally-low straps that can be pulled high by header configuration.

**[VERIFIED-HW]** Status outputs include:

```text
PD3 LED_BLUE
PD4 LED_AMBER
PC14 BUZZER
```

A separate green power LED is also present in the schematic.

**[VERIFIED-HW]** SWDIO and SWCLK are assigned to PA13 and PA14 and appear as schematic global nets.

**[OPEN]** No dedicated SWD connector or obvious SWD test-point group is present in the supplied schematic. NRST is also not visibly exposed as a service connector. Factory/debug access should be settled before hardware freeze.

---

## 14. SPI3 and SD status on the full board

This is a major distinction between **project intent** and **current hardware evidence**.

**[VERIFIED-HW]** The `.ioc` enables SPI3:

```text
PC10 SPI3_SCK
PC11 SPI3_MISO
PC12 SPI3_MOSI
PA15 SPI3_CS
```

**[VERIFIED-HW]** In the current schematic, the SPI3 labels appear at the MCU side but no SD card socket, flash IC, or second connected SPI3 endpoint is present.

**[VERIFIED-HW]** The 3D board render also does not visibly establish an SD socket.

**[DECIDED]** Project discussion wants SD-based storage/logging to be possible, with SD over SPI and logging/file work kept low priority.

**[OPEN]** Therefore the truthful current statement is:

> **SPI3 is reserved/configured for a possible storage role, but an onboard SD card is not present in the supplied full-board schematic snapshot.**

Do not advertise `CAP_SD` for the H743 target until the final hardware actually contains and validates that storage endpoint.

---

## 15. H743 board render

**[VERIFIED-HW]** `vevo.png` visually corroborates the broad layout concept, including the central LQFP100 MCU, USB-C, two CAN-side connectors, three-rail output headers, the lower Flex/receiver bank, onboard sensors, and the 2x2 configuration header.

**[IMPORTANT]** The render should be used by the configurator as optional board artwork only after connector naming and orientation are matched to the final PCB revision. It must not be used as a substitute for capability discovery.

The render cannot prove copper keepouts, ground-plane cuts, controlled impedance, signal continuity, or hidden-side components.

---

# Part II — Vektor Core Reduced hardware

## 16. Reduced board source of truth

**[VERIFIED-HW]** The reduced target is defined by `reduced-board-hwdef.txt` as STM32F405xx with an 8 MHz crystal and 1024 KB flash.

Key hwdef facts:

```text
MCU STM32F4xx STM32F405xx
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 1024
I2C_ORDER I2C1
SERIAL_ORDER OTG1 USART1 EMPTY USART3 USART6
```

The file also defines 64 KB of reserved flash start space, 15360 bytes of HAL storage, and flash storage page 2.

**[VERIFIED-HW]** The current custom firmware string is `VEKTORAP`. The hwdef sets `STM32_ST_USE_TIMER 5`, enables `HAL_DEFAULT_INS_FAST_SAMPLE 3`, and includes `../include/minimize_fpv_osd.inc`. These are current reduced-target implementation details, not requirements for the H743 target.

### 16.1 USB and serial

**[VERIFIED-HW]** Native USB FS uses PA11/PA12 through OTG1.

**[VERIFIED-HW]** External serial resources are:

```text
USART1 PA9 TX / PA10 RX
USART3 PB10 TX / PB11 RX
USART6 PC6 TX / PC7 RX
```

**[VERIFIED-HW]** PC0 controls external USART1 RX inversion for SBUS-style reception.

### 16.2 PWM outputs

**[VERIFIED-HW]** The reduced board exposes six PWM outputs:

```text
PWM1 PB0 TIM3_CH3
PWM2 PB1 TIM3_CH4
PWM3 PA3 TIM2_CH4
PWM4 PA2 TIM2_CH3
PWM5 PA1 TIM2_CH2
PWM6 PA0 TIM2_CH1
```

This creates two physical timer groups:

```text
TIM3 : 2 channels
TIM2 : 4 channels
```

### 16.3 BMI088

**[VERIFIED-HW]** The reduced board uses a BMI088 on SPI1:

```text
PA5 SPI1_SCK
PA6 SPI1_MISO
PA7 SPI1_MOSI
PA4 BMI088_G_CS
PB6 BMI088_A_CS
PC4 DRDY1_BMI088_G
PB7 DRDY2_BMI088_A
```

The hwdef specifies `ROTATION_PITCH_180` and enables the BMI088 as separate accelerometer and gyro SPI devices in MODE3 at 10 MHz.

### 16.4 Storage

**[VERIFIED-HW]** Unlike the current full-board schematic, the reduced hwdef explicitly contains storage hardware definitions:

```text
SPI3 on PC10/PC11/PC12
SDCARD1_CS on PA13
FLASH_CS on PB3
FATFS I/O enabled
SPI dataflash enabled for logging
```

The SD card and dataflash share SPI3 in the current hwdef.

**[DECIDED]** Logging and file operations remain background work even on the board that has actual storage hardware.

### 16.5 ADC and optional sensors

**[VERIFIED-HW]** ADC resources include:

```text
PC2 BATT_VOLTAGE_SENS
PC1 BATT_CURRENT_SENS
PC5 USB_SENSE
```

Default battery scales are defined as 10.1 for voltage and 17.0 for current.

**[VERIFIED-HW]** I2C1 is present on PB8/PB9.

**[VERIFIED-HW]** The hwdef enables optional BMP280 probing at I2C address `0x76`, allows startup without a barometer, and probes external I2C compasses. There is no onboard compass defined in the reduced hwdef.

### 16.6 Reduced-board capabilities that are absent

**[VERIFIED-HW]** The supplied reduced hwdef defines no CAN endpoint.

**[VERIFIED-HW]** It defines six PWM outputs, not the full H743 12-output plus six-Flex arrangement.

**[DECIDED]** The reduced firmware must not advertise CAN or full-board Flex capabilities that are not physically present.

---

# Part III — Common firmware architecture

## 17. Software base

**[DECIDED]** The intended base is:

- ChibiOS
- Waf
- selected ArduPilot libraries
- an architecture closer in spirit to `AP_Periph` than to Rover or Plane

**[IMPLEMENTED]** A standalone `Vektor` Waf application now exists under `Tools/Vektor`. It builds for SITL and for the reduced `revo-mini` F405 target. The full H743 hardware build remains blocked on its dedicated ChibiOS hwdef. Architecture not identified as implemented in Section 41 remains design intent.

---

## 18. Layering and dependency direction

The common architecture is:

```text
HAL / BSP
  -> hardware endpoints
      -> core services
          -> device / estimation services
              -> application components
                  -> protocol adapters and configurator-facing state
```

### 18.1 HAL / BSP

**[DECIDED]** Owns physical details such as:

- MCU startup and clocks
- GPIO
- timers
- DMA
- SPI
- I2C
- UART
- CAN/FDCAN where present
- USB
- ADC
- storage backends
- physical onboard sensor instances

### 18.2 Hardware endpoints

**[DECIDED]** Translate physical resources into generic Vektor resources, for example:

- PWM output banks
- PWM input/capture
- RPM/Hall capture
- ADC inputs
- digital/end-switch inputs
- UART receiver endpoint
- CAN RX and actuator TX endpoint
- storage endpoint

### 18.3 Core services

**[DECIDED]** Common services should include:

- component registry
- capability registry
- typed signal routing
- scheduler
- parameter service
- persistence
- observables
- actions and events
- file service
- protocol adapters

### 18.4 Device and estimation layer

**[DECIDED]** Board-specific sensors should feed a common attitude/motion service so that application components do not depend on ICM-20602 versus BMI088 details.

The supplied architecture notes propose ArduPilot AHRS/EKF infrastructure as reusable machinery.

**[DRAFT]** The exact estimator stack is not proven by implementation here. A future source-code truth base should replace this statement with the actual selected AP_AHRS/EKF configuration.

### 18.5 Application components

**[DECIDED]** Initial high-level components include:

- Receiver / Command Source
- Attitude / Motion
- VSP1
- VSP2
- Actuator Feedback
- hardware output endpoints
- future domain-level Vektor components

Unsupported components should not be instantiated or advertised on a target.

---

## 19. Component contract

**[DECIDED]** A component may expose:

- **Inputs** — typed logical signals consumed by the component
- **Outputs** — typed logical signals produced by the component
- **Parameters** — persistent configuration
- **Observables** — read-only live state and diagnostics
- **Actions** — one-shot operations

**[DECIDED]** Application components must be boot-agnostic and transport-agnostic. They should not know whether a signal originated from a UART, CAN, USB transaction, or physical PWM input unless the source identity is itself meaningful application data.

---

## 20. Signal model

**[DECIDED]** Use typed signals rather than untyped floats everywhere.

Candidate signal types include:

```text
bool
normalized scalar
integer/scalar
angle
angular rate
RPM
voltage
position
pulse width
vector2
vector3
quaternion
```

**[DECIDED]** A routed signal should carry at least:

```text
value
timestamp
quality = VALID | STALE | INVALID
```

**[DECIDED]** Realtime consumers generally use newest-value semantics. Old visualization samples should not build up and replay later.

---

## 21. Routing model

**[DECIDED]** Routing connects logical outputs to logical inputs using stable port identities.

A route conceptually remains:

```text
source_output_id -> destination_input_id
```

**[DECIDED]** Physical I/O is independent of application logic. A VSP output may eventually target a PWM channel or a DroneCAN actuator endpoint without changing VSP math.

**[DECIDED]** Command sources should likewise be interchangeable. A logical channel can originate from UART receiver data or CAN-derived actuator commands.

**[DECIDED]** Mixed command sourcing is allowed. The intended architecture permits some command channels to come from UART and others from CAN when the user explicitly routes them that way.

---

## 22. Parameters and persistence

**[DECIDED]** `AP_Param` is the authoritative persistent parameter store.

Both native Vektor Protocol parameter operations and MAVLink `PARAM_*` operations should act on the same underlying parameters.

**[DECIDED]** A common `ParameterService` owns:

- validation
- update
- persistence
- component notification
- live reconfiguration where supported

Suggested namespaces remain:

```text
APP_ application and machine behavior
HW_  physical and electrical endpoint behavior
SYS_ runtime/system behavior
NET_ communication/network behavior
```

**[DECIDED]** Fixed MCU pin names and timer names should not become ordinary user parameters.

**[DRAFT]** Parameter descriptors may define apply policy:

```text
LIVE
RECONFIGURE
REBOOT
```

---

# Part IV — Scheduling and realtime behavior

## 23. Scheduling model

**[DECIDED]** Vektor does not use one giant PLC scan loop. Data flow and scheduling are separate concerns.

Three timing classes are intended.

### 23.1 Event-driven hardware work

Examples:

- IMU data-ready
- SPI DMA completion
- UART frame reception
- CAN reception
- RPM/Hall edges
- timer update events

### 23.2 Deterministic control and estimation work

Examples:

- AHRS/estimation
- attitude/motion processing
- command routing needed by control
- VSP
- VRS

### 23.3 Lower-priority service work

Examples:

- Vektor telemetry
- MAVLink telemetry
- diagnostics
- parameter housekeeping
- file transfer
- SD/dataflash logging
- statistics

**[DECIDED]** Logging is explicitly low priority. It must not define or block the control architecture.

---

## 24. Board-specific scheduler profiles

**[DECIDED]** The H743 and F405 are allowed to run different update rates.

**[DECIDED]** Application components must be time-aware. They should use actual timestamps and `dt`, not assume one compile-time loop period.

A conceptual API remains valid:

```cpp
component.update(now_us, dt);
```

**[DRAFT]** Earlier planning suggested approximately 1 kHz-class control on H743 and approximately 500 Hz-class control on F405. Those values are not yet validated benchmarks and are not protocol guarantees.

**[DECIDED]** VSP/VRS belongs in the fast control path. This remains important when physical servos are configured around 330 Hz. The H743 has ample intended headroom, but the final rate should still be chosen from measured latency and control requirements rather than from CPU capability alone.

**[DECIDED]** PWM frame generation remains timer-driven and is not forced to equal the control-loop frequency.

---

# Part V — VSP and VRS domain model

## 25. VSP component

**[DECIDED]** Each VSP is one user-visible component.

Nominal logical inputs:

```text
X
Y
```

Nominal logical outputs:

```text
SERVO_A
SERVO_B
```

Internal behavior may include:

- axis trim and scaling
- Voith geometry/mixing
- ring limiting
- output saturation handling
- calibration
- optional VRS
- VRS authority limiting
- actuator/RPM feedback integration later

**[DECIDED]** VRS remains part of the VSP feature set in the normal user model. It is not a separate PLC block that the user must manually wire into every VSP chain.

**[DECIDED]** Attitude/motion can be consumed internally by VSP when VRS is enabled.

---

## 26. VRS behavior inherited from CLIM6

The following items are retained as domain behavior, not as full-board hardware restrictions.

**[LEGACY-VRS]** Roll stabilization is primarily rate damping from gyro roll rate:

```text
u = -Kd * roll_rate
```

A small slow roll-angle term may be added when useful, but it should not force the vessel to fight a harmless static list indefinitely.

**[LEGACY-VRS]** Accelerometer information is useful for slow attitude reference and drift correction. It should not dominate the fast disturbance-rejection path.

**[LEGACY-VRS]** VRS correction is an overlay on the ordinary VSP command, not a replacement for propulsion or steering commands.

**[LEGACY-VRS]** Hard mechanical/ring limits remain authoritative. Automatic calibration or identification may learn behavior inside known authority, but must not expand authority automatically.

**[LEGACY-VRS]** Saturation handling and anti-windup are required if an integral term is introduced.

**[LEGACY-VRS]** On IMU failure, VRS correction should be removed predictably while preserving the base command wherever architecture allows.

### 26.1 Commissioning direction

**[LEGACY-VRS]** The preferred commissioning process is deterministic bounded identification rather than opaque machine learning.

Useful stages include:

```text
sensor stillness/bias check
neutral trim determination
bounded VSP excitation
response-sign and gain identification
conservative gain selection
validation
persistent profile save
```

Every calibration stage should have bounds, timeout, sanity checks, and safe fallback.

### 26.2 Legacy CLIM6 vessel profile

The earlier reference vessel used:

- two aft VSP units
- four fast digital servos total
- CLS2875H servos
- nominal 8.4 V servo rail
- approximately 330 Hz servo command rate
- very large external servo-side bulk capacitance

**[IMPORTANT]** These are reference application details. They are not global requirements of every Vektor Core Evo installation.

---

# Part VI — CAN and DroneCAN architecture

## 27. DroneCAN direction

**[DECIDED]** DroneCAN support is planned after the basic Vektor infrastructure, but the software boundaries should accommodate it from the beginning.

**[DECIDED]** The full H743 target should be able to operate in two actuator-command roles:

1. **Receive role** — listen to actuator-command data from an external autopilot and expose up to 16 received command channels as logical Vektor sources.
2. **Transmit role** — emit actuator commands for DroneCAN actuators, allowing logical outputs such as VSP commands to target CAN rather than local PWM.

**[DECIDED]** These are actuator-direction modes. The board should still be able to listen to normal DroneCAN sensor and power-module messages while performing the selected actuator role.

**[DECIDED]** Standard DroneCAN/DSDL data such as power measurements should enter normal Vektor observability or signal services rather than being trapped inside a CAN-specific UI path.

**[DECIDED]** CAN-derived input channels should be usable like UART receiver channels, including mixed-source configurations.

**[DRAFT]** Exact DroneCAN node-ID management, DSDL subset, actuator message types, timeouts, channel mapping, and parameter names are not yet defined in the supplied sources.

**[DECIDED]** A CAN inspector is a desirable later configurator feature, not a prerequisite for the first firmware/configurator skeleton.

---

# Part VII — Vektor Protocol truth base

> **Historical summary:** Sections 29 through 36 below preserve the state of the original protocol draft. `VEKTOR_SERIAL_PROTOCOL_TRUTH.md` now freezes the v1 wire contract, including framing, CRC, IDs, flags, typed values, retry behavior, routing messages, telemetry quality, and timestamps. Use that document—not the unresolved markers below—for implementation and interoperability.

## 28. Protocol role

**[DECIDED]** Vektor Protocol is the primary native service/configuration protocol for the Vektor configurator.

**[DECIDED]** It is an adapter over the common component model. It is not the parameter database and it is not application-specific VSP logic.

**[DECIDED]** MAVLink remains a parallel compatibility adapter over the same state where useful.

---

## 29. Current framing draft

**[DRAFT]** The protocol draft uses COBS on a byte stream with zero-byte packet delimiting.

Decoded packet shape:

```text
version        u8
message_type   u8
flags          u8
sequence       u16
payload_len    u16
payload        byte[payload_len]
crc32          u32
```

**[DRAFT]** Multibyte fields are little-endian.

**[DRAFT]** CRC covers decoded header and payload, excluding the CRC field itself.

**[OPEN]** The exact CRC32 polynomial, initialization, reflection, final XOR, and test vector have not been specified. These must be frozen before independent implementations are expected to interoperate.

**[OPEN]** Message-type numeric values and flag-bit assignments are not defined in the source draft.

---

## 30. Protocol traffic classes and priority

**[DECIDED]** Control and configuration traffic must take precedence over background transfer.

Current intended priority is:

```text
1. control responses, actions, errors
2. important events
3. fast realtime attitude/control observables
4. ordinary telemetry
5. file transfer and background bulk work
```

**[DECIDED]** Realtime visualization uses newest-value-wins behavior. Old unsent samples may be replaced rather than queued.

**[DECIDED]** File transfer and logging must yield naturally to control work.

**[OPEN]** The draft calls control traffic reliable and ordered, but does not yet define a complete application-layer retransmission/ACK mechanism for a lossy UART transport. USB CDC and UART transport semantics should be documented separately.

---

## 31. Stable IDs and discovery

**[DRAFT]** Components, fields, actions, observables, and routable ports receive stable 32-bit IDs generated at build time, with duplicate detection.

**[OPEN]** The ID-generation algorithm and stability rules across renamed objects or firmware versions are not yet specified.

### 31.1 HELLO

The current consolidated handshake should report at least:

```text
protocol_version
firmware_version
hardware_type
hardware_revision
board_id
schema_hash
capability_hash
device_id
capability_flags
```

**[DECIDED]** `schema_hash` and `capability_hash` allow descriptor caching and avoid downloading unchanged metadata on every connection.

**[OPEN]** Exact hash algorithm, byte ordering, width, and lifecycle rules are not yet frozen.

### 31.2 DESCRIBE

**[DECIDED]** Discovery must cover both application objects and hardware endpoints.

Descriptor categories need to represent at least:

- board identity
- component instances
- component inputs and outputs
- parameters
- observables
- actions
- PWM banks and timer groups
- Flex ports and supported modes
- UART endpoints and electrical features
- CAN endpoints
- exposed ADC channels
- storage areas
- attitude/sensor services
- runtime rate limits

**[DECIDED]** Hardware capability and application component inventory are separate concepts.

---

## 32. Parameters over Vektor Protocol

**[DRAFT]** Core operations include:

```text
GET
SET
VALUE
GET_MANY
SET_MANY
VALUES
GET_ALL_PARAMS
```

**[DECIDED]** Firmware validates and persists through the same `AP_Param` objects available to MAVLink.

**[DECIDED]** A successful SET should return the accepted value rather than assuming the requested value was accepted unchanged.

**[OPEN]** Exact typed-value binary encoding, string length encoding, enum metadata, and bulk-message size limits are not yet fixed.

---

## 33. Realtime subscriptions

**[DRAFT]** The PC requests one or more observables and a desired period or rate.

Firmware returns an accepted rate and subscription ID.

Compact telemetry then carries:

```text
subscription_id
timestamp
values...
```

The subscription definition supplies field order and types so IDs do not need to repeat in every sample.

**[DECIDED]** The client must adapt to accepted runtime limits instead of assuming H743 and F405 offer identical stream rates.

### 33.1 Attitude stream

**[DRAFT]** A fast attitude payload may contain:

```text
sequence
timestamp_us
q[4] float32
gyro[3] float32
```

Earlier targets of roughly 250 to 500 Hz are useful for the full board when the estimator and transport support them. The reduced board may accept a lower rate.

**[OPEN]** Timestamp width, epoch, wrap handling, and synchronization semantics are not yet frozen.

---

## 34. Actions and events

**[DECIDED]** One-shot operations are actions, not magic parameter values.

Examples include:

- VSP calibration
- zero feedback
- reset statistics
- start identification
- compass calibration
- reboot
- reboot to bootloader

Long operations should expose progress through observables and completion or failure through events.

Events cover asynchronous occurrences such as sensor invalidity, CAN bus-off, configuration reload, calibration completion, and file-transfer completion/failure.

**[OPEN]** Operation-ID width, cancellation semantics, and action argument encoding remain unspecified.

---

## 35. File service

**[DECIDED]** File access uses logical areas rather than raw MCU addresses or unrestricted host-like filesystem paths.

Candidate areas remain:

```text
CONFIG
LOGS
CALIBRATION
USER
UPDATE
```

Core operations remain:

```text
FILE_LIST
FILE_STAT
FILE_OPEN
FILE_READ
FILE_WRITE
FILE_CLOSE
```

Optional delete, rename, and mkdir operations should only be added for a real product need.

**[DECIDED]** Chunk operations carry offsets so retry/resume is possible.

**[DECIDED]** Final writes should be checked by length and preferably CRC/hash. Atomic replacement should use staging where required.

**[IMPORTANT]** The logical file-service API may exist on both boards even though storage backends differ. The reduced board has verified SD/dataflash support. The current H743 schematic does not yet have a verified SD device.

---

## 36. Initial protocol message set

The draft v1 set is:

```text
HELLO
DESCRIBE
GET
SET
VALUE
GET_MANY
SET_MANY
VALUES
SUBSCRIBE
UNSUBSCRIBE
TELEMETRY
ACTION
ACTION_ACK
EVENT
FILE_LIST
FILE_STAT
FILE_OPEN
FILE_READ
FILE_WRITE
FILE_CLOSE
PING
ERROR
```

**[OPEN]** Routing configuration is conceptually part of the product but the supplied protocol draft does not yet freeze dedicated route-management message names or their binary schema.

---

# Part VIII — Configurator

## 37. Runtime-driven UI

**[DECIDED]** The configurator is one multiplatform PC application for Windows, macOS, and Linux.

**[DECIDED]** Connection flow is capability-driven:

```text
connect
  -> HELLO
  -> identify board/firmware/hashes
  -> DESCRIBE if metadata is not cached
  -> build hardware model
  -> build component model
  -> render only supported capabilities
```

Examples:

- six PWM outputs on the reduced board means show six
- twelve primary outputs on the full board means show twelve
- no CAN on reduced means no CAN routing UI
- no verified SD on a board means no SD-specific UI
- the six Flex channels should be rendered separately from the dedicated UART7_RX row even though they share one physical header bank

**[DECIDED]** The UI may use board-specific artwork, connector diagrams, and friendly labels as presentation metadata, but generic connection and configuration must not depend on hard-coded board artwork.

---

## 38. Configurator interaction style

**[DECIDED]** The configurator should be graphics-heavy enough to make I/O and routing understandable, with drag/drop where useful.

**[DECIDED]** It should not force the user into a pure PLC graph as the only way to configure the machine.

**[DECIDED]** A plain, utilitarian desktop application appearance is acceptable. It does not need a futuristic visual language or a web-dashboard aesthetic.

**[DRAFT]** Avalonia is a candidate multiplatform framework, not a frozen architectural requirement in the supplied sources.

**[DECIDED]** A GUI skeleton can be developed before firmware exists by using the board render and dummy capability/component data, provided the dummy data is not mistaken for actual protocol behavior.

---

# Part IX — Known contradictions and corrections

## 39. Corrections to earlier briefs

### 39.1 The old G0B1 CLIM6 hardware is not Core Evo hardware

**[CORRECTION]** The CLIM6 brief describes a previous STM32G0B1 controller and SPI2 pin plan. Those details must not be imported into the H743 Core Evo hwdef.

The current full Vektor board uses STM32H743VIT6 and ICM-20602 on SPI4.

### 39.2 “Six Flex ports” does not mean six physical rows

**[CORRECTION]** There are six timer Flex channels plus one UART7_RX row in a seven-row header bank.

### 39.3 Full-board SD is not yet hardware fact

**[CORRECTION]** Previous consolidated text refers generally to full-board SPI storage/SD direction. The current H743 schematic has SPI3 signals but no SD socket or storage IC. SD is verified only on the reduced F405 source at present.

### 39.4 Enabled MCU UART does not imply connector capability

**[CORRECTION]** USART2 and UART7_TX exist in the `.ioc` but are not externally exposed in the current schematic. Capability discovery must represent external usable endpoints.

### 39.5 H743 performance targets are not current CubeMX clock facts

**[CORRECTION]** The H743 chip can support far more performance, but the supplied `.ioc` currently reports a 64 MHz CPU/system configuration. Proposed 1 kHz-class loops remain scheduling design targets until the actual ChibiOS board and firmware are profiled.

### 39.6 FDCAN does not imply CAN FD at the connector

**[CORRECTION]** The MCU has FDCAN blocks, while the current board uses SN65HVD232 transceivers. Advertise the physical CAN capability actually validated.

---

# Part X — Current red flags and open engineering items

## 40. Hardware and hwdef issues to resolve

The following items should be treated as actionable before declaring the H743 board definition stable.

- **[OPEN] Correct `SPI4_INT` direction.** PE3 is currently configured as a GPIO output in the `.ioc`, while it is an IMU interrupt input net.
- **[OPEN] Freeze the H743 clock tree.** Current CubeMX is 64 MHz and contains a questionable USB kernel-clock value. The ArduPilot/ChibiOS board definition must be deliberate.
- **[OPEN] Freeze CAN nominal bit timing.** Do not inherit the `.ioc` 2.6875 Mbit/s calculated value as product behavior.
- **[OPEN] Decide full-board SD hardware.** SPI3 is reserved but no storage device is present in the schematic snapshot.
- **[OPEN] Decide what to do with USART2 and UART7_TX.** Remove unused peripheral enablement or explicitly expose the signals in a future hardware revision.
- **[OPEN] Confirm SWD/manufacturing access.** SWD nets exist, but no dedicated connector/test-pad group is evidenced.
- **[OPEN] Verify CAN termination policy.** One bus appears directly terminated while the other has a solder-jumper path.
- **[OPEN] Verify ADC sampling time.** Current 1.5-cycle CubeMX setting should be checked against the VCC divider source impedance.
- **[OPEN] Decide external ESD strategy.** No obvious USB/CAN TVS protection is visible in the current schematic snapshot.
- **[OPEN] Confirm USB-only +5 V behavior.** Measure the +5 V connector rail when the board is powered through VBUS only.
- **[OPEN] Define exact board revision and board ID values** that the protocol will report.

---

## 41. Firmware implementation status

The current `Tools/Vektor` scaffold implements and tests:

- standalone `vektor` Waf application entry point;
- reduced `revo-mini` F405 build selection and H743/F405 capability records;
- bounded typed signal primitives with timestamps and quality;
- a minimal `AP_Param`-backed system parameter set;
- a common component/field schema registry used for stable-ID lookup,
  parameter enumeration, and protocol descriptor generation;
- runtime uptime and loop-timing observables;
- Vektor Protocol v1 COBS framing, CRC-32/ISO-HDLC, bounded stream parsing, and the canonical frame vector;
- `PING`, `HELLO`, paged board/component/field/endpoint/timer-group/runtime-limit `DESCRIBE`, `ERROR`, typed `GET`/`SET`, bulk parameter operations, and duplicate-request replay/conflict handling;
- deterministic nonzero schema/capability hashes over ID-sorted descriptor records, with initialization-time stable-ID collision validation;
- bounded `SUBSCRIBE`/`UNSUBSCRIBE` sessions and compact volatile `TELEMETRY` for the built-in realtime observables, with scheduler-rate negotiation and newest-sample behavior;
- host tests covering codec/parser behavior, schema registry serialization, replay caching, runtime timing, subscription scheduling, and an end-to-end UART telemetry session.

The following remain design or bring-up work rather than implemented product behavior:

- full H743 ChibiOS hwdef and hardware build;
- build-generated schema tables and build-time collision failure (the current common registry is hand-authored and validates IDs during initialization);
- hardware endpoint drivers, scheduler profiles, and measured realtime limits;
- common attitude service across ICM-20602 and BMI088;
- VSP/VRS components and persistent signal routing with cycle validation;
- action/event services, DroneCAN integration, and MAVLink coexistence;
- logical file-service backends and low-priority logging;
- configurator protocol client and UI.

---

## 42. Remaining protocol implementation gaps

`VEKTOR_SERIAL_PROTOCOL_TRUTH.md` freezes the v1 interoperability details that were open in the original draft. The current firmware still needs:

- route service handlers and persistent route validation;
- action schemas, action/event handlers, and operation lifecycle behavior;
- file-service handlers and board-specific chunk limits where storage is advertised;
- a finalized physical USB/UART port-selection policy and MAVLink coexistence validation.

---

# Part XI — Capability model for the two current targets

## 43. Truthful capability matrix

This table distinguishes physical evidence from intended generic abstractions.

| Capability | H743 Core Evo current hardware | F405 Reduced current hwdef |
|---|---|---|
| MCU | STM32H743VIT6 | STM32F405xx |
| Native USB FS | Yes | Yes |
| Primary PWM outputs | 12 | 6 |
| Timer Flex channels | 6 | None defined as Flex |
| Dedicated receiver/input row | UART7_RX in Flex bank | USART inputs through board UARTs |
| Onboard IMU | ICM-20602 on SPI4 | BMI088 on SPI1 |
| Onboard magnetometer | MMC5983MA on I2C1 | No onboard compass defined |
| External I2C | I2C2 on GNSS connector, I2C1 occupied by onboard compass | I2C1 |
| CAN | Two controller/transceiver ports | None in current hwdef |
| CAN FD physical guarantee | No | No CAN |
| External serial connectors | USART1, UART5, UART8, GNSS USART3 | USART1, USART3, USART6 in hwdef |
| Configured but unexposed UART | USART2, UART7_TX | Not assessed from schematic because only hwdef supplied |
| VCC ADC sensing | PC4 with 56k/10k divider and 100 nF | Battery voltage/current and USB sense ADCs |
| SD card | Not present in supplied schematic | Yes, SPI3 hwdef |
| Dataflash | Not present in supplied schematic | Yes, SPI3 hwdef |
| Configuration straps | CONF1, CONF2 | Not defined in supplied hwdef |
| Status LEDs | Blue, amber, plus power LED | Blue and red |
| Buzzer | PC14 output | Not defined in supplied hwdef |

---

## 44. What the client should discover instead of hard-code

**[DECIDED]** Runtime descriptors should make at least these facts discoverable:

```text
board identity and revision
firmware version
available application components
physical PWM channel count
PWM timer groups and accepted rates
Flex count and per-port supported modes
dedicated receiver endpoints
UART endpoint direction and electrical features
CAN presence and controller count
ADC observables
sensor/attitude service presence
storage areas and backends
max realtime subscription rate
control/update profile information where meaningful
```

The H743 seven-row Flex/receiver bank is a good test case for why runtime descriptions must model capabilities accurately rather than infer them from connector width.

---

# Part XII — Safety and failure model

## 45. Local failure isolation

**[DECIDED]** Because the system is power-on operational, failure behavior must be local and predictable.

Required principles include:

- stale-input detection
- `VALID`, `STALE`, and `INVALID` propagation
- sensor communication validity checks
- bounded output values
- VSP ring and VRS authority limits
- predictable lost-command-source behavior
- watchdog integration
- CAN bus-off handling on the full board
- parameter range validation
- bounded calibration actions
- storage operations that cannot stall control work

**[DECIDED]** A failed visualization stream, file transfer, or logging task must not be able to starve VSP/VRS control.

---

# Part XIII — Implementation order derived from the truth base

## 46. First full-board bring-up tasks

1. Create the H743 ChibiOS/ArduPilot hwdef from the verified pin map rather than from prose.
2. Correct PE3 `SPI4_INT` to input/interrupt behavior.
3. Establish final oscillator and clock configuration and verify USB timing.
4. Bring up native USB.
5. Bring up ICM-20602 on SPI4 with DRDY.
6. Bring up MMC5983MA on I2C1.
7. Register the 12 main PWM outputs with their four timer groups.
8. Register six Flex timer channels with their two timer groups.
9. Register `UART7_RX` as a separate receiver-capable endpoint despite its physical location in the Flex header bank.
10. Register USART1, UART5, UART8, and the USART3/I2C2 GNSS connector according to actual external connectivity.
11. Register VCC ADC sensing on PC4 and validate ADC acquisition time.
12. Bring up CAN1/CAN2 at a deliberately selected classic CAN bitrate and verify termination.
13. Leave SPI3/storage capability disabled until real storage hardware is present and tested.
14. Establish manufacturing/debug access before hardware freeze.

---

## 47. Common firmware skeleton order

1. Board identity and capability registry.
2. Common typed signal structure with timestamp and quality.
3. Component registry.
4. ParameterService around AP_Param.
5. Generic PWM endpoint and timer-group metadata.
6. Command-source abstraction.
7. Common attitude/motion service.
8. VSP component with normalized X/Y and ring-limited logical outputs.
9. Internal optional VRS.
10. Routing service.
11. Vektor Protocol framing, HELLO, and DESCRIBE.
12. GET/SET and bulk parameter operations.
13. Realtime subscriptions with requested-versus-accepted rates.
14. Actions and events.
15. Reduced-board build against the same application model.
16. Generic file-service abstraction, using actual reduced-board storage first.
17. DroneCAN source and actuator endpoints on H743.
18. Low-priority logging.
19. MAVLink compatibility adapter where it provides concrete value.

---

# Part XIV — Rules for future work

## 48. Things future work may assume

Future Vektor work may treat the following as current project invariants unless a later source explicitly changes them:

- One common application architecture supports H743 and F405 targets.
- Hardware inventory is board-defined and runtime-described.
- The configurator adapts to descriptors rather than board-name branching.
- Application components do not directly own MCU pins or bus drivers.
- AP_Param is the persistent parameter authority.
- Vektor Protocol is the native rich configuration/service adapter.
- MAVLink is a parallel compatibility adapter, not a second parameter database.
- VSP is a high-level component.
- VRS is normally an internal VSP feature.
- VSP/VRS belongs to the fast control path.
- PWM timing is a hardware-endpoint concern with timer-group constraints.
- CAN input channels and UART receiver channels enter a common routing model.
- VSP outputs can eventually target PWM or DroneCAN actuators.
- Logging and file work are lower priority than control and realtime service.
- The H743 board currently has 12 primary timer outputs, six timer Flex channels, and one separate UART7_RX row in the same Flex header bank.
- The reduced F405 board currently has six PWM outputs and no CAN endpoint in its supplied hwdef.

---

## 49. Things future work must **not** assume

Do not assume any of the following without a newer verified source:

- that the H743 runs at 480 MHz or any other maximum clock
- that the current `.ioc` clock tree is final
- that full-board SD hardware exists
- that SPI3 should automatically be advertised as storage
- that every enabled UART in CubeMX is externally available
- that all six Flex timer pins support ADC mode
- that the seven-row Flex header means seven Flex timer ports
- that FDCAN means CAN FD is electrically supported
- that the `.ioc` 2.6875 Mbit/s CAN calculation is the desired bus rate
- that every message defined by the frozen Vektor Protocol v1 wire contract is already implemented
- that the existing firmware scaffold already drives the described physical endpoints or implements VSP/VRS
- that an Avalonia GUI implementation already exists
- that old STM32G0B1 CLIM6 pin mappings apply to Core Evo
- that every Vektor application needs GPS, magnetometer, barometer, SD, MAVLink, or DroneCAN
- that H743 and F405 must run identical loop rates
- that visual board artwork is authoritative hardware metadata

---

# Part XV — Short canonical architecture summary

## 50. Current working model

```text
                        PC / Mac / Linux Configurator
                                   |
                             Vektor Protocol
                                   |
                         HELLO + DESCRIBE
                                   |
                     capability-driven client model
                                   |
             +---------------------+---------------------+
             |                                           |
     Vektor Core Evo H743                      Vektor Core Reduced F405
     12 PWM + 6 Flex + RX                      6 PWM
     ICM-20602 + MMC5983MA                     BMI088
     dual classic-CAN physical ports           no CAN in current hwdef
     no verified SD in current schematic       SD + dataflash in hwdef
             |                                           |
             +---------------------+---------------------+
                                   |
                         SAME VEKTOR APPLICATION
                                   |
       +---------------------------+---------------------------+
       |                |                   |                  |
  Components       typed routing        AP_Param          observability
       |                                    |                  |
Command Sources                         ParameterService       actions/events
 UART / CAN                                  |
       |                                     +--> Vektor Protocol
       |                                     +--> MAVLink
       v
   VSP1 / VSP2
   optional VRS
       |
 logical actuator commands
       |
   endpoint routing
      / \
     /   \
   PWM   DroneCAN actuator output
```

The central separation remains:

> **The board defines physical reality. The Vektor application defines logical machine behavior. The protocol exposes both without requiring the client to know the PCB in advance.**

---

# Part XVI — Revision policy for this truth base

## 51. How this file should evolve

This file should be updated when one of the following becomes available:

- a newer KiCad schematic or PCB revision
- the final H743 ArduPilot/ChibiOS hwdef
- a revised reduced-board hwdef
- material changes to the Vektor firmware source or its verified test/build status
- measured scheduler/latency results
- a frozen Vektor Protocol specification with numeric encodings
- DroneCAN implementation details
- an implemented configurator protocol model
- final board IDs and hardware revision naming
- full-board SD/storage hardware, if added

When a future source contradicts this file, update the specific fact and preserve the reason for the change. Do not silently convert a draft item into a verified item.

---

## 52. Current highest-priority truth gaps

If only a small number of things are resolved next, these provide the greatest increase in certainty:

1. Finalize the H743 hwdef and clock tree.
2. Correct and validate ICM-20602 DRDY/`SPI4_INT` behavior.
3. Confirm the six Flex electrical modes pin by pin.
4. Decide whether the full board actually receives an SD socket or other SPI3 storage.
5. Freeze classic CAN bitrate and termination behavior for both H743 ports.
6. Define SWD/reset manufacturing access.
7. Implement the first VSP component with typed routable inputs and outputs.
8. Implement and persist the route model, then prove the same VSP/route behavior on H743 and F405 builds.
9. Replace the hand-authored schema registry arrays with build-generated tables and build-time collision checks.
10. Measure actual control, attitude, USB, CAN, and file-transfer scheduling behavior before turning proposed rates into product claims.
