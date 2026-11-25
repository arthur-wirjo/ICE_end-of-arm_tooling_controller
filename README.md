# ICE_end-of-arm_tooling_controller: Custom EOAT Servo & IO Interface

<img width="300" alt="ICE_EOAT_PCB_Layout" src="https://github.com/user-attachments/assets/d49dc73c-fd0c-4243-92b0-a54d4ccf5f29" />
<img width="290" alt="ICE_EOAT_PCB_3D_Viewer" src="https://github.com/user-attachments/assets/a26d4fdb-f94b-4e18-8b1e-05bae88245e8" />
<img width="300" alt="ICE_EOAT_PCB_V1 0" src="https://github.com/user-attachments/assets/0528be01-e217-49a9-adc4-c3686cf55d5b" />
<img width="300" alt="EOAT_PCB_Test_1" src="https://github.com/user-attachments/assets/8e923c3d-f69f-4c2f-b185-20f6c6872876" />

A custom-designed, 64mm circular End-of-Arm Tooling (EOAT) controller PCB designed for the Dobot Magician E6 robot arm. It bridges the robot's 24V industrial PNP digital I/O with a custom servo-driven gripper, providing multi-rail power regulation, opto-isolated signal shifting, current sensing, and dual-protocol servo control.

## Overview

The Dobot Magician E6 features an end-effector interface with 24V power and 24V PNP digital I/O (`24V`, `DO_1`, `DO_2`, `DI_1`, `DI_2`, `GND`). To use a custom servo-driven gripper with the arm, I needed an intermediate controller board capable of stepping down the 24V supply and translating the 24V I/O signals into standard hobby PWM or serial bus servo commands.

A major design constraint was the physical footprint: the robot's end-effector is 65mm in diameter. This PCB is designed with a 64mm diameter and matching M6 mounting holes so it can be bolted directly inline between the robot flange and the custom gripper. Additionally, because the board was assembled using a reflow soldering hotplate, the layout was heavily optimized to pack almost all SMD components onto the front side while keeping the footprint compact.

## Key Features

* **Direct Flange Mounting:** 64mm circular PCB profile with M6 mounting holes matching the Dobot Magician E6 end-effector (65mm diameter).
* **Dual Servo Compatibility:** Supports both standard 5V PWM servos (e.g., MG996R) and 12V Half-Duplex UART bus servos (e.g., Feetech ST3215).
* **Multi-Rail Power Regulation:** Dual high-efficiency buck converters stepping 24V down to 12V and 5V for the servos, plus an LDO providing a clean 3.3V rail for logic.
* **Opto-Isolated 24V I/O:** Bidirectional level shifting between the robot's 24V PNP digital I/O (`DI_1`, `DI_2`, `DO_1`, `DO_2`) and the 3.3V MCU logic.
* **Gripper Force / Current Sensing:** Low-side/in-line 50mΩ shunt resistor paired with a 20V/V current sense amplifier to monitor servo load and detect object grip or stall conditions.
* **Reflow-Optimized Layout:** High-density single-sided component placement designed specifically for hotplate reflow assembly.
* **Visual Diagnostics:** Dedicated onboard status LEDs for all power rails (24V, 12V, 5V, 3.3V), optocoupler I/O activity, and MCU heartbeat.

## Hardware

### Computing & Control
* **Microcontroller:** STM32G030F6P6TR (Compact mainstream Arm Cortex-M0+ MCU handling I/O logic, ADC current reading, PWM generation, and half-duplex UART).
* **Supported Actuators:** MG996R (5V + PWM) or Feetech ST3215 (12V + Half-Duplex UART).

### Power Stage & Protection
* **12V & 5V Buck Converters:** 2x TI LMR16030SDDAR (60V, 3A simple switcher buck regulators stepping 24V down to 12V and 5V, paired with 15µH 5A power inductors and SS56 Schottky diodes).
* **3.3V Logic LDO:** Microne ME6211A33PG-N (Steps the 5V rail down to 3.3V for the STM32).
* **Input Protection:** Littelfuse SMBJ26CA Bidirectional TVS Diode (Clamps transient voltage spikes on the 24V robot supply rail).

### Signal Conditioning & Sensing
* **24V I/O Isolation:** 4x Toshiba TLP291(GB-TP,SE) Optocouplers (Translates 24V PNP signals to 3.3V MCU inputs and vice versa).
* **PWM Level Shifter:** TI SN74LVC1T45DBVR Dual-Supply Bus Transceiver (Shifts 3.3V MCU timer output to 5V logic for standard PWM servos).
* **Current Sensing:** FOJAN 50mΩ 1W Shunt Resistor (1206) paired with a TI INA180A1IDBVR Current Sense Amplifier (Gain = 20V/V).

### Connectors & Interface
* **Robot I/O Input:** 6-pin JST-XH SMD (`S6B-XH-SM4-TB`) interfacing with the robot's M8 6-pin aviation connector.
* **UART Servo Output:** 3-pin 2.50mm pitch Molex 5268-03A / `HX25003-3WAP` (For ST3215 serial bus servo).
* **PWM Servo Output:** 3-pin 2.54mm pitch header (`PZ2.54-1x3P`).
* **Debugging:** 5-pin 2.54mm right-angle SMD header (`PZ2.54-1x5P-SMD-RA`) for ST-Link SWD programming.

## Firmware & Control Architecture

The firmware is written in C using the STM32 HAL framework and structured around a deterministic, non-blocking super-loop (`HAL_GetTick()`) without an RTOS to keep flash and RAM overhead minimal on the STM32G030F6P6.

* **50 Hz (20 ms) Control & Current Sensing Task:** Reads opto-isolated Dobot digital outputs, samples and filters servo current via the ADC, and updates either the PWM timer compare register or transmits a Feetech UART packet on state transitions.
* **1 Hz (500 ms toggle) Heartbeat Task:** Blinks the onboard status LED (`PB8`) to confirm normal MCU execution.

### Gripper Control Truth Table

The Dobot's two digital outputs (`DO_1` and `DO_2`) are mapped to four distinct operating modes:

| `D2_OUT` (`PA2`) | `D1_OUT` (`PA1`) | Command Mode | PWM Servo (MG996R) Behavior | UART Servo (Feetech ST3215) Behavior |
| :---: | :---: | :--- | :--- | :--- |
| **LOW (0)** | **LOW (0)** | **Hard Open** | Driven firmly to Open position (`1000 µs`). | Driven to Open position (`1000`) at 80% max torque. |
| **LOW (0)** | **HIGH (1)** | **Hard Close** | Driven firmly to Closed position (`2000 µs`) with 1.2A overload protection. | Driven to Closed position (`3000`) at 80% max torque. |
| **HIGH (1)** | **LOW (0)** | **Soft Open (Free / Idle)** | PWM pulses disabled (`CCR1 = 0`) allowing the gripper fingers to be moved freely by hand. | Torque Enable register (`0x28`) set to `0` so the gripper is back-drivable by hand. |
| **HIGH (1)** | **HIGH (1)** | **Soft Close (Compliant Grip)** | Steps closed incrementally and locks position with a slight back-off as soon as current reaches `450 mA`. | Commands Close with internal torque limit throttled to 25% (`Reg 0x30 = 250`) and reduced speed. |

### Current Sensing & Cable Protection
With a **50 mΩ** sense resistor and a **20 V/V** amplifier gain (`INA180A1`), the analog output scales cleanly to **1.0 V/A** (**1 mV = 1 mA**). Raw 12-bit ADC readings are smoothed using a fast integer exponential moving average filter:
```c
g_servo_current_mA = ((3 * g_servo_current_mA) + current_mA) >> 2;
```
To protect the robot's 1.5A-rated M8 aviation cable and prevent servo burn-out under stall conditions, the firmware enforces a **450 mA** threshold for compliant soft gripping and a **1200 mA (1.2A)** hard overload cutoff.

### MCU Pinout & STM32CubeMX Configuration

* **System Clock:** 64 MHz (Internal HSI + PLL)

| Pin | Signal / Label | Peripheral Mode | Key Configuration Parameters |
| :--- | :--- | :--- | :--- |
| `PA1` | `D1_OUT` | `GPIO_Input` | Active-High (External 10kΩ pull-down on PCB), `GPIO_NOPULL` |
| `PA2` | `D2_OUT` | `GPIO_Input` | Active-High (External 10kΩ pull-down on PCB), `GPIO_NOPULL` |
| `PA3` | `D1_IN` | `GPIO_Output` | Push-Pull, Active-High optocoupler drive to robot `DI_1` |
| `PA4` | `D2_IN` | `GPIO_Output` | Push-Pull, Active-High optocoupler drive to robot `DI_2` |
| `PA6` | `PWM_SERVO` | `TIM3_CH1` | Prescaler = `63` (1 MHz / 1 µs tick), Period = `19999` (50 Hz / 20 ms) |
| `PA7` | `CURR_SENSE` | `ADC1_IN7` | 12-bit resolution, `PCLK/4`, `79.5 Cycles` sampling time, calibrated on boot |
| `PB6` | `UART_SERVO` | `USART1_TX` | Half-Duplex Single-Wire mode, `1,000,000` bps (1 Mbps), 8N1 |
| `PB8` | `HEARTBEAT` | `GPIO_Output` | Push-Pull status LED, toggled every 500 ms |
| `PA13` | `SYS_SWDIO` | `SYS` | Serial Wire Debug Data |
| `PA14` | `SYS_SWCLK` | `SYS` | Serial Wire Debug Clock |

## Current Status

* **V1.0 (Completed & Operational):** Hardware bring-up, STM32G0 firmware development, and full integration testing with the Dobot Magician E6 robot arm and gripper are complete. All power rails (12V, 5V, 3.3V), opto-isolated 24V I/O channels, current sensing feedback, and servo control modes operated as designed on the first test run.
* **Future Enhancements:** Implement outbound digital feedback (`D1_IN`, `D2_IN`) to report object grip detection and fault/stall states back to the Dobot controller.

## Project Timeline

| Date | Milestone |
|---|---|
| Nov 4, 2026 | Project planning and component research |
| Nov 5, 2026 | Started schematic capture |
| Nov 6, 2026 | Schematic completed; started 64mm circular PCB layout |
| Nov 7, 2026 | PCB layout completed; PCBs and BOM ordered |
| Nov 14, 2026 | PCB and components arrived |
| Nov 15, 2026 | PCB soldered |
| Nov 20, 2026 | STM32 Firmware written; PCB tested and working with robot arm |
