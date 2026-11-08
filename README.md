# ICE_end-of-arm_tooling_controller: Custom EOAT Servo & IO Interface

<img width="300" alt="ICE_EOAT_PCB_Layout" src="https://github.com/user-attachments/assets/d49dc73c-fd0c-4243-92b0-a54d4ccf5f29" />
<img width="290" alt="ICE_EOAT_PCB_3D_Viewer" src="https://github.com/user-attachments/assets/a26d4fdb-f94b-4e18-8b1e-05bae88245e8" />

A custom-designed, 64mm circular End-of-Arm Tooling (EOAT) controller PCB designed for the Dobot Magician E6 robot arm. It bridges the robot's 24V industrial PNP digital I/O with a custom servo-driven gripper, providing multi-rail power regulation, opto-isolated signal shifting, current sensing, and dual-protocol servo control.

## Overview

The Dobot Magician E6 features an end-effector interface with 24V power and 24V PNP digital I/O (`24V`, `DO_1`, `DO_2`, `DI_1`, `DI_2`, `GND`). To use a custom servo-driven gripper with the arm, I needed an intermediate controller board capable of stepping down the 24V supply and translating the 24V I/O signals into standard hobby PWM or serial bus servo commands.

A major design constraint was the physical footprint: the robot's end-effector is 65mm in diameter. This PCB is designed with a 64mm diameter and matching M6 mounting holes so it can be bolted directly inline between the robot flange and the custom gripper. Additionally, because the board will be assembled using a reflow soldering hotplate, the layout was heavily optimized to pack almost all SMD components onto the front side while keeping the footprint compact.

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
* **Debugging:** 5-pin 2.54mm right-angleSMD header (`PZ2.54-1x5P-SMD-RA`) for ST-Link SWD programming.

## Current Status

* **V1.0 (In Progress):** Schematic and 64mm circular PCB layout are complete. PCBs and components have been ordered and are awaiting arrival for hotplate reflow assembly and initial bring-up.

## Project Timeline

| Date | Milestone |
|---|---|
| Nov 4, 2026 | Project planning and component research |
| Nov 5, 2026 | Started schematic capture |
| Nov 6, 2026 | Schematic completed; started 64mm circular PCB layout |
| Nov 7, 2026 | PCB layout completed; PCBs and BOM ordered |
