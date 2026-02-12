# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**MDRobot** — A metal detecting robot platform that autonomously or manually scans open areas using a towed coil array. The robot uses GPS, beacon triangulation, IMU, and wheel encoders for positioning, and logs detections with coordinates for color-map generation.

## Current Controller

**WAGO PFC100 (750-8100)** running **e!COCKPIT**. Robot is driveable via WebVisu and Modbus registers.

| Field | Value |
|-------|-------|
| Model | WAGO 750-8100 (PFC100) |
| IP | 10.0.0.223 |
| MAC | 00:30:DE:42:B5:23 |
| Runtime | e!COCKPIT (CODESYS 3.5 based) |
| WBM | https://10.0.0.223 |
| WebVisu | http://10.0.0.223:8080 |
| Status | Running — robot is driveable |

| Interface | Protocol | Purpose |
|-----------|----------|---------|
| WAGO WBM | HTTPS :443 | Web Based Management |
| WAGO WebVisu | HTTP :8080 | Operator visualization / drive control |
| WAGO ↔ Motors | Modbus registers | Motor speed/direction via Sabertooth amp |

**Dev environment:** e!COCKPIT runs in a VMware VM at **10.0.0.75** (RDP) on the Windows PC (10.0.0.31).

| Component | Tool | Deploy Method |
|-----------|------|---------------|
| e!COCKPIT project | e!COCKPIT IDE | Upload to WAGO PFC100 at 10.0.0.223 |

## Original Architecture (archived)

The robot was originally controlled by a Raspberry Pi running Codesys v3.5.8 with a Teensy 3.1 I2C slave. All original hardware (Pi, Teensy, IMU, GPS, beacons, pendant) has been removed. The source code and documentation remains in this repo for reference.

### Three-Tier Control System

1. **High-level** — Windows PC (Azure GUI, SQL historian, Kinect, Unreal Engine 3D interface). Connects to supervisor Pi via Modbus TCP.
2. **Supervisor** — Raspberry Pi running **Codesys v3.5.8** softPLC runtime. Hosts web GUI, Pi Camera streaming, and orchestrates all subsystems. A secondary Pi (RoboPi) handles ultrasonic sensors and wheel encoders.
3. **Low-level I/O** — **Teensy 3.1** microcontroller as I2C slave (address `0x2B`) to the supervisor Pi. Handles relay control, 12-bit ADC, RTC, and watchdog safety (500ms heartbeat — all relays fail-safe OFF if Pi stops responding).

Motor control: Codesys → I2C DAC (LTC 2619) → analog signals → Sabertooth 2x25 motor amplifier.

Positioning: GPS Hat + 3-corner beacon triangulation (each beacon is a Pi with GPS + Modbus server) + dead reckoning from wheel encoders + 10-DOF IMU sensor fusion.

Operator control: ESP8266 hand pendant (joystick + E-stop) over WiFi, or laptop via VNC to onboard Windows PC.

### Source Code

| File | Language | Purpose |
|------|----------|---------|
| `Source Code/sketch_Teensy_MDRobot.ino` | Arduino/C++ | Teensy 3.1 I2C slave firmware — relays, ADC, RTC, watchdog |
| `Source Code/gpsdData.py` | Python | GPS daemon → Modbus TCP bridge (writes lat/lon as 32-bit floats to registers) |
| `Source Code/Robot_R4_022116.zip` (+.z01-.z06) | Codesys ST | Main PLC project archive — all POUs, FBs, methods in Structured Text |
| `Source Code/Robot_R4_pdfs/` | PDF | Exported documentation for all 24 Codesys components |

### Teensy I2C Wire Format (16+ bytes)

```
[0]    Counter (watchdog toggle)     [1]    Relay bits (4 SSRs)
[2-3]  ADC Ch0 (12-bit lo/hi)       [4-5]  ADC Ch1
[6-7]  ADC Ch2                       [8-9]  ADC Ch3
[10]   Digital inputs (4 switches)   [11-13] Hour/Min/Sec
[14-15] Day/Month                    [16-17] Year (lo/hi)
```

### Codesys POUs (documented in `Robot_R4_pdfs/`)

Key components: `POU_Global_System` (main loop), `POU_Teensy` (I2C master), `POU_Estop_Control`, `POU_Robot_Drive_1/2` + `Drive_Select`, `POU_Read_GPS`, `POU_Modbus1/2` (beacon polling), `POU_Camera`, `POU_LTC_2619` (DAC), IMU drivers (`BMP085`, `LSM303`, `L3GD20H`), `OSCAT_FILTER_MAV_DW` (moving average filter).

### Original Build & Deploy

| Component | Tool | Deploy Method |
|-----------|------|---------------|
| Teensy firmware (.ino) | Arduino IDE or PlatformIO | USB via Teensy Loader |
| Codesys project (.zip) | Codesys IDE v3.5.8 | Upload to Pi runtime from IDE |
| Python GPS bridge | Python 2/3 | Copy to Pi, run directly |

**Python dependencies (for gpsdData.py):** `gpsd` daemon, `python-gps`, `pymodbus`

**Note:** `gpsdData.py` has a hardcoded Modbus server IP (`192.168.0.100`) that must be configured for your network.

**Teensy libraries:** `Wire.h`, `I2C_Anything.h`, `Time.h`/`TimeLib.h`

### Original Network & Protocols

| Interface | Protocol | Purpose |
|-----------|----------|---------|
| Pi ↔ Teensy | I2C (addr 0x2B) | Relay/ADC/RTC control |
| Pi ↔ IMU/DAC | I2C | Sensor reads, motor DAC output |
| Pi ↔ Beacons | Modbus TCP | Position triangulation polling |
| Pi ↔ GPS Hat | UART via gpsd | GPS coordinates |
| Pi ↔ Pendant | WiFi | Teleoperation commands |
| Teensy serial | UART @19200 | Debug output |

## Hardware Design

- `Board Files/MDRobot_supervisor_design_files.zip` — Schematics and PCB layouts (original Pi baseboard)
- `Board Files/MDRobot_supervisor_gerbers.zip` — Gerber files for PCB manufacturing
