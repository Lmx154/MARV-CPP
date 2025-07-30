# 🚀 MARV Project - Modular Avionics for Rockets and Vehicles

A C++ Arduino project for Raspberry Pi Pico 2 (RP2350) with three separate sketches: Flight Controller (FC), Radio, and Ground Station (GS). This project implements a UAV system where the FC handles flight operations and sensors, the Radio manages wireless communications, and the GS monitors telemetry. ✈️📡🛰️

## 🗂️ Project Structure

```
lmx154-marv/
├── fc/                     # Flight Controller Sketch
│   ├── fc.ino             # FC main application code
│   └── mcp2515_driver.cpp # MCP2515 CAN controller driver (using ArduinoMCP2515 library)
├── radio/                 # Radio Sketch
│   ├── radio.ino          # Radio main application code
│   └── mcp2515_driver.cpp # MCP2515 CAN controller driver (using ArduinoMCP2515 library)
├── gs/                    # Ground Station Sketch
│   └── gs.ino             # GS main application code
└── platformio.ini         # PlatformIO configuration for building all sketches (optional)
```

## 🛠️ Hardware Requirements

- **Microcontrollers**: Three Raspberry Pi Pico 2 boards (RP2350A) for FC, Radio, and GS. 🖥️
- **Flight Controller (FC)**: 🛩️
  - Sensors: BMP388 barometer (0x77), PCF8563 RTC (0x51), and MS5611-01BA03 barometer (0x76) (I2C1 on GP2 SDA, GP3 SCL); BMI088 IMU (SPI0 on GP18 CLK, GP19 MOSI, GP16 MISO, GP20 CS GYRO, GP17 CS ACCEL); BMM350 magnetometer (0x14) and ICM-20948 IMU (0x69) (I2C0 on GP20 SDA, GP21 SCL); Ublox NEO-M9N GPS (UART0 on GP0 TX, GP1 RX).
  - CAN: MCP2515 (SPI1 on GP10 CLK, GP11 MOSI, GP12 MISO, GP13 CS) for communication with Radio.
  - SD Card: SPI on GP6 CLK, GP7 MOSI, GP8 MISO, GP9 CS.
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Radio**: 📻
  - LoRa: E32900T30D (UART0 on GP0 TX, GP1 RX).
  - CAN: MCP2515 (SPI0 on GP18 CLK, GP19 MOSI, GP16 MISO, GP17 CS) for communication with FC.
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Ground Station (GS)**: 🖥️
  - LoRa: E32900T30D (UART0 on GP0 TX, GP1 RX).
  - LED: GPIO25.
  - Debug UART: UART1 on GP4 TX (to CP2102 RX), GP5 RX (from CP2102 TX) for USB serial logging.
- **Flashing/Debugging**: Use the Arduino IDE or PlatformIO for uploading sketches via USB bootloader or SWD. For SWD, use a Raspberry Pi Pico as a probe with official firmware. 🧪
- **Debugging Hardware**: CP2102 USB-to-serial converters connected to the debug UART pins on each board for viewing logs on a host computer. 🐞

For detailed pinout diagrams and configurations, refer to `docs/hardware.md`. 📝

## 📡 Protocols and High-Level Overview

- **Intra-Module (FC-Radio)**: MultiWii Serial Protocol (MSP) over CAN bus for lightweight, binary data transfer (e.g., telemetry, commands). Frames include headers, payloads, and CRC-16-CCITT checksums for integrity. 🔗
- **Inter-Module (Radio-GS)**: MAVLink v2 over LoRa (915 MHz) for standardized telemetry and commands, with RSSI/SNR appended. 📶
- **Checksums**: CRC-16-CCITT on all CAN packets to detect corruption. ✅
- **High-Level View**: The FC acquires sensor data (e.g., IMU, GPS), performs estimation (AHRS/EKF) and control, and sends raw data via CAN to the Radio. The Radio encodes/decodes MAVLink and transmits over LoRa to the GS, which forwards to a computer (e.g., QGroundControl). Reverse flow handles commands. This modular setup ensures the FC focuses on real-time flight without comms overhead. 🔄

## 🧑‍💻 Installation from Scratch

1. **Install Arduino IDE**: Download from https://www.arduino.cc/en/software. 🛠️
2. **Add RP2350 Board Support**: Install the official Raspberry Pi Pico Arduino core via Boards Manager (search for "Raspberry Pi Pico"). Ensure RP2350 support is included (available as of 2025).
3. **Install Libraries**: Use Library Manager to install:
   - Wire (for I2C)
   - SPI
   - SD (for SD card)
   - Adafruit_BMP3XX (for BMP388)
   - Adafruit_Sensor (base for sensors)
   - Adafruit_BMI088 (if available, else custom)
   - Adafruit_ICM20948
   - Adafruit_BMM350 (if available)
   - TinyGPS++ (for Ublox GPS)
   - ArduinoMCP2515 (for CAN)
   - MAVLink (for MAVLink v2)
   - FreeRTOS (for task management)
   - Other sensor-specific libs as needed.
4. **Set Up Debugger**: For SWD, flash probe firmware to another Pico following official docs.

## 🏗️ Build and Upload Commands

Use Arduino IDE or PlatformIO.

### Building and Uploading Specific Sketches

**Flight Controller:** 🛩️
- Open `fc/fc.ino` in Arduino IDE.
- Select board: Raspberry Pi Pico 2.
- Build and upload via USB.

**Radio:** 📻
- Open `radio/radio.ino`.
- Build and upload.

**Ground Station:** 🖥️
- Open `gs/gs.ino`.
- Build and upload.

For PlatformIO: Use `platformio.ini` to build all.

## 🖥️ Viewing Debug Output

Each board outputs logs over UART1 (GP4 TX, GP5 RX). Connect CP2102 and use a serial terminal (e.g., Arduino Serial Monitor) at 115200 baud.

1. Connect CP2102: Board TX (GP4) to CP2102 RX; Board RX (GP5) to CP2102 TX; share GND.
2. Upload sketch.
3. Open Serial Monitor.

Logs prefixed with [INFO], [WARN], or [ERROR].

## 🔄 Development Workflow

1. **Choose target**: FC, Radio, or GS. 🎯
2. **Edit files**: In respective directories. ✏️
3. **Build and test**: Use Arduino IDE to verify/compile. 🧪
4. **Upload**: Upload to device. ⚡

## ⚙️ Technical Details

- **Target**: RP2350 (ARM Cortex-M33). 🎯
- **HAL**: Arduino core for RP2350.
- **Framework**: FreeRTOS for real-time concurrency. ⏱️
- **Libraries**: Use standard Arduino libraries for peripherals and sensors.

## 📦 Dependencies

- Arduino core
- FreeRTOS
- Sensor libraries (Adafruit, etc.)
- MAVLink
- ArduinoMCP2515

## 📄 License

MIT License - See project metadata for details. 📝

