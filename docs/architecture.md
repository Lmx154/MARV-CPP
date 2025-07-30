# High-Level Design Document for RP2350-Based UAV Flight Computer Firmware

## Document Overview

This document furnishes a comprehensive high-level design for the firmware of an unmanned aerial vehicle (UAV) flight computer employing the RP2350 microcontroller, designated as the Raspberry Pi Pico 2. Intended for embedded programmers, it delineates the architecture, operational principles, functionality, features, and strategies for mitigating errors arising from C++ resource management, peripheral interfaces such as I2C, SPI, UART, and PIO, dual-core processing, and associated technologies. The design leverages readily available Arduino libraries for components including sensor drivers, Attitude and Heading Reference System (AHRS), and Extended Kalman Filter (EKF), while using FreeRTOS for real-time task management. This methodology exploits C++ features for safety and concurrency, while optimizing the RP2350’s hardware attributes, encompassing dual Cortex-M33 cores (configurable as RISC-V), hardware floating-point unit (FPU), digital signal processing (DSP) instructions, 520 KB SRAM, and programmable I/O (PIO) blocks.

The firmware operates within the Arduino environment, utilizing the Arduino core for RP2350 for hardware abstraction. As of July 21, 2025, the Arduino core delivers stable support for RP2350 peripherals, incorporating recent enhancements for GPIO reliability and PIO programming, as documented in the RP2350 datasheet (version dated August 2024). The system derives inspiration from open-source initiatives such as madflight, ArduPilot, and Betaflight, adapting their modular frameworks while exceeding them in safety through C++ compile-time checks and in performance via RP2350-specific optimizations.

Communication responsibilities, including MAVLink encoding and decoding over LoRa, are delegated to a separate microcontroller (Radio MC), allowing the RP2350 flight computer (FC) to concentrate exclusively on core flight operations. Data exchange between the Radio MC and FC occurs via CAN with cyclic redundancy check (CRC-16-CCITT) for integrity verification, ensuring robust, non-blocking transfers without encumbering the FC’s dual cores.

Key objectives encompass attaining exceptional flight smoothness via extensible control algorithms, rigorous telemetry validation, and phased development incorporating optimization cycles. During the telemetry-only phase, testing will integrate an ArduPilot drone, facilitating data monitoring through a ground station while the Radio MC manages MAVLink interactions.

## System Requirements and Assumptions

- **Hardware**: RP2350 microcontroller interfacing with sensors via I2C (BMP388 barometer and PCF8563 RTC on I2C1; BMM350 magnetometer and ICM-20948 IMU on I2C0), SPI (BMI088 IMU, SD card), UART (Ublox NEO-M9N GPS), and PIO for actuators (e.g., servos). A separate Radio MC handles LoRa communications, connected to the FC via CAN for raw data transfer.
- **Software Environment**: Arduino IDE/PlatformIO, FreeRTOS for task scheduling, Arduino core for HAL. Use readily available libraries such as Adafruit for sensors, Madgwick for AHRS, and EKF implementations from open-source repos; custom floating-point mathematics for DSP utilization where needed.
- **UAV Applications**: Drones, rockets, RC planes, emphasizing initial telemetry precision prior to control activation.
- **Communication Protocol**: MAVLink managed exclusively by the Radio MC for ground station interactions. CAN between Radio MC and FC employs custom framed binary packets with CRC-16-CCITT for data integrity, containing pre-parsed commands or telemetry payloads.
- **Performance Goals**: Sub-millisecond control loop latency, <5% CPU utilization for sensor/estimation tasks via DMA/PIO, and resilient operation with fault tolerance.

## Firmware Architecture Overview

The firmware adopts a three-layer structure to foster modularity and maintainability:

- **Hardware Layer**: Interfaces with Arduino core for peripheral governance, administering configurations and shared resources, excluding communication peripherals now delegated to the Radio MC.
- **Driver Layer**: Abstracts sensor interactions through classes, using Arduino libraries.
- **Application Layer**: Coordinates FreeRTOS tasks across dual cores (Core 0 for estimation/control, Core 1 for logging and CAN reception from Radio MC), incorporating the state machine, parameter system, AHRS/EKF, and control logic.

This architecture prioritizes real-time execution, with both cores dedicated to flight-critical functions. The Radio MC processes MAVLink packets, forwarding parsed data (e.g., commands) to the FC via CAN with CRC-protected frames.

### How the Architecture Works

- **Initialization**: Upon boot, the hardware layer configures pins, DMA channels, and PIO state machines via Arduino core. Shared resources (e.g., split I2C buses) are managed with mutexes in FreeRTOS.
- **Data Acquisition**: Sensor drivers use libraries to read data, commencing DMA transfers for SPI devices. Raw data undergoes parsing with parameter-derived calibrations.
- **Estimation and Control**: Core 0 executes AHRS for quaternion fusion and EKF for state estimation. Control tasks utilize these outputs to generate actuator commands via PIO.
- **Communication and Logging**: Core 1 manages SD card logging via DMA and receives CAN data from Radio MC, verifying CRC and queuing parsed payloads (e.g., mode change commands) for Core 0.
- **State Management**: The state machine regulates transitions, activating tasks according to modes (e.g., sensors only in TelemetryOnly).
- **Parameter Interaction**: Flash-stored parameters load to RAM, with updates from Radio MC-forwarded commands influencing operations like EKF covariances.
- **Error Handling**: FreeRTOS priorities avert task disputes; mutexes secure exclusive bus access.

The Radio MC independently handles LoRa interactions, encoding/decoding MAVLink, and forwards validated, CRC-appended raw data to the FC’s CAN, ensuring the FC remains unburdened by protocol overhead.

## Functionality and Features

### Core Functionality

- High-frequency sensor acquisition (IMU at 100 Hz) with DMA offload, using Arduino libraries.
- AHRS for quaternion-based attitude computation (e.g., Madgwick library), EKF for integrated state estimation (position, velocity, orientation) via available libraries.
- Logging to SD card (configurable 1-50 Hz) with RTC timestamps.
- Actuator governance via PIO PWM for servos/motors.
- CAN reception of raw data from Radio MC, with CRC-16-CCITT validation for integrity.

### Advanced Features

- Extensible flight modes (Stabilize, Hover, Loiter, Freestyle) through class-based control, modeled after ArduPilot.
- Parameter system for runtime adjustments (e.g., EKF activation, filter thresholds), with validation and flash persistence.
- Calibration procedures for sensors, persisting biases/scales in parameters.
- Telemetry-only mode for validation, with Radio MC relaying data to ground stations via MAVLink.
- Provisions for advanced algorithms (e.g., IEKF, SMC) selectable via parameters.

### Safety and Reliability

- Watchdog timers in FreeRTOS for hang detection.
- CRC-16-CCITT verification on CAN packets from Radio MC to mitigate corruption.
- Low-power modes in Recovery leveraging RP2350’s sleep capabilities.

## Error Prevention Measures

Embedded systems on RP2350 encounter challenges from C++ resource management, peripheral disputes, and hardware specifics. Mitigation strategies include:

- **C++ Resource Management**: Use RAII for automatic cleanup; FreeRTOS mutexes for mutable access, safeguarded by task priorities. Classes enforce compile-time checks, averting data races. Pin buffers to prevent use-after-free in DMA contexts.
- **I2C Conflicts**: Mutex guarantees exclusive access; incorporate timeouts and recovery (e.g., clock stretching handling) per Arduino Wire library.
- **SPI/DMA Issues**: Share DMA channels via classes; completion callbacks invoke tasks, with status inspections addressing RP2350 FIFO overflows.
- **CAN from Radio MC**: DMA for reception; CRC-16-CCITT validation in receive tasks discards corrupted frames. Baud rate synchronization prevents overruns; queues buffer data to avoid blocking.
- **RP2350 Dual-Core Pitfalls**: FreeRTOS core affinity precludes cross-core races; atomic operations for shared SRAM params.
- **LoRa/MAVLink Delegation**: Radio MC isolation prevents FC exposure to radio noise or packet errors; CAN CRC adds redundancy.
- **General**: Compile-time pin conflict checks; runtime health assessments via params terminate unsafe states.

## Visual Roadmaps for Data Flow

### Overall Data Flow Diagram

```mermaid
graph TD
    subgraph Peripherals
        IMU[IMU SPI]
        GPS[GPS UART]
        Baro[Baro I2C1]
        Mag[Mag I2C0]
        RTC[RTC I2C1]
        SD[SD SPI]
        Actuators[Actuators PIO]
        IMU2[ICM-20948 I2C0]
    end

    subgraph External
        RadioMC[Radio MC<br>MAVLink over LoRa]
    end

    subgraph Hardware Layer
        BusManagers[I2C/SPI/DMA/PIO Managers]
        CanRx[CAN RX from Radio MC<br>with DMA]
    end

    subgraph Driver Layer
        SensorDrivers[Sensor Drivers<br>Class-Based Read/Parse/Calibrate using Libraries]
    end

    subgraph Application Layer - Core 0
        ReadTasks[Sensor Read Tasks<br>DMA-Initiated]
        AHRS[AHRS Fusion via Library]
        EKF[EKF Estimation via Library]
        Control[Control Task<br>PID/SMC/etc. via Classes]
        SM[State Machine]
        Cal[Calibration Tasks]
    end

    subgraph Application Layer - Core 1
        Log[Logging Task<br>DMA Writes]
        CanParse[CAN Parse Task<br>CRC Validation & Queue]
    end

    subgraph Shared
        Params[Parameter System<br>RAM/Flash]
        Queues[Inter-Core Queues]
    end

    Peripherals -->|Raw Data| BusManagers
    BusManagers -->|Managed Access| SensorDrivers
    SensorDrivers -->|Parsed Data| ReadTasks
    ReadTasks -->|Fused Inputs| AHRS
    AHRS -->|Attitude| EKF
    EKF -->|Estimates| Control
    Control -->|Commands| Actuators
    EKF -->|Telemetry Data| Queues
    Queues --> CanParse
    RadioMC -->|Raw Parsed Data<br>with CRC| CanRx
    CanRx -->|DMA Buffer| CanParse
    CanParse -->|Validated Payloads| Queues
    Queues --> Control
    Queues --> SM
    SM -->|Enables| ReadTasks
    SM -->|Enables| AHRS
    SM -->|Enables| EKF
    SM -->|Enables| Control
    SM -->|Enables| Log
    SM -->|Triggers| Cal
    Cal -->|Updates| Params
    CanParse -->|Param Commands| Params
    AllTasks[All Tasks] -->|Read/Write| Params
    EKF -->|Data| Log
    Log -->|Writes| SD
```

### CAN Data Flow Between Radio MC and FC

```mermaid
sequenceDiagram
    participant RadioMC as Radio MC
    participant FCCan as FC CAN (DMA RX)
    participant FCParse as FC Parse Task (Core 1)

    RadioMC->>FCCan: Send Framed Packet (Header + Payload + CRC)
    FCCan->>FCCan: DMA Transfer to Buffer
    FCCan->>FCParse: Completion Interrupt
    FCParse->>FCParse: Verify CRC
    alt Valid CRC
        FCParse->>FCParse: Extract Payload (e.g., Command Struct)
        FCParse->>Queues: Queue for Core 0 (e.g., Mode Change)
    else Invalid
        FCParse->>Log: Log Error; Discard
    end
    Note over RadioMC,FCParse: Payloads are pre-parsed; no MAVLink in FC
```

## Phased Development and MVP Checkpoint Roadmap

Development proceeds in phases with embedded optimization cycles, yielding testable products at each MVP. Optimizations emphasize profiling (e.g., cycle counts), error minimization, and smoothness (e.g., estimate variance). ArduPilot drone testing validates telemetry via Radio MC-ground station links.

### Phase 1: Basic Functionality and Optimization

**Objective**: Configure hardware, sensors, and basic data handling, excluding communication.

- **MVP 1.1: Hardware and Driver Setup**
  - Establish hardware layer (configs, managers) and driver layer (classes for sensors using libraries).
  - Functionality: Acquire and parse raw sensor data.
  - Testable Product: Serial output of parsed values.
  - Testing: Bench verification on ArduPilot drone; assess read accuracy.
  - Improvements: Refine class interfaces for resource issues.
- **Optimization 1.1**: Benchmark DMA latencies; optimize bus access to <5% CPU usage.

- **MVP 1.2: Basic Application Tasks**
  - Introduce FreeRTOS tasks for reads; initial state machine (Idle/Armed/TelemetryOnly).
  - Functionality: Mode-based data collection.
  - Testable Product: Sketch with transitions.
  - Testing: Simulate modes; log data consistency.
  - Improvements: Prioritize tasks to eliminate jitter.
- **Optimization 1.2**: Dual-core balancing; reduce inter-core queue delays.

### Phase 2: CAN Reception from Radio MC and Optimization

**Objective**: Integrate CAN for raw data from Radio MC, with CRC-16-CCITT.

- **MVP 2.1: CAN Setup**
  - Configure CAN DMA RX on Core 1; implement CRC validation.
  - Functionality: Receive and parse raw payloads.
  - Testable Product: Queue validated data for telemetry.
  - Testing: Simulate Radio MC; verify CRC rejection rates (<1%).
  - Improvements: Adjust frame formats for minimal overhead.
- **Optimization 2.1**: Profile CAN throughput; enhance DMA buffer management.

- **MVP 2.2: Parameter System Integration**
  - Enable param updates from CAN payloads.
  - Functionality: Remote tweaks via simulated Radio MC.
  - Testable Product: Configurable system.
  - Testing: Change params; confirm effects.
  - Improvements: Strengthen validation to prevent invalid updates.
- **Optimization 2.2**: Optimize flash sync; test CRC under noise.

### Phase 3: Estimation and Basic Control with Optimization

**Objective**: Deploy AHRS/EKF and initial control.

- **MVP 3.1: AHRS and EKF Implementation**
  - AHRS/EKF using libraries (e.g., Madgwick for AHRS, open-source EKF).
  - Functionality: Estimates in TelemetryOnly.
  - Testable Product: Logged fused data.
  - Testing: ArduPilot drone bench; compare to references.
  - Improvements: Covariance tuning for <1° drift.
- **Optimization 3.1**: FPU/DSP utilization; parallelize EKF.

- **MVP 3.2: Basic Control and Modes**
  - Class-based PID; initial modes.
  - Functionality: Actuator outputs.
  - Testable Product: Stable test rig operation.
  - Testing: Measure overshoot (<5%).
  - Improvements: Calibration linkage.
- **Optimization 3.2**: Loop profiling; SMC integration testing.

### Phase 4: Advanced Features and Final Optimization

**Objective**: Complete calibrations, modes, and extensibility.

- **MVP 4.1: Calibrations and Modes**
  - Calibration tasks; full modes.
  - Functionality: Algorithm selection via params.
  - Testable Product: Mode-switchable system.
  - Testing: Flights; smoothness evaluation.
  - Improvements: IEKF testing.
- **Optimization 4.1**: Core load optimization; power reduction.

- **MVP 4.2: End-to-End Validation**
  - Full integration; fault tests.
  - Functionality: Optimized firmware.
  - Testable Product: Mission-ready.
  - Testing: Vs. ArduPilot; RMS position error <0.5 m.
  - Improvements: Iterative refinements from logs.