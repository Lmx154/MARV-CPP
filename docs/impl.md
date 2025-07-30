# Implementation Document for RP2350-Based UAV Flight Computer Firmware

## Document Overview

This implementation document serves as a detailed guideline for developing the firmware of an unmanned aerial vehicle (UAV) flight computer based on the RP2350 microcontroller. It translates the high-level design into actionable steps, providing pseudo code structures, feature checklists, and implementation strategies for each MVP checkpoint in the phased development roadmap. The focus is on creating modular, efficient code in C++ using the Arduino environment, leveraging readily available libraries for sensors, AHRS, EKF, and communication protocols, while using FreeRTOS for real-time task management.

Pseudo code is used throughout to illustrate key structures and algorithms, emphasizing computational efficiency (e.g., preferring O(1) operations for real-time tasks, minimizing floating-point operations where possible, and leveraging hardware features like DMA and FPU for offloading). Modularity is maintained through class-based abstractions, allowing for extensible components without runtime overhead. Developers should follow this as a checklist, implementing one MVP at a time, with testing and optimization integrated at each step.

Key principles:
- **Efficiency**: Use floating-point arithmetic with FPU support; ensure loops are bounded and avoid dynamic allocations.
- **Modularity**: Define classes and interfaces for drivers and algorithms to enable swapping (e.g., PID vs. SMC) with compile-time resolution.
- **Safety**: Leverage FreeRTOS mutexes and queues for shared access; use atomic operations for inter-core communication.
- **Testing**: Each MVP includes a testable product and verification steps, focusing on bench tests with an ArduPilot drone for telemetry validation.

## System Requirements and Assumptions

Implementation assumes the hardware and software environment as specified in the high-level design. Developers must:
- Verify Arduino core compatibility with the RP2350 datasheet (August 8, 2024 version) for peripherals like I2C, SPI, UART, PIO, and DMA.
- Use libraries like Adafruit for sensors, Madgwick for AHRS, and open-source EKF libs; custom implementations only where no library exists.
- Ensure all code runs in Arduino context, with error handling routed to a watchdog reset.

## Firmware Architecture Implementation

Implement the three-layer architecture as follows:

1. **Hardware Layer**: Use Arduino core to configure peripherals. Create manager classes for shared resources (e.g., I2C bus) with FreeRTOS mutexes.
   - Pseudo code for a bus manager (e.g., for I2C):
     ```
     class I2CManager {
     private:
         SemaphoreHandle_t mutex;
         // Wire instance from Arduino

     public:
         I2CManager() { mutex = xSemaphoreCreateMutex(); }
         bool acquire() { return xSemaphoreTake(mutex, pdMS_TO_TICKS(10)); }  // Timeout
         void release() { xSemaphoreGive(mutex); }
     };
     ```
     - Efficiency: O(1) acquire/release; use in FreeRTOS tasks to ensure safe access.

2. **Driver Layer**: Define classes for sensor interactions using libraries. Each driver implements read/parse/calibrate methods.
   - Pseudo code for a sensor interface:
     ```
     class SensorDriver {
     public:
         virtual RawData read_raw() = 0;  // Use library calls, O(n) small
         virtual ParsedData parse(RawData raw) = 0;  // Floating-point ops, O(1)
         virtual bool calibrate(Params* params) = 0;  // Update biases, O(1)
     };
     ```
     - Modularity: Allows swapping sensors without changing application logic.

3. **Application Layer**: Use FreeRTOS for tasks on dual cores. Core 0 for high-priority estimation/control; Core 1 for logging/CAN.
   - Shared resources: Use queues (FreeRTOS QueueHandle_t) for inter-core data.
   - Pseudo code for a queue:
     ```
     QueueHandle_t dataQueue = xQueueCreate(10, sizeof(DataStruct));  // Fixed size

     bool push(DataStruct item) {
         return xQueueSend(dataQueue, &item, pdMS_TO_TICKS(1));  // O(1)
     }

     DataStruct pop() {
         DataStruct item;
         if (xQueueReceive(dataQueue, &item, pdMS_TO_TICKS(10))) return item;
         return {};  // O(1)
     }
     ```
     - Efficiency: Bounded size; no locks needed for queues.

## Implementation of Functionality and Features

- **Core Functionality**:
  - Sensor Acquisition: Use DMA for SPI/IMU reads via libraries; callback triggers FreeRTOS task for parsing.
  - AHRS/EKF: Use libraries (e.g., MadgwickAHRS.h, KalmanFilter libs) with quaternion math (O(n) small matrices).
  - Logging: DMA writes to SD via SD library; format as binary with timestamps (O(1) per entry).
  - Actuators: PIO for PWM via Arduino core; configure for parallel output.
  - CAN Reception: DMA RX with CRC-16-CCITT check via ArduinoMCP2515 (O(n) over packet, n<100 bytes).

- **Advanced Features**:
  - Flight Modes: Enum-based state machine; classes for controllers (e.g., PID: O(1) per update).
  - Parameter System: Struct with fixed fields; load from flash, validate on update (O(1) lookups).
  - Calibration: Accumulate samples (bounded buffer), compute averages/biases (O(n) with n=100-1000).
  - Telemetry-Only: Disable control tasks; queue data for CAN TX to Radio.

- **Safety and Reliability**:
  - Watchdog: FreeRTOS tick hook resets.
  - CRC: Polynomial computation, precompute table for O(n) efficiency.

## Implementation of Error Prevention Measures

- **Resource Management**: Use RAII for cleanup; FreeRTOS mutexes for mutables.
- **I2C Conflicts**: Timeout in acquire; recovery via Wire reset.
- **SPI/DMA**: Check status in callbacks; retry on overflow.
- **CAN**: Validate CRC-16-CCITT before queuing; baud rate: 115200.
- **Dual-Core**: Affinity in FreeRTOS; atomics for shared params.
- **General**: Static pin assignments; health checks in idle task.

## Phased Development and MVP Checkpoint Implementation

Follow the phases sequentially. Each MVP includes:
- **Checklist**: Features to implement.
- **Pseudo Code Snippets**: Key structures/algorithms.
- **Testing and Optimization**: Steps and metrics.

### Phase 1: Basic Functionality and Optimization

**Objective**: Set up hardware/drivers and basic tasks.

- **MVP 1.1: Hardware and Driver Setup**
  - **Checklist**:
    - Configure peripherals (I2C, SPI, DMA) in setup().
    - Implement bus managers for I2C/SPI.
    - Create class-based drivers for BMP388 (Adafruit_BMP3XX), BMM350, PCF8563, BMI088, ICM-20948 (Adafruit_ICM20948), Ublox NEO-M9N (TinyGPS++).
    - Parse raw data with floating-point scaling.
    - Output parsed data via serial (for testing).
  - **Pseudo Code Snippet** (Driver Example for BMI088 IMU):
    ```
    class BMI088Driver : public SensorDriver {
    private:
        Adafruit_BMI088 bmi;  // Library instance
        Vector3 accel_bias;

    public:
        RawData read_raw() {
            bmi.readSensors();  // Library call, O(1)
            return {bmi.accel_x, bmi.accel_y, bmi.accel_z, ...};
        }

        ParsedData parse(RawData raw) {
            float accel_x = (raw.accel_x * scale) - accel_bias.x;  // O(1)
            // Similarly for others
            return {accel: {accel_x, ...}, gyro: {...}};
        }

        bool calibrate(Params* params) {
            accel_bias = params->get("accel_bias");  // O(1)
            return true;
        }
    };
    ```
  - **Testable Product**: Sketch that reads and serial-prints sensor data at 10 Hz.
  - **Testing**: Connect to ArduPilot drone; verify accuracy (±1% error) via known references.
  - **Improvements**: Adjust classes for resource errors; ensure no data races.

- **Optimization 1.1**: Profile DMA setup/teardown (target <100 µs); measure CPU usage during reads (target <5% at 100 Hz).

- **MVP 1.2: Basic Application Tasks**
  - **Checklist**:
    - Set up FreeRTOS tasks for sensor reads (high priority).
    - Implement state machine enum (Idle, Armed, TelemetryOnly).
    - Enable/disable tasks based on mode.
    - Collect data in a struct for logging/serial.
  - **Pseudo Code Snippet** (FreeRTOS Task for Sensor Read):
    ```
    void read_baro_task(void* pvParameters) {
        I2CManager i2c;
        BMI088Driver baro_driver;
        while (true) {
            if (i2c.acquire()) {
                RawData raw = baro_driver.read_raw();
                i2c.release();
                ParsedData parsed = baro_driver.parse(raw);
                // Store in shared struct or queue
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    ```
  - **Testable Product**: Sketch with mode transitions via dummy input; consistent data collection.
  - **Testing**: Simulate modes; check log for no jitter (>99% on-time).
  - **Improvements**: Assign priorities to minimize preemption delays.

- **Optimization 1.2**: Balance tasks across cores (reads on Core 0); profile queue latencies (<10 µs).

### Phase 2: CAN Reception from Radio MC and Optimization

**Objective**: Add CAN RX with CRC-16-CCITT; integrate parameters.

- **MVP 2.1: CAN Setup**
  - **Checklist**:
    - Configure CAN DMA RX on Core 1 using ArduinoMCP2515.
    - Implement frame format: Header (2 bytes len/type) + Payload + CRC (2 bytes).
    - Validate CRC-16-CCITT in task.
    - Queue valid payloads (e.g., telemetry requests).
  - **Pseudo Code Snippet** (CAN Parse Task):
    ```
    void parse_can_task(void* pvParameters) {
        uint8_t buffer[LEN];
        while (true) {
            // Read from CAN lib
            uint16_t crc_calc = crc16_ccitt(buffer, len);  // O(n)
            if (crc_calc != buffer[len]) { log_error(); continue; }
            Payload payload = extract_payload(buffer);
            push_to_queue(payload);  // O(1)
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    ```
  - **Testable Product**: Receive and queue data from simulated Radio MC.
  - **Testing**: Send corrupted frames; verify rejection (<1% false positive).
  - **Improvements**: Optimize frame size (minimize headers).

- **Optimization 2.1**: Profile throughput (target 1 kB/s); buffer sizing to avoid overruns.

- **MVP 2.2: Parameter System Integration**
  - **Checklist**:
    - Define Params struct with fixed fields (e.g., array of key-value).
    - Load from flash on boot; update from CAN payloads.
    - Validate updates (e.g., range checks).
  - **Pseudo Code Snippet** (Params Update):
    ```
    struct Params {
        float values[NUM_PARAMS];
        const char* keys[NUM_PARAMS];
    };

    bool update(const char* key, float value) {
        for (int i = 0; i < NUM_PARAMS; i++) {  // O(k), k small
            if (strcmp(keys[i], key) == 0) {
                if (value >= MIN && value <= MAX) { values[i] = value; return true; }
            }
        }
        return false;
    }
    ```
  - **Testable Product**: Remote param changes affect drivers.
  - **Testing**: Update via sim; confirm no invalid states.
  - **Improvements**: Use hash for O(1) if needed, but array sufficient.

- **Optimization 2.2**: Minimize flash writes (batch); test CRC with noise injection.

### Phase 3: Estimation and Basic Control with Optimization

**Objective**: Add AHRS/EKF and control.

- **MVP 3.1: AHRS and EKF Implementation**
  - **Checklist**:
    - AHRS: Use Madgwick library (quaternions, O(1) per update).
    - EKF: Use open-source library (15-state, matrix ops).
    - Integrate CAN commands to toggle.
    - Log estimates.
  - **Pseudo Code Snippet** (AHRS Update):
    ```
    class AHRS {
    private:
        Madgwick filter;  // Library

    public:
        void update(Vector3 gyro, Vector3 accel, Vector3 mag, float dt) {
            filter.updateIMU(gyro.x, gyro.y, gyro.z, accel.x, ... , dt);  // O(1)
            quat = filter.getQuaternion();
        }
    };
    ```
  - **Testable Product**: Fused data in logs.
  - **Testing**: Bench vs. ArduPilot; drift <1°/min.
  - **Improvements**: Tune gains via params.

- **Optimization 3.1**: Use FPU for matrices; parallelize prediction/correction if possible.

- **MVP 3.2: Basic Control and Modes**
  - **Checklist**:
    - Class for controllers (e.g., PID).
    - Link to state machine for mode enables.
    - Output to PIO actuators.
  - **Pseudo Code Snippet** (PID Controller Class):
    ```
    class PID {
    private:
        float kp, ki, kd, integral;

    public:
        float update(float setpoint, float measured, float dt) {
            float err = setpoint - measured;
            integral += err * dt;  // Anti-windup if needed
            float deriv = (err - prev_err) / dt;
            return kp * err + ki * integral + kd * deriv;  // O(1)
        }
    };
    ```
  - **Testable Product**: Actuator responses in test rig.
  - **Testing**: Overshoot <5%; stability checks.
  - **Improvements**: Link calibrations to gains.

- **Optimization 3.2**: Profile control loop (<1 ms); test alternative controllers.

### Phase 4: Advanced Features and Final Optimization

**Objective**: Complete and validate.

- **MVP 4.1: Calibrations and Modes**
  - **Checklist**:
    - Implement calibration tasks (accumulate, compute).
    - Full modes (Stabilize, etc.) via classes.
    - Select algorithms via params.
  - **Pseudo Code Snippet** (Calibration):
    ```
    Vector3 calibrate_accel(Vector3 samples[100]) {
        Vector3 sum = {0,0,0};
        for (int i = 0; i < 100; i++) { sum += samples[i]; }  // O(n)
        return sum / 100.0f;
    }
    ```
  - **Testable Product**: Mode switches; calibrated ops.
  - **Testing**: Flight tests; evaluate smoothness.
  - **Improvements**: Test IEKF variant.

- **Optimization 4.1**: Balance core loads (<50% each); enable sleep modes.

- **MVP 4.2: End-to-End Validation**
  - **Checklist**:
    - Integrate all; add fault injection.
    - Full telemetry via Radio MC.
  - **Pseudo Code Snippet** (Fault Handling):
    ```
    void health_check(Params* params) {
        if (sensor_error > params->threshold) { state_machine.transition(Recovery); }  // O(1)
    }
    ```
  - **Testable Product**: Mission-ready firmware.
  - **Testing**: Compare to ArduPilot; RMS error <0.5 m.
  - **Improvements**: Refine from logs; iterate optimizations.

