# FRDM-K22F Quadcopter Flight Controller

An out-of-tree Zephyr RTOS flight controller firmware for quadcopters built on the NXP FRDM-K22F development board.

## Features
- **400Hz Flight Loop**: Real-time flight control loop run in a high-priority cooperative thread.
- **IMU Sensor Fusion**: Attitude estimation (Roll/Pitch) using a **Mahony complementary filter** powered by the **FRDM-STBC-AGM01** shield (FXOS8700 Accelerometer + FXAS21002 Gyroscope).
- **CMSIS-DSP Hardware Acceleration**: Leverages the Cortex-M4F single-precision FPU using ARM CMSIS-DSP library for high-speed PID updates and fast math (`arm_sqrt_f32`).
- **Barometer Subsystem**: Altitude estimation and PID tracking utilizing the Infineon DPS310 sensor over a shared I2C bus.
- **Auto Landing Assist**: Configurable manual or automatic failsafe landing sequence that gently grounds the drone and disarms the motors.
- **Dual RC Input Interfaces**:
  - Continuous interrupt-driven PWM capture (5-6 channels).
  - Single-wire iBUS serial protocol parsing (up to 14 channels).

## Hardware & Pin Mapping
For full wiring instructions, pin designations, and schematics, refer to the [Wiring Diagram](wiring_diagram.md).

All core hardware devices (sensors, receivers) are resolved using Zephyr `chosen` node properties defined in [frdm_k22f.overlay](file:///home/rholl/quadcopter/quad-app/boards/frdm_k22f.overlay). If you swap components or hardware configurations, simply update the mappings under the `chosen` block (e.g. `flt_ctl,gyro`, `flt_ctl,accel`, `flt_ctl,baro`) without altering the source logic.

## Getting Started

### Workspace Setup
This project uses `west` for dependency management. To set up the workspace:

1. Initialize a new West workspace:
   ```bash
   west init -m <URL-to-this-repository>
   west update
   ```
   *Note: Our custom `west.yml` uses an allowlist to pull only the exact modules needed (CMSIS-DSP, MCUX-SDK-NG, hal_nxp), saving gigabytes of disk space.*

2. Set up your python environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r zephyr/scripts/requirements.txt
   ```

### Building the Firmware
Build the application targeting the FRDM-K22F board with the AGM01 sensor shield:
```bash
west build -b frdm_k22f --shield frdm_stbc_agm01 quad-app -p always
```


## Logging & Telemetry
System logs are outputs over UART (115200 baud) in deferred mode to ensure printing telemetry does not block the 400Hz control thread.

Example Output:
```text
[INF] Starting IMU Gyro Calibration. Please keep the drone perfectly still...
[INF] Calibration Complete. Biases - Roll: 0.02, Pitch: -0.01, Yaw: 0.05
[INF] Quadcopter Flight Control Workspace Online. Starting loop...
[INF] ST:FLYING ACRO | Alt:1.45m (Tgt:1.50m) | EstAng:(0.5, -0.2) | Thr:1480
```

## Shell Commands
The flight controller includes an interactive shell subsystem enabled by default (`CONFIG_SHELL=y`). You can use the following commands over the UART interface:

* **Mute/Unmute Telemetry Output**:
  To prevent the 40Hz telemetry logs from flooding the console while typing commands, mute them via:
  ```bash
  log disable flight_blackbox
  ```
  To restore logging output, run:
  ```bash
  log enable flight_blackbox
  ```

* **Trigger Magnetometer Calibration**:
  To manually trigger a 15-second magnetometer hard-iron calibration at runtime:
  ```bash
  mag_calib
  ```
  During calibration, rotate the device slowly in all directions to capture minimum and maximum values along all three axes.

