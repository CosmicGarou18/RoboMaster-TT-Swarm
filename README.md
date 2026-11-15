# RoboMaster TT Autonomous Swarm

An autonomous multi-drone project built around the **DJI RoboMaster TT (Tello Talent)**, focusing on stable flight control, velocity calibration, dead reckoning, and coordinated multi-drone navigation.

The project aims to develop a three-drone autonomous swarm consisting of one master and two slave drones arranged in a T-formation. The system is designed to eventually support shared-coordinate positioning, formation maintenance, and real-time collision avoidance.

## Project Overview

The project is being developed in stages, beginning with reliable single-drone flight before progressing towards multi-drone coordination.

The current implementation focuses on establishing accurate flight control through PID tuning, measuring the relationship between commanded RC speeds and actual drone velocities, and developing the foundations for position estimation in a shared coordinate system.

## Hardware

- **Drone:** DJI RoboMaster TT (Tello Talent), model TLW004.
- **Extension Module:** RMTTOC ESP32-D2WD dual-core module.
- **Distance Sensor:** Front-facing VL53L0X-style Time-of-Flight (ToF) sensor connected through I2C.
- **Positioning:** Onboard Visual Positioning System (VPS), using a bottom-facing camera and infrared sensing.
- **Communication:** ESP-NOW wireless communication between drones and the ground station.
- **Ground Station:** A second RMTTOC module connected to a PC through USB.

## Software & Development Environment

- Arduino IDE 2
- ESP32 Arduino Core 2.0.17
- C++ / Arduino
- DJI RMTT Libraries
- FreeRTOS
- ESP-NOW
- Python (PC-side PID tuning and flight analysis)

The ESP32 firmware uses a dual-core FreeRTOS architecture to separate communication and flight-control operations.

- **Core 0:** Handles Serial1 communication with the drone, incoming data, command transmission, and heartbeat monitoring.
- **Core 1:** Handles flight-control logic, sensor readings, ESP-NOW communication, and control calculations.

A command queue and mutex-protected telemetry structure allow both cores to communicate without interfering with flight operations.

## Key Implementations

### 1. PID Flight Control

PID controllers are used to stabilise drone movement and maintain desired flight behaviour.

Two primary control systems have been developed:

- **Altitude Hold:** Maintains a target vertical position.
- **Lateral Wall Following:** Controls horizontal movement to maintain a desired distance from a wall.

The lateral PID gains were evaluated through repeated hardware-based gain sweeps, comparing different combinations of proportional, integral, and derivative gains.

The selected working configurations are:

| Controller | Kp | Ki | Kd |
|---|---:|---:|---:|
| Lateral | 0.25 | 0.0 | 0.8 |
| Altitude | 6.3 | 0.0 | 1.9 |

The lateral controller's derivative term also incorporates filtering and clamping to reduce the effects of sensor noise.

### 2. Velocity Calibration

A major challenge is that RC commands specify drone tilt and movement rates rather than directly specifying velocity.

Consequently, the relationship between commanded RC speed and actual velocity must be measured experimentally.

A dedicated `velocity_finder.ino` script was developed to measure real-world velocity by flying the drone towards a wall from a known starting distance.

The script:
- Executes flights at specified RC speed values.
- Measures elapsed flight time until the front ToF sensor detects the wall.
- Calculates velocity using the known physical distance travelled.
- Uses yaw-based heading correction to maintain a straight flight path.
- Uses altitude PID control to maintain stable vertical positioning.

The resulting calibration data will be used to construct a velocity lookup table for subsequent position estimation.

### 3. Position Estimation & Dead Reckoning

The project uses calibrated velocity measurements as the foundation for estimating drone position.

The proposed position model integrates velocity over time:

\[
x_t=x_{t-1}+v_t\cos(\theta_t)\Delta t
\]

\[
y_t=y_{t-1}+v_t\sin(\theta_t)\Delta t
\]

Where:
- \(x,y\) represent horizontal position.
- \(v\) represents calibrated velocity.
- \(\theta\) represents drone yaw.
- \(\Delta t\) represents the time interval.

Optical-flow velocity measurements were investigated but found unreliable under the project's operating conditions. Mission pads were also evaluated as a potential position-correction mechanism but proved unsuitable for continuous correction at operational flight speeds.

Front-facing ToF measurements remain the preferred source of direct distance information whenever a wall is within sensor range.

### 4. ESP-NOW Communication & Swarm Architecture

ESP-NOW provides the communication framework for the planned multi-drone system.

Each drone is intended to broadcast its position and orientation to a ground station, allowing the system to construct a shared representation of the swarm.

The proposed architecture includes:
- A master drone establishing the shared coordinate origin.
- Slave drones initialising their positions relative to predefined formation offsets.
- Broadcast communication of drone position and yaw.
- Pairwise distance calculations to detect potentially unsafe proximity.
- Command routing for individual drone control.

The initial formation consists of three drones arranged in a T-formation.

This shared-coordinate system is currently at the design stage and has not yet been fully implemented.

### 5. Sensor Integration & Safety

The front-facing ToF sensor provides distance measurements for wall-following and safety monitoring.

Several sensor-validation mechanisms have been incorporated to prevent invalid readings from triggering incorrect flight behaviour.

These include:
- Explicit validation of sensor out-of-range values.
- Strict response-format validation before parsing measurements.
- Filtering of abnormal distance readings.
- Consecutive-reading checks before triggering emergency actions.
- Distance-based watchdogs and controlled emergency landing conditions.

Sensor sentinel values are verified against the specific library and access method rather than assumed to be consistent across different implementations.

## Current Project Status

**Implemented and tested:**
- ESP32-to-drone Serial1 communication and control handshake.
- Dual-core FreeRTOS architecture.
- ESP-NOW broadcast communication.
- Altitude hold PID controller.
- Lateral wall-following PID controller.
- Hardware-based lateral PID gain sweeps.
- Front-facing ToF sensor integration.
- Velocity-finder script with sensor validation and emergency-stop debounce.

**Currently in progress:**
- Experimental RC-speed-to-velocity calibration.
- Construction of the velocity calibration table.
- Dead-reckoning and coordinate integration.
- Shared-coordinate positioning for multiple drones.
- Autonomous formation control and collision avoidance.

## Future Development

1. Complete velocity calibration across different RC command values.
2. Implement dead-reckoning using calibrated velocity and yaw measurements.
3. Establish a common coordinate frame for all drones.
4. Implement master–slave formation control.
5. Develop inter-drone distance monitoring and collision avoidance.
6. Investigate wider-field-of-view depth sensing for improved environmental awareness.

---

**Project Focus:** Autonomous Robotics · Flight Control · PID Optimisation · Embedded Systems · Sensor Fusion · Wireless Communication · Multi-Agent Coordination
