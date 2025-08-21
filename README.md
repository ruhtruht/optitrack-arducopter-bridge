# optitrack-arducopter-bridge
Make a Arducopter Drone Fly in Optitrack Setup via NatNetSDK and pyMavlink
# MoCapMAV – OptiTrack-Based Indoor Drone Navigation Using MAVLink

This repository contains a Python-based system for integrating OptiTrack motion capture data with MAVLink-enabled drones. It enables GPS-independent indoor navigation by converting motion capture position estimates (via the NatNet SDK) into real-time MAVLink messages, which are sent to a Pixhawk-based flight controller running ArduCopter firmware.

The system was developed and tested in the context of an academic research project focused on autonomous aerial tracking of ground robots in indoor environments.

## Overview

The project demonstrates a working pipeline that:

- Receives real-time 6DOF position and orientation data from the OptiTrack system via the NatNet SDK
- Converts position data from ENU to NED coordinates using quaternion-based transformations
- Sends MAVLink VISION_POSITION_ESTIMATE messages at 20 Hz
- Commands basic autonomous flight operations (arming, takeoff, land, waypoint tracking) in GUIDED mode
- Integrates safety mechanisms including connection monitoring, AGV signal timeouts, and emergency land triggers

## Architecture

OptiTrack Cameras → Motive → NatNet SDK (UDP) → Python Receiver  
                         ↓  
               Coordinate Transformation (ENU → NED)  
                         ↓  
                 MAVLink Command Generation  
                         ↓  
              ArduPilot Flight Controller (Pixhawk)

## Components

The implementation consists of modular Python scripts:

- qmed_follows_agv.py — Main controller for autonomous drone tracking
- mavlink_controller.py — Encapsulation of pymavlink communication
- natnet_tracker.py — UDP-based NatNet data receiver
- coordinate_transform.py — Quaternion-based rotation and coordinate conversion
- safety_config.py — Static safety parameter definitions
- safety_checks.py — Real-time safety and timeout logic
- vision_position_only.py — Position-only testing mode (no autonomous control)
- hovertest.py — Minimal takeoff and hover script for stability testing

Associated system logs and additional validation scripts are included in the /Anhang directory.

## System Requirements

- Python ≥ 3.8
- pymavlink
- numpy
- Access to an OptiTrack motion capture system with NatNet broadcasting enabled
- A MAVLink-capable drone (tested with Pixhawk on ArduCopter 4.x)
- Network access to both the drone and OptiTrack streaming computer

## Setup Instructions

1. Clone this repository and install dependencies:

   pip install pymavlink numpy

2. Start OptiTrack Motive and ensure NatNet streaming is enabled.

3. Configure your drone to accept external vision position estimates (EKF2/3 settings).

4. Launch the tracking and flight control script:

   python qmed_follows_agv.py

## Validation and Testing

Due to safety considerations, full autonomous flights in indoor environments were not conducted with the QMED platform. Instead, validation was performed through hover tests, position-only transmissions, and detailed analysis of system log files.

Example output from system logs demonstrates:

- Successful lock of position estimates
- Execution of pre-flight safety checks
- Takeoff command dispatch
- Real-time position updates with target-relative coordinates

Full logs are available in /Anhang/.

## Related Work

- crazyflie_bk: https://github.com/briankim13/crazyflie_bk  
  An indoor navigation project using OptiTrack and ROS on a Crazyflie Nano drone (9x9 cm, 19 g)

- RaspberryPi4_and_Pixhawk: https://github.com/nickolaus65/RasberryPi4_and_Pixhawk  
  Demonstrates MAVLink communication via serial interface between a Pixhawk and a Raspberry Pi

Both projects utilize lightweight mini quadrotor platforms optimized for indoor use. This stands in contrast to the QMED drone used in the current project, which due to its high thrust requirement and open rotor design is less suited for confined laboratory spaces.

## Academic Context

This implementation was developed as part of a bachelor's thesis investigating GPS-free drone navigation and the integration of external motion capture systems with ArduPilot flight controllers. The focus was on software modularity, transform accuracy, and reliable MAVLink communication under constrained indoor conditions.

## License

This repository is released under the MIT License. Usage for academic, research, and prototyping purposes is encouraged.

## Citation

To cite this project in academic work, please use the following format:

[Author Name]. (2025). MoCapMAV: OptiTrack-Based Indoor Drone Navigation Using MAVLink [Computer software]. https://github.com/<your-username>/MoCapMAV
