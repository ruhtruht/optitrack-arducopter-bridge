

# optitrack-arducopter-bridge

This repository contains a Python-based system for integrating OptiTrack motion capture data with MAVLink-enabled drones. It enables GPS-independent indoor navigation by converting motion capture position estimates (via the NatNet SDK) into real-time MAVLink messages, which are sent to a Pixhawk-based flight controller running ArduCopter firmware.

The system was developed and tested in the context of a bachelor's thesis on autonomous aerial tracking of ground robots in constrained indoor environments.

## Overview

The software stack implements a complete pipeline for indoor drone localization and navigation:

- Real-time 6DOF motion capture via the OptiTrack NatNet SDK
- Quaternion-based ENU to NED coordinate transformation
- High-frequency (20 Hz) MAVLink message generation
- Autopilot control using ArduPilot in GUIDED mode
- System-level safety features: timeout detection, emergency land procedures, position validation

## Architecture

```

OptiTrack Cameras → Motive → NatNet SDK (UDP) → Python Receiver
↓
Coordinate Transformation (ENU → NED)
↓
MAVLink Command Generation
↓
ArduPilot Flight Controller (Pixhawk)

```

## Repository Structure

```

.
├── qmed\_follows\_agv.py           # Main system controller
├── mavlink\_controller.py         # MAVLink communication interface
├── natnet\_tracker.py             # NatNet UDP receiver
├── coordinate\_transform.py       # ENU → NED transformation logic
├── safety\_config.py              # Configurable safety parameters
├── safety\_checks.py              # Online safety checks and failsafes
├── hovertest.py                  # Simple takeoff and hover stability test
├── vision\_position\_only.py       # Position-only validation script
├── cache/                        # System logs and recorded telemetry
├── sdk/                          # Example scripts from the OptiTrack SDK
└── README.md

````

## System Requirements

- Python ≥ 3.8  
- `pymavlink`, `numpy`  
- An OptiTrack system with Motive and NatNet streaming enabled  
- A MAVLink-capable drone (e.g. Pixhawk on ArduCopter 4.x)  
- Local network communication between PC and drone via telemetry or Wi-Fi

## Flight Controller Configuration

To enable the ArduPilot EKF to accept external vision data, several parameters must be changed from their defaults. The following table summarizes the necessary adjustments:

| Parameter         | New Value | Default |
|-------------------|-----------|---------|
| `AHRS_GPS_USE`     | 0.0       | 1.0     |
| `EK3_GPS_CHECK`    | 0.0       | 31.0    |
| `EK3_SRC1_POSXY`   | 6.0       | 3.0     |
| `EK3_SRC1_POSZ`    | 6.0       | 3.0     |
| `EK3_SRC1_VELXY`   | 6.0       | 3.0     |
| `EK3_SRC1_VELZ`    | 6.0       | 3.0     |
| `EK3_SRC1_YAW`     | 6.0       | 1.0     |
| `GPS_TYPE`         | 0.0       | 14.0    |
| `VISO_TYPE`        | 1.0       | 0.0     |

These values configure the EKF to use MAVLink VISION_POSITION_ESTIMATE data for state estimation instead of GPS.

## Setup Instructions

1. Clone the repository:

   ```bash
   git clone https://github.com/ruhtruht/optitrack-arducopter-bridge.git
   cd optitrack-arducopter-bridge
````

2. Install dependencies:

   ```bash
   pip install pymavlink numpy
   ```

3. Start Motive and enable NatNet 3D data streaming.

4. Ensure your flight controller is configured with the parameters listed above.

5. Run the main script:

   ```bash
   python qmed_follows_agv.py
   ```

## Validation and Testing

Due to safety concerns, full indoor flight testing was not performed with the 3 kg QMED drone platform. Instead, validation was conducted through simulated position-only transmissions, controlled hover tests, and in-depth analysis of system logs (see `/cache/`).

All functional components—including MAVLink messaging, transformation accuracy, mode switching, and safety logic—were verified in real-time using log-based evaluation and telemetry monitoring.

## Related Work

* [crazyflie\_bk](https://github.com/briankim13/crazyflie_bk):
  An indoor navigation project using OptiTrack and ROS on a Crazyflie Nano drone (9 × 9 cm, 19 g).

* [RaspberryPi4\_and\_Pixhawk](https://github.com/nickolaus65/RasberryPi4_and_Pixhawk):
  Demonstrates serial MAVLink communication between a Raspberry Pi and Pixhawk controller.

Both projects use microdrones optimized for indoor testing. In contrast, this project highlights the challenges of performing vision-based indoor flight with larger multirotor platforms (QMED, \~3 kg), emphasizing the need for lighter and safer UAV designs in confined laboratory spaces.

## Academic Context

This implementation was developed as part of a bachelor’s thesis investigating GPS-free drone navigation using motion capture. The focus was on clean modular design, reliable MAVLink transmission, and precise spatial transformations in environments where traditional GNSS positioning is unavailable.

## License

MIT License. Open for research, prototyping, and academic teaching.

## Citation

```
@misc{optitrack_arducopter_bridge_2025,
  author = {ruhtruht},
  title = {MoCapMAV: OptiTrack-Based Indoor Drone Navigation Using MAVLink},
  year = {2025},
  howpublished = {\url{https://github.com/ruhtruht/optitrack-arducopter-bridge}},
  note = {Bachelor’s Thesis Project}
}
```

```

Wenn du willst, kann ich zusätzlich noch ein `.bib`-Eintrag oder ein akademisches Abstract draus machen.
```
