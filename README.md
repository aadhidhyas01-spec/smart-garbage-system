# AI Robotic Truck — Autonomous Smart Waste Management System

An autonomous, multi-compartment municipal waste management robotic truck simulation powered by **ROS 2**, **Gazebo Sim**, **AI Vision IoT**, and a real-time **Telemetry Dashboard**.

---

## 🌟 Overview

The **AI Robotic Truck** (`civiclens_gazebo`) is designed for urban residential neighborhoods. It autonomously detects overflowing dustbins using solar-powered AI camera IoT stations, classifies waste types using computer vision, navigates a 5-street municipal grid, utilizes a 6-DOF robotic arm for automated bin retrieval, segregates refuse into dedicated compartments, crushes plastic polymers with an onboard crusher, and discharges payload at a Central Base Station.

---

## 🏗️ System Architecture

```
SMART DUSTBIN (DB-01 to DB-05)
     │
     ▼
CivicLens Solar AI Camera Station
     │
     ├── Fill-Level Gauge Monitoring (>90% Trigger)
     ├── OpenCV YOLO Waste Classification (Bio / Non-Bio / Plastic)
     └── 5G IoT Transmission (Priority Alerts & GPS Coordinates)
            │
            ▼
      Central AI System
            │
            ▼
    Mission / Route Planner (civiclens_autonomy)
            │
            ▼
   Autonomous Waste Truck
            │
     ┌──────┴──────────────────────────────────────┐
     ▼                                             ▼
 Kinematic Navigation                       6-DOF Robotic Arm
     │                                             │
     ▼                                             ▼
 5-Street Waypoints                        Physical Pickup & Sorting
                                                   │
          ┌────────────────────────────────────────┼────────────────────────────────────────┐
          ▼                                        ▼                                        ▼
Compartment 1 (Bio)                      Compartment 2 (Non-Bio)                  Compartment 3 (Plastic)
  [Vibrant Green]                          [Electric Cyan]                          [Yellow + Crusher]
     │                                        │                                        │
     └────────────────────────────────────────┼────────────────────────────────────────┘
                                              ▼
                                 Central Base Station Unload
                                              │
                                              ▼
                           Municipal Recycling & Polymer Shredder
```

---

## 🚛 Key Features

- **Autonomous Waypoint Navigation**: Executes round-trip collection routes across residential streets (`DB-01` through `DB-05`).
- **High-Fidelity Vehicle Design & Cab**: Aerodynamic cab with tinted windshield, side windows, doors, side mirrors, grille, dual LED headlights, indicators, and heavy-duty bumper.
- **Mounted Perception Sensors**:
  - **Ultrasonic Sensors**: Dual front bumper sonar transducers for close-range docking (< 1.0 m).
  - **IR Proximity Sensor**: Infrared edge detection on the front bumper.
  - **3D LiDAR Sensor**: Cab roof spinning puck with active 360° laser scanning.
  - **GPS Module**: Cab roof GNSS antenna dome & RTK positioning.
  - **IMU Sensor**: Central chassis inertial measurement unit monitoring orientation and tilt.
- **Undercarriage Powertrain & Electronics**:
  - **Electric Drive Motor**: High-torque traction motor with radial cooling fins and active speed control.
  - **Battery System**: High-voltage lithium energy storage pack with orange shielded conduits.
  - **Control Electronics**: Power distribution unit with aluminum heatsink fins.
  - **Main Controller**: Vehicle ECU device management unit.
- **6-DOF Robotic Arm**: Turret base, shoulder pitch, elbow pitch, forearm roll, wrist pitch, and mechanical gripper claw with rubber friction pads for physical bin handling and continuous waste bag tracking during pickup.
- **Tri-Compartment Segregated Storage**:
  - **Compartment 1 (Green)**: Biodegradable & organic waste with embossed **Leaf** emblem.
  - **Compartment 2 (Blue)**: Non-biodegradable, metals, and dry recyclables with embossed **Recycling 3-arrow** emblem.
  - **Compartment 3 (Yellow)**: Compacted plastic polymers with embossed **Bottle** emblem.
- **Onboard Plastic Crusher**: Dual counter-rotating toothed steel rollers achieving $4.2\times$ volumetric reduction ($76.2\%$ volume saved).
- **Live Telemetry & Web Operations Dashboard**: Real-time fleet mapping, YOLO vision bounding boxes, sensor telemetry HUD (ultrasonic, motor RPM, battery %, crusher torque), compartment fill gauges, and event logs served at `http://localhost:8080`.

---

## 🗺️ 5-Street City Layout & Missions

| Street | Dustbin ID | Coordinates $(X, Y)$ | Waste Classification | Fill Level | Targeted Compartment |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Street 1: Greenfield Road** | `DB-01` | $(30.0, -3.5)$ | **BIODEGRADABLE** | $94.5\%$ (FULL) | Compartment 1 (Green) |
| **Street 2: Market Lane** | `DB-02` | $(53.5, 30.0)$ | **NON_BIODEGRADABLE** | $96.2\%$ (FULL) | Compartment 2 (Cyan) |
| **Street 3: Heritage Avenue** | `DB-03` | $(30.0, 53.5)$ | **PLASTIC** | $98.4\%$ (FULL) | Compartment 3 (Yellow + Crusher) |
| **Street 4: Sunrise Boulevard** | `DB-04` | $(-3.5, 30.0)$ | **BIODEGRADABLE** | $92.8\%$ (FULL) | Compartment 1 (Green) |
| **Street 5: Tech Park Way** | `DB-05` | $(80.0, -3.5)$ | **PLASTIC** | $95.7\%$ (FULL) | Compartment 3 (Yellow + Crusher) |

---

## 🚀 Getting Started

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 (Lyrical / Humble / Iron / Jazzy)
- Gazebo Sim (GZ Sim)
- Python 3 & OpenCV

### Build
```bash
source /opt/ros/lyrical/setup.bash # or your ROS 2 distro
colcon build --symlink-install
```

### Run
```bash
./run.sh
```
This executes Gazebo Sim, loads city worlds and models, launches ROS 2 controllers, runs the Central Mission Planner, and launches the live web dashboard at `http://localhost:8080`.

---

## 📡 ROS 2 Topics Reference

| Topic | Type | Purpose |
| :--- | :--- | :--- |
| `/civiclens/truck_command` | `std_msgs/msg/String` | Waypoint navigation and arm manipulation commands (`GOTO`, `PICKUP`, `UNLOAD`). |
| `/civiclens/truck_state` | `std_msgs/msg/String` | Action completion signals (`DONE`) from Gazebo Model Plugin to ROS 2. |
| `/civiclens/truck_telemetry` | `std_msgs/msg/String` | High-frequency JSON telemetry of truck position $(x, y, yaw)$, arm state, and compartment loads. |
| `/civiclens/mission_status` | `std_msgs/msg/String` | Mission round, target street, step description, and overall progress percentage. |
| `/civiclens/iot_alerts` | `std_msgs/msg/String` | Real-time smart dustbin fill level alarms (>90% full trigger) and classification metadata. |
| `/civiclens/fleet_analytics` | `std_msgs/msg/String` | Aggregated payload metrics (kg collected, CO₂ saved, crusher compaction ratio). |

---

## 📄 License
Apache 2.0 / MIT License.
