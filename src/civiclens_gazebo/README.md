# CivicLens AI — Autonomous Smart Waste Management System

CivicLens AI is a smart, autonomous municipal waste-management system designed for urban residential neighborhoods. It detects overflowing dustbins using solar-powered AI camera IoT stations, classifies the waste using OpenCV YOLO-style models, dispatches an autonomous multi-compartment waste truck with a 6-DOF robotic arm, sorts waste into segregated compartments (`BIODEGRADABLE`, `NON_BIODEGRADABLE`, `PLASTIC`), compacts polymers using an onboard plastic crusher, and transports waste back to a Central Recycling Base Station.

---

## 🏗️ Solution Architecture

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

## 🗺️ 5-Street City Layout & Round-Trip Missions

CivicLens AI operates across 5 distinct residential streets arranged on a 50m × 50m municipal grid block:

| Street | Dustbin ID | Coordinates $(X, Y)$ | Waste Classification | Fill Level | Targeted Compartment |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Street 1: Greenfield Road** | `DB-01` | $(30.0, -3.5)$ | **BIODEGRADABLE** | $94.5\%$ (FULL) | Compartment 1 (Green) |
| **Street 2: Market Lane** | `DB-02` | $(53.5, 30.0)$ | **NON_BIODEGRADABLE** | $96.2\%$ (FULL) | Compartment 2 (Blue) |
| **Street 3: Heritage Avenue** | `DB-03` | $(30.0, 53.5)$ | **PLASTIC** | $98.4\%$ (FULL) | Compartment 3 (Yellow + Crusher) |
| **Street 4: Sunrise Boulevard** | `DB-04` | $(-3.5, 30.0)$ | **BIODEGRADABLE** | $92.8\%$ (FULL) | Compartment 1 (Green) |
| **Street 5: Tech Park Way** | `DB-05` | $(80.0, -3.5)$ | **PLASTIC** | $95.7\%$ (FULL) | Compartment 3 (Yellow + Crusher) |

### Demand-Driven Round-Trip Sequence:
The autonomous truck executes 5 distinct collection cycles, always returning to the Central Base Station to discharge and reset:
$$\text{BASE} \to \text{STREET 1} \to \text{BASE} \to \text{STREET 2} \to \text{BASE} \to \text{STREET 3} \to \text{BASE} \to \text{STREET 4} \to \text{BASE} \to \text{STREET 5} \to \text{FINAL BASE}$$

---

## 🚛 Autonomous Truck & Segregated Compartments

The truck model (`model://civiclens_truck`) features:
- **6-DOF Articulated Robotic Arm**: Turret base, shoulder pitch, elbow pitch, forearm roll, wrist pitch, and mechanical gripper claw.
- **Physical Attachment & Dynamic Tracking**: Waste objects (`waste_1` to `waste_5`) are lifted from the bin and placed inside the exact compartment, riding secured on the truck bed during transit.
- **4 Dedicated Zones**:
  1. **Compartment 1 (Green, $X = +0.40$)**: Biodegradable & wet compost refuse.
  2. **Compartment 2 (Blue, $X = 0.0$)**: Non-biodegradable, metals, and dry recyclables.
  3. **Compartment 3 (Yellow, $X = -0.40$)**: Compacted plastic polymers.
  4. **Plastic Crusher Unit (Charcoal, $X = -0.80$)**: High-torque ingestion teeth with $4.2\times$ volumetric reduction.

---

## 🌐 Real-Time Telemetry & Operations Dashboard

A real-time web dashboard is served at **`http://localhost:8080`**:
- **Interactive 2D Street Grid & Fleet Map (HTML5 Canvas)**: Real-time rendering of all 5 streets, dustbin statuses (overflowing vs serviced), and live autonomous truck position, heading, and wheel tracks.
- **Live AI Camera Feeds**: Real-time OpenCV synthetic camera frames with YOLO-style bounding boxes (`[BIODEGRADABLE 94.5%]`, `[PLASTIC 98.4%]`) and fill-level gauges.
- **3-Compartment Fill Meters**: Live payload weight (kg) per compartment.
- **Plastic Crusher Ratio Monitor**: Live compaction efficiency ($4.2 : 1$) and truck volume saved ($76.2\%$).
- **Live Event Log**: Timestamped terminal feed for all IoT alerts and fleet dispatches.

---

## 🚀 Quickstart Guide

### 1. Build the Workspace
```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

### 2. Launch Complete System (One Command)
```bash
./run.sh
```
This automatically sets all paths, launches Gazebo Sim, connects the ROS 2 bridges, runs the AI Vision IoT stations, starts the Central Autonomy Mission Planner, initializes the Fleet Manager, and opens the Operations Dashboard at **`http://localhost:8080`**.

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

