#!/usr/bin/env python3
"""
CivicLens AI — Real-Time Operations & Telemetry Web Dashboard
Hosts a glassmorphic, high-tech web interface on port 8080 with interactive 2D street map,
live OpenCV AI vision feeds with YOLO detections, 3-compartment truck payload meters,
plastic crusher status, perception sensors (Ultrasonic, IR, LiDAR, GPS, IMU), powertrain HUD,
and full interactive system architecture blueprint based on the user's diagram.
"""

import os
import sys
import time
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CAMERA_FRAME_DIR = "/tmp/civiclens_cameras"
os.makedirs(CAMERA_FRAME_DIR, exist_ok=True)

# Shared Global State between ROS 2 subscriber thread and HTTP server
STATE = {
    "truck": {
        "x": 0.0,
        "y": -10.0,
        "yaw": 1.57,
        "state": "IDLE",
        "bin_id": 1,
        "waste_type": "BIODEGRADABLE",
        "arm_step": 0,
        "bio_count": 0,
        "nonbio_count": 0,
        "plastic_count": 0,
        "ultrasonic_dist_m": 5.0,
        "motor_speed_kmh": 0.0,
        "motor_rpm": 0,
        "battery_soc_pct": 98.6,
        "battery_voltage": 74.0,
        "crusher_status": "IDLE",
        "crusher_rpm": 0.0,
        "crusher_torque_nm": 12.0,
        "lidar_range_m": 18.5,
        "imu_pitch_deg": 0.15
    },
    "mission": {
        "round": 1,
        "total_rounds": 5,
        "street": "Street 1: Greenfield Rd",
        "bin_code": "DB-01",
        "command": "GOTO 0 0 1.57",
        "description": "Leaving Base towards Intersection",
        "step_index": 0,
        "total_steps": 42,
        "progress": 0.0,
        "complete": False
    },
    "analytics": {
        "total_mass_kg": 0.0,
        "bio_mass_kg": 0.0,
        "nonbio_mass_kg": 0.0,
        "plastic_mass_kg": 0.0,
        "crusher_compaction_ratio": "4.2:1",
        "volume_saved_pct": 76.2,
        "bins_serviced": 0,
        "unload_cycles": 0,
        "co2_saved_kg": 0.0,
        "fleet_efficiency_pct": 98.7
    },
    "bins": [
        {"id": 1, "code": "DB-01", "street": "Greenfield Road (St 1)", "type": "BIODEGRADABLE", "fill": 94.5, "status": "FULL", "x": 30.0, "y": -3.5},
        {"id": 2, "code": "DB-02", "street": "Market Lane (St 2)", "type": "NON_BIODEGRADABLE", "fill": 96.2, "status": "FULL", "x": 53.5, "y": 30.0},
        {"id": 3, "code": "DB-03", "street": "Heritage Avenue (St 3)", "type": "PLASTIC", "fill": 98.4, "status": "FULL", "x": 30.0, "y": 53.5},
        {"id": 4, "code": "DB-04", "street": "Sunrise Boulevard (St 4)", "type": "BIODEGRADABLE", "fill": 92.8, "status": "FULL", "x": -3.5, "y": 30.0},
        {"id": 5, "code": "DB-05", "street": "Tech Park Way (St 5)", "type": "PLASTIC", "fill": 95.7, "status": "FULL", "x": 80.0, "y": -3.5}
    ],
    "events": [
        {"time": time.strftime("%H:%M:%S"), "msg": "CivicLens AI System Online. Fleet Dispatched from Central Base Depot.", "type": "info"}
    ]
}

STATE_LOCK = threading.Lock()
CMD_PUBLISHER = None

HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CivicLens AI — Smart Waste Management &amp; Robotic Manipulation Dashboard</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700;800;900&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-base: #060911;
      --bg-card: rgba(16, 23, 38, 0.85);
      --bg-card-border: rgba(255, 255, 255, 0.08);
      --bg-card-hover: rgba(23, 34, 56, 0.95);
      --neon-bio: #00e676;
      --neon-nonbio: #00b0ff;
      --neon-plastic: #ffd600;
      --neon-crusher: #ff9100;
      --neon-danger: #ff1744;
      --text-main: #f0f4f8;
      --text-muted: #889bb3;
      --accent-cyan: #00f2fe;
    }
    * { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Outfit', sans-serif; }
    body {
      background: radial-gradient(circle at 15% 15%, #0e1a33 0%, var(--bg-base) 60%);
      color: var(--text-main);
      min-height: 100vh;
      overflow-x: hidden;
      padding-bottom: 24px;
    }
    .header {
      background: rgba(10, 14, 26, 0.90);
      backdrop-filter: blur(16px);
      border-bottom: 1px solid var(--bg-card-border);
      padding: 16px 28px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      position: sticky;
      top: 0;
      z-index: 100;
    }
    .brand { display: flex; align-items: center; gap: 14px; }
    .brand-logo {
      width: 46px; height: 46px;
      background: linear-gradient(135deg, #00c6ff, #0072ff);
      border-radius: 12px;
      display: flex; align-items: center; justify-content: center;
      box-shadow: 0 0 20px rgba(0, 198, 255, 0.4);
      font-size: 24px;
    }
    .brand-title h1 { font-size: 19px; font-weight: 800; letter-spacing: 0.5px; }
    .brand-title span { color: var(--accent-cyan); font-weight: 500; font-size: 12px; letter-spacing: 0.8px; }
    .header-badges { display: flex; gap: 12px; align-items: center; }
    .badge {
      background: rgba(255,255,255,0.05);
      border: 1px solid var(--bg-card-border);
      padding: 6px 14px;
      border-radius: 20px;
      font-size: 12px;
      display: flex; align-items: center; gap: 8px;
      font-family: 'JetBrains Mono', monospace;
    }
    .pulse-dot {
      width: 8px; height: 8px; border-radius: 50%;
      background: var(--neon-bio);
      box-shadow: 0 0 10px var(--neon-bio);
      animation: pulse 1.8s infinite;
    }
    @keyframes pulse { 0% { transform: scale(0.9); opacity: 0.8; } 50% { transform: scale(1.3); opacity: 1; } 100% { transform: scale(0.9); opacity: 0.8; } }
    
    /* System Architecture Banner */
    .system-blueprint-banner {
      background: linear-gradient(90deg, rgba(0, 198, 255, 0.1), rgba(0, 114, 255, 0.05));
      border: 1px solid rgba(0, 242, 254, 0.2);
      border-radius: 14px;
      margin: 18px 28px 0 28px;
      padding: 14px 20px;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .blueprint-title {
      font-size: 13px;
      font-weight: 800;
      color: var(--accent-cyan);
      letter-spacing: 1px;
      text-transform: uppercase;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .blueprint-flow {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
      font-size: 11px;
      font-weight: 600;
      color: var(--text-muted);
    }
    .flow-step {
      background: rgba(255,255,255,0.06);
      padding: 4px 10px;
      border-radius: 6px;
      color: #fff;
      display: flex; align-items: center; gap: 5px;
      border: 1px solid rgba(255,255,255,0.06);
    }
    .flow-step.active {
      background: rgba(0, 242, 254, 0.2);
      border-color: var(--accent-cyan);
      color: var(--accent-cyan);
      box-shadow: 0 0 10px rgba(0, 242, 254, 0.3);
    }
    .flow-arrow { color: var(--accent-cyan); font-weight: bold; }

    .main-grid {
      display: grid;
      grid-template-columns: 1.15fr 0.85fr;
      gap: 20px;
      padding: 18px 28px;
    }
    @media (max-width: 1200px) { .main-grid { grid-template-columns: 1fr; } }

    .card {
      background: var(--bg-card);
      backdrop-filter: blur(14px);
      border: 1px solid var(--bg-card-border);
      border-radius: 18px;
      padding: 20px;
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.35);
      transition: border-color 0.25s ease;
    }
    .card:hover { border-color: rgba(0, 242, 254, 0.3); }
    .card-title {
      font-size: 14px;
      font-weight: 700;
      letter-spacing: 0.8px;
      text-transform: uppercase;
      color: var(--text-muted);
      margin-bottom: 14px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    /* Canvas Map */
    #cityCanvas {
      width: 100%;
      height: 360px;
      background: #0b111e;
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,0.06);
      display: block;
    }

    /* Camera Feed Section */
    .cam-container {
      position: relative;
      border-radius: 14px;
      overflow: hidden;
      border: 1px solid rgba(0, 242, 254, 0.25);
      background: #000;
    }
    .cam-feed {
      width: 100%;
      height: 260px;
      object-fit: cover;
      display: block;
    }
    .cam-tag {
      position: absolute;
      top: 10px; left: 10px;
      background: rgba(0,0,0,0.75);
      border: 1px solid var(--neon-bio);
      color: var(--neon-bio);
      font-size: 11px;
      font-family: 'JetBrains Mono', monospace;
      padding: 4px 10px;
      border-radius: 6px;
      display: flex; align-items: center; gap: 6px;
    }
    .cam-selector {
      display: flex;
      gap: 8px;
      margin-top: 10px;
      flex-wrap: wrap;
    }
    .cam-btn {
      background: rgba(255,255,255,0.05);
      border: 1px solid var(--bg-card-border);
      color: var(--text-muted);
      padding: 5px 12px;
      border-radius: 8px;
      font-size: 11px;
      cursor: pointer;
      font-weight: 600;
      transition: all 0.2s;
    }
    .cam-btn.active, .cam-btn:hover {
      background: rgba(0, 242, 254, 0.15);
      border-color: var(--accent-cyan);
      color: #fff;
    }

    /* Mission Status Banner */
    .mission-banner {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
      margin-bottom: 16px;
    }
    .stat-pill {
      background: rgba(255,255,255,0.03);
      border: 1px solid var(--bg-card-border);
      border-radius: 12px;
      padding: 12px 14px;
    }
    .stat-pill .label { font-size: 10px; color: var(--text-muted); text-transform: uppercase; font-weight: 700; }
    .stat-pill .value { font-size: 17px; font-weight: 800; margin-top: 4px; font-family: 'JetBrains Mono', monospace; }

    /* Compartments */
    .comp-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 10px;
    }
    .comp-box {
      border-radius: 14px;
      padding: 14px;
      background: rgba(255,255,255,0.02);
      border: 1px solid var(--bg-card-border);
      position: relative;
    }
    .comp-box.bio { border-top: 3px solid var(--neon-bio); }
    .comp-box.nonbio { border-top: 3px solid var(--neon-nonbio); }
    .comp-box.plastic { border-top: 3px solid var(--neon-plastic); }
    .comp-meter {
      height: 8px;
      background: rgba(255,255,255,0.08);
      border-radius: 4px;
      margin-top: 10px;
      overflow: hidden;
    }
    .comp-bar {
      height: 100%;
      border-radius: 4px;
      transition: width 0.4s ease;
    }

    /* Vehicle Perception & Powertrain HUD Grid */
    .sensor-hud-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
    }
    .sensor-tile {
      background: rgba(255,255,255,0.02);
      border: 1px solid var(--bg-card-border);
      border-radius: 12px;
      padding: 10px 12px;
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .sensor-tile .st-head {
      font-size: 10px;
      font-weight: 700;
      color: var(--text-muted);
      text-transform: uppercase;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .sensor-tile .st-val {
      font-size: 15px;
      font-weight: 800;
      font-family: 'JetBrains Mono', monospace;
      color: #fff;
    }
    .sensor-tile .st-sub {
      font-size: 10px;
      color: var(--accent-cyan);
    }

    /* Interactive Dispatch Buttons */
    .btn-strip {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 12px;
    }
    .action-btn {
      background: rgba(0, 198, 255, 0.12);
      border: 1px solid rgba(0, 242, 254, 0.3);
      color: #fff;
      padding: 8px 14px;
      border-radius: 8px;
      font-size: 12px;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.2s;
    }
    .action-btn:hover {
      background: rgba(0, 242, 254, 0.3);
      border-color: var(--accent-cyan);
      box-shadow: 0 0 14px rgba(0, 242, 254, 0.4);
      transform: translateY(-1px);
    }

    /* Event Log Terminal */
    .terminal {
      background: #080c14;
      border: 1px solid rgba(255,255,255,0.06);
      border-radius: 12px;
      padding: 12px;
      height: 150px;
      overflow-y: auto;
      font-family: 'JetBrains Mono', monospace;
      font-size: 11px;
      display: flex;
      flex-direction: column;
      gap: 5px;
    }
    .term-line { display: flex; gap: 10px; line-height: 1.4; }
    .term-time { color: var(--text-muted); flex-shrink: 0; }
    .term-msg { color: #d0d8e2; }
    .term-alert { color: var(--neon-danger); font-weight: 700; }
    .term-action { color: var(--neon-plastic); }
    .term-success { color: var(--neon-bio); }
  </style>
</head>
<body>

  <header class="header">
    <div class="brand">
      <div class="brand-logo">🚛</div>
      <div class="brand-title">
        <h1>CIVICLENS AI — ROBOTIC WASTE FLEET</h1>
        <span>AI VISION • ROBOTIC MANIPULATION • MULTI-COMPARTMENT SORTING</span>
      </div>
    </div>
    <div class="header-badges">
      <div class="badge"><div class="pulse-dot"></div>5G IOT ONLINE</div>
      <div class="badge" id="clock">00:00:00</div>
      <div class="badge" style="border-color: var(--accent-cyan); color: var(--accent-cyan);">ROS 2 + GAZEBO SIM</div>
    </div>
  </header>

  <!-- High-Level Working Flow Banner (Directly from User Diagram) -->
  <div class="system-blueprint-banner">
    <div class="blueprint-title">
      <span>Smart Automated Waste Management Flow Architecture</span>
      <span style="font-size: 11px; font-weight: 500; color: var(--text-muted);">End-to-End Autonomous Lifecycle</span>
    </div>
    <div class="blueprint-flow">
      <div class="flow-step" id="flow_cam">📷 AI Camera Detection</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_ai">🧠 Central AI &amp; Route</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_nav">🚛 Truck Autonomous Transit</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_dock">📡 Ultrasonic/LiDAR Docking</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_arm">🦾 Robotic Arm Dustbin Grasp</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_sort">♻️ 3-Way Sort &amp; Crusher</div>
      <span class="flow-arrow">→</span>
      <div class="flow-step" id="flow_cloud">☁️ Cloud Database Record</div>
    </div>
  </div>

  <main class="main-grid">
    <!-- LEFT COLUMN -->
    <div style="display: flex; flex-direction: column; gap: 18px;">
      
      <!-- Top Mission Progress Banner -->
      <div class="card">
        <div class="card-title">
          <span>Active Mission Round-Trip Sequence</span>
          <span id="roundBadge" style="color: var(--accent-cyan); font-family: 'JetBrains Mono'; font-weight: 700;">ROUND 1 / 5</span>
        </div>
        <div class="mission-banner">
          <div class="stat-pill">
            <div class="label">Target Street</div>
            <div class="value" id="curStreet" style="color: #fff; font-size: 14px;">Street 1: Greenfield</div>
          </div>
          <div class="stat-pill">
            <div class="label">Target Bin</div>
            <div class="value" id="curBin" style="color: var(--neon-plastic);">DB-01</div>
          </div>
          <div class="stat-pill">
            <div class="label">Truck Action</div>
            <div class="value" id="truckState" style="color: var(--neon-bio); font-size: 15px;">CRUISING</div>
          </div>
          <div class="stat-pill">
            <div class="label">Overall Progress</div>
            <div class="value" id="missionProg">0.0%</div>
          </div>
        </div>

        <div style="font-size: 12px; color: var(--text-muted); margin-bottom: 6px; display: flex; justify-content: space-between;">
          <span id="missionDesc">En Route to Greenfield Road</span>
          <span id="activeCmd" style="font-family: 'JetBrains Mono'; color: var(--accent-cyan);">GOTO 30 0 0.0</span>
        </div>
        <div style="height: 6px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden;">
          <div id="progBar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #00c6ff, #0072ff); transition: width 0.5s;"></div>
        </div>
      </div>

      <!-- 2D Live Street & Truck Canvas Map -->
      <div class="card">
        <div class="card-title">
          <span>5-Street Autonomous Fleet Grid Map</span>
          <span style="font-size: 12px; color: var(--text-muted); font-weight: 400;">50m x 50m Municipal Zone</span>
        </div>
        <canvas id="cityCanvas" width="700" height="360"></canvas>
      </div>

      <!-- Vehicle Perception Sensors & Powertrain HUD (from User's Blueprint) -->
      <div class="card">
        <div class="card-title">
          <span>Truck Sensors &amp; Powertrain Telemetry HUD</span>
          <span style="font-size: 11px; color: var(--neon-bio);">● ALL SUBSYSTEMS NOMINAL</span>
        </div>
        <div class="sensor-hud-grid">
          
          <!-- Ultrasonic Sensor -->
          <div class="sensor-tile">
            <div class="st-head">🔊 Ultrasonic Sonar</div>
            <div class="st-val" id="hud_ultrasonic">0.82 m</div>
            <div class="st-sub" id="hud_ultrasonic_sub">Front Bumper Docking</div>
          </div>

          <!-- Drive Motor -->
          <div class="sensor-tile">
            <div class="st-head">⚡ Drive Motor</div>
            <div class="st-val" id="hud_motor">14.4 km/h</div>
            <div class="st-sub" id="hud_rpm">850 RPM • Speed Ctrl</div>
          </div>

          <!-- Battery System -->
          <div class="sensor-tile">
            <div class="st-head">🔋 Battery System</div>
            <div class="st-val" id="hud_battery">98.6 %</div>
            <div class="st-sub" id="hud_voltage">74.0 V • High-Voltage</div>
          </div>

          <!-- Plastic Crusher -->
          <div class="sensor-tile">
            <div class="st-head">⚙️ Plastic Crusher</div>
            <div class="st-val" id="hud_crusher" style="color: var(--neon-plastic);">IDLE</div>
            <div class="st-sub" id="hud_crusher_sub">4.2:1 Ratio • Powder</div>
          </div>

          <!-- 3D LiDAR -->
          <div class="sensor-tile">
            <div class="st-head">📡 3D LiDAR Sensor</div>
            <div class="st-val" id="hud_lidar">18.5 m</div>
            <div class="st-sub">360° Obstacle Avoidance</div>
          </div>

          <!-- IR Proximity -->
          <div class="sensor-tile">
            <div class="st-head">🔴 IR Sensor</div>
            <div class="st-val" style="color: var(--neon-bio);">CLEAR</div>
            <div class="st-sub">Front Bumper Optical</div>
          </div>

          <!-- GPS Module -->
          <div class="sensor-tile">
            <div class="st-head">📍 GPS Module</div>
            <div class="st-val">12.9716° N</div>
            <div class="st-sub">12 Sats Locked • RTK</div>
          </div>

          <!-- 6-DOF Robotic Arm -->
          <div class="sensor-tile">
            <div class="st-head">🦾 Robotic Arm</div>
            <div class="st-val" id="hud_arm_step" style="color: var(--accent-cyan);">STOWED</div>
            <div class="st-sub">Object Grasping Claw</div>
          </div>

        </div>
      </div>

      <!-- Fleet Analytics Summary -->
      <div class="card">
        <div class="card-title">Fleet Segregation &amp; Carbon Analytics</div>
        <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px;">
          <div class="stat-pill">
            <div class="label">Total Mass Diverted</div>
            <div class="value" id="totalKg" style="color: #fff;">0.0 kg</div>
          </div>
          <div class="stat-pill">
            <div class="label">Crusher Ratio</div>
            <div class="value" style="color: var(--neon-crusher);">4.2 : 1</div>
          </div>
          <div class="stat-pill">
            <div class="label">Truck Volume Saved</div>
            <div class="value" style="color: var(--neon-bio);">76.2%</div>
          </div>
          <div class="stat-pill">
            <div class="label">CO₂ Saved</div>
            <div class="value" id="co2Saved" style="color: var(--neon-nonbio);">0.0 kg</div>
          </div>
        </div>
      </div>

    </div>

    <!-- RIGHT COLUMN -->
    <div style="display: flex; flex-direction: column; gap: 18px;">

      <!-- Live AI Camera Feed with YOLO Detection -->
      <div class="card">
        <div class="card-title">
          <span>AI Vision Camera System</span>
          <span id="activeCamCode" style="color: var(--neon-bio); font-family: 'JetBrains Mono';">CAM: DB-01</span>
        </div>

        <div class="cam-container">
          <img id="camImg" src="/api/camera?bin=1" class="cam-feed" alt="Live Camera Feed">
          <div class="cam-tag">
            <div class="pulse-dot"></div>
            <span>YOLO-V11 DUSTBIN DETECTOR</span>
          </div>
        </div>

        <div class="cam-selector">
          <button class="cam-btn active" onclick="switchCam('auto')">Auto Active</button>
          <button class="cam-btn" onclick="switchCam(1)">DB-01 (Bio)</button>
          <button class="cam-btn" onclick="switchCam(2)">DB-02 (Non-Bio)</button>
          <button class="cam-btn" onclick="switchCam(3)">DB-03 (Plastic)</button>
          <button class="cam-btn" onclick="switchCam(4)">DB-04 (Bio)</button>
          <button class="cam-btn" onclick="switchCam(5)">DB-05 (Plastic)</button>
        </div>
      </div>

      <!-- Truck Segregated Compartments (Green Leaf, Blue Recycle, Yellow Bottle) -->
      <div class="card">
        <div class="card-title">
          <span>3 Segregated Compartments &amp; Crusher</span>
          <span style="font-size: 11px; color: var(--text-muted);">Molded Bed Bins</span>
        </div>
        <div class="comp-grid">
          
          <div class="comp-box bio">
            <div style="font-size: 10px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 1 🌿</div>
            <div style="font-size: 14px; font-weight: 800; color: var(--neon-bio); margin-top: 3px;">BIODEGRADABLE</div>
            <div style="font-size: 19px; font-family: 'JetBrains Mono'; margin-top: 4px;" id="bioKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="bioBar" style="width: 0%; background: var(--neon-bio);"></div></div>
          </div>

          <div class="comp-box nonbio">
            <div style="font-size: 10px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 2 ♻️</div>
            <div style="font-size: 14px; font-weight: 800; color: var(--neon-nonbio); margin-top: 3px;">NON-BIODEGRADABLE</div>
            <div style="font-size: 19px; font-family: 'JetBrains Mono'; margin-top: 4px;" id="nonbioKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="nonbioBar" style="width: 0%; background: var(--neon-nonbio);"></div></div>
          </div>

          <div class="comp-box plastic">
            <div style="font-size: 10px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 3 🍾</div>
            <div style="font-size: 14px; font-weight: 800; color: var(--neon-plastic); margin-top: 3px;">PLASTIC + CRUSHER</div>
            <div style="font-size: 19px; font-family: 'JetBrains Mono'; margin-top: 4px;" id="plasticKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="plasticBar" style="width: 0%; background: var(--neon-plastic);"></div></div>
          </div>

        </div>
      </div>

      <!-- Manual Mission & Arm Dispatch Controller -->
      <div class="card">
        <div class="card-title">Manual Fleet Dispatch &amp; Arm Controls</div>
        <div class="btn-strip">
          <button class="action-btn" onclick="sendCmd('GOTO 30 -1.0 -1.57')">📍 Goto DB-01</button>
          <button class="action-btn" onclick="sendCmd('GOTO 51.0 30 0.0')">📍 Goto DB-02</button>
          <button class="action-btn" onclick="sendCmd('GOTO 30 51.0 1.57')">📍 Goto DB-03</button>
          <button class="action-btn" onclick="sendCmd('PICKUP 1 BIODEGRADABLE')">🦾 Pick Bio</button>
          <button class="action-btn" onclick="sendCmd('PICKUP 3 PLASTIC')">⚙️ Pick &amp; Crush Plastic</button>
          <button class="action-btn" onclick="sendCmd('UNLOAD')">♻️ Base Unload</button>
        </div>
      </div>

      <!-- Live Event Operations Log -->
      <div class="card">
        <div class="card-title">Real-Time IoT &amp; Mission Event Log</div>
        <div class="terminal" id="termLog">
          <div class="term-line"><span class="term-time">[00:00:01]</span><span class="term-msg">System initialized. Connected to ROS 2 topic bus.</span></div>
        </div>
      </div>

    </div>
  </main>

  <script>
    let selectedCam = 'auto';
    let activeBinId = 1;

    function switchCam(val) {
      selectedCam = val;
      document.querySelectorAll('.cam-btn').forEach(btn => btn.classList.remove('active'));
      event.target.classList.add('active');
    }

    async function sendCmd(cmdStr) {
      try {
        await fetch('/api/command', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ command: cmdStr })
        });
      } catch (err) {
        console.error("Failed to send command:", err);
      }
    }

    // Refresh Camera Feed
    setInterval(() => {
      const camId = (selectedCam === 'auto') ? activeBinId : selectedCam;
      document.getElementById('camImg').src = '/api/camera?bin=' + camId + '&t=' + Date.now();
      document.getElementById('activeCamCode').innerText = 'CAM: DB-0' + camId;
    }, 400);

    // Update Clock
    setInterval(() => {
      const now = new Date();
      document.getElementById('clock').innerText = now.toTimeString().split(' ')[0];
    }, 1000);

    // Fetch Telemetry Poll (5 Hz)
    async function pollTelemetry() {
      try {
        const res = await fetch('/api/telemetry');
        const data = await res.json();
        
        // Update mission banner
        if (data.mission) {
          document.getElementById('roundBadge').innerText = 'ROUND ' + data.mission.round + ' / 5';
          document.getElementById('curStreet').innerText = data.mission.street;
          document.getElementById('curBin').innerText = data.mission.bin_code;
          document.getElementById('missionDesc').innerText = data.mission.description;
          document.getElementById('activeCmd').innerText = data.mission.command;
          document.getElementById('missionProg').innerText = data.mission.progress + '%';
          document.getElementById('progBar').style.width = data.mission.progress + '%';
          activeBinId = data.truck.bin_id || 1;
        }

        if (data.truck) {
          document.getElementById('truckState').innerText = data.truck.state;

          // Sensors & Powertrain HUD
          if (data.truck.ultrasonic_dist_m !== undefined) {
            document.getElementById('hud_ultrasonic').innerText = data.truck.ultrasonic_dist_m.toFixed(2) + ' m';
            if (data.truck.ultrasonic_dist_m < 1.0) {
              document.getElementById('hud_ultrasonic_sub').innerText = 'Docked at Dustbin';
              document.getElementById('hud_ultrasonic').style.color = 'var(--neon-bio)';
            } else {
              document.getElementById('hud_ultrasonic_sub').innerText = 'Front Bumper Sonar';
              document.getElementById('hud_ultrasonic').style.color = '#fff';
            }
          }

          if (data.truck.motor_speed_kmh !== undefined) {
            document.getElementById('hud_motor').innerText = data.truck.motor_speed_kmh.toFixed(1) + ' km/h';
            document.getElementById('hud_rpm').innerText = Math.round(data.truck.motor_rpm || 0) + ' RPM • Speed Ctrl';
          }

          if (data.truck.battery_soc_pct !== undefined) {
            document.getElementById('hud_battery').innerText = data.truck.battery_soc_pct.toFixed(1) + ' %';
            document.getElementById('hud_voltage').innerText = (data.truck.battery_voltage || 74.0).toFixed(1) + ' V • High-Voltage';
          }

          if (data.truck.crusher_status) {
            document.getElementById('hud_crusher').innerText = data.truck.crusher_status;
            if (data.truck.crusher_status === 'CRUSHING') {
              document.getElementById('hud_crusher').style.color = 'var(--neon-danger)';
              document.getElementById('hud_crusher_sub').innerText = 'Active 450 RPM • 125 Nm';
            } else {
              document.getElementById('hud_crusher').style.color = 'var(--neon-plastic)';
              document.getElementById('hud_crusher_sub').innerText = '4.2:1 Ratio • Powder';
            }
          }

          if (data.truck.lidar_range_m !== undefined) {
            document.getElementById('hud_lidar').innerText = data.truck.lidar_range_m.toFixed(1) + ' m';
          }

          // Arm step label
          const armSteps = ["APPROACH", "GRASP DUST", "LIFTING", "SORT SWING", "DEPOSITING", "STOWED"];
          const curStep = data.truck.arm_step || 0;
          if (data.truck.state === 'PICKING') {
            document.getElementById('hud_arm_step').innerText = armSteps[curStep] || "ACTIVE";
          } else {
            document.getElementById('hud_arm_step').innerText = "STOWED";
          }

          // Highlight Flow Step in Blueprint
          document.querySelectorAll('.flow-step').forEach(el => el.classList.remove('active'));
          if (data.truck.state === 'MOVING') {
            document.getElementById('flow_nav').classList.add('active');
          } else if (data.truck.state === 'PICKING') {
            if (curStep <= 1) document.getElementById('flow_dock').classList.add('active');
            else if (curStep <= 2) document.getElementById('flow_arm').classList.add('active');
            else document.getElementById('flow_sort').classList.add('active');
          } else if (data.truck.state === 'UNLOADING') {
            document.getElementById('flow_cloud').classList.add('active');
          } else {
            document.getElementById('flow_ai').classList.add('active');
          }
        }

        if (data.analytics) {
          document.getElementById('totalKg').innerText = data.analytics.total_mass_kg + ' kg';
          document.getElementById('co2Saved').innerText = data.analytics.co2_saved_kg + ' kg';
          document.getElementById('bioKg').innerText = data.analytics.bio_mass_kg + ' kg';
          document.getElementById('nonbioKg').innerText = data.analytics.nonbio_mass_kg + ' kg';
          document.getElementById('plasticKg').innerText = data.analytics.plastic_mass_kg + ' kg';
          
          document.getElementById('bioBar').style.width = Math.min(100, (data.analytics.bio_mass_kg / 100) * 100) + '%';
          document.getElementById('nonbioBar').style.width = Math.min(100, (data.analytics.nonbio_mass_kg / 100) * 100) + '%';
          document.getElementById('plasticBar').style.width = Math.min(100, (data.analytics.plastic_mass_kg / 100) * 100) + '%';
        }

        // Draw Map
        drawCityMap(data);

        // Update Terminal Events
        if (data.events) {
          const term = document.getElementById('termLog');
          term.innerHTML = '';
          data.events.slice(-15).forEach(e => {
            const line = document.createElement('div');
            line.className = 'term-line';
            let cls = 'term-msg';
            if (e.msg.includes('OVERFLOW')) cls = 'term-alert';
            else if (e.msg.includes('PICKUP') || e.msg.includes('Cruising')) cls = 'term-action';
            else if (e.msg.includes('Unloading') || e.msg.includes('SUCCESS')) cls = 'term-success';
            line.innerHTML = `<span class="term-time">[${e.time}]</span><span class="${cls}">${e.msg}</span>`;
            term.appendChild(line);
          });
          term.scrollTop = term.scrollHeight;
        }

      } catch (err) {
        console.error("Telemetry fetch error:", err);
      }
    }

    setInterval(pollTelemetry, 300);

    // 2D City Canvas Renderer
    const canvas = document.getElementById('cityCanvas');
    const ctx = canvas.getContext('2d');

    function drawCityMap(data) {
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);

      // Coordinate Transform: City is approx -15 <= X <= 95, -18 <= Y <= 65
      const minX = -15, maxX = 95;
      const minY = -18, maxY = 65;

      function toScreen(x, y) {
        const sx = ((x - minX) / (maxX - minX)) * (w - 60) + 30;
        const sy = h - (((y - minY) / (maxY - minY)) * (h - 60) + 30);
        return { x: sx, y: sy };
      }

      // Draw Roads
      ctx.lineWidth = 18;
      ctx.strokeStyle = '#1e283c';
      ctx.lineCap = 'round';

      // Road Y=0
      let p1 = toScreen(minX, 0), p2 = toScreen(maxX, 0);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();

      // Road Y=50
      p1 = toScreen(0, 50); p2 = toScreen(60, 50);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();

      // Road X=0
      p1 = toScreen(0, -12); p2 = toScreen(0, 50);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();

      // Road X=50
      p1 = toScreen(50, 0); p2 = toScreen(50, 50);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();

      // Road Dashed Centerlines
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffd600';
      ctx.setLineDash([6, 6]);
      p1 = toScreen(minX, 0); p2 = toScreen(maxX, 0);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
      ctx.setLineDash([]);

      // Base Station
      const basePos = toScreen(0, -10);
      ctx.fillStyle = '#0072ff';
      ctx.beginPath(); ctx.arc(basePos.x, basePos.y, 14, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#00c6ff'; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px JetBrains Mono';
      ctx.fillText('BASE DEPOT', basePos.x - 30, basePos.y + 24);

      // Smart Dustbins
      if (data.bins) {
        data.bins.forEach(b => {
          const bp = toScreen(b.x, b.y);
          let col = '#00e676';
          if (b.type === 'NON_BIODEGRADABLE') col = '#00b0ff';
          if (b.type === 'PLASTIC') col = '#ffd600';

          ctx.fillStyle = col;
          ctx.beginPath(); ctx.arc(bp.x, bp.y, 8, 0, Math.PI * 2); ctx.fill();
          ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();

          ctx.fillStyle = '#fff';
          ctx.font = 'bold 9px JetBrains Mono';
          ctx.fillText(b.code, bp.x - 12, bp.y - 12);
        });
      }

      // Draw Autonomous Truck (Top-Down Blueprint & Real-Time Compartments)
      if (data.truck) {
        const tp = toScreen(data.truck.x, data.truck.y);
        ctx.save();
        ctx.translate(tp.x, tp.y);
        ctx.rotate(-data.truck.yaw + Math.PI/2);

        // Truck chassis & perimeter shadow
        ctx.fillStyle = '#151c28';
        ctx.shadowColor = '#00f2fe';
        ctx.shadowBlur = 12;
        ctx.fillRect(-15, -30, 30, 60);

        // White Cab Shell & Windshield
        ctx.fillStyle = '#f8fafc';
        ctx.shadowBlur = 0;
        ctx.fillRect(-14, -29, 28, 16);

        // Cab Windshield (Dark Tint)
        ctx.fillStyle = '#0f172a';
        ctx.fillRect(-12, -27, 24, 6);

        // Cab Roof Solar Panels (Sapphire)
        ctx.fillStyle = '#0284c7';
        ctx.fillRect(-10, -20, 8, 5);
        ctx.fillRect(2, -20, 8, 5);

        // Emergency Flashing Beacon Bar (Amber)
        ctx.fillStyle = '#f59e0b';
        ctx.shadowColor = '#f59e0b';
        ctx.shadowBlur = (Date.now() % 600 < 300) ? 10 : 2;
        ctx.fillRect(-7, -22, 14, 2);
        ctx.shadowBlur = 0;

        // Robotic Arm Base & Turret (Between Cab & Bed)
        ctx.fillStyle = '#94a3b8';
        ctx.beginPath();
        ctx.arc(0, -11, 4.5, 0, Math.PI * 2);
        ctx.fill();

        // Arm Link Indicator line
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(0, -11);
        if (data.truck.state === 'PICKING') {
          // Arm extended forward or back depending on step
          if (data.truck.arm_step <= 2) {
            ctx.lineTo(0, -28); // Reaching forward
          } else {
            ctx.lineTo(0, 5);  // Swinging backward
          }
        } else {
          ctx.lineTo(0, -5);   // Stowed
        }
        ctx.stroke();

        // Bed Side Rails with Compartment Color Trim
        ctx.fillStyle = '#334155';
        ctx.fillRect(-15, -6, 2, 36);
        ctx.fillRect(13, -6, 2, 36);

        // ================= TRUCK COMPARTMENTS (COMPARTMENT-WISE TOP VIEW) =================
        // Comp 1: Bio / Organic (Emerald Green)
        const isBioActive = (data.truck.state === 'PICKING' && data.truck.waste_type === 'BIODEGRADABLE');
        ctx.fillStyle = isBioActive ? '#00ff88' : '#059669';
        if (isBioActive) { ctx.shadowColor = '#00ff88'; ctx.shadowBlur = 8; }
        ctx.fillRect(-12, -6, 24, 8);
        ctx.shadowBlur = 0;
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 6px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('BIO 🍃', 0, 0);

        // Bulkhead Divider 1
        ctx.fillStyle = '#e2e8f0';
        ctx.fillRect(-13, 3, 26, 1.5);

        // Comp 2: Non-Bio / Recyclable (Electric Blue)
        const isNonBioActive = (data.truck.state === 'PICKING' && data.truck.waste_type === 'NON_BIODEGRADABLE');
        ctx.fillStyle = isNonBioActive ? '#38bdf8' : '#0284c7';
        if (isNonBioActive) { ctx.shadowColor = '#38bdf8'; ctx.shadowBlur = 8; }
        ctx.fillRect(-12, 5.5, 24, 8);
        ctx.shadowBlur = 0;
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 6px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('RECYCLE ♻', 0, 11.5);

        // Bulkhead Divider 2
        ctx.fillStyle = '#e2e8f0';
        ctx.fillRect(-13, 14.5, 26, 1.5);

        // Comp 3: Plastic (Safety Yellow)
        const isPlasticActive = (data.truck.state === 'PICKING' && data.truck.waste_type === 'PLASTIC');
        ctx.fillStyle = isPlasticActive ? '#fef08a' : '#eab308';
        if (isPlasticActive) { ctx.shadowColor = '#facc15'; ctx.shadowBlur = 8; }
        ctx.fillRect(-12, 17, 24, 7);
        ctx.shadowBlur = 0;
        ctx.fillStyle = '#0f172a';
        ctx.font = 'bold 6px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('PLASTIC', 0, 22.5);

        // Bulkhead Divider 3
        ctx.fillStyle = '#475569';
        ctx.fillRect(-13, 24.5, 26, 1.5);

        // Comp 4: Onboard Plastic Crusher (Industrial Orange + Hazard Stripes)
        ctx.fillStyle = '#ea580c';
        ctx.fillRect(-12, 26.5, 24, 7.5);
        // Hazard stripe pattern
        ctx.fillStyle = '#0f172a';
        ctx.fillRect(-10, 26.5, 4, 7.5);
        ctx.fillRect(-2, 26.5, 4, 7.5);
        ctx.fillRect(6, 26.5, 4, 7.5);
        if (data.truck.crusher_status === 'CRUSHING') {
          ctx.fillStyle = '#ffea00';
          ctx.font = 'bold 5px Inter, sans-serif';
          ctx.fillText('⚡ CRUSH', 0, 32);
        }

        // Heading Direction Arrow (Cyan)
        ctx.fillStyle = '#00f2fe';
        ctx.beginPath();
        ctx.moveTo(0, -36); ctx.lineTo(-6, -30); ctx.lineTo(6, -30); ctx.closePath();
        ctx.fill();

        ctx.restore();
      }
    }
  </script>
</body>
</html>
"""

class CivicLensHTTPHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_DASHBOARD.encode('utf-8'))

        elif path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with STATE_LOCK:
                payload = json.dumps(STATE)
            self.wfile.write(payload.encode('utf-8'))

        elif path == "/api/camera":
            query = urllib.parse.parse_qs(parsed.query)
            bin_id = query.get("bin", ["1"])[0]
            
            img_file = os.path.join(CAMERA_FRAME_DIR, f"cam_db_{bin_id}.jpg")
            if not os.path.exists(img_file):
                img_file = os.path.join(CAMERA_FRAME_DIR, "cam_latest.jpg")

            if os.path.exists(img_file):
                try:
                    with open(img_file, "rb") as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                    self.end_headers()
                    self.wfile.write(data)
                except Exception:
                    self.send_response(500)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            try:
                body = json.loads(post_data.decode('utf-8'))
                cmd_str = body.get("command", "")
                if cmd_str and CMD_PUBLISHER:
                    msg = String()
                    msg.data = cmd_str
                    CMD_PUBLISHER.publish(msg)
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(b'{"status":"ok"}')
                    return
            except Exception:
                pass
            self.send_response(400)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


class DashboardROSNode(Node):
    def __init__(self):
        super().__init__('civiclens_dashboard_node')
        global CMD_PUBLISHER
        CMD_PUBLISHER = self.create_publisher(String, '/civiclens/truck_command', 10)

        self.telemetry_sub = self.create_subscription(
            String, '/civiclens/truck_telemetry', self.on_truck_telemetry, 10)
        self.mission_sub = self.create_subscription(
            String, '/civiclens/mission_status', self.on_mission_status, 10)
        self.analytics_sub = self.create_subscription(
            String, '/civiclens/fleet_analytics', self.on_analytics, 10)
        self.alert_sub = self.create_subscription(
            String, '/civiclens/iot_alerts', self.on_iot_alert, 10)
        self.cmd_sub = self.create_subscription(
            String, '/civiclens/truck_command', self.on_command, 10)

        self.get_logger().info("Dashboard ROS 2 Subscriber Bridge Initialized with Perception & Powertrain HUD.")

    def on_truck_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
            with STATE_LOCK:
                STATE["truck"].update(data)
        except Exception:
            pass

    def on_mission_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            with STATE_LOCK:
                STATE["mission"].update(data)
        except Exception:
            pass

    def on_analytics(self, msg: String):
        try:
            data = json.loads(msg.data)
            with STATE_LOCK:
                STATE["analytics"].update(data)
        except Exception:
            pass

    def on_iot_alert(self, msg: String):
        try:
            data = json.loads(msg.data)
            entry = {
                "time": time.strftime("%H:%M:%S"),
                "msg": f"IoT Alert: Smart Bin {data.get('code')} at {data.get('street')} is {data.get('fill_level')}% FULL ({data.get('waste_type')})",
                "type": "alert"
            }
            with STATE_LOCK:
                STATE["events"].append(entry)
                if len(STATE["events"]) > 50:
                    STATE["events"].pop(0)
        except Exception:
            pass

    def on_command(self, msg: String):
        cmd = msg.data
        entry = {
            "time": time.strftime("%H:%M:%S"),
            "msg": f"Fleet Command: {cmd}",
            "type": "action"
        }
        with STATE_LOCK:
            STATE["events"].append(entry)
            if len(STATE["events"]) > 50:
                STATE["events"].pop(0)


def run_http_server(port=8080):
    server_address = ('', port)
    httpd = HTTPServer(server_address, CivicLensHTTPHandler)
    print(f"[CivicLens Dashboard] HTTP Telemetry Server running at http://0.0.0.0:{port}")
    httpd.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    ros_node = DashboardROSNode()

    # Start HTTP server in a daemon thread
    http_thread = threading.Thread(target=run_http_server, args=(8080,), daemon=True)
    http_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
