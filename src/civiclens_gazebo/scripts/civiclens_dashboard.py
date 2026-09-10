#!/usr/bin/env python3
"""
CivicLens AI — Real-Time Operations & Telemetry Web Dashboard
Hosts a glassmorphic, high-tech web interface on port 8080 with interactive 2D street map,
live OpenCV AI vision feeds with YOLO detections, 3-compartment truck payload meters,
plastic crusher status, and real-time mission telemetry.
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
        "plastic_count": 0
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
  <title>CivicLens AI — Autonomous Waste Management Telemetry Dashboard</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700;800&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-base: #060911;
      --bg-card: rgba(16, 23, 38, 0.75);
      --bg-card-border: rgba(255, 255, 255, 0.08);
      --bg-card-hover: rgba(23, 34, 56, 0.85);
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
      background: rgba(10, 14, 26, 0.85);
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
      width: 44px; height: 44px;
      background: linear-gradient(135deg, #00c6ff, #0072ff);
      border-radius: 12px;
      display: flex; align-items: center; justify-content: center;
      box-shadow: 0 0 20px rgba(0, 198, 255, 0.4);
      font-size: 22px;
    }
    .brand-title h1 { font-size: 20px; font-weight: 800; letter-spacing: 0.5px; }
    .brand-title span { color: var(--accent-cyan); font-weight: 400; font-size: 13px; letter-spacing: 1px; }
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
    
    .main-grid {
      display: grid;
      grid-template-columns: 1.15fr 0.85fr;
      gap: 20px;
      padding: 24px 28px;
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
      font-size: 15px;
      font-weight: 700;
      letter-spacing: 0.8px;
      text-transform: uppercase;
      color: var(--text-muted);
      margin-bottom: 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    /* Canvas Map */
    #cityCanvas {
      width: 100%;
      height: 380px;
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
      height: 280px;
      object-fit: cover;
      display: block;
    }
    .cam-tag {
      position: absolute;
      top: 12px; left: 12px;
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
      margin-top: 12px;
      flex-wrap: wrap;
    }
    .cam-btn {
      background: rgba(255,255,255,0.05);
      border: 1px solid var(--bg-card-border);
      color: var(--text-muted);
      padding: 6px 12px;
      border-radius: 8px;
      font-size: 12px;
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
      gap: 12px;
      margin-bottom: 20px;
    }
    .stat-pill {
      background: rgba(255,255,255,0.03);
      border: 1px solid var(--bg-card-border);
      border-radius: 12px;
      padding: 14px;
    }
    .stat-pill .label { font-size: 11px; color: var(--text-muted); text-transform: uppercase; font-weight: 600; }
    .stat-pill .value { font-size: 19px; font-weight: 800; margin-top: 4px; font-family: 'JetBrains Mono', monospace; }

    /* Compartments */
    .comp-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin-top: 12px;
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

    /* Event Log Terminal */
    .terminal {
      background: #080c14;
      border: 1px solid rgba(255,255,255,0.06);
      border-radius: 12px;
      padding: 14px;
      height: 170px;
      overflow-y: auto;
      font-family: 'JetBrains Mono', monospace;
      font-size: 11px;
      display: flex;
      flex-direction: column;
      gap: 6px;
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
      <div class="brand-logo">♻️</div>
      <div class="brand-title">
        <h1>CIVICLENS AI</h1>
        <span>AUTONOMOUS WASTE FLEET TELEMETRY &amp; SEGREGATION</span>
      </div>
    </div>
    <div class="header-badges">
      <div class="badge"><div class="pulse-dot"></div>FLEET 5G LIVE</div>
      <div class="badge" id="clock">00:00:00</div>
      <div class="badge" style="border-color: var(--accent-cyan); color: var(--accent-cyan);">ROS 2 + GAZEBO SIM</div>
    </div>
  </header>

  <main class="main-grid">
    <!-- LEFT COLUMN -->
    <div style="display: flex; flex-direction: column; gap: 20px;">
      
      <!-- Top Mission Progress Banner -->
      <div class="card">
        <div class="card-title">
          <span>Active Mission Round-Trip Sequence</span>
          <span id="roundBadge" style="color: var(--accent-cyan); font-family: 'JetBrains Mono'; font-weight: 700;">ROUND 1 / 5</span>
        </div>
        <div class="mission-banner">
          <div class="stat-pill">
            <div class="label">Target Street</div>
            <div class="value" id="curStreet" style="color: #fff; font-size: 15px;">Street 1: Greenfield</div>
          </div>
          <div class="stat-pill">
            <div class="label">Target Bin</div>
            <div class="value" id="curBin" style="color: var(--neon-plastic);">DB-01</div>
          </div>
          <div class="stat-pill">
            <div class="label">Truck Action</div>
            <div class="value" id="truckState" style="color: var(--neon-bio); font-size: 16px;">CRUISING</div>
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
          <span style="font-size: 12px; color: var(--text-muted); font-weight: 400;">50m x 50m Municipal Block</span>
        </div>
        <canvas id="cityCanvas" width="700" height="380"></canvas>
      </div>

      <!-- Fleet Analytics Summary -->
      <div class="card">
        <div class="card-title">Fleet Segregation &amp; Carbon Analytics</div>
        <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px;">
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
            <div class="label">CO₂ Footprint Reduced</div>
            <div class="value" id="co2Saved" style="color: var(--neon-nonbio);">0.0 kg</div>
          </div>
        </div>
      </div>

    </div>

    <!-- RIGHT COLUMN -->
    <div style="display: flex; flex-direction: column; gap: 20px;">

      <!-- Live AI Camera Feed with YOLO Detection -->
      <div class="card">
        <div class="card-title">
          <span>Solar-Powered Smart Bin AI Vision</span>
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

      <!-- Truck Segregated Compartments -->
      <div class="card">
        <div class="card-title">Autonomous Truck 3-Compartment Payload</div>
        <div class="comp-grid">
          
          <div class="comp-box bio">
            <div style="font-size: 11px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 1</div>
            <div style="font-size: 16px; font-weight: 800; color: var(--neon-bio); margin-top: 4px;">BIODEGRADABLE</div>
            <div style="font-size: 20px; font-family: 'JetBrains Mono'; margin-top: 6px;" id="bioKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="bioBar" style="width: 0%; background: var(--neon-bio);"></div></div>
          </div>

          <div class="comp-box nonbio">
            <div style="font-size: 11px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 2</div>
            <div style="font-size: 16px; font-weight: 800; color: var(--neon-nonbio); margin-top: 4px;">NON-BIODEGRADABLE</div>
            <div style="font-size: 20px; font-family: 'JetBrains Mono'; margin-top: 6px;" id="nonbioKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="nonbioBar" style="width: 0%; background: var(--neon-nonbio);"></div></div>
          </div>

          <div class="comp-box plastic">
            <div style="font-size: 11px; color: var(--text-muted); font-weight: 700;">COMPARTMENT 3 + CRUSHER</div>
            <div style="font-size: 16px; font-weight: 800; color: var(--neon-plastic); margin-top: 4px;">PLASTIC POLYMERS</div>
            <div style="font-size: 20px; font-family: 'JetBrains Mono'; margin-top: 6px;" id="plasticKg">0.0 kg</div>
            <div class="comp-meter"><div class="comp-bar" id="plasticBar" style="width: 0%; background: var(--neon-plastic);"></div></div>
          </div>

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

      // Coordinate Transform: City is approx -10 <= X <= 90, -15 <= Y <= 60
      // Map to canvas
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
      ctx.beginPath(); ctx.arc(basePos.x, basePos.y, 16, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px Outfit';
      ctx.textAlign = 'center';
      ctx.fillText('BASE', basePos.x, basePos.y + 4);

      // Draw Dustbins
      if (data.bins) {
        data.bins.forEach(b => {
          const bp = toScreen(b.x, b.y);
          ctx.beginPath();
          ctx.arc(bp.x, bp.y, 11, 0, Math.PI * 2);
          
          if (b.type === 'BIODEGRADABLE') ctx.fillStyle = '#00e676';
          else if (b.type === 'NON_BIODEGRADABLE') ctx.fillStyle = '#00b0ff';
          else ctx.fillStyle = '#ffd600';
          ctx.fill();

          ctx.strokeStyle = (b.id === activeBinId) ? '#fff' : 'rgba(255,255,255,0.4)';
          ctx.lineWidth = (b.id === activeBinId) ? 3 : 1.5;
          ctx.stroke();

          ctx.fillStyle = '#fff';
          ctx.font = 'bold 9px JetBrains Mono';
          ctx.fillText(b.code, bp.x, bp.y - 14);
        });
      }

      // Draw Autonomous Truck
      if (data.truck) {
        const tp = toScreen(data.truck.x, data.truck.y);
        ctx.save();
        ctx.translate(tp.x, tp.y);
        ctx.rotate(-data.truck.yaw + Math.PI/2); // Adjust orientation for screen coords

        // Truck chassis
        ctx.fillStyle = '#00f2fe';
        ctx.shadowColor = '#00f2fe';
        ctx.shadowBlur = 12;
        ctx.fillRect(-10, -18, 20, 36);

        // Cab
        ctx.fillStyle = '#111927';
        ctx.fillRect(-8, -16, 16, 10);

        // Heading arrow
        ctx.fillStyle = '#ffd600';
        ctx.beginPath();
        ctx.moveTo(0, -22); ctx.lineTo(-5, -16); ctx.lineTo(5, -16); ctx.closePath();
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
                except Exception as e:
                    self.send_response(500)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress noisy HTTP request logging in ROS terminal
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

        self.get_logger().info("Dashboard ROS 2 Subscriber Bridge Initialized.")

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
