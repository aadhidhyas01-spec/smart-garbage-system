#!/usr/bin/env python3
"""
CivicLens AI — Smart Dustbin AI Vision & IoT Telemetry Station
Simulates the 5 solar-powered AI camera stations with real-time OpenCV
YOLO-style object detection, fill-level monitoring (>90% trigger), and IoT JSON alerting.
"""

import os
import sys
import time
import json
import datetime
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CAMERA_FRAME_DIR = "/tmp/civiclens_cameras"
os.makedirs(CAMERA_FRAME_DIR, exist_ok=True)

class SmartDustbinStation:
    def __init__(self, bin_id, street_name, waste_type, fill_level, gps_lat, gps_lon):
        self.bin_id = bin_id
        self.code = f"DB-{bin_id:02d}"
        self.street_name = street_name
        self.waste_type = waste_type
        self.fill_level = fill_level
        self.gps_lat = gps_lat
        self.gps_lon = gps_lon
        self.status = "FULL" if fill_level >= 90.0 else "NORMAL"
        self.priority = "HIGH" if fill_level >= 90.0 else "LOW"
        self.solar_power = 98.5
        self.serviced = False
        self.last_alert_time = 0.0

    def service_bin(self):
        self.serviced = True
        self.fill_level = 5.0
        self.status = "SERVICED"
        self.priority = "NORMAL"

class CivicLensAIVision(Node):
    def __init__(self):
        super().__init__('civiclens_ai_vision')

        self.alert_pub = self.create_publisher(String, '/civiclens/iot_alerts', 10)
        self.vision_status_pub = self.create_publisher(String, '/civiclens/vision_status', 10)

        # Listen to truck commands to detect when a bin has been serviced
        self.cmd_sub = self.create_subscription(
            String, '/civiclens/truck_command', self.on_truck_command, 10)
        self.state_sub = self.create_subscription(
            String, '/civiclens/truck_state', self.on_truck_state, 10)
        self.telemetry_sub = self.create_subscription(
            String, '/civiclens/truck_telemetry', self.on_truck_telemetry, 10)

        # Initialize the 5 Smart Dustbins
        self.stations = {
            1: SmartDustbinStation(1, "Greenfield Road (Street 1)", "BIODEGRADABLE", 94.5, 12.9716, 77.5946),
            2: SmartDustbinStation(2, "Market Lane (Street 2)", "NON_BIODEGRADABLE", 96.2, 12.9725, 77.5958),
            3: SmartDustbinStation(3, "Heritage Avenue (Street 3)", "PLASTIC", 98.4, 12.9734, 77.5971),
            4: SmartDustbinStation(4, "Sunrise Boulevard (Street 4)", "BIODEGRADABLE", 92.8, 12.9742, 77.5985),
            5: SmartDustbinStation(5, "Tech Park Way (Street 5)", "PLASTIC", 95.7, 12.9750, 77.5999),
        }

        self.active_display_bin = 1
        self.current_truck_action = "IDLE"
        self.active_truck_bin = 1

        # Periodic vision processing & IoT broadcast timer (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("CivicLens AI Vision & IoT Station Node Initialized for 5 Smart Dustbins.")

    def on_truck_command(self, msg: String):
        tokens = msg.data.split()
        if not tokens:
            return
        if tokens[0] == "PICKUP" and len(tokens) >= 2:
            try:
                bin_id = int(tokens[1])
                self.active_truck_bin = bin_id
                self.active_display_bin = bin_id
                self.get_logger().info(f"Truck initiating pickup at Smart Dustbin DB-{bin_id:02d}")
            except ValueError:
                pass

    def on_truck_state(self, msg: String):
        pass

    def on_truck_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_truck_action = data.get("state", "IDLE")
            bid = data.get("bin_id", 0)
            if bid in self.stations:
                self.active_display_bin = bid
                # If truck completed pickup or arm dropped waste, mark bin as serviced
                if self.current_truck_action in ["PICKING", "IDLE"] and data.get("arm_step", 0) >= 4:
                    if not self.stations[bid].serviced:
                        self.stations[bid].service_bin()
                        self.get_logger().info(f"DB-{bid:02d} fill level reset to 5% after autonomous robotic pickup!")
        except Exception:
            pass

    def timer_callback(self):
        now = time.time()

        # 1. Publish IoT alerts for overflowing bins
        alert_payloads = []
        for bid, st in self.stations.items():
            alert_payloads.append({
                "bin_id": bid,
                "code": st.code,
                "street": st.street_name,
                "waste_type": st.waste_type,
                "fill_level": round(st.fill_level, 1),
                "status": st.status,
                "priority": st.priority,
                "solar_power": st.solar_power,
                "lat": st.gps_lat,
                "lon": st.gps_lon,
                "serviced": st.serviced
            })

            if st.status == "FULL" and (now - st.last_alert_time > 3.0):
                st.last_alert_time = now
                alert_msg = String()
                alert_msg.data = json.dumps({
                    "alert_type": "OVERFLOW_DETECTED",
                    "bin_id": bid,
                    "code": st.code,
                    "street": st.street_name,
                    "fill_level": round(st.fill_level, 1),
                    "waste_type": st.waste_type,
                    "priority": "HIGH",
                    "timestamp": datetime.datetime.now().isoformat()
                })
                self.alert_pub.publish(alert_msg)

        # Publish consolidated status for dashboard
        stat_msg = String()
        stat_msg.data = json.dumps(alert_payloads)
        self.vision_status_pub.publish(stat_msg)

        # 2. Render Synthetic OpenCV YOLO Vision Feeds for each station
        for bid, st in self.stations.items():
            frame = self.generate_synthetic_camera_feed(st)
            file_path = os.path.join(CAMERA_FRAME_DIR, f"cam_db_{bid}.jpg")
            cv2.imwrite(file_path, frame)
            if bid == self.active_display_bin:
                cv2.imwrite(os.path.join(CAMERA_FRAME_DIR, "cam_latest.jpg"), frame)

    def generate_synthetic_camera_feed(self, st: SmartDustbinStation) -> np.ndarray:
        """
        Synthesizes a realistic 640x480 camera view looking down at the smart dustbin
        with simulated waste objects, OpenCV YOLO bounding boxes, and IoT HUD.
        """
        width, height = 640, 480
        # Background: Concrete sidewalk texture
        frame = np.full((height, width, 3), (45, 45, 48), dtype=np.uint8)

        # Add asphalt / road border on the left
        cv2.rectangle(frame, (0, 0), (120, height), (32, 32, 34), -1)
        cv2.line(frame, (120, 0), (120, height), (220, 220, 220), 2) # Curb line

        # Dustbin Top Rim Perspective (Dark green plastic rim)
        bin_center = (380, 260)
        cv2.ellipse(frame, bin_center, (170, 130), 0, 0, 360, (25, 80, 30), -1)
        cv2.ellipse(frame, bin_center, (155, 115), 0, 0, 360, (15, 45, 18), -1)

        # Render waste contents inside bin depending on fill level
        if not st.serviced:
            # Bulging garbage bag visual
            cv2.ellipse(frame, (375, 255), (140, 100), 0, 0, 360, (28, 28, 30), -1)
            cv2.circle(frame, (350, 240), 45, (38, 38, 40), -1)
            cv2.circle(frame, (410, 260), 50, (32, 32, 35), -1)
            cv2.circle(frame, (370, 280), 40, (22, 22, 24), -1)

            # Bag tie tape
            cv2.ellipse(frame, (375, 230), (35, 12), -15, 0, 360, (20, 180, 220), -1)

            # Simulated YOLO Detection Bounding Box
            box_x1, box_y1, box_x2, box_y2 = 230, 140, 520, 380
            
            if st.waste_type == "BIODEGRADABLE":
                box_color = (0, 220, 60)      # Neon Green
                label = f"BIODEGRADABLE [{st.fill_level:.1f}%]"
                sub_label = "Organic / Food Refuse"
            elif st.waste_type == "NON_BIODEGRADABLE":
                box_color = (240, 140, 20)     # Neon Blue/Cyan
                label = f"NON-BIODEGRADABLE [{st.fill_level:.1f}%]"
                sub_label = "Dry Recyclable / Metal / Glass"
            else: # PLASTIC
                box_color = (20, 210, 255)     # Vibrant Yellow
                label = f"PLASTIC [{st.fill_level:.1f}%] -> CRUSHER REQ"
                sub_label = "Polymers / PET Containers"

            # Draw glowing bounding box
            cv2.rectangle(frame, (box_x1, box_y1), (box_x2, box_y2), box_color, 2)
            # Corner accents
            c_len = 16
            for (cx, cy, dx, dy) in [
                (box_x1, box_y1, 1, 1), (box_x2, box_y1, -1, 1),
                (box_x1, box_y2, 1, -1), (box_x2, box_y2, -1, -1)
            ]:
                cv2.line(frame, (cx, cy), (cx + dx * c_len, cy), box_color, 4)
                cv2.line(frame, (cx, cy), (cx, cy + dy * c_len), box_color, 4)

            # Label banner
            cv2.rectangle(frame, (box_x1, box_y1 - 32), (box_x1 + 310, box_y1), box_color, -1)
            cv2.putText(frame, label, (box_x1 + 6, box_y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, sub_label, (box_x1, box_y2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)
        else:
            # Empty / Serviced Bin
            cv2.putText(frame, "[ DUSTBIN EMPTIED & SERVICED ]", (220, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 240, 120), 2, cv2.LINE_AA)

        # ================= AI HUD OVERLAY =================
        # Top Header Glassmorphic Bar
        cv2.rectangle(frame, (0, 0), (width, 50), (18, 20, 24), -1)
        cv2.line(frame, (0, 50), (width, 50), (60, 65, 75), 1)

        # Branding
        cv2.putText(frame, "CIVICLENS AI VISION", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"CAM: {st.code} | {st.street_name}", (14, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 190, 200), 1, cv2.LINE_AA)

        # Solar & IoT Badge
        cv2.circle(frame, (width - 155, 25), 5, (0, 230, 0), -1)
        cv2.putText(frame, f"SOLAR: {st.solar_power}%", (width - 142, 29),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "IoT 5G: LINKED", (width - 260, 29),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (100, 200, 255), 1, cv2.LINE_AA)

        # Bottom Telemetry Bar
        cv2.rectangle(frame, (0, height - 60), (width, height), (18, 20, 24), -1)
        cv2.line(frame, (0, height - 60), (width, height - 60), (60, 65, 75), 1)

        # Fill Gauge Bar
        gauge_x, gauge_y, gauge_w, gauge_h = 14, height - 42, 220, 18
        cv2.rectangle(frame, (gauge_x, gauge_y), (gauge_x + gauge_w, gauge_y + gauge_h), (35, 40, 45), -1)
        cv2.rectangle(frame, (gauge_x, gauge_y), (gauge_x + gauge_w, gauge_y + gauge_h), (80, 85, 95), 1)

        fill_pixels = int((st.fill_level / 100.0) * gauge_w)
        gauge_color = (0, 0, 230) if st.fill_level >= 90.0 else (0, 210, 80)
        cv2.rectangle(frame, (gauge_x + 1, gauge_y + 1), (gauge_x + fill_pixels, gauge_y + gauge_h - 1), gauge_color, -1)
        cv2.putText(frame, f"FILL: {st.fill_level:.1f}% ({st.status})", (gauge_x + 10, gauge_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        # GPS Coordinates & Timestamp
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, f"GPS: {st.gps_lat:.4f} N, {st.gps_lon:.4f} E | {now_str}", (260, height - 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 180, 190), 1, cv2.LINE_AA)
        
        # Priority Alert Status
        if st.status == "FULL":
            cv2.putText(frame, "DISPATCH ALERT: SENT", (260, height - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (50, 80, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "STATUS: ALL CLEAR", (260, height - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 120), 1, cv2.LINE_AA)

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = CivicLensAIVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
