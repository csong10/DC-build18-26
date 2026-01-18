#!/usr/bin/env python3
"""
RC Car "Phone Driver" - Autonomous Navigation
==============================================
The Phone is mounted ON the robot (Robot = Phone Position).
The Robot drives to the HARDCODED destination below.

Includes:
- Face Detection (Safety Stop)
- Obstacle Avoidance (Path of Least Resistance)
- GPS Navigation (Phone -> Hardcoded Point)
"""

import threading
import time
import socket
import math
import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# ===================== 📍 DESTINATION SETTINGS 📍 ===================== #
# --------------------------------------------------------------------- #
#       ENTER THE GPS COORDINATES OF YOUR TARGET DESTINATION BELOW      #
# --------------------------------------------------------------------- #

DESTINATION_LAT = 40.443322  # <--- REPLACE WITH TARGET LATITUDE
DESTINATION_LON = -79.943644 # <--- REPLACE WITH TARGET LONGITUDE

# --------------------------------------------------------------------- #

# Camera Config
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# ARDUINO CONNECTION Config
ARDUINO_PORT = '/dev/ttyACM0'  
ARDUINO_BAUDRATE = 9600

# Thresholds
DANGER_DISTANCE = 40.0        # Stop if face is closer than this
SAFE_DISTANCE = 100.0         # Avoid obstacles closer than this
OBSTACLE_AREA_THRESHOLD = 0.08
WAYPOINT_REACHED_DISTANCE = 3.0 # Meters
HEADING_TOLERANCE = 15.0        # Degrees

# GPS Config
GPS_UDP_IP = "0.0.0.0"
GPS_UDP_PORT = 11123
GPS_TIMEOUT = 5
GPS_ENABLED = True

# ===================== MATH FUNCTIONS ===================== #

def parse_nmea_coordinate(coord_str, direction):
    if not coord_str: return None
    try:
        coord = float(coord_str)
        deg = int(coord / 100)
        mins = coord - (deg * 100)
        val = deg + (mins / 60.0)
        if direction in ['S', 'W']: val = -val
        return val
    except: return None

def parse_gprmc(line):
    try:
        parts = line.split(',')
        if len(parts) < 8 or parts[2] != 'A': return None
        lat = parse_nmea_coordinate(parts[3], parts[4])
        lon = parse_nmea_coordinate(parts[5], parts[6])
        course = float(parts[8]) if parts[8] else 0.0
        if lat and lon: return {'lat': lat, 'lon': lon, 'course': course}
    except: pass
    return None

def parse_gpgga(line):
    try:
        parts = line.split(',')
        if len(parts) < 10 or int(parts[6] or 0) == 0: return None
        lat = parse_nmea_coordinate(parts[2], parts[3])
        lon = parse_nmea_coordinate(parts[4], parts[5])
        if lat and lon: return {'lat': lat, 'lon': lon}
    except: pass
    return None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000 # Earth radius in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def calc_bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def bearing_to_turn(current, target):
    diff = (target - current + 360) % 360
    if diff < HEADING_TOLERANCE or diff > (360 - HEADING_TOLERANCE): return 'forward'
    return 'right' if diff <= 180 else 'left'

# ===================== MAIN CONTROLLER ===================== #

class RCCarController:
    def __init__(self):
        # State
        self.robot_lat = None
        self.robot_lon = None
        self.robot_heading = 0.0
        self.dist_to_dest = None
        self.bearing_to_dest = None
        
        self.face_dist = None
        self.obstacle_detected = False
        self.clearer_path = "equal"
        
        self.running = True
        self.current_mode = "STARTING"
        self.data_lock = threading.Lock()
        
        # Hardware
        self.arduino = None
        self.picam2 = None
        self.gps_socket = None
        self.face_cascade = None
        
        self.CMD_MAP = {"forward": b'F', "backward": b'B', "left": b'L', "right": b'R', "stop": b'S'}

    def initialize(self):
        # Arduino
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
            time.sleep(2)
            print("[INIT] ✓ Arduino Connected")
        except: print("[INIT] ✗ Arduino Failed"); return False

        # Camera
        try:
            self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()
            print("[INIT] ✓ Camera Ready")
        except: print("[INIT] ✗ Camera Failed"); return False

        # GPS
        try:
            self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gps_socket.bind((GPS_UDP_IP, GPS_UDP_PORT))
            self.gps_socket.settimeout(GPS_TIMEOUT)
            print("[INIT] ✓ GPS Listening")
        except: print("[INIT] ✗ GPS Failed"); return False
        
        return True

    def gps_thread(self):
        while self.running:
            try:
                data, _ = self.gps_socket.recvfrom(1024)
                line = data.decode('utf-8').strip()
                
                res = None
                if '$GPRMC' in line:
                    res = parse_gprmc(line)
                elif '$GPGGA' in line:
                    res = parse_gpgga(line)
                
                if res:
                    # Calculate navigation FROM Robot (Phone) TO Destination
                    d = haversine(res['lat'], res['lon'], DESTINATION_LAT, DESTINATION_LON)
                    b = calc_bearing(res['lat'], res['lon'], DESTINATION_LAT, DESTINATION_LON)
                    
                    with self.data_lock:
                        self.robot_lat = res['lat']
                        self.robot_lon = res['lon']
                        
                        # Only update heading if we have speed/course info (GPRMC)
                        if 'course' in res and res['course'] > 0: 
                            self.robot_heading = res['course']
                            
                        self.dist_to_dest = d
                        self.bearing_to_dest = b
            except: pass

    def camera_thread(self):
        while self.running:
            try:
                frame = self.picam2.capture_array()
                if frame is None: continue
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                # Face Detection
                f_dist = None
                if self.face_cascade:
                    faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
                    if len(faces) > 0:
                        w = faces[0][2]
                        f_dist = (14.0 * 500) / w # Approx focal calc
                
                # Obstacle Detection
                edges = cv2.Canny(cv2.GaussianBlur(gray, (5,5), 0), 50, 150)
                roi = edges[160:, :] # Bottom 2/3 of screen
                h, w = roi.shape
                
                left_sum = np.sum(roi[:, :w//2])
                right_sum = np.sum(roi[:, w//2:])
                clr = 'left' if left_sum < right_sum else 'right'
                obs = (np.sum(roi > 0) / (w*h)) > OBSTACLE_AREA_THRESHOLD
                
                with self.data_lock:
                    self.face_dist = f_dist
                    self.obstacle_detected = obs
                    self.clearer_path = clr
                time.sleep(0.05)
            except: pass

    def motor_thread(self):
        last_cmd = None
        while self.running:
            with self.data_lock:
                f_dist = self.face_dist
                obs = self.obstacle_detected
                clr = self.clearer_path
                dist = self.dist_to_dest
                bear = self.bearing_to_dest
                head = self.robot_heading
            
            cmd = "stop"
            
            # PRIORITY 1: Safety (Face)
            if f_dist and f_dist < DANGER_DISTANCE:
                self.current_mode = "STOP (Face)"
                cmd = "stop"
            
            # PRIORITY 2: Obstacle Avoidance
            elif obs:
                self.current_mode = f"AVOID ({clr})"
                cmd = clr # 'left' or 'right'
            
            # PRIORITY 3: GPS Navigation
            elif dist is not None:
                if dist < WAYPOINT_REACHED_DISTANCE:
                    self.current_mode = "ARRIVED"
                    cmd = "stop"
                else:
                    turn = bearing_to_turn(head, bear)
                    self.current_mode = f"NAV: {turn} ({dist:.1f}m)"
                    cmd = turn
            
            # PRIORITY 4: No GPS Signal
            else:
                self.current_mode = "WAITING GPS"
                cmd = "stop"

            if cmd != last_cmd:
                self.arduino.write(self.CMD_MAP.get(cmd, b'S'))
                print(f"[MOTOR] {cmd.upper()} | {self.current_mode}")
                last_cmd = cmd
            time.sleep(0.1)
            
    def status_thread(self):
        """Prints status updates every 2 seconds"""
        while self.running:
            with self.data_lock:
                lat = self.robot_lat if self.robot_lat else 0.0
                lon = self.robot_lon if self.robot_lon else 0.0
                print(f"[STATUS] {self.current_mode} | Robot: {lat:.6f},{lon:.6f} | Hdg: {self.robot_heading:.1f}")
            time.sleep(2.0)

    def start(self):
        if not self.initialize(): return
        
        # Start all threads
        threading.Thread(target=self.camera_thread, daemon=True).start()
        threading.Thread(target=self.gps_thread, daemon=True).start()
        threading.Thread(target=self.motor_thread, daemon=True).start()
        threading.Thread(target=self.status_thread, daemon=True).start()
        
        print("\n" + "="*50)
        print(" RC CAR AUTONOMOUS DRIVER (PHONE MODE)")
        print(f" DESTINATION: {DESTINATION_LAT}, {DESTINATION_LON}")
        print(" CTRL+C to Stop")
        print("="*50 + "\n")
        
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            self.arduino.write(b'S')
            self.arduino.close()
            self.picam2.stop()
            self.gps_socket.close()
            print("\n[MAIN] System Stopped.")

if __name__ == "__main__":
    RCCarController().start()
