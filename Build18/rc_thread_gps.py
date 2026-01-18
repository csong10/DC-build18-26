#!/usr/bin/env python3
"""
RC Car Threaded Controller - Camera + GPS Navigation
=====================================================

This script controls an RC car using:
- GPS navigation to phone location (via GPS2IP app)
- Camera vision for obstacle avoidance
- Path of least resistance turning logic
- Motor control via Arduino USB Serial

Run with: sudo python3 rc_car_threaded.py

GPS2IP Setup (iPhone):
1. Install GPS2IP app
2. Settings: UDP Push, IP = Pi's IP, Port = 11123
3. Enable GGA and RMC messages
4. Enable Background Mode
5. Start streaming
"""

import threading
import time
import socket
import math
import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# ===================== CONFIGURATION ===================== #

# Camera Config
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# ARDUINO CONNECTION Config
ARDUINO_PORT = '/dev/ttyACM0'  
ARDUINO_BAUDRATE = 9600

# Decision Thresholds
DANGER_DISTANCE = 40.0        # Stop if obstacle closer than this (cm)
SAFE_DISTANCE = 100.0         # Slow down under this distance

# Obstacle Avoidance Config
OBSTACLE_AREA_THRESHOLD = 0.08
TURN_DURATION = 0.5

# GPS Config (for GPS2IP app)
GPS_UDP_IP = "0.0.0.0"
GPS_UDP_PORT = 11123
GPS_TIMEOUT = 5

# Navigation Config
WAYPOINT_REACHED_DISTANCE = 3.0   # meters
HEADING_TOLERANCE = 15.0          # degrees

# Robot starting position (update to your location)
ROBOT_START_LAT = 40.4433
ROBOT_START_LON = -79.9436

# Enable/Disable GPS navigation
GPS_ENABLED = True

# ===================== GPS FUNCTIONS ===================== #

def parse_nmea_coordinate(coord_str, direction):
    """Parse NMEA coordinate to decimal degrees."""
    if not coord_str:
        return None
    try:
        coord = float(coord_str)
        degrees = int(coord / 100)
        minutes = coord - (degrees * 100)
        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal
    except:
        return None

def parse_gprmc(sentence):
    """Parse $GPRMC sentence."""
    try:
        parts = sentence.split(',')
        if len(parts) < 8 or parts[2] != 'A':
            return None
        
        lat = parse_nmea_coordinate(parts[3], parts[4])
        lon = parse_nmea_coordinate(parts[5], parts[6])
        
        if lat and lon:
            return {'lat': lat, 'lon': lon, 'valid': True}
    except:
        pass
    return None

def parse_gpgga(sentence):
    """Parse $GPGGA sentence."""
    try:
        parts = sentence.split(',')
        if len(parts) < 10 or int(parts[6] or 0) == 0:
            return None
        
        lat = parse_nmea_coordinate(parts[2], parts[3])
        lon = parse_nmea_coordinate(parts[4], parts[5])
        
        if lat and lon:
            return {'lat': lat, 'lon': lon, 'valid': True}
    except:
        pass
    return None

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS points in meters."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing from point 1 to point 2 (0-360, 0=North)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    
    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lambda)
    
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

def bearing_to_turn(current_heading, target_bearing):
    """Determine turn direction to reach target bearing."""
    diff = (target_bearing - current_heading + 360) % 360
    
    if diff < HEADING_TOLERANCE or diff > (360 - HEADING_TOLERANCE):
        return 'straight', diff
    elif diff <= 180:
        return 'right', diff
    else:
        return 'left', 360 - diff

# ===================== CAMERA FUNCTIONS ===================== #

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0:
        return float('inf')
    return (known_width * focal_length) / pixel_width

def analyze_path_clearance(edges, width, height):
    """Analyze which side has fewer obstacles."""
    mid = width // 2
    left_edges = np.sum(edges[:, :mid] > 0)
    right_edges = np.sum(edges[:, mid:] > 0)
    
    total = left_edges + right_edges
    if total == 0:
        return 'equal', left_edges, right_edges
    
    diff_ratio = abs(left_edges - right_edges) / total
    
    if diff_ratio < 0.1:
        return 'equal', left_edges, right_edges
    elif left_edges < right_edges:
        return 'left', left_edges, right_edges
    else:
        return 'right', left_edges, right_edges

# ===================== MAIN CONTROLLER ===================== #

class RCCarController:
    def __init__(self):
        # Camera data
        self.face_distance = None
        self.face_position = None
        self.obstacle_detected = False
        self.obstacle_distance = 100.0
        self.clearer_path = "equal"
        
        # GPS data
        self.goal_lat = None
        self.goal_lon = None
        self.goal_distance = None
        self.goal_bearing = None
        self.robot_lat = ROBOT_START_LAT
        self.robot_lon = ROBOT_START_LON
        self.robot_heading = 0.0  # Assume facing North initially
        
        self.running = True
        self.current_mode = "STARTING"
        
        # Locks
        self.data_lock = threading.Lock()
        
        # Hardware
        self.arduino = None
        self.picam2 = None
        self.face_cascade = None
        self.focal_length = None
        self.gps_socket = None
        
        self.CMD_MAP = {
            "forward": b'F',
            "backward": b'B',
            "left": b'L',
            "right": b'R',
            "stop": b'S',
        }

    def initialize(self):
        print("[INIT] Starting initialization...")
        
        # Arduino
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
            time.sleep(2)
            print(f"[INIT] ✓ Arduino on {ARDUINO_PORT}")
        except Exception as e:
            print(f"[INIT] ✗ Arduino failed: {e}")
            return False
        
        # Camera
        try:
            self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            if self.face_cascade.empty():
                print("[INIT] ⚠ Haar cascade not found")
                self.face_cascade = None
            
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)
            
            self.focal_length = calculate_focal_length(
                CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH
            )
            print("[INIT] ✓ Camera ready")
        except Exception as e:
            print(f"[INIT] ✗ Camera failed: {e}")
            return False
        
        # GPS
        if GPS_ENABLED:
            try:
                self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.gps_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.gps_socket.bind((GPS_UDP_IP, GPS_UDP_PORT))
                self.gps_socket.settimeout(GPS_TIMEOUT)
                print(f"[INIT] ✓ GPS listening on port {GPS_UDP_PORT}")
            except Exception as e:
                print(f"[INIT] ⚠ GPS failed: {e} - running without GPS")
                self.gps_socket = None
        else:
            print("[INIT] GPS disabled")
        
        print("[INIT] ✓ All systems ready!")
        return True

    def get_best_turn(self, obstacle_pos, clearer_path):
        """Get best turn direction using path of least resistance."""
        if clearer_path == 'left':
            return 'left'
        elif clearer_path == 'right':
            return 'right'
        
        if obstacle_pos == 'left':
            return 'right'
        elif obstacle_pos == 'right':
            return 'left'
        
        return 'right'

    def camera_thread(self):
        print("[CAM] Thread started")
        
        while self.running:
            try:
                if self.picam2 is None:
                    time.sleep(1)
                    continue
                
                frame = self.picam2.capture_array()
                if frame is None:
                    continue
                
                height, width = frame.shape[:2]
                frame_area = height * width
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                # Face detection
                face_dist = None
                face_pos = None
                
                if self.face_cascade is not None:
                    faces = self.face_cascade.detectMultiScale(
                        gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)
                    )
                    
                    if len(faces) > 0:
                        faces = sorted(faces, key=lambda x: x[2], reverse=True)
                        x, y, w, h = faces[0]
                        face_dist = distance_to_camera(FACE_WIDTH_CM, self.focal_length, w)
                        face_x = x + w // 2
                        
                        if face_x < width // 3:
                            face_pos = 'left'
                        elif face_x > 2 * width // 3:
                            face_pos = 'right'
                        else:
                            face_pos = 'center'
                
                # Edge detection for obstacles
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blurred, 50, 150)
                roi = edges[height//3:, :]
                
                clearer_path, _, _ = analyze_path_clearance(roi, width, height - height//3)
                
                # Obstacle detection
                obstacle_detected = False
                obstacle_dist = 100.0
                
                if face_dist is None:
                    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        area_ratio = area / frame_area
                        if area_ratio > OBSTACLE_AREA_THRESHOLD:
                            obstacle_detected = True
                            obstacle_dist = max(0, 100 - (area_ratio * 500))
                            break
                
                with self.data_lock:
                    self.face_distance = face_dist
                    self.face_position = face_pos
                    self.obstacle_detected = obstacle_detected
                    self.obstacle_distance = obstacle_dist
                    self.clearer_path = clearer_path
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"[CAM] Error: {e}")
                time.sleep(0.5)
        
        print("[CAM] Thread stopped")

    def gps_thread(self):
        if self.gps_socket is None:
            print("[GPS] No socket - thread exiting")
            return
        
        print("[GPS] Thread started - waiting for phone location...")
        
        while self.running:
            try:
                data, addr = self.gps_socket.recvfrom(1024)
                line = data.decode('utf-8').strip()
                
                result = None
                if '$GPRMC' in line or '$GNRMC' in line:
                    result = parse_gprmc(line)
                elif '$GPGGA' in line or '$GNGGA' in line:
                    result = parse_gpgga(line)
                
                if result:
                    goal_lat = result['lat']
                    goal_lon = result['lon']
                    
                    distance = haversine_distance(
                        self.robot_lat, self.robot_lon, goal_lat, goal_lon
                    )
                    bearing = calculate_bearing(
                        self.robot_lat, self.robot_lon, goal_lat, goal_lon
                    )
                    
                    with self.data_lock:
                        self.goal_lat = goal_lat
                        self.goal_lon = goal_lon
                        self.goal_distance = distance
                        self.goal_bearing = bearing
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[GPS] Error: {e}")
                time.sleep(1)
        
        print("[GPS] Thread stopped")

    def motor_thread(self):
        print("[MOTOR] Thread started")
        last_command = None
        avoiding_since = None
        
        while self.running:
            with self.data_lock:
                face_dist = self.face_distance
                face_pos = self.face_position
                obstacle = self.obstacle_detected
                obstacle_dist = self.obstacle_distance
                clearer = self.clearer_path
                goal_dist = self.goal_distance
                goal_bearing = self.goal_bearing
            
            command = "forward"
            
            # PRIORITY 1: Face too close - STOP
            if face_dist is not None and face_dist < DANGER_DISTANCE:
                self.current_mode = "STOPPED_FACE"
                command = "stop"
                avoiding_since = None
            
            # PRIORITY 2: Face or obstacle - AVOID
            elif (face_dist is not None and face_dist < SAFE_DISTANCE) or \
                 (obstacle and obstacle_dist < 30):
                self.current_mode = "AVOIDING"
                
                if avoiding_since is None:
                    avoiding_since = time.time()
                
                if time.time() - avoiding_since < TURN_DURATION * 2:
                    turn_dir = self.get_best_turn(face_pos or 'center', clearer)
                    command = turn_dir
                elif time.time() - avoiding_since < TURN_DURATION * 4:
                    command = "backward"
                else:
                    avoiding_since = None
            
            # PRIORITY 3: Navigate to GPS goal
            elif goal_dist is not None and goal_bearing is not None:
                avoiding_since = None
                
                if goal_dist < WAYPOINT_REACHED_DISTANCE:
                    self.current_mode = "GOAL_REACHED"
                    command = "stop"
                else:
                    turn_dir, turn_angle = bearing_to_turn(self.robot_heading, goal_bearing)
                    
                    if turn_dir == 'straight' or turn_angle < HEADING_TOLERANCE:
                        self.current_mode = "NAVIGATING"
                        command = "forward"
                    else:
                        self.current_mode = "TURNING"
                        command = turn_dir
            
            # PRIORITY 4: No GPS - cautious forward
            elif obstacle and obstacle_dist < 60:
                self.current_mode = "CAUTIOUS"
                avoiding_since = None
                command = "forward"
            
            # PRIORITY 5: Clear path
            else:
                self.current_mode = "FORWARD"
                avoiding_since = None
                command = "forward"
            
            if command != last_command:
                self.send_command(command)
                last_command = command
            
            time.sleep(0.1)
        
        self.send_command("stop")
        print("[MOTOR] Thread stopped")

    def send_command(self, cmd_str):
        if self.arduino and self.arduino.is_open and cmd_str in self.CMD_MAP:
            self.arduino.write(self.CMD_MAP[cmd_str])
            print(f"[MOTOR] {cmd_str.upper()} | Mode: {self.current_mode}")

    def status_thread(self):
        print("[STATUS] Thread started")
        
        while self.running:
            with self.data_lock:
                face_dist = self.face_distance
                obstacle = self.obstacle_detected
                obstacle_dist = self.obstacle_distance
                clearer = self.clearer_path
                goal_dist = self.goal_distance
                goal_bearing = self.goal_bearing
                goal_lat = self.goal_lat
                goal_lon = self.goal_lon
            
            parts = [f"Mode: {self.current_mode}"]
            
            if face_dist:
                parts.append(f"Face: {face_dist:.1f}cm")
            if obstacle:
                parts.append(f"Obstacle: {obstacle_dist:.0f}")
            if goal_dist and goal_lat and goal_lon:
                parts.append(f"Phone: [{goal_lat:.5f}, {goal_lon:.5f}]")
                parts.append(f"Goal: {goal_dist:.1f}m @ {goal_bearing:.0f}°")
            
            parts.append(f"Clearer: {clearer}")
            
            print(f"[STATUS] {' | '.join(parts)}")
            time.sleep(2)
        
        print("[STATUS] Thread stopped")

    def start(self):
        if not self.initialize():
            print("[ERROR] Initialization failed")
            return
        
        threads = [
            threading.Thread(target=self.camera_thread, daemon=True, name="Camera"),
            threading.Thread(target=self.motor_thread, daemon=True, name="Motor"),
            threading.Thread(target=self.status_thread, daemon=True, name="Status"),
        ]
        
        if GPS_ENABLED and self.gps_socket:
            threads.append(threading.Thread(target=self.gps_thread, daemon=True, name="GPS"))
        
        for t in threads:
            t.start()
        
        print("\n" + "="*50)
        print("RC CAR - GPS NAVIGATION + OBSTACLE AVOIDANCE")
        print("="*50)
        print(f"GPS: {'Enabled' if GPS_ENABLED else 'Disabled'}")
        print(f"Robot position: {self.robot_lat:.6f}, {self.robot_lon:.6f}")
        print("Press Ctrl+C to stop")
        print("="*50 + "\n")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[MAIN] Shutting down...")
            self.stop()

    def stop(self):
        self.running = False
        time.sleep(0.5)
        
        if self.arduino:
            self.arduino.write(b'S')
            self.arduino.close()
        
        if self.picam2:
            self.picam2.stop()
        
        if self.gps_socket:
            self.gps_socket.close()
        
        print("[MAIN] Stopped")


if __name__ == "__main__":
    controller = RCCarController()
    controller.start()
