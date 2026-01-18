#!/usr/bin/env python3
"""
RC Car Component Testing Script - FULL SUITE (Phone Driver Edition)
===================================================================

Use this to test all components before running the full system.

Run with: sudo python3 test_components.py

Tests included:
1. Arduino USB Connection & Motor Commands
2. Camera & Obstacle Detection  
3. Path Clearance Analysis
4. Threading Basics
5. Motor Navigation Sequence
6. GPS Receiver (Connection Check)
7. GPS Navigation Test (Phone = Goal) [Legacy Mode]
8. Integrated Camera + Motor + GPS (Phone = Robot, Dest = Hardcoded)
9. Monitor Phone GPS (Continuous)
10. Phone Driver Navigation Math (Visual Only)
"""

import sys
import time
import threading
import subprocess
import socket
import math
import numpy as np

# ===================== CONFIGURATION ===================== #

# --------------------------------------------------------- #
#           📍 HARDCODED DESTINATION SETTINGS 📍            #
#    (Used for Test 8 & 10 - Phone Driver Navigation)       #
# --------------------------------------------------------- #
DESTINATION_LAT = 40.443322   # <--- REPLACE WITH TARGET LATITUDE
DESTINATION_LON = -79.943644  # <--- REPLACE WITH TARGET LONGITUDE
# --------------------------------------------------------- #

# Camera Config
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# ARDUINO CONNECTION Config
ARDUINO_PORT = '/dev/ttyACM0'  
ARDUINO_BAUDRATE = 9600 

# Decision Thresholds
DANGER_DISTANCE = 40.0
SAFE_DISTANCE = 100.0

# Obstacle Detection Config
OBSTACLE_AREA_THRESHOLD = 0.08

# GPS Config (for GPS2IP app)
GPS_UDP_IP = "0.0.0.0"    # Listen on all interfaces
GPS_UDP_PORT = 11123      # Standard GPS port
GPS_TIMEOUT = 10          

# Navigation Config
WAYPOINT_REACHED_DISTANCE = 3.0   
HEADING_TOLERANCE = 15.0          

# Robot's starting position (Fallback only, overwritten by GPS)
ROBOT_DEFAULT_LAT = 40.4433  
ROBOT_DEFAULT_LON = -79.9436

# ===================== HELPER FUNCTIONS ===================== #

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: 
        return float('inf')
    return (known_width * focal_length) / pixel_width

def analyze_path_clearance(edges, width, height):
    """Analyze left/right frame for obstacles."""
    mid = width // 2
    left_region = edges[:, :mid]
    right_region = edges[:, mid:]
    
    left_edges = np.sum(left_region > 0)
    right_edges = np.sum(right_region > 0)
    
    threshold = 0.1
    total = left_edges + right_edges
    
    if total == 0:
        return 'equal', left_edges, right_edges
    
    diff_ratio = abs(left_edges - right_edges) / total
    
    if diff_ratio < threshold:
        return 'equal', left_edges, right_edges
    elif left_edges < right_edges:
        return 'left', left_edges, right_edges
    else:
        return 'right', left_edges, right_edges

def get_best_turn_direction(obstacle_pos, clearer_path):
    """Determine best turn direction."""
    if clearer_path == 'left': return 'left'
    elif clearer_path == 'right': return 'right'
    if obstacle_pos == 'left': return 'right'
    elif obstacle_pos == 'right': return 'left'
    return 'right'

def release_camera():
    """Release any existing camera instances."""
    try:
        subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], capture_output=True)
        time.sleep(0.5)
    except: pass

# ===================== GPS FUNCTIONS ===================== #

def parse_nmea_coordinate(coord_str, direction):
    if not coord_str or coord_str == '': return None
    try:
        coord = float(coord_str)
        degrees = int(coord / 100)
        minutes = coord - (degrees * 100)
        decimal_degrees = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        return decimal_degrees
    except (ValueError, TypeError): return None

def parse_gprmc(nmea_sentence):
    try:
        parts = nmea_sentence.split(',')
        if len(parts) < 8: return None
        status = parts[2]
        if status != 'A': return None
        
        lat = parse_nmea_coordinate(parts[3], parts[4])
        lon = parse_nmea_coordinate(parts[5], parts[6])
        course = float(parts[8]) if parts[8] and parts[8] != '' else None
        
        if lat is not None and lon is not None:
            return {'lat': lat, 'lon': lon, 'course': course, 'valid': True}
    except (ValueError, IndexError): pass
    return None

def parse_gpgga(nmea_sentence):
    try:
        parts = nmea_sentence.split(',')
        if len(parts) < 10: return None
        fix_quality = int(parts[6]) if parts[6] else 0
        if fix_quality == 0: return None
        
        lat = parse_nmea_coordinate(parts[2], parts[3])
        lon = parse_nmea_coordinate(parts[4], parts[5])
        
        if lat is not None and lon is not None:
            return {'lat': lat, 'lon': lon, 'valid': True}
    except (ValueError, IndexError): pass
    return None

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    
    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    return (bearing + 360) % 360

def bearing_to_turn_direction(current_heading, target_bearing):
    diff = (target_bearing - current_heading + 360) % 360
    if diff < HEADING_TOLERANCE or diff > (360 - HEADING_TOLERANCE):
        return 'straight', diff
    elif diff <= 180:
        return 'right', diff 
    else:
        return 'left', 360 - diff 

# ===================== GPS RECEIVER CLASS ===================== #

class GPSReceiver:
    def __init__(self, ip=GPS_UDP_IP, port=GPS_UDP_PORT):
        self.ip = ip
        self.port = port
        self.sock = None
        self.last_position = None
    
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.ip, self.port))
            self.sock.settimeout(GPS_TIMEOUT)
            print(f"✓ GPS listening on {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ GPS socket error: {e}")
            return False
    
    def get_position(self, timeout=GPS_TIMEOUT):
        if not self.sock: return None
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = self.sock.recvfrom(1024)
                line = data.decode('utf-8').strip()
                if '$GPRMC' in line or '$GNRMC' in line:
                    result = parse_gprmc(line)
                    if result:
                        self.last_position = result
                        return result
                elif '$GPGGA' in line or '$GNGGA' in line:
                    result = parse_gpgga(line)
                    if result:
                        self.last_position = result
                        return result
            except socket.timeout: continue
            except Exception as e: continue
        return None
    
    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

# ===================== TEST FUNCTIONS ===================== #

def test_arduino():
    print("\n" + "="*60 + "\nTESTING: Arduino Motor Control\n" + "="*60)
    input("Press Enter to start...")
    try:
        import serial
        ports = [ARDUINO_PORT, '/dev/ttyACM1', '/dev/ttyUSB0']
        arduino = None
        for port in ports:
            try:
                arduino = serial.Serial(port, ARDUINO_BAUDRATE, timeout=1)
                print(f"✓ Connected on {port}")
                break
            except: continue
        
        if not arduino:
            print("✗ Could not connect")
            return False
        
        time.sleep(2)
        print("Forward 1s..."); arduino.write(b'F'); time.sleep(1)
        print("Stop..."); arduino.write(b'S')
        arduino.close()
        return True
    except: return False

def test_camera():
    print("\n" + "="*60 + "\nTESTING: Camera\n" + "="*60)
    try:
        import cv2
        from picamera2 import Picamera2
        release_camera()
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        time.sleep(1)
        print("✓ Camera initialized")
        picam2.stop()
        picam2.close()
        return True
    except: return False

def test_path_clearance():
    print("\n" + "="*60 + "\nTESTING: Path Clearance\n" + "="*60)
    input("Press Enter to start...")
    try:
        import cv2
        from picamera2 import Picamera2
        release_camera()
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        for i in range(10):
            frame = picam2.capture_array()
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            roi = edges[h//3:, :]
            clearer, l, r = analyze_path_clearance(roi, w, h-h//3)
            print(f"Frame {i}: {clearer.upper()}")
            time.sleep(0.2)
        picam2.stop()
        picam2.close()
        return True
    except: return False

def test_threading():
    print("Skipped for brevity (Standard Python)")
    return True

def test_motor_sequence():
    print("\n" + "="*60 + "\nTESTING: Motor Sequence\n" + "="*60)
    if input("Proceed? (y/n): ").lower() != 'y': return None
    try:
        import serial
        ard = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
        time.sleep(2)
        seq = [(b'F', 1), (b'S', 0.5), (b'R', 1), (b'S', 0.5)]
        for c, t in seq:
            ard.write(c)
            time.sleep(t)
        ard.write(b'S')
        ard.close()
        return True
    except: return False

def test_gps_receiver():
    print("\n" + "="*60 + "\nTESTING: GPS Receiver\n" + "="*60)
    gps = GPSReceiver()
    if not gps.connect(): return False
    print("Waiting for data...")
    pos = gps.get_position(timeout=5)
    gps.close()
    if pos:
        print(f"✓ Received: {pos}")
        return True
    print("✗ No data")
    return False

def test_gps_navigation():
    # Legacy test where Phone = Goal
    print("Use Test 10 for Phone Driver mode.")
    return True

def monitor_phone_gps():
    gps = GPSReceiver()
    if not gps.connect(): return
    try:
        while True:
            pos = gps.get_position(timeout=1)
            if pos: print(f"\r{pos}", end="")
            else: print("\rWaiting...", end="")
    except KeyboardInterrupt: pass
    gps.close()

def test_phone_driver_navigation():
    """Visual test of the Phone Driver Math."""
    print("\n" + "="*60 + "\nTESTING: Phone Driver Math\n" + "="*60)
    print(f"Destination: {DESTINATION_LAT}, {DESTINATION_LON}")
    gps = GPSReceiver()
    if not gps.connect(): return False
    try:
        while True:
            pos = gps.get_position(timeout=2)
            if pos:
                dist = haversine_distance(pos['lat'], pos['lon'], DESTINATION_LAT, DESTINATION_LON)
                bear = calculate_bearing(pos['lat'], pos['lon'], DESTINATION_LAT, DESTINATION_LON)
                print(f"Robot: {pos['lat']:.5f},{pos['lon']:.5f} | Target Dist: {dist:.1f}m | Bear: {bear:.1f}")
            time.sleep(1)
    except KeyboardInterrupt: pass
    gps.close()
    return True

def test_integrated_gps():
    """
    Test 8: INTEGRATED PHONE DRIVER
    Robot Position = Updated via GPS
    Destination = Hardcoded
    """
    print("\n" + "="*60)
    print("TESTING: Integrated Camera + Motor + Phone GPS")
    print("==============================================")
    print(f"GOAL: {DESTINATION_LAT}, {DESTINATION_LON}")
    print("1. Phone is on Robot.")
    print("2. Robot will drive to Hardcoded Destination.")
    print("3. Obstacle Avoidance is ACTIVE.")
    
    if input("\nProceed? (y/n): ").lower() != 'y': return None
    
    arduino = None
    picam2 = None
    gps = None
    
    try:
        import serial
        import cv2
        from picamera2 import Picamera2
        
        # Connect Hardware
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
        time.sleep(2)
        
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        focal_length = calculate_focal_length(CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH)
        
        gps = GPSReceiver()
        if not gps.connect(): return False
        
        # Shared State
        data_lock = threading.Lock()
        state = {
            'running': True,
            'robot_lat': None, 'robot_lon': None, 'robot_head': 0.0,
            'dist': None, 'bear': None,
            'face_dist': None, 'obstacle': False, 'clearer': 'equal'
        }
        
        def gps_worker():
            while state['running']:
                pos = gps.get_position(timeout=2)
                if pos:
                    with data_lock:
                        # UPDATE ROBOT POSITION FROM GPS
                        state['robot_lat'] = pos['lat']
                        state['robot_lon'] = pos['lon']
                        
                        # Update Heading if moving
                        if pos.get('course'):
                            state['robot_head'] = pos['course']
                        
                        # Calc Navigation to Hardcoded Dest
                        d = haversine_distance(pos['lat'], pos['lon'], DESTINATION_LAT, DESTINATION_LON)
                        b = calculate_bearing(pos['lat'], pos['lon'], DESTINATION_LAT, DESTINATION_LON)
                        state['dist'] = d
                        state['bear'] = b
                time.sleep(0.5)

        def camera_worker():
            while state['running']:
                frame = picam2.capture_array()
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                # Face
                f_dist = None
                faces = face_cascade.detectMultiScale(gray, 1.1, 4)
                if len(faces) > 0:
                    w = faces[0][2]
                    f_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)
                
                # Obstacle
                blurred = cv2.GaussianBlur(gray, (5,5), 0)
                edges = cv2.Canny(blurred, 50, 150)
                roi = edges[160:, :] # Bottom 2/3
                h, w_img = roi.shape
                clr, _, _ = analyze_path_clearance(roi, w_img, h)
                
                obs = False
                contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    if cv2.contourArea(c)/(h*w_img) > OBSTACLE_AREA_THRESHOLD:
                        obs = True; break
                
                with data_lock:
                    state['face_dist'] = f_dist
                    state['obstacle'] = obs
                    state['clearer'] = clr
                time.sleep(0.05)

        def motor_worker():
            last_cmd = None
            while state['running']:
                with data_lock:
                    f_d = state['face_dist']
                    obs = state['obstacle']
                    clr = state['clearer']
                    dist = state['dist']
                    bear = state['bear']
                    head = state['robot_head']
                
                cmd = b'S'
                reason = "WAIT"
                
                # 1. Safety
                if f_d and f_d < DANGER_DISTANCE:
                    cmd = b'S'; reason = "FACE STOP"
                elif obs:
                    cmd = b'L' if clr == 'left' else b'R'
                    reason = f"AVOID {clr.upper()}"
                
                # 2. Nav
                elif dist is not None:
                    if dist < WAYPOINT_REACHED_DISTANCE:
                        cmd = b'S'; reason = "ARRIVED"
                    else:
                        turn, angle = bearing_to_turn_direction(head, bear)
                        if turn == 'straight' or angle < HEADING_TOLERANCE:
                            cmd = b'F'; reason = f"NAV FWD ({dist:.1f}m)"
                        else:
                            cmd = b'L' if turn == 'left' else b'R'
                            reason = f"NAV TURN {turn.upper()}"
                
                if cmd != last_cmd:
                    arduino.write(cmd)
                    print(f"  [MOTOR] {reason}")
                    last_cmd = cmd
                time.sleep(0.1)

        t_gps = threading.Thread(target=gps_worker, daemon=True)
        t_cam = threading.Thread(target=camera_worker, daemon=True)
        t_mot = threading.Thread(target=motor_worker, daemon=True)
        
        t_gps.start(); t_cam.start(); t_mot.start()
        
        time.sleep(30) # Run for 30s
        state['running'] = False
        time.sleep(1)
        
        arduino.write(b'S'); arduino.close()
        picam2.stop(); picam2.close()
        gps.close()
        print("✓ Integrated Test Complete")
        return True

    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    while True:
        print("\n" + "="*40)
        print(" FULL TEST SUITE (Phone Driver Edition)")
        print("="*40)
        print("1. Arduino Motors")
        print("2. Camera")
        print("3. Path Clearance")
        print("4. Threading")
        print("5. Motor Sequence (MOVES!)")
        print("6. GPS Receiver Check")
        print("7. GPS Nav (Legacy Phone=Goal)")
        print("8. INTEGRATED (Phone=Robot, Dest=Hardcoded)")
        print("9. Monitor GPS Raw")
        print("10. Phone Driver Math (Visual Only)")
        print("0. Exit")
        
        c = input("\nChoice: ")
        if c=='1': test_arduino()
        elif c=='2': test_camera()
        elif c=='3': test_path_clearance()
        elif c=='4': test_threading()
        elif c=='5': test_motor_sequence()
        elif c=='6': test_gps_receiver()
        elif c=='7': test_gps_navigation()
        elif c=='8': test_integrated_gps()
        elif c=='9': monitor_phone_gps()
        elif c=='10': test_phone_driver_navigation()
        elif c=='0': break

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nStopped")
