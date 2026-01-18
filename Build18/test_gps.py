#!/usr/bin/env python3

"""

RC Car Component Testing Script - Camera, Motor & GPS

======================================================



Use this to test all components before running the full system.



Run with: sudo python3 test_components.py



Tests included:

1. Arduino USB Connection & Motor Commands

2. Camera & Obstacle Detection  

3. Path Clearance Analysis

4. Threading Basics

5. Motor Navigation Sequence

6. GPS Receiver (iPhone GPS2IP)

7. GPS Navigation Test

8. Integrated Camera + Motor + GPS

9. Monitor Phone GPS (Continuous)

"""



import sys

import time

import threading

import subprocess

import socket

import math

import numpy as np



# ===================== CONFIGURATION ===================== #



# Camera Config

FACE_WIDTH_CM = 14.0

CALIBRATION_DISTANCE_CM = 50.0

CALIBRATION_PIX_WIDTH = 140



# ARDUINO CONNECTION Config

ARDUINO_PORT = '/dev/ttyACM0'  

ARDUINO_BAUDRATE = 9600  # Must match Arduino Serial.begin(9600)



# Decision Thresholds

DANGER_DISTANCE = 40.0

SAFE_DISTANCE = 100.0



# Obstacle Detection Config

OBSTACLE_AREA_THRESHOLD = 0.08



# GPS Config (for GPS2IP app)

GPS_UDP_IP = "0.0.0.0"    # Listen on all interfaces

GPS_UDP_PORT = 11123      # Standard GPS port (match GPS2IP settings)

GPS_TIMEOUT = 10          # Seconds to wait for GPS data



# Navigation Config

WAYPOINT_REACHED_DISTANCE = 3.0   # meters - consider waypoint reached

HEADING_TOLERANCE = 15.0          # degrees - acceptable heading error



# Robot's starting position (set this to your known starting point or use GPS)

# Example: CMU campus coordinates

ROBOT_DEFAULT_LAT = 40.4433  # Example: Pittsburgh area

ROBOT_DEFAULT_LON = -79.9436



# ===================== HELPER FUNCTIONS ===================== #



def calculate_focal_length(known_dist, known_width, width_pix):

    return (width_pix * known_dist) / known_width



def distance_to_camera(known_width, focal_length, pixel_width):

    if pixel_width == 0: 

        return float('inf')

    return (known_width * focal_length) / pixel_width



def analyze_path_clearance(edges, width, height):

    """

    Analyze left and right portions of the frame to determine

    which direction has fewer obstacles (path of least resistance).

    """

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

    """Determine best turn direction based on obstacle and path clearance."""

    if clearer_path == 'left':

        return 'left'

    elif clearer_path == 'right':

        return 'right'

    

    if obstacle_pos == 'left':

        return 'right'

    elif obstacle_pos == 'right':

        return 'left'

    

    return 'right'



def release_camera():

    """Release any existing camera instances."""

    try:

        subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], capture_output=True)

        time.sleep(0.5)

    except:

        pass



# ===================== GPS FUNCTIONS ===================== #



def parse_nmea_coordinate(coord_str, direction):

    """

    Parse NMEA coordinate format to decimal degrees.

    NMEA format: DDDMM.MMMMM (degrees and minutes)

    """

    if not coord_str or coord_str == '':

        return None

    

    try:

        # NMEA format: DDDMM.MMMMM or DDMM.MMMMM

        coord = float(coord_str)

        

        # Extract degrees and minutes

        degrees = int(coord / 100)

        minutes = coord - (degrees * 100)

        

        # Convert to decimal degrees

        decimal_degrees = degrees + (minutes / 60.0)

        

        # Apply direction (S and W are negative)

        if direction in ['S', 'W']:

            decimal_degrees = -decimal_degrees

        

        return decimal_degrees

    except (ValueError, TypeError):

        return None



def parse_gprmc(nmea_sentence):

    """

    Parse $GPRMC sentence for latitude, longitude, and speed.

    Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...

    """

    try:

        parts = nmea_sentence.split(',')

        

        if len(parts) < 8:

            return None

        

        status = parts[2]

        if status != 'A':  # 'A' = valid, 'V' = invalid

            return None

        

        lat = parse_nmea_coordinate(parts[3], parts[4])

        lon = parse_nmea_coordinate(parts[5], parts[6])

        

        # Speed in knots (convert to m/s if needed)

        speed_knots = float(parts[7]) if parts[7] else 0

        speed_ms = speed_knots * 0.514444

        

        # Course/heading in degrees

        course = float(parts[8]) if parts[8] and parts[8] != '' else None

        

        if lat is not None and lon is not None:

            return {

                'lat': lat,

                'lon': lon,

                'speed_knots': speed_knots,

                'speed_ms': speed_ms,

                'course': course,

                'valid': True

            }

    except (ValueError, IndexError) as e:

        pass

    

    return None



def parse_gpgga(nmea_sentence):

    """

    Parse $GPGGA sentence for latitude, longitude, and fix quality.

    Format: $GPGGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,altitude,...

    """

    try:

        parts = nmea_sentence.split(',')

        

        if len(parts) < 10:

            return None

        

        fix_quality = int(parts[6]) if parts[6] else 0

        if fix_quality == 0:  # No fix

            return None

        

        lat = parse_nmea_coordinate(parts[2], parts[3])

        lon = parse_nmea_coordinate(parts[4], parts[5])

        

        satellites = int(parts[7]) if parts[7] else 0

        altitude = float(parts[9]) if parts[9] else 0

        

        if lat is not None and lon is not None:

            return {

                'lat': lat,

                'lon': lon,

                'fix_quality': fix_quality,

                'satellites': satellites,

                'altitude': altitude,

                'valid': True

            }

    except (ValueError, IndexError) as e:

        pass

    

    return None



def haversine_distance(lat1, lon1, lat2, lon2):

    """

    Calculate the great-circle distance between two points on Earth.

    Returns distance in meters.

    """

    R = 6371000  # Earth's radius in meters

    

    phi1 = math.radians(lat1)

    phi2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)

    delta_lambda = math.radians(lon2 - lon1)

    

    a = math.sin(delta_phi / 2) ** 2 + \

        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2

    

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    

    return R * c



def calculate_bearing(lat1, lon1, lat2, lon2):

    """

    Calculate the initial bearing from point 1 to point 2.

    Returns bearing in degrees (0-360, where 0 = North).

    """

    phi1 = math.radians(lat1)

    phi2 = math.radians(lat2)

    delta_lambda = math.radians(lon2 - lon1)

    

    x = math.sin(delta_lambda) * math.cos(phi2)

    y = math.cos(phi1) * math.sin(phi2) - \

        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    

    bearing = math.atan2(x, y)

    bearing = math.degrees(bearing)

    

    # Normalize to 0-360

    bearing = (bearing + 360) % 360

    

    return bearing



def bearing_to_turn_direction(current_heading, target_bearing):

    """

    Determine which way to turn to reach target bearing.

    Returns: 'left', 'right', or 'straight'

    """

    diff = (target_bearing - current_heading + 360) % 360

    

    if diff < HEADING_TOLERANCE or diff > (360 - HEADING_TOLERANCE):

        return 'straight', diff

    elif diff <= 180:

        return 'right', diff  # Turn right (clockwise)

    else:

        return 'left', 360 - diff  # Turn left (counter-clockwise)





# ===================== GPS RECEIVER CLASS ===================== #



class GPSReceiver:

    """Receives and parses GPS data from iPhone GPS2IP app via UDP."""

    

    def __init__(self, ip=GPS_UDP_IP, port=GPS_UDP_PORT):

        self.ip = ip

        self.port = port

        self.sock = None

        self.last_position = None

        self.last_update = None

        self.running = False

    

    def connect(self):

        """Create UDP socket to receive GPS data."""

        try:

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            self.sock.bind((self.ip, self.port))

            self.sock.settimeout(GPS_TIMEOUT)

            print(f"✓ GPS receiver listening on {self.ip}:{self.port}")

            return True

        except Exception as e:

            print(f"✗ GPS socket error: {e}")

            return False

    

    def get_position(self, timeout=GPS_TIMEOUT):

        """

        Wait for and return GPS position from iPhone.

        Returns dict with lat, lon, and other data, or None if timeout.

        """

        if not self.sock:

            return None

        

        start_time = time.time()

        

        while time.time() - start_time < timeout:

            try:

                data, addr = self.sock.recvfrom(1024)

                line = data.decode('utf-8').strip()

                

                # Try parsing different NMEA sentence types

                if '$GPRMC' in line or '$GNRMC' in line:

                    result = parse_gprmc(line)

                    if result:

                        self.last_position = result

                        self.last_update = time.time()

                        return result

                

                elif '$GPGGA' in line or '$GNGGA' in line:

                    result = parse_gpgga(line)

                    if result:

                        self.last_position = result

                        self.last_update = time.time()

                        return result

                        

            except socket.timeout:

                continue

            except Exception as e:

                print(f"GPS parse error: {e}")

                continue

        

        return None

    

    def close(self):

        """Close the UDP socket."""

        if self.sock:

            self.sock.close()

            self.sock = None





# ===================== TEST FUNCTIONS ===================== #



def test_arduino():

    """Test USB Serial communication with Arduino."""

    print("\n" + "="*60)

    print("TESTING: Arduino USB Connection & Motor Control")

    print("="*60)

    print(f"Port: {ARDUINO_PORT}")

    print(f"Baudrate: {ARDUINO_BAUDRATE}")

    print("\nEnsure Arduino is plugged into USB and code is uploaded!")

    input("Press Enter to start test...")

    

    try:

        import serial

        

        ports_to_try = [ARDUINO_PORT, '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']

        arduino = None

        connected_port = None

        

        for port in ports_to_try:

            try:

                print(f"  Trying {port}...")

                arduino = serial.Serial(port, ARDUINO_BAUDRATE, timeout=1)

                connected_port = port

                break

            except serial.SerialException:

                continue

        

        if arduino is None:

            print("✗ ERROR: Could not open any serial port")

            return False

        

        print(f"✓ Connected to {connected_port}!")

        time.sleep(2)

        

        commands = [

            (b'F', "Forward", 1.0),

            (b'S', "Stop", 0.5),

            (b'B', "Backward", 1.0),

            (b'S', "Stop", 0.5),

            (b'L', "Turn Left", 0.8),

            (b'S', "Stop", 0.5),

            (b'R', "Turn Right", 0.8),

            (b'S', "Stop", 0.3)

        ]

        

        print("\nSending test commands...")

        print("-" * 40)

        

        for cmd_byte, desc, duration in commands:

            arduino.write(cmd_byte)

            print(f"  {desc:15} ({cmd_byte.decode()}) - {duration}s")

            time.sleep(duration)

        

        arduino.write(b'S')

        arduino.close()

        

        print("-" * 40)

        print("✓ Arduino motor test complete!")

        

        response = input("\nDid all motors respond correctly? (y/n): ").strip().lower()

        return response == 'y'

        

    except Exception as e:

        print(f"✗ Error: {e}")

        return False





def test_camera():

    """Test camera and obstacle detection."""

    print("\n" + "="*60)

    print("TESTING: Camera & Obstacle Detection")

    print("="*60)

    print("Position objects/face in front of the camera")

    input("Press Enter to start test...")

    

    picam2 = None

    

    try:

        import cv2

        from picamera2 import Picamera2

        

        cascade_path = 'haarcascade_frontalface_default.xml'

        face_cascade = cv2.CascadeClassifier(cascade_path)

        has_cascade = not face_cascade.empty()

        

        if has_cascade:

            print("✓ Haar cascade loaded")

        else:

            print("⚠ Haar cascade not found")

        

        focal_length = calculate_focal_length(

            CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH

        )

        

        release_camera()

        

        print("\nInitializing camera...")

        picam2 = Picamera2()

        config = picam2.create_video_configuration(

            main={"format": "RGB888", "size": (640, 480)}

        )

        picam2.configure(config)

        picam2.start()

        time.sleep(1)

        print("✓ Camera initialized!")

        

        print("\nProcessing 50 frames...")

        print("-" * 40)

        

        detections = {'face': 0, 'obstacle': 0, 'clear': 0}

        

        for i in range(50):

            frame = picam2.capture_array()

            height, width = frame.shape[:2]

            frame_area = height * width

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            

            detected_this_frame = False

            

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            edges = cv2.Canny(blurred, 50, 150)

            roi_top = height // 3

            roi = edges[roi_top:, :]

            

            clearer_path, left_edges, right_edges = analyze_path_clearance(

                roi, width, height - roi_top

            )

            

            if has_cascade:

                faces = face_cascade.detectMultiScale(

                    gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)

                )

                

                if len(faces) > 0:

                    faces = sorted(faces, key=lambda x: x[2], reverse=True)

                    x, y, w, h = faces[0]

                    face_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)

                    

                    print(f"  Frame {i+1:2}/50: FACE at {face_dist:.1f}cm | Clearer: {clearer_path}")

                    detections['face'] += 1

                    detected_this_frame = True

            

            if not detected_this_frame:

                contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                

                for contour in contours:

                    area = cv2.contourArea(contour)

                    area_ratio = area / frame_area

                    

                    if area_ratio > OBSTACLE_AREA_THRESHOLD:

                        print(f"  Frame {i+1:2}/50: OBSTACLE | Clearer: {clearer_path}")

                        detections['obstacle'] += 1

                        detected_this_frame = True

                        break

            

            if not detected_this_frame:

                detections['clear'] += 1

                if i % 10 == 0:

                    print(f"  Frame {i+1:2}/50: Clear")

            

            time.sleep(0.1)

        

        picam2.stop()

        picam2.close()

        

        print("-" * 40)

        print(f"\nFaces: {detections['face']} | Obstacles: {detections['obstacle']} | Clear: {detections['clear']}")

        print("✓ Camera test complete!")

        return True

        

    except Exception as e:

        print(f"✗ Error: {e}")

        if picam2:

            try:

                picam2.stop()

                picam2.close()

            except:

                pass

        return False





def test_gps_receiver():

    """Test GPS data reception from iPhone GPS2IP app."""

    print("\n" + "="*60)

    print("TESTING: GPS Receiver (iPhone GPS2IP)")

    print("="*60)

    print("\nSetup Instructions:")

    print("1. Install GPS2IP app on your iPhone")

    print("2. In GPS2IP settings:")

    print("   - Connection Method: UDP Push")

    print(f"   - IP Address: Your Raspberry Pi's IP (run 'hostname -I')")

    print(f"   - Port: {GPS_UDP_PORT}")

    print("   - Enable GGA and RMC messages")

    print("   - Enable Background Mode")

    print("3. Start GPS2IP streaming")

    print("\nWaiting for GPS data...")

    

    response = input("\nIs GPS2IP streaming? (y/n): ").strip().lower()

    if response != 'y':

        print("Test skipped - start GPS2IP first")

        return None

    

    gps = GPSReceiver()

    

    if not gps.connect():

        return False

    

    print(f"\nListening for GPS data (timeout: {GPS_TIMEOUT}s)...")

    print("-" * 40)

    

    positions_received = 0

    

    for i in range(10):

        print(f"\nAttempt {i+1}/10...")

        position = gps.get_position(timeout=5)

        

        if position:

            positions_received += 1

            print(f"  ✓ Position received!")

            print(f"    Latitude:  {position['lat']:.6f}°")

            print(f"    Longitude: {position['lon']:.6f}°")

            

            if 'speed_ms' in position:

                print(f"    Speed:     {position['speed_ms']:.2f} m/s")

            if 'satellites' in position:

                print(f"    Satellites: {position['satellites']}")

            if 'altitude' in position:

                print(f"    Altitude:  {position['altitude']:.1f} m")

        else:

            print(f"  ✗ No GPS data received")

    

    gps.close()

    

    print("-" * 40)

    print(f"\nReceived {positions_received}/10 GPS positions")

    

    if positions_received > 0:

        print("✓ GPS receiver test complete!")

        return True

    else:

        print("✗ No GPS data received")

        print("\nTroubleshooting:")

        print("  1. Check GPS2IP is running and streaming")

        print("  2. Verify IP address matches Pi's IP")

        print(f"  3. Verify port is {GPS_UDP_PORT}")

        print("  4. Check if firewall is blocking UDP")

        print("  5. Try: sudo ufw allow 11123/udp")

        return False



def monitor_phone_gps():

    """

    Continuously monitor and display phone GPS location.

    Loops until interrupted to allow verification of signal reception.

    """

    print("\n" + "="*60)

    print("MONITORING: Phone GPS Location (Continuous)")

    print("="*60)

    print("Ensure GPS2IP is running on your phone.")

    print("Press Ctrl+C to stop monitoring.")

    

    gps = GPSReceiver()

    if not gps.connect():

        return False

    

    print(f"\nListening on port {GPS_UDP_PORT}...")

    print("Walk around with your phone to see coordinates update.\n")

    

    try:

        while True:

            pos = gps.get_position(timeout=2)

            if pos:

                # Use carriage return \r to overwrite the line for a clean display

                speed = pos.get('speed_ms', 0)

                print(f"\r[PHONE] Lat: {pos['lat']:.6f} | Lon: {pos['lon']:.6f} | Spd: {speed:.1f}m/s   ", end="", flush=True)

            else:

                print(f"\r[PHONE] Waiting for signal...                                     ", end="", flush=True)

            

            time.sleep(0.2)

            

    except KeyboardInterrupt:

        print("\n\nStopped monitoring.")

    finally:

        gps.close()

    return True



def test_gps_navigation():

    """Test GPS navigation calculations."""

    print("\n" + "="*60)

    print("TESTING: GPS Navigation Calculations")

    print("="*60)

    print("\nThis test will:")

    print("1. Get your phone's GPS position (goal)")

    print("2. Calculate distance and bearing from robot to phone")

    print("3. Show which direction the robot should turn")

    

    response = input("\nProceed? (y/n): ").strip().lower()

    if response != 'y':

        return None

    

    gps = GPSReceiver()

    

    if not gps.connect():

        return False

    

    # Get robot's position (use default or get from another GPS)

    print(f"\nRobot position (default): {ROBOT_DEFAULT_LAT:.6f}, {ROBOT_DEFAULT_LON:.6f}")

    use_default = input("Use default robot position? (y/n): ").strip().lower()

    

    if use_default == 'y':

        robot_lat = ROBOT_DEFAULT_LAT

        robot_lon = ROBOT_DEFAULT_LON

    else:

        try:

            robot_lat = float(input("Enter robot latitude: "))

            robot_lon = float(input("Enter robot longitude: "))

        except ValueError:

            print("Invalid coordinates, using default")

            robot_lat = ROBOT_DEFAULT_LAT

            robot_lon = ROBOT_DEFAULT_LON

    

    print(f"\nRobot at: {robot_lat:.6f}, {robot_lon:.6f}")

    print("\nWaiting for phone GPS (goal position)...")

    print("Walk around with your phone to see navigation updates")

    print("Press Ctrl+C to stop")

    print("-" * 40)

    

    # Assume robot is facing North (0 degrees) initially

    robot_heading = 0.0

    

    try:

        while True:

            position = gps.get_position(timeout=3)

            

            if position:

                phone_lat = position['lat']

                phone_lon = position['lon']

                

                # Calculate distance and bearing

                distance = haversine_distance(robot_lat, robot_lon, phone_lat, phone_lon)

                bearing = calculate_bearing(robot_lat, robot_lon, phone_lat, phone_lon)

                

                # Determine turn direction

                turn_dir, turn_angle = bearing_to_turn_direction(robot_heading, bearing)

                

                print(f"\n  Phone at: {phone_lat:.6f}, {phone_lon:.6f}")

                print(f"  Distance: {distance:.1f} meters")

                print(f"  Bearing:  {bearing:.1f}° (from North)")

                print(f"  Robot heading: {robot_heading:.1f}°")

                print(f"  Turn: {turn_dir.upper()} by {turn_angle:.1f}°")

                

                if distance < WAYPOINT_REACHED_DISTANCE:

                    print(f"  🎯 GOAL REACHED! (within {WAYPOINT_REACHED_DISTANCE}m)")

                elif turn_dir == 'straight':

                    print(f"  → GO FORWARD")

                else:

                    print(f"  → TURN {turn_dir.upper()} then FORWARD")

            else:

                print("  Waiting for GPS...")

            

            time.sleep(1)

            

    except KeyboardInterrupt:

        print("\n\nNavigation test stopped")

    

    gps.close()

    print("✓ GPS navigation test complete!")

    return True





def test_integrated_gps():

    """Test integrated camera + motor + GPS navigation."""

    print("\n" + "="*60)

    print("TESTING: Integrated Camera + Motor + GPS")

    print("="*60)

    print("\nThis test combines:")

    print("1. GPS navigation to phone location")

    print("2. Camera-based obstacle avoidance")

    print("3. Path of least resistance turning")

    print("\n⚠ WARNING: The car will move!")

    

    response = input("\nProceed? (y/n): ").strip().lower()

    if response != 'y':

        return None

    

    arduino = None

    picam2 = None

    gps = None

    

    try:

        import serial

        import cv2

        from picamera2 import Picamera2

        

        # Connect Arduino

        ports_to_try = [ARDUINO_PORT, '/dev/ttyACM1', '/dev/ttyUSB0']

        for port in ports_to_try:

            try:

                arduino = serial.Serial(port, ARDUINO_BAUDRATE, timeout=1)

                print(f"✓ Arduino connected on {port}")

                break

            except:

                continue

        

        if arduino is None:

            print("✗ Could not connect to Arduino")

            return False

        

        time.sleep(2)

        

        # Initialize camera

        release_camera()

        print("Initializing camera...")

        picam2 = Picamera2()

        config = picam2.create_video_configuration(

            main={"format": "RGB888", "size": (640, 480)}

        )

        picam2.configure(config)

        picam2.start()

        time.sleep(1)

        print("✓ Camera ready!")

        

        # Load face cascade

        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

        has_cascade = not face_cascade.empty()

        focal_length = calculate_focal_length(

            CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH

        )

        

        # Connect GPS

        gps = GPSReceiver()

        if not gps.connect():

            print("⚠ GPS not available - running without navigation")

            gps = None

        

        # Get robot position

        robot_lat = ROBOT_DEFAULT_LAT

        robot_lon = ROBOT_DEFAULT_LON

        robot_heading = 0.0  # Assume facing North

        

        # Shared state

        data_lock = threading.Lock()

        shared_state = {

            'obstacle_detected': False,

            'obstacle_distance': 100,

            'clearer_path': 'equal',

            'face_detected': False,

            'face_distance': None,

            'goal_lat': None,

            'goal_lon': None,

            'goal_distance': None,

            'goal_bearing': None,

            'running': True

        }

        

        def camera_worker():

            while shared_state['running']:

                frame = picam2.capture_array()

                height, width = frame.shape[:2]

                frame_area = height * width

                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

                

                blurred = cv2.GaussianBlur(gray, (5, 5), 0)

                edges = cv2.Canny(blurred, 50, 150)

                roi = edges[height//3:, :]

                

                clearer_path, _, _ = analyze_path_clearance(roi, width, height - height//3)

                

                face_detected = False

                face_dist = None

                obstacle_detected = False

                obstacle_dist = 100

                

                if has_cascade:

                    faces = face_cascade.detectMultiScale(

                        gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)

                    )

                    if len(faces) > 0:

                        faces = sorted(faces, key=lambda x: x[2], reverse=True)

                        x, y, w, h = faces[0]

                        face_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)

                        face_detected = True

                

                if not face_detected:

                    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    for contour in contours:

                        area = cv2.contourArea(contour)

                        if area / frame_area > OBSTACLE_AREA_THRESHOLD:

                            obstacle_detected = True

                            obstacle_dist = max(0, 100 - (area / frame_area * 500))

                            break

                

                with data_lock:

                    shared_state['face_detected'] = face_detected

                    shared_state['face_distance'] = face_dist

                    shared_state['obstacle_detected'] = obstacle_detected

                    shared_state['obstacle_distance'] = obstacle_dist

                    shared_state['clearer_path'] = clearer_path

                

                time.sleep(0.05)

        

        def gps_worker():

            if gps is None:

                return

            

            while shared_state['running']:

                position = gps.get_position(timeout=2)

                

                if position:

                    goal_lat = position['lat']

                    goal_lon = position['lon']

                    

                    distance = haversine_distance(robot_lat, robot_lon, goal_lat, goal_lon)

                    bearing = calculate_bearing(robot_lat, robot_lon, goal_lat, goal_lon)

                    

                    with data_lock:

                        shared_state['goal_lat'] = goal_lat

                        shared_state['goal_lon'] = goal_lon

                        shared_state['goal_distance'] = distance

                        shared_state['goal_bearing'] = bearing

                

                time.sleep(1)

        

        def motor_worker():

            last_command = None

            

            while shared_state['running']:

                with data_lock:

                    face = shared_state['face_detected']

                    face_dist = shared_state['face_distance']

                    obstacle = shared_state['obstacle_detected']

                    obstacle_dist = shared_state['obstacle_distance']

                    clearer = shared_state['clearer_path']

                    goal_dist = shared_state['goal_distance']

                    goal_bearing = shared_state['goal_bearing']

                

                # Priority 1: Safety - Face too close

                if face and face_dist and face_dist < DANGER_DISTANCE:

                    command = b'S'

                    cmd_name = f"STOP (face at {face_dist:.0f}cm)"

                

                # Priority 2: Obstacle avoidance

                elif (face and face_dist and face_dist < SAFE_DISTANCE) or \

                     (obstacle and obstacle_dist < 30):

                    turn_dir = get_best_turn_direction('center', clearer)

                    command = b'L' if turn_dir == 'left' else b'R'

                    cmd_name = f"AVOID: {turn_dir.upper()} (clearer={clearer})"

                

                # Priority 3: Navigate to GPS goal

                elif goal_dist is not None and goal_bearing is not None:

                    if goal_dist < WAYPOINT_REACHED_DISTANCE:

                        command = b'S'

                        cmd_name = f"GOAL REACHED ({goal_dist:.1f}m)"

                    else:

                        turn_dir, turn_angle = bearing_to_turn_direction(robot_heading, goal_bearing)

                        if turn_dir == 'straight' or turn_angle < HEADING_TOLERANCE:

                            command = b'F'

                            cmd_name = f"NAV: FORWARD ({goal_dist:.1f}m to goal)"

                        else:

                            command = b'L' if turn_dir == 'left' else b'R'

                            cmd_name = f"NAV: {turn_dir.upper()} {turn_angle:.0f}° ({goal_dist:.1f}m)"

                

                # Priority 4: No GPS, just go forward

                else:

                    command = b'F'

                    cmd_name = "FORWARD (no GPS)"

                

                if command != last_command:

                    arduino.write(command)

                    print(f"  [MOTOR] {cmd_name}")

                    last_command = command

                

                time.sleep(0.1)

        

        print("\nStarting integrated test for 30 seconds...")

        print("The robot will navigate toward your phone while avoiding obstacles")

        print("-" * 40)

        

        threads = [

            threading.Thread(target=camera_worker, daemon=True),

            threading.Thread(target=motor_worker, daemon=True),

        ]

        

        if gps:

            threads.append(threading.Thread(target=gps_worker, daemon=True))

        

        for t in threads:

            t.start()

        

        time.sleep(30)

        

        shared_state['running'] = False

        time.sleep(0.5)

        

        # Cleanup

        print("\nCleaning up...")

        arduino.write(b'S')

        arduino.close()

        picam2.stop()

        picam2.close()

        if gps:

            gps.close()

        

        print("-" * 40)

        print("✓ Integrated test complete!")

        

        response = input("\nDid the robot navigate correctly? (y/n): ").strip().lower()

        return response == 'y'

        

    except Exception as e:

        print(f"✗ Error: {e}")

        import traceback

        traceback.print_exc()

        

        if arduino:

            try:

                arduino.write(b'S')

                arduino.close()

            except:

                pass

        if picam2:

            try:

                picam2.stop()

                picam2.close()

            except:

                pass

        if gps:

            gps.close()

        

        return False





def test_threading():

    """Test basic threading functionality."""

    print("\n" + "="*60)

    print("TESTING: Threading Basics")

    print("="*60)

    

    try:

        data_lock = threading.Lock()

        shared_state = {'count': 0, 'running': True}

        results = {'thread1': 0, 'thread2': 0}

        

        def worker1():

            while shared_state['running'] and results['thread1'] < 20:

                with data_lock:

                    shared_state['count'] += 1

                    results['thread1'] += 1

                time.sleep(0.05)

        

        def worker2():

            while shared_state['running'] and results['thread2'] < 10:

                with data_lock:

                    results['thread2'] += 1

                time.sleep(0.1)

        

        print("Starting threads...")

        t1 = threading.Thread(target=worker1, daemon=True)

        t2 = threading.Thread(target=worker2, daemon=True)

        

        t1.start()

        t2.start()

        t1.join(timeout=3)

        t2.join(timeout=3)

        shared_state['running'] = False

        

        print(f"Thread 1: {results['thread1']} iterations")

        print(f"Thread 2: {results['thread2']} iterations")

        

        if results['thread1'] >= 15 and results['thread2'] >= 8:

            print("✓ Threading test passed!")

            return True

        return False

        

    except Exception as e:

        print(f"✗ Error: {e}")

        return False





def test_path_clearance():

    """Test path clearance analysis."""

    print("\n" + "="*60)

    print("TESTING: Path Clearance Analysis")

    print("="*60)

    print("Block LEFT or RIGHT side of camera to test")

    input("Press Enter to start...")

    

    picam2 = None

    

    try:

        import cv2

        from picamera2 import Picamera2

        

        release_camera()

        

        picam2 = Picamera2()

        config = picam2.create_video_configuration(

            main={"format": "RGB888", "size": (640, 480)}

        )

        picam2.configure(config)

        picam2.start()

        time.sleep(1)

        print("✓ Camera ready!")

        

        print("\nAnalyzing for 30 frames...")

        print("-" * 40)

        

        for i in range(30):

            frame = picam2.capture_array()

            height, width = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            edges = cv2.Canny(blurred, 50, 150)

            roi = edges[height//3:, :]

            

            clearer, left, right = analyze_path_clearance(roi, width, height - height//3)

            

            indicator = "◀◀◀ LEFT" if clearer == 'left' else \

                       "RIGHT ▶▶▶" if clearer == 'right' else "=== EQUAL ==="

            

            print(f"  Frame {i+1:2}: L:{left:6} R:{right:6} | {indicator}")

            time.sleep(0.2)

        

        picam2.stop()

        picam2.close()

        

        print("✓ Path clearance test complete!")

        return True

        

    except Exception as e:

        print(f"✗ Error: {e}")

        if picam2:

            try:

                picam2.stop()

                picam2.close()

            except:

                pass

        return False





def test_motor_sequence():

    """Test motor sequence."""

    print("\n" + "="*60)

    print("TESTING: Motor Sequence")

    print("="*60)

    print("⚠ The car will move!")

    

    response = input("Proceed? (y/n): ").strip().lower()

    if response != 'y':

        return None

    

    try:

        import serial

        

        arduino = None

        for port in [ARDUINO_PORT, '/dev/ttyACM1', '/dev/ttyUSB0']:

            try:

                arduino = serial.Serial(port, ARDUINO_BAUDRATE, timeout=1)

                print(f"✓ Connected on {port}")

                break

            except:

                continue

        

        if not arduino:

            print("✗ Could not connect")

            return False

        

        time.sleep(2)

        

        sequence = [

            (b'F', "Forward", 1.5),

            (b'S', "Stop", 0.3),

            (b'R', "Right", 0.6),

            (b'S', "Stop", 0.2),

            (b'F', "Forward", 1.0),

            (b'L', "Left", 0.6),

            (b'F', "Forward", 0.8),

            (b'S', "Stop", 0.5),

        ]

        

        print("\nExecuting sequence...")

        for cmd, name, dur in sequence:

            print(f"  {name}...")

            arduino.write(cmd)

            time.sleep(dur)

        

        arduino.close()

        print("✓ Sequence complete!")

        

        return input("Did it work? (y/n): ").strip().lower() == 'y'

        

    except Exception as e:

        print(f"✗ Error: {e}")

        return False





def main():

    """Main menu."""

    print("\n" + "="*60)

    print("RC CAR TEST SUITE")

    print("Camera + Motor + GPS Navigation")

    print("="*60)

    print(f"\nArduino: {ARDUINO_PORT} @ {ARDUINO_BAUDRATE}")

    print(f"GPS Port: {GPS_UDP_PORT}")

    

    while True:

        print("\n" + "-"*40)

        print("Tests:")

        print("  1. Arduino Motor Control")

        print("  2. Camera & Obstacle Detection")

        print("  3. Path Clearance Analysis")

        print("  4. Threading Basics")

        print("  5. Motor Sequence")

        print("  6. GPS Receiver (iPhone GPS2IP)")

        print("  7. GPS Navigation Calculations")

        print("  8. Integrated Camera + Motor + GPS")

        print("  9. Monitor Phone GPS (Continuous)")

        print("  0. Exit")

        

        choice = input("\nChoice: ").strip()

        

        results = {}

        

        if choice == '1':

            results['arduino'] = test_arduino()

        elif choice == '2':

            results['camera'] = test_camera()

        elif choice == '3':

            results['path_clearance'] = test_path_clearance()

        elif choice == '4':

            results['threading'] = test_threading()

        elif choice == '5':

            r = test_motor_sequence()

            if r is not None:

                results['motor_sequence'] = r

        elif choice == '6':

            r = test_gps_receiver()

            if r is not None:

                results['gps_receiver'] = r

        elif choice == '7':

            r = test_gps_navigation()

            if r is not None:

                results['gps_navigation'] = r

        elif choice == '8':

            r = test_integrated_gps()

            if r is not None:

                results['integrated_gps'] = r

        elif choice == '9':

            monitor_phone_gps()

        elif choice == '0':

            print("\nGoodbye!")

            break

        else:

            print("Invalid choice")

            continue

        

        if results:

            print("\n" + "="*60)

            print("SUMMARY")

            print("="*60)

            for name, passed in results.items():

                status = "✓ PASSED" if passed else "✗ FAILED" if passed is False else "⏭ SKIPPED"

                print(f"  {name:20} {status}")





if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("\n\nInterrupted")

    except Exception as e:

        print(f"\nError: {e}")

        import traceback

        traceback.print_exc()
