#!/usr/bin/env python3
"""
RC Car Component Testing Script - Camera & Motor Focus
=======================================================

Use this to test camera vision and motor control components
before running the full obstacle avoidance system.

Run with: sudo python3 test_components.py

Tests included:
1. Arduino USB Connection & Motor Commands
2. Camera & Obstacle Detection  
3. Threading Basics
4. Integrated Camera + Motor Test (with path of least resistance)
5. Motor Navigation Sequence
"""

import sys
import time
import threading
import subprocess
import numpy as np

# ===================== CONFIGURATION ===================== #
# (Matching rc_car_threaded.py settings)

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
    
    Returns: ('left', 'right', or 'equal'), left_edge_count, right_edge_count
    """
    # Split frame into left and right halves
    mid = width // 2
    left_region = edges[:, :mid]
    right_region = edges[:, mid:]
    
    # Count edge pixels in each region (more edges = more obstacles)
    left_edges = np.sum(left_region > 0)
    right_edges = np.sum(right_region > 0)
    
    # Add a threshold to avoid jitter when counts are similar
    threshold = 0.1  # 10% difference required
    total = left_edges + right_edges
    
    if total == 0:
        return 'equal', left_edges, right_edges
    
    diff_ratio = abs(left_edges - right_edges) / total
    
    if diff_ratio < threshold:
        return 'equal', left_edges, right_edges
    elif left_edges < right_edges:
        return 'left', left_edges, right_edges  # Left has fewer obstacles
    else:
        return 'right', left_edges, right_edges  # Right has fewer obstacles

def get_best_turn_direction(obstacle_pos, clearer_path):
    """
    Determine the best direction to turn based on:
    1. Where the obstacle is
    2. Which path is clearer
    
    Returns: 'left' or 'right'
    """
    # If one path is clearly better, use it
    if clearer_path == 'left':
        return 'left'
    elif clearer_path == 'right':
        return 'right'
    
    # If paths are equal, turn away from obstacle
    if obstacle_pos == 'left':
        return 'right'
    elif obstacle_pos == 'right':
        return 'left'
    
    # Default: turn right (arbitrary choice for center obstacles with equal paths)
    return 'right'

def release_camera():
    """Helper function to release any existing camera instances"""
    try:
        subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], capture_output=True)
        time.sleep(0.5)
    except:
        pass

# ===================== TEST FUNCTIONS ===================== #

def test_arduino():
    """Test USB Serial communication with Arduino"""
    print("\n" + "="*60)
    print("TESTING: Arduino USB Connection & Motor Control")
    print("="*60)
    print(f"Port: {ARDUINO_PORT}")
    print(f"Baudrate: {ARDUINO_BAUDRATE}")
    print("\nEnsure Arduino is plugged into USB and code is uploaded!")
    input("Press Enter to start test...")
    
    try:
        import serial
        
        # Try configured port first, then alternatives
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
            print("  1. Check USB cable connection")
            print("  2. Try: ls /dev/tty* (look for ttyACM0 or ttyUSB0)")
            print("  3. Ensure Arduino code is uploaded")
            return False
        
        print(f"✓ Connected to {connected_port}!")
        if connected_port != ARDUINO_PORT:
            print(f"  NOTE: Update ARDUINO_PORT in rc_car_threaded.py to '{connected_port}'")
        
        time.sleep(2)  # Wait for Arduino reset
        
        # Test commands matching Arduino code
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
        
        print("\nSending test commands (watch the motors!)...")
        print("-" * 40)
        
        for cmd_byte, desc, duration in commands:
            arduino.write(cmd_byte)
            print(f"  {desc:15} ({cmd_byte.decode()}) - {duration}s")
            time.sleep(duration)
        
        arduino.write(b'S')  # Final stop
        arduino.close()
        
        print("-" * 40)
        print("✓ Arduino motor test complete!")
        
        response = input("\nDid all motors respond correctly? (y/n): ").strip().lower()
        return response == 'y'
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except ImportError:
        print("✗ ERROR: pyserial not installed")
        print("  Install with: pip3 install pyserial")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False


def test_camera():
    """Test camera and obstacle detection with path clearance analysis"""
    print("\n" + "="*60)
    print("TESTING: Camera & Obstacle Detection")
    print("="*60)
    print(f"Face calibration: {FACE_WIDTH_CM}cm at {CALIBRATION_DISTANCE_CM}cm = {CALIBRATION_PIX_WIDTH}px")
    print(f"Danger distance: {DANGER_DISTANCE}cm")
    print(f"Safe distance: {SAFE_DISTANCE}cm")
    print("\nPosition objects/face in front of the camera to test detection")
    input("Press Enter to start test...")
    
    picam2 = None
    
    try:
        import cv2
        from picamera2 import Picamera2
        
        # Check for Haar cascade file
        cascade_path = 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)
        has_cascade = not face_cascade.empty()
        
        if has_cascade:
            print("✓ Haar cascade loaded for face detection")
        else:
            print("⚠ Haar cascade not found - face detection disabled")
            print("  Download from: https://github.com/opencv/opencv/tree/master/data/haarcascades")
        
        # Calculate focal length
        focal_length = calculate_focal_length(
            CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH
        )
        print(f"  Calculated focal length: {focal_length:.2f}")
        
        # Try to release camera if it's busy
        release_camera()
        
        # Initialize camera with retry
        print("\nInitializing camera...")
        max_retries = 3
        for attempt in range(max_retries):
            try:
                picam2 = Picamera2()
                config = picam2.create_video_configuration(
                    main={"format": "RGB888", "size": (640, 480)}
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(1)
                print("✓ Camera initialized!")
                break
            except RuntimeError as e:
                if attempt < max_retries - 1:
                    print(f"  Camera busy, retrying ({attempt + 1}/{max_retries})...")
                    release_camera()
                    time.sleep(2)
                else:
                    raise RuntimeError(
                        "Camera is busy. Try running:\n"
                        "  sudo pkill -f libcamera\n"
                        "  sudo pkill -f python3\n"
                        "Or reboot: sudo reboot"
                    ) from e
        
        print("\nProcessing 50 frames (5 seconds)...")
        print("Move objects/hands/face in front of camera")
        print("-" * 40)
        
        detections = {'face': 0, 'obstacle': 0, 'clear': 0}
        
        for i in range(50):
            frame = picam2.capture_array()
            height, width = frame.shape[:2]
            frame_area = height * width
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            detected_this_frame = False
            
            # --- Edge detection for path clearance ---
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            roi_top = height // 3
            roi = edges[roi_top:, :]
            
            clearer_path, left_edges, right_edges = analyze_path_clearance(
                roi, width, height - roi_top
            )
            
            # --- Face Detection ---
            if has_cascade:
                faces = face_cascade.detectMultiScale(
                    gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)
                )
                
                if len(faces) > 0:
                    faces = sorted(faces, key=lambda x: x[2], reverse=True)
                    x, y, w, h = faces[0]
                    face_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)
                    face_x = x + w // 2
                    
                    # Determine face position
                    if face_x < width // 3:
                        face_pos = "LEFT"
                    elif face_x > 2 * width // 3:
                        face_pos = "RIGHT"
                    else:
                        face_pos = "CENTER"
                    
                    status = "DANGER!" if face_dist < DANGER_DISTANCE else \
                             "CAUTION" if face_dist < SAFE_DISTANCE else "OK"
                    
                    best_turn = get_best_turn_direction(face_pos.lower(), clearer_path)
                    
                    print(f"  Frame {i+1:2}/50: FACE at {face_dist:.1f}cm ({face_pos}) [{status}] "
                          f"| Clearer: {clearer_path} -> Turn {best_turn.upper()}")
                    detections['face'] += 1
                    detected_this_frame = True
            
            # --- Obstacle Detection ---
            if not detected_this_frame:
                contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    area_ratio = area / frame_area
                    
                    if area_ratio > OBSTACLE_AREA_THRESHOLD:
                        x, y, w, h = cv2.boundingRect(contour)
                        cx = x + w // 2
                        
                        if cx < width // 3:
                            position = "LEFT"
                        elif cx > 2 * width // 3:
                            position = "RIGHT"
                        else:
                            position = "CENTER"
                        
                        dist_est = max(0, 100 - (area_ratio * 500))
                        best_turn = get_best_turn_direction(position.lower(), clearer_path)
                        
                        print(f"  Frame {i+1:2}/50: OBSTACLE at {position} (dist: {dist_est:.0f}) "
                              f"| Clearer: {clearer_path} -> Turn {best_turn.upper()}")
                        detections['obstacle'] += 1
                        detected_this_frame = True
                        break
            
            if not detected_this_frame:
                detections['clear'] += 1
                if i % 10 == 0:
                    print(f"  Frame {i+1:2}/50: Clear | Path clearance - L:{left_edges} R:{right_edges}")
            
            time.sleep(0.1)
        
        picam2.stop()
        try:
            picam2.close()
        except:
            pass
        
        print("-" * 40)
        print("\nDetection Summary:")
        print(f"  Faces detected:     {detections['face']} frames")
        print(f"  Obstacles detected: {detections['obstacle']} frames")
        print(f"  Clear frames:       {detections['clear']} frames")
        
        print("\n✓ Camera test complete!")
        return True
        
    except ImportError as e:
        print(f"✗ Required library not installed: {e}")
        print("  Install with: sudo apt-get install python3-opencv python3-picamera2")
        return False
    except Exception as e:
        print(f"✗ Camera error: {e}")
        import traceback
        traceback.print_exc()
        
        if picam2:
            try:
                picam2.stop()
                picam2.close()
            except:
                pass
        
        return False


def test_threading():
    """Test basic threading functionality"""
    print("\n" + "="*60)
    print("TESTING: Threading Basics")
    print("="*60)
    print("Testing thread synchronization with shared state...")
    
    try:
        # Simulate shared state pattern
        data_lock = threading.Lock()
        shared_state = {
            'obstacle_detected': False,
            'obstacle_position': 'center',
            'clearer_path': 'equal',
            'running': True
        }
        results = {'camera_updates': 0, 'motor_commands': 0}
        
        def mock_camera_thread():
            count = 0
            while shared_state['running'] and count < 20:
                with data_lock:
                    shared_state['obstacle_detected'] = (count % 5 == 0)
                    shared_state['obstacle_position'] = ['left', 'center', 'right'][count % 3]
                    shared_state['clearer_path'] = ['left', 'right', 'equal'][count % 3]
                    results['camera_updates'] += 1
                time.sleep(0.05)
                count += 1
        
        def mock_motor_thread():
            count = 0
            last_command = None
            while shared_state['running'] and count < 10:
                with data_lock:
                    obstacle = shared_state['obstacle_detected']
                    position = shared_state['obstacle_position']
                    clearer = shared_state['clearer_path']
                
                if obstacle:
                    command = get_best_turn_direction(position, clearer)
                else:
                    command = 'forward'
                
                if command != last_command:
                    results['motor_commands'] += 1
                    last_command = command
                
                time.sleep(0.1)
                count += 1
        
        print("\nStarting mock camera and motor threads...")
        
        t1 = threading.Thread(target=mock_camera_thread, daemon=True)
        t2 = threading.Thread(target=mock_motor_thread, daemon=True)
        
        start_time = time.time()
        t1.start()
        t2.start()
        
        t1.join(timeout=3)
        t2.join(timeout=3)
        shared_state['running'] = False
        elapsed = time.time() - start_time
        
        print(f"\nResults (completed in {elapsed:.2f}s):")
        print(f"  Camera updates: {results['camera_updates']}")
        print(f"  Motor command changes: {results['motor_commands']}")
        
        if results['camera_updates'] >= 15 and results['motor_commands'] >= 3:
            print("\n✓ Threading test passed!")
            return True
        else:
            print("\n✗ Threading test failed")
            return False
            
    except Exception as e:
        print(f"✗ Threading error: {e}")
        return False


def test_integrated():
    """Test camera and motor working together with path of least resistance"""
    print("\n" + "="*60)
    print("TESTING: Integrated Camera + Motor")
    print("="*60)
    print("This tests camera detection controlling motor commands")
    print("using PATH OF LEAST RESISTANCE logic for turn direction")
    print("\n⚠ WARNING: The car will move! Keep it on a safe surface!")
    
    response = input("\nProceed with integrated test? (y/n): ").strip().lower()
    if response != 'y':
        print("Test skipped")
        return None
    
    arduino = None
    picam2 = None
    
    try:
        import serial
        import cv2
        from picamera2 import Picamera2
        
        # Connect to Arduino
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
        
        # Load face cascade
        cascade_path = 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)
        has_cascade = not face_cascade.empty()
        
        if has_cascade:
            print("✓ Haar cascade loaded for face detection")
        else:
            print("⚠ Haar cascade not found - using edge detection only")
        
        # Calculate focal length
        focal_length = calculate_focal_length(
            CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH
        )
        
        # Try to release camera if busy
        print("Checking camera availability...")
        release_camera()
        
        # Initialize camera with retry
        print("Initializing camera...")
        max_retries = 3
        for attempt in range(max_retries):
            try:
                picam2 = Picamera2()
                config = picam2.create_video_configuration(
                    main={"format": "RGB888", "size": (640, 480)}
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(1)
                print("✓ Camera ready!")
                break
            except RuntimeError as e:
                if attempt < max_retries - 1:
                    print(f"  Camera busy, retrying ({attempt + 1}/{max_retries})...")
                    release_camera()
                    time.sleep(2)
                else:
                    raise RuntimeError(
                        "Camera is busy. Try: sudo pkill -f libcamera && sudo reboot"
                    ) from e
        
        # Shared state
        data_lock = threading.Lock()
        shared_state = {
            'obstacle_detected': False,
            'obstacle_position': 'center',
            'obstacle_distance': 100,
            'face_detected': False,
            'face_distance': None,
            'face_position': 'center',
            'clearer_path': 'equal',
            'running': True
        }
        
        def camera_worker():
            while shared_state['running']:
                frame = picam2.capture_array()
                height, width = frame.shape[:2]
                frame_area = height * width
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                # Edge detection for path clearance
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blurred, 50, 150)
                roi_top = height // 3
                roi = edges[roi_top:, :]
                
                clearer_path, left_edges, right_edges = analyze_path_clearance(
                    roi, width, height - roi_top
                )
                
                face_detected = False
                face_dist = None
                face_pos = 'center'
                obstacle_detected = False
                obstacle_pos = 'center'
                obstacle_dist = 100
                
                # Face Detection
                if has_cascade:
                    faces = face_cascade.detectMultiScale(
                        gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)
                    )
                    
                    if len(faces) > 0:
                        faces = sorted(faces, key=lambda x: x[2], reverse=True)
                        x, y, w, h = faces[0]
                        face_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)
                        face_x = x + w // 2
                        face_detected = True
                        
                        if face_x < width // 3:
                            face_pos = 'left'
                        elif face_x > 2 * width // 3:
                            face_pos = 'right'
                        else:
                            face_pos = 'center'
                
                # Edge-based obstacle detection (fallback)
                if not face_detected:
                    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        area_ratio = area / frame_area
                        if area_ratio > OBSTACLE_AREA_THRESHOLD:
                            obstacle_detected = True
                            x, y, w, h = cv2.boundingRect(contour)
                            cx = x + w // 2
                            if cx < width // 3:
                                obstacle_pos = 'left'
                            elif cx > 2 * width // 3:
                                obstacle_pos = 'right'
                            obstacle_dist = max(0, 100 - (area_ratio * 500))
                            break
                
                with data_lock:
                    shared_state['face_detected'] = face_detected
                    shared_state['face_distance'] = face_dist
                    shared_state['face_position'] = face_pos
                    shared_state['obstacle_detected'] = obstacle_detected
                    shared_state['obstacle_position'] = obstacle_pos
                    shared_state['obstacle_distance'] = obstacle_dist
                    shared_state['clearer_path'] = clearer_path
                
                time.sleep(0.05)
        
        def motor_worker():
            last_command = None
            while shared_state['running']:
                with data_lock:
                    face = shared_state['face_detected']
                    face_dist = shared_state['face_distance']
                    face_pos = shared_state['face_position']
                    obstacle = shared_state['obstacle_detected']
                    obstacle_pos = shared_state['obstacle_position']
                    obstacle_dist = shared_state['obstacle_distance']
                    clearer = shared_state['clearer_path']
                
                # Decision logic with path of least resistance
                
                # Priority 1: Face detection (safety)
                if face and face_dist is not None:
                    if face_dist < DANGER_DISTANCE:
                        command = b'S'
                        cmd_name = f"STOP (face at {face_dist:.0f}cm)"
                    elif face_dist < SAFE_DISTANCE:
                        turn_dir = get_best_turn_direction(face_pos, clearer)
                        command = b'L' if turn_dir == 'left' else b'R'
                        cmd_name = f"{turn_dir.upper()} (face {face_pos}, clearer={clearer})"
                    else:
                        command = b'F'
                        cmd_name = f"FORWARD (face far at {face_dist:.0f}cm)"
                
                # Priority 2: General obstacle
                elif obstacle and obstacle_dist < 30:
                    turn_dir = get_best_turn_direction(obstacle_pos, clearer)
                    command = b'L' if turn_dir == 'left' else b'R'
                    cmd_name = f"{turn_dir.upper()} (obstacle {obstacle_pos}, clearer={clearer})"
                
                elif obstacle and obstacle_dist < 60:
                    command = b'F'
                    cmd_name = "FORWARD (cautious)"
                
                else:
                    command = b'F'
                    cmd_name = "FORWARD"
                
                if command != last_command:
                    arduino.write(command)
                    print(f"  [MOTOR] {cmd_name}")
                    last_command = command
                
                time.sleep(0.1)
        
        print("\nRunning integrated test for 15 seconds...")
        print("Move hand/object/face in front of camera")
        print("Watch how it chooses turn direction based on clearer path!")
        print("-" * 40)
        
        cam_thread = threading.Thread(target=camera_worker, daemon=True)
        motor_thread = threading.Thread(target=motor_worker, daemon=True)
        
        cam_thread.start()
        motor_thread.start()
        
        time.sleep(15)
        
        shared_state['running'] = False
        time.sleep(0.5)
        
        # Cleanup
        print("\nCleaning up...")
        try:
            arduino.write(b'S')
            arduino.close()
            print("  Arduino closed")
        except:
            pass
        
        try:
            picam2.stop()
            picam2.close()
            print("  Camera released")
        except:
            pass
        
        print("-" * 40)
        print("✓ Integrated test complete!")
        
        response = input("\nDid the car respond correctly with smart turning? (y/n): ").strip().lower()
        return response == 'y'
        
    except ImportError as e:
        print(f"✗ Missing library: {e}")
        return False
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
        
        return False


def test_motor_sequence():
    """Test motor sequence for obstacle avoidance maneuvers"""
    print("\n" + "="*60)
    print("TESTING: Motor Navigation Sequence")
    print("="*60)
    print("This runs a predefined movement sequence")
    print("\n⚠ WARNING: The car will move!")
    
    response = input("\nProceed? (y/n): ").strip().lower()
    if response != 'y':
        print("Test skipped")
        return None
    
    try:
        import serial
        
        ports_to_try = [ARDUINO_PORT, '/dev/ttyACM1', '/dev/ttyUSB0']
        arduino = None
        
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
        
        sequence = [
            ("Forward", b'F', 1.5),
            ("Stop", b'S', 0.3),
            ("Turn Right", b'R', 0.6),
            ("Stop", b'S', 0.2),
            ("Forward", b'F', 1.0),
            ("Stop", b'S', 0.3),
            ("Turn Left", b'L', 0.6),
            ("Stop", b'S', 0.2),
            ("Forward", b'F', 1.0),
            ("Backward", b'B', 0.5),
            ("Turn Right", b'R', 0.4),
            ("Forward", b'F', 0.8),
            ("Stop", b'S', 0.5),
        ]
        
        print("\nExecuting sequence...")
        print("-" * 40)
        
        for name, cmd, duration in sequence:
            print(f"  {name}...")
            arduino.write(cmd)
            time.sleep(duration)
        
        arduino.close()
        print("-" * 40)
        print("✓ Sequence complete!")
        
        response = input("\nDid sequence execute correctly? (y/n): ").strip().lower()
        return response == 'y'
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def test_path_clearance():
    """Test path clearance analysis specifically"""
    print("\n" + "="*60)
    print("TESTING: Path Clearance Analysis")
    print("="*60)
    print("This tests the 'path of least resistance' detection")
    print("Block one side of the camera view to see it detect the clearer side")
    input("Press Enter to start test...")
    
    picam2 = None
    
    try:
        import cv2
        from picamera2 import Picamera2
        
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
        
        print("\nAnalyzing path clearance for 30 frames...")
        print("Block LEFT or RIGHT side of camera view to test")
        print("-" * 40)
        
        for i in range(30):
            frame = picam2.capture_array()
            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            roi = edges[height//3:, :]
            
            clearer, left_count, right_count = analyze_path_clearance(
                roi, width, height - height//3
            )
            
            # Visual indicator
            if clearer == 'left':
                indicator = "◀◀◀ LEFT CLEARER"
            elif clearer == 'right':
                indicator = "RIGHT CLEARER ▶▶▶"
            else:
                indicator = "=== EQUAL ==="
            
            print(f"  Frame {i+1:2}/30: L:{left_count:6} R:{right_count:6} | {indicator}")
            time.sleep(0.2)
        
        picam2.stop()
        picam2.close()
        
        print("-" * 40)
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


def main():
    """Run test menu"""
    print("\n" + "="*60)
    print("RC CAR COMPONENT TEST SUITE")
    print("Camera & Motor - Path of Least Resistance")
    print("="*60)
    print("\nConfiguration:")
    print(f"  Arduino Port: {ARDUINO_PORT}")
    print(f"  Arduino Baudrate: {ARDUINO_BAUDRATE}")
    print(f"  Danger Distance: {DANGER_DISTANCE}cm")
    print(f"  Safe Distance: {SAFE_DISTANCE}cm")
    
    while True:
        print("\n" + "-"*40)
        print("Available tests:")
        print("  1. Arduino Motor Control")
        print("  2. Camera & Obstacle Detection")
        print("  3. Path Clearance Analysis")
        print("  4. Threading Basics")
        print("  5. Motor Navigation Sequence")
        print("  6. Integrated Camera + Motor")
        print("  7. Run ALL tests")
        print("  0. Exit")
        
        choice = input("\nEnter test number: ").strip()
        
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
            result = test_motor_sequence()
            if result is not None:
                results['motor_sequence'] = result
        elif choice == '6':
            result = test_integrated()
            if result is not None:
                results['integrated'] = result
        elif choice == '7':
            print("\n" + "="*60)
            print("RUNNING ALL TESTS")
            print("="*60)
            results['threading'] = test_threading()
            results['path_clearance'] = test_path_clearance()
            results['camera'] = test_camera()
            results['arduino'] = test_arduino()
            result = test_motor_sequence()
            if result is not None:
                results['motor_sequence'] = result
            result = test_integrated()
            if result is not None:
                results['integrated'] = result
        elif choice == '0':
            print("\nExiting...")
            break
        else:
            print("Invalid choice!")
            continue
        
        if results:
            print("\n" + "="*60)
            print("TEST SUMMARY")
            print("="*60)
            
            for test_name, passed in results.items():
                if passed is None:
                    status = "⏭ SKIPPED"
                elif passed:
                    status = "✓ PASSED"
                else:
                    status = "✗ FAILED"
                print(f"  {test_name.upper():20} {status}")
            
            actual_results = [v for v in results.values() if v is not None]
            if actual_results and all(actual_results):
                print("\n🎉 All tests passed! Ready to run:")
                print("   sudo python3 rc_car_threaded.py")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
