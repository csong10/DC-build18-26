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
4. Integrated Camera + Motor Test
"""

import sys
import time
import threading

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
    if pixel_width == 0: return float('inf')
    return (known_width * focal_length) / pixel_width

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
    """Test camera and obstacle detection (matching rc_car_threaded.py logic)"""
    print("\n" + "="*60)
    print("TESTING: Camera & Obstacle Detection")
    print("="*60)
    print(f"Face calibration: {FACE_WIDTH_CM}cm at {CALIBRATION_DISTANCE_CM}cm = {CALIBRATION_PIX_WIDTH}px")
    print(f"Danger distance: {DANGER_DISTANCE}cm")
    print(f"Safe distance: {SAFE_DISTANCE}cm")
    print("\nPosition objects/face in front of the camera to test detection")
    input("Press Enter to start test...")
    
    try:
        import cv2
        from picamera2 import Picamera2
        import numpy as np
        
        # Check for Haar cascade file
        cascade_path = 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)
        has_cascade = not face_cascade.empty()
        
        if has_cascade:
            print("✓ Haar cascade loaded for face detection")
        else:
            print("⚠ Haar cascade not found - face detection disabled")
            print("  Download from: https://github.com/opencv/opencv/tree/master/data/haarcascades")
        
        # Calculate focal length (same as main script)
        focal_length = calculate_focal_length(
            CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH
        )
        print(f"  Calculated focal length: {focal_length:.2f}")
        
        # Try to release camera if it's busy
        release_camera()
        
        # Initialize camera with retry
        print("\nInitializing camera...")
        picam2 = None
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
            
            # --- Face Detection (same logic as main script) ---
            if has_cascade:
                faces = face_cascade.detectMultiScale(
                    gray, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30)
                )
                
                if len(faces) > 0:
                    faces = sorted(faces, key=lambda x: x[2], reverse=True)
                    x, y, w, h = faces[0]
                    face_dist = distance_to_camera(FACE_WIDTH_CM, focal_length, w)
                    face_x = x + w // 2
                    
                    status = "DANGER!" if face_dist < DANGER_DISTANCE else \
                             "CAUTION" if face_dist < SAFE_DISTANCE else "OK"
                    
                    print(f"  Frame {i+1:2}/50: FACE at {face_dist:.1f}cm, x={face_x} [{status}]")
                    detections['face'] += 1
                    detected_this_frame = True
            
            # --- Obstacle Detection (same logic as main script) ---
            if not detected_this_frame:
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blurred, 50, 150)
                
                roi_top = height // 3
                roi = edges[roi_top:, :]
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
                        print(f"  Frame {i+1:2}/50: OBSTACLE at {position} "
                              f"(size: {area_ratio*100:.1f}%, dist: {dist_est:.0f})")
                        detections['obstacle'] += 1
                        detected_this_frame = True
                        break
            
            if not detected_this_frame:
                detections['clear'] += 1
                if i % 10 == 0:
                    print(f"  Frame {i+1:2}/50: Clear")
            
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
        
        # Try to release camera on error
        try:
            if picam2:
                picam2.stop()
                picam2.close()
        except:
            pass
        
        return False


def test_threading():
    """Test basic threading functionality (matching main script architecture)"""
    print("\n" + "="*60)
    print("TESTING: Threading Basics")
    print("="*60)
    print("Testing thread synchronization with shared state...")
    
    try:
        # Simulate the shared state pattern from rc_car_threaded.py
        data_lock = threading.Lock()
        shared_state = {
            'obstacle_detected': False,
            'obstacle_position': 'center',
            'face_distance': None,
            'running': True
        }
        results = {'camera_updates': 0, 'motor_commands': 0}
        
        def mock_camera_thread():
            """Simulates camera thread"""
            count = 0
            while shared_state['running'] and count < 20:
                with data_lock:
                    # Simulate detection changes
                    shared_state['obstacle_detected'] = (count % 5 == 0)
                    shared_state['obstacle_position'] = ['left', 'center', 'right'][count % 3]
                    results['camera_updates'] += 1
                time.sleep(0.05)
                count += 1
        
        def mock_motor_thread():
            """Simulates motor thread"""
            count = 0
            last_command = None
            while shared_state['running'] and count < 10:
                with data_lock:
                    obstacle = shared_state['obstacle_detected']
                    position = shared_state['obstacle_position']
                
                # Decision logic similar to main script
                if obstacle:
                    if position == 'left':
                        command = 'right'
                    elif position == 'right':
                        command = 'left'
                    else:
                        command = 'stop'
                else:
                    command = 'forward'
                
                if command != last_command:
                    results['motor_commands'] += 1
                    last_command = command
                
                time.sleep(0.1)
                count += 1
        
        print("\nStarting mock camera and motor threads...")
        
        t1 = threading.Thread(target=mock_camera_thread, name="MockCamera", daemon=True)
        t2 = threading.Thread(target=mock_motor_thread, name="MockMotor", daemon=True)
        
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
            print("\n✗ Threading test failed - unexpected counts")
            return False
            
    except Exception as e:
        print(f"✗ Threading error: {e}")
        return False


def release_camera():
    """Helper function to release any existing camera instances"""
    import subprocess
    try:
        # Kill any existing picamera/libcamera processes
        subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], capture_output=True)
        time.sleep(0.5)
    except:
        pass

def test_integrated():
    """Test camera and motor working together"""
    print("\n" + "="*60)
    print("TESTING: Integrated Camera + Motor")
    print("="*60)
    print("This will test camera detection controlling motor commands")
    print("The car will move forward and stop/turn when obstacles are detected")
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
        
        # Try to release camera if it's busy
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
                        "Camera is busy. Try running:\n"
                        "  sudo pkill -f libcamera\n"
                        "  sudo pkill -f python3\n"
                        "Or reboot: sudo reboot"
                    ) from e
        
        # Shared state (matching main script)
        data_lock = threading.Lock()
        shared_state = {
            'obstacle_detected': False,
            'obstacle_position': 'center',
            'obstacle_distance': 100,
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
                contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                detected = False
                pos = 'center'
                dist = 100
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    area_ratio = area / frame_area
                    if area_ratio > OBSTACLE_AREA_THRESHOLD:
                        detected = True
                        x, y, w, h = cv2.boundingRect(contour)
                        cx = x + w // 2
                        if cx < width // 3:
                            pos = 'left'
                        elif cx > 2 * width // 3:
                            pos = 'right'
                        dist = max(0, 100 - (area_ratio * 500))
                        break
                
                with data_lock:
                    shared_state['obstacle_detected'] = detected
                    shared_state['obstacle_position'] = pos
                    shared_state['obstacle_distance'] = dist
                
                time.sleep(0.05)
        
        def motor_worker():
            last_command = None
            while shared_state['running']:
                with data_lock:
                    detected = shared_state['obstacle_detected']
                    position = shared_state['obstacle_position']
                    distance = shared_state['obstacle_distance']
                
                # Decision logic matching main script
                if detected and distance < 30:
                    if position == 'left':
                        command = b'R'
                        cmd_name = "RIGHT"
                    elif position == 'right':
                        command = b'L'
                        cmd_name = "LEFT"
                    else:
                        command = b'S'
                        cmd_name = "STOP"
                elif detected and distance < 60:
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
        
        print("\nRunning integrated test for 10 seconds...")
        print("Move hand/object in front of camera to see motor response")
        print("-" * 40)
        
        cam_thread = threading.Thread(target=camera_worker, daemon=True)
        motor_thread = threading.Thread(target=motor_worker, daemon=True)
        
        cam_thread.start()
        motor_thread.start()
        
        time.sleep(10)
        
        shared_state['running'] = False
        time.sleep(0.5)
        
        # Cleanup - ensure resources are released
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
        
        response = input("\nDid the car respond correctly to obstacles? (y/n): ").strip().lower()
        return response == 'y'
        
    except ImportError as e:
        print(f"✗ Missing library: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        
        # Cleanup on error
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
    """Test a predefined motor sequence for obstacle avoidance maneuvers"""
    print("\n" + "="*60)
    print("TESTING: Motor Navigation Sequence")
    print("="*60)
    print("This runs a predefined movement sequence to verify")
    print("motor control for obstacle avoidance maneuvers")
    print("\n⚠ WARNING: The car will move! Keep it on a safe surface!")
    
    response = input("\nProceed with motor sequence test? (y/n): ").strip().lower()
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
        
        # Simulate obstacle avoidance sequence
        sequence = [
            ("Forward", b'F', 1.5, "Moving forward..."),
            ("Stop", b'S', 0.3, "Stopping (obstacle ahead)..."),
            ("Turn Right", b'R', 0.6, "Turning right to avoid..."),
            ("Stop", b'S', 0.2, "Stop turning..."),
            ("Forward", b'F', 1.0, "Continue forward..."),
            ("Stop", b'S', 0.3, "Stopping (another obstacle)..."),
            ("Turn Left", b'L', 0.6, "Turning left to avoid..."),
            ("Stop", b'S', 0.2, "Stop turning..."),
            ("Forward", b'F', 1.0, "Continue forward..."),
            ("Backward", b'B', 0.5, "Backing up (stuck)..."),
            ("Turn Right", b'R', 0.4, "Turning..."),
            ("Forward", b'F', 0.8, "Moving forward..."),
            ("Stop", b'S', 0.5, "Final stop"),
        ]
        
        print("\nExecuting obstacle avoidance sequence...")
        print("-" * 40)
        
        for name, cmd, duration, description in sequence:
            print(f"  {description}")
            arduino.write(cmd)
            time.sleep(duration)
        
        arduino.close()
        print("-" * 40)
        print("✓ Motor sequence test complete!")
        
        response = input("\nDid the sequence execute correctly? (y/n): ").strip().lower()
        return response == 'y'
        
    except ImportError:
        print("✗ pyserial not installed")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    """Run test menu"""
    print("\n" + "="*60)
    print("RC CAR COMPONENT TEST SUITE")
    print("Camera & Motor Focus (Bluetooth Disabled)")
    print("="*60)
    print("\nConfiguration:")
    print(f"  Arduino Port: {ARDUINO_PORT}")
    print(f"  Arduino Baudrate: {ARDUINO_BAUDRATE}")
    print(f"  Danger Distance: {DANGER_DISTANCE}cm")
    print(f"  Safe Distance: {SAFE_DISTANCE}cm")
    print("\nThis script helps verify each component before running")
    print("the full obstacle avoidance system.")
    
    while True:
        print("\n" + "-"*40)
        print("Available tests:")
        print("  1. Arduino Motor Control")
        print("  2. Camera & Obstacle Detection")
        print("  3. Threading Basics")
        print("  4. Motor Navigation Sequence")
        print("  5. Integrated Camera + Motor")
        print("  6. Run ALL tests")
        print("  0. Exit")
        
        choice = input("\nEnter test number: ").strip()
        
        results = {}
        
        if choice == '1':
            results['arduino'] = test_arduino()
        elif choice == '2':
            results['camera'] = test_camera()
        elif choice == '3':
            results['threading'] = test_threading()
        elif choice == '4':
            result = test_motor_sequence()
            if result is not None:
                results['motor_sequence'] = result
        elif choice == '5':
            result = test_integrated()
            if result is not None:
                results['integrated'] = result
        elif choice == '6':
            print("\n" + "="*60)
            print("RUNNING ALL TESTS")
            print("="*60)
            results['threading'] = test_threading()
            results['camera'] = test_camera()
            results['arduino'] = test_arduino()
            result = test_motor_sequence()
            if result is not None:
                results['motor_sequence'] = result
            result = test_integrated()
            if result is not None:
                results['integrated'] = result
        elif choice == '0':
            print("\nExiting test suite...")
            break
        else:
            print("Invalid choice!")
            continue
        
        # Print summary if tests were run
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
            elif any(v == False for v in actual_results):
                print("\n⚠ Some tests failed. Fix issues before running full system.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
