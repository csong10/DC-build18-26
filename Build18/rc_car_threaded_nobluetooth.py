#!/usr/bin/env python3
"""
RC Car Threaded Controller - Camera-Based Obstacle Avoidance
=============================================================

This script controls an RC car using:
- Camera vision for obstacle detection (faces + general obstacles)
- Motor control via Arduino USB Serial
- Threaded architecture for concurrent processing

Based on original rc_car_threaded.py with Bluetooth removed.

Run with: sudo python3 rc_car_threaded.py
"""

import threading
import time
import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# ===================== CONFIGURATION ===================== #

# Camera Config (preserved from original)
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# ARDUINO CONNECTION Config (preserved from original)
ARDUINO_PORT = '/dev/ttyACM0'  
ARDUINO_BAUDRATE = 115200        

# Decision Thresholds (preserved from original)
DANGER_DISTANCE = 40.0        # Stop if obstacle closer than this (cm)
SAFE_DISTANCE = 100.0         # Slow down / be cautious under this distance

# Obstacle Avoidance Config (NEW)
OBSTACLE_AREA_THRESHOLD = 0.08    # Minimum contour area ratio to detect as obstacle (8% of frame)
OBSTACLE_CLOSE_THRESHOLD = 0.15   # Contour area ratio indicating obstacle is very close (15%)
FORWARD_DURATION = 2.0            # How long to move forward before checking (seconds)
TURN_DURATION = 0.5               # How long to turn when avoiding (seconds)
SEARCH_TURN_DURATION = 0.3        # How long to turn when searching for path

# ===================== CAMERA FUNCTIONS ===================== #

def calculate_focal_length(known_dist, known_width, width_pix):
    """Calculate camera focal length from calibration values"""
    return (width_pix * known_dist) / known_width

def distance_to_camera(known_width, focal_length, pixel_width):
    """Estimate distance to object based on its pixel width"""
    if pixel_width == 0: 
        return float('inf')
    return (known_width * focal_length) / pixel_width

# ===================== MAIN CONTROLLER ===================== #

class RCCarController:
    def __init__(self):
        # Shared data
        self.closest_face_distance = None
        self.closest_face_x = None
        self.face_frame_width = None
        
        # Obstacle detection data (NEW)
        self.obstacle_detected = False
        self.obstacle_position = "center"  # "left", "center", "right"
        self.obstacle_distance_estimate = 100.0  # 0-100 scale (lower = closer)
        
        self.running = True
        
        # Locks & Communication
        self.data_lock = threading.Lock()
        self.arduino = None
        
        # Components
        self.picam2 = None
        self.face_cascade = None
        self.focal_length = None
        self.current_mode = "STARTING"
        
        # Command Mapping (Python String -> Arduino Char)
        self.CMD_MAP = {
            "forward": b'F',
            "backward": b'B',
            "left": b'L',
            "right": b'R',
            "stop": b'S',
        }

    def initialize_components(self):
        print("[INIT] Initializing components...")
        
        # 1. Connect to Arduino via USB
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
            time.sleep(2)  # CRITICAL: Wait for Arduino to reboot after USB connection
            print(f"[INIT] ✓ Arduino connected on {ARDUINO_PORT}")
        except Exception as e:
            print(f"[ERROR] Arduino connection failed: {e}")
            print("  -> Is the USB cable plugged in?")
            print("  -> Did you upload the Arduino code?")
            return False

        # 2. Camera
        try:
            self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            if self.face_cascade.empty():
                print("[WARN] Haar cascade XML file missing - face detection disabled")
                print("  -> Download from: https://github.com/opencv/opencv/tree/master/data/haarcascades")
                self.face_cascade = None
            
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)  # Let camera warm up
            
            self.focal_length = calculate_focal_length(
                CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH
            )
            print("[INIT] ✓ Camera initialized")
        except Exception as e:
            print(f"[ERROR] Camera failed: {e}")
            return False
        
        print("[INIT] ✓ All components ready!")
        return True

    # --- CAMERA THREAD ---

    def camera_thread(self):
        """
        Camera thread that detects:
        1. Faces (using Haar cascade) - for distance estimation
        2. General obstacles (using edge detection + contours)
        """
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
                
                # --- Face Detection ---
                face_dist = None
                face_x = None
                
                if self.face_cascade is not None:
                    faces = self.face_cascade.detectMultiScale(
                        gray, 
                        scaleFactor=1.05, 
                        minNeighbors=4, 
                        minSize=(30, 30)
                    )
                    
                    if len(faces) > 0:
                        # Sort faces by width (largest = closest)
                        faces = sorted(faces, key=lambda x: x[2], reverse=True)
                        x, y, w, h = faces[0]
                        face_dist = distance_to_camera(FACE_WIDTH_CM, self.focal_length, w)
                        face_x = x + w // 2
                
                # --- General Obstacle Detection (Edge + Contour) ---
                obstacle_detected = False
                obstacle_position = "center"
                obstacle_distance = 100.0
                
                # Apply Gaussian blur and edge detection
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blurred, 50, 150)
                
                # Focus on lower 2/3 of frame (ground-level obstacles)
                roi_top = height // 3
                roi = edges[roi_top:, :]
                
                # Find contours
                contours, _ = cv2.findContours(
                    roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                
                # Find significant contours
                significant_contours = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    area_ratio = area / frame_area
                    if area_ratio > OBSTACLE_AREA_THRESHOLD:
                        significant_contours.append((contour, area_ratio))
                
                if significant_contours:
                    # Find the largest obstacle
                    largest = max(significant_contours, key=lambda x: x[1])
                    contour, area_ratio = largest
                    
                    # Get bounding box to determine position
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    
                    # Determine position
                    if center_x < width // 3:
                        obstacle_position = "left"
                    elif center_x > 2 * width // 3:
                        obstacle_position = "right"
                    else:
                        obstacle_position = "center"
                    
                    # Estimate distance (inverse of area ratio)
                    obstacle_distance = max(0, 100 - (area_ratio * 500))
                    obstacle_detected = True
                
                # --- Update shared state ---
                with self.data_lock:
                    self.closest_face_distance = face_dist
                    self.closest_face_x = face_x
                    self.face_frame_width = width
                    
                    self.obstacle_detected = obstacle_detected
                    self.obstacle_position = obstacle_position
                    self.obstacle_distance_estimate = obstacle_distance
                
                time.sleep(0.05)  # ~20 FPS
                
            except Exception as e:
                print(f"[CAM ERR] {e}")
                time.sleep(0.5)
        
        print("[CAM] Thread stopped")

    # --- MOTOR CONTROL THREAD ---

    def motor_control_thread(self):
        """
        Motor control thread with obstacle avoidance logic.
        
        Priority:
        1. Face detected close -> STOP (safety for people)
        2. Obstacle detected -> AVOID (turn away)
        3. Path clear -> FORWARD
        """
        print("[MOTOR] Thread started")
        last_command = None
        avoiding_since = None

        while self.running:
            with self.data_lock:
                face_dist = self.closest_face_distance
                face_x = self.closest_face_x
                frame_width = self.face_frame_width
                
                obstacle = self.obstacle_detected
                obstacle_pos = self.obstacle_position
                obstacle_dist = self.obstacle_distance_estimate

            command = "forward"  # Default: move forward
            
            # === PRIORITY 1: Face Detection (Safety) ===
            if face_dist is not None:
                if face_dist < DANGER_DISTANCE:
                    # Person too close - STOP
                    self.current_mode = "STOPPED_FACE"
                    command = "stop"
                    print(f"[MOTOR] Face detected at {face_dist:.1f}cm - STOPPING")
                    
                elif face_dist < SAFE_DISTANCE:
                    # Person in view, not too close - turn away
                    self.current_mode = "AVOIDING_FACE"
                    if frame_width and face_x:
                        center = frame_width // 2
                        if face_x > center:
                            command = "left"   # Face on right, turn left
                        else:
                            command = "right"  # Face on left, turn right
                    else:
                        command = "right"  # Default turn
                    print(f"[MOTOR] Face at {face_dist:.1f}cm, position={face_x} - turning {command}")
            
            # === PRIORITY 2: Obstacle Avoidance ===
            elif obstacle:
                if obstacle_dist < 30:
                    # Obstacle very close - stop and turn
                    self.current_mode = "AVOIDING_OBSTACLE"
                    
                    if avoiding_since is None:
                        avoiding_since = time.time()
                        command = "stop"
                        print(f"[MOTOR] Obstacle at {obstacle_pos}! Stopping...")
                    
                    elif time.time() - avoiding_since < TURN_DURATION:
                        # Turn away from obstacle
                        if obstacle_pos == "left":
                            command = "right"
                        elif obstacle_pos == "right":
                            command = "left"
                        else:
                            command = "right"  # Center obstacle, default right
                        print(f"[MOTOR] Turning {command} to avoid {obstacle_pos} obstacle")
                    
                    elif time.time() - avoiding_since < TURN_DURATION * 2:
                        # Continue turning
                        if obstacle_pos == "left":
                            command = "right"
                        elif obstacle_pos == "right":
                            command = "left"
                        else:
                            command = "right"
                    
                    else:
                        # Been avoiding too long, try backing up
                        if time.time() - avoiding_since < TURN_DURATION * 3:
                            command = "backward"
                            print("[MOTOR] Backing up...")
                        else:
                            avoiding_since = None  # Reset and try again
                
                elif obstacle_dist < 60:
                    # Obstacle approaching - slow down (shorter movements)
                    self.current_mode = "CAUTIOUS"
                    avoiding_since = None
                    command = "forward"
                    print(f"[MOTOR] Obstacle ahead ({obstacle_dist:.0f}) - cautious forward")
                
                else:
                    # Obstacle detected but far away
                    avoiding_since = None
                    command = "forward"
            
            # === PRIORITY 3: Path Clear ===
            else:
                self.current_mode = "FORWARD"
                avoiding_since = None
                command = "forward"

            # Send command only if it changed
            if command != last_command:
                self.send_to_arduino(command)
                last_command = command
            
            # Adjust sleep based on mode
            if self.current_mode == "CAUTIOUS":
                time.sleep(0.15)  # Slower updates when being cautious
            else:
                time.sleep(0.1)

        # Ensure motors stop on exit
        self.send_to_arduino("stop")
        print("[MOTOR] Thread stopped")

    def send_to_arduino(self, command_str):
        """Maps string command to byte character and sends via USB"""
        if self.arduino and self.arduino.is_open:
            if command_str in self.CMD_MAP:
                byte_cmd = self.CMD_MAP[command_str]
                self.arduino.write(byte_cmd)
                print(f"[ARDUINO] Sent: {command_str.upper()} -> {byte_cmd}")
            else:
                print(f"[ARDUINO] Unknown command: {command_str}")

    def status_thread(self):
        """Periodic status updates"""
        print("[STATUS] Thread started")
        while self.running:
            with self.data_lock:
                face_dist = self.closest_face_distance
                obstacle = self.obstacle_detected
                obstacle_pos = self.obstacle_position
                obstacle_dist = self.obstacle_distance_estimate
            
            status_parts = [f"Mode: {self.current_mode}"]
            
            if face_dist:
                status_parts.append(f"Face: {face_dist:.1f}cm")
            
            if obstacle:
                status_parts.append(f"Obstacle: {obstacle_pos} ({obstacle_dist:.0f})")
            
            print(f"[STATUS] {' | '.join(status_parts)}")
            time.sleep(2)
        
        print("[STATUS] Thread stopped")

    def start(self):
        """Initialize and start all threads"""
        if not self.initialize_components():
            print("[ERROR] Initialization failed - exiting")
            return
        
        # Create threads
        threads = [
            threading.Thread(target=self.camera_thread, daemon=True, name="Camera"),
            threading.Thread(target=self.motor_control_thread, daemon=True, name="Motor"),
            threading.Thread(target=self.status_thread, daemon=True, name="Status"),
        ]
        
        # Start all threads
        for t in threads:
            t.start()
        
        print("\n" + "="*50)
        print("=== RC CAR OBSTACLE AVOIDANCE RUNNING ===")
        print("="*50)
        print("Modes: FORWARD | CAUTIOUS | AVOIDING | STOPPED")
        print("Press Ctrl+C to stop")
        print("="*50 + "\n")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[MAIN] Shutdown requested...")
            self.stop()

    def stop(self):
        """Clean shutdown"""
        print("[MAIN] Stopping all threads...")
        self.running = False
        time.sleep(0.5)
        
        if self.arduino:
            self.arduino.write(b'S')  # Ensure stop
            self.arduino.close()
            print("[MAIN] Arduino disconnected")
        
        if self.picam2:
            self.picam2.stop()
            print("[MAIN] Camera stopped")
        
        print("[MAIN] System stopped.")


if __name__ == "__main__":
    controller = RCCarController()
    controller.start()
