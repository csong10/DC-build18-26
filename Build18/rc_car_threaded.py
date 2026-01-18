import threading
import time
import cv2
import numpy as np
import serial
from queue import Queue
from bluepy.btle import Scanner
from picamera2 import Picamera2

# ===================== CONFIGURATION ===================== #

# Bluetooth Beacon Config
TARGET_UUID = "DC672026BD1866667777674206742067".lower()
SAMPLES_PER_READING = 5
PATH_LOSS_N = 2.0

# Camera Config
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# UART Config
UART_PORT = '/dev/ttyAMA0'  # Default UART port on Raspberry Pi
UART_BAUDRATE = 115200       # Adjust to match your STM32 configuration

# Decision Thresholds
MIN_BLUETOOTH_DISTANCE = 0.5  # meters - stop when this close to beacon
DANGER_DISTANCE = 80.0        # cm - stop immediately if obstacle this close
SAFE_DISTANCE = 150.0         # cm - obstacle far enough to proceed

# ===================== BLUETOOTH BEACON FUNCTIONS ===================== #

def parse_ibeacon(scan_entry):
    """Extract iBeacon data from scan entry"""
    for (adtype, desc, value) in scan_entry.getScanData():
        if desc == "Manufacturer":
            raw = value.lower()
            if raw.startswith("4c000215") and len(raw) >= 50:
                uuid = raw[8:40]
                major = int(raw[40:44], 16)
                minor = int(raw[44:48], 16)
                tx_power = int(raw[48:50], 16)
                if tx_power > 127:
                    tx_power -= 256
                return uuid, major, minor, tx_power
    return None

def estimate_distance(rssi: float, tx_power: int, n: float = PATH_LOSS_N):
    """Estimate distance using log-distance path loss model"""
    if rssi == 0:
        return None
    ratio_db = tx_power - rssi
    distance = 10 ** (ratio_db / (10 * n))
    return distance

# ===================== CAMERA FUNCTIONS ===================== #

def calculate_focal_length(known_dist, known_width, width_pix):
    """Calculate focal length for distance estimation"""
    return (width_pix * known_dist) / known_width

def distance_to_camera(known_width, focal_length, pixel_width):
    """Calculate distance to object based on pixel width"""
    if pixel_width == 0:
        return 0
    return (known_width * focal_length) / pixel_width

# ===================== MAIN RC CAR CONTROLLER ===================== #

class RCCarController:
    def __init__(self):
        # Shared data between threads
        self.bluetooth_distance = None
        self.bluetooth_rssi = None
        self.closest_face_distance = None
        self.closest_face_x = None
        self.face_frame_width = None
        self.running = True
        
        # Thread synchronization
        self.data_lock = threading.Lock()
        
        # UART for STM32 communication
        self.uart = None
        self.command_queue = Queue()
        
        # Components
        self.scanner = None
        self.picam2 = None
        self.face_cascade = None
        self.focal_length = None
        
        # Mode tracking
        self.current_mode = "SEARCHING"  # SEARCHING, FOLLOWING_BEACON, AVOIDING_OBSTACLE
        
    def initialize_components(self):
        """Initialize all hardware components"""
        print("[INIT] Initializing components...")
        
        # Initialize UART
        try:
            self.uart = serial.Serial(UART_PORT, UART_BAUDRATE, timeout=1)
            print(f"[INIT] UART initialized on {UART_PORT} at {UART_BAUDRATE} baud")
        except Exception as e:
            print(f"[ERROR] UART initialization failed: {e}")
            print("[WARNING] Running without UART - commands will be printed only")
        
        # Initialize Bluetooth Scanner
        try:
            self.scanner = Scanner()
            print("[INIT] Bluetooth scanner initialized")
        except Exception as e:
            print(f"[ERROR] Bluetooth scanner initialization failed: {e}")
        
        # Initialize Camera
        try:
            self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            if self.face_cascade.empty():
                raise Exception("Haar cascade XML file not found")
            
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            
            # Calculate focal length for distance estimation
            self.focal_length = calculate_focal_length(
                CALIBRATION_DISTANCE_CM, 
                FACE_WIDTH_CM, 
                CALIBRATION_PIX_WIDTH
            )
            
            print("[INIT] Camera initialized with face detection")
        except Exception as e:
            print(f"[ERROR] Camera initialization failed: {e}")
        
        print("[INIT] All components initialized\n")
    
    # ===================== BLUETOOTH THREAD ===================== #
    
    def bluetooth_thread(self):
        """Continuously scan for Bluetooth beacon and estimate distance"""
        print("[BT] Bluetooth thread started")
        
        while self.running:
            try:
                if self.scanner is None:
                    time.sleep(1)
                    continue
                
                rssi_values = []
                tx_power_values = []
                
                # Take multiple samples for averaging
                for _ in range(SAMPLES_PER_READING):
                    if not self.running:
                        break
                    
                    devices = self.scanner.scan(0.4)
                    for dev in devices:
                        data = parse_ibeacon(dev)
                        if data:
                            uuid, major, minor, tx_power = data
                            if uuid == TARGET_UUID:
                                rssi_values.append(dev.rssi)
                                tx_power_values.append(tx_power)
                    time.sleep(0.05)
                
                # Calculate average and distance
                if rssi_values:
                    avg_rssi = sum(rssi_values) / len(rssi_values)
                    avg_tx_power = int(sum(tx_power_values) / len(tx_power_values))
                    distance = estimate_distance(avg_rssi, avg_tx_power)
                    
                    with self.data_lock:
                        self.bluetooth_distance = distance
                        self.bluetooth_rssi = avg_rssi
                else:
                    with self.data_lock:
                        self.bluetooth_distance = None
                        self.bluetooth_rssi = None
                
                time.sleep(0.1)  # Brief pause between scan cycles
                
            except Exception as e:
                print(f"[BT ERROR] {e}")
                time.sleep(1)
        
        print("[BT] Bluetooth thread stopped")
    
    # ===================== CAMERA THREAD ===================== #
    
    def camera_thread(self):
        """Continuously process camera feed for face detection"""
        print("[CAM] Camera thread started")
        
        while self.running:
            try:
                if self.picam2 is None or self.face_cascade is None:
                    time.sleep(1)
                    continue
                
                # Capture frame
                frame = self.picam2.capture_array()
                if frame is None:
                    continue
                
                # Convert to grayscale for face detection
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                
                # Detect faces
                faces = self.face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.05,
                    minNeighbors=4,
                    minSize=(30, 30)
                )
                
                # Process detected faces
                if len(faces) > 0:
                    detected_objects = []
                    for (x, y, w, h) in faces:
                        dist = distance_to_camera(FACE_WIDTH_CM, self.focal_length, w)
                        detected_objects.append((dist, x, y, w, h))
                    
                    # Sort by distance (closest first)
                    detected_objects.sort(key=lambda x: x[0])
                    
                    # Get closest face
                    closest = detected_objects[0]
                    dist, x, y, w, h = closest
                    
                    with self.data_lock:
                        self.closest_face_distance = dist
                        self.closest_face_x = x + w // 2  # Center X coordinate
                        self.face_frame_width = frame.shape[1]  # Frame width for centering
                else:
                    with self.data_lock:
                        self.closest_face_distance = None
                        self.closest_face_x = None
                        self.face_frame_width = None
                
                time.sleep(0.05)  # ~20 FPS camera processing
                
            except Exception as e:
                print(f"[CAM ERROR] {e}")
                time.sleep(0.5)
        
        print("[CAM] Camera thread stopped")
    
    # ===================== DECISION & MOTOR CONTROL THREAD ===================== #
    
    def motor_control_thread(self):
        """Make decisions and send commands to STM32 based on sensor data"""
        print("[MOTOR] Motor control thread started")
        
        while self.running:
            try:
                # Read all sensor data safely
                with self.data_lock:
                    bt_distance = self.bluetooth_distance
                    bt_rssi = self.bluetooth_rssi
                    face_distance = self.closest_face_distance
                    face_x = self.closest_face_x
                    frame_width = self.face_frame_width
                
                command = None
                
                # Decision Logic - Priority System
                # 1. HIGHEST PRIORITY: Avoid obstacles (faces)
                if face_distance is not None and face_x is not None and frame_width is not None:
                    self.current_mode = "AVOIDING_OBSTACLE"
                    command = self.avoid_obstacle(face_distance, face_x, frame_width)
                
                # 2. If no obstacles, follow beacon
                elif bt_distance is not None:
                    self.current_mode = "FOLLOWING_BEACON"
                    command = self.decide_beacon_following(bt_distance)
                
                # 3. No beacon and no obstacles - search
                else:
                    self.current_mode = "SEARCHING"
                    command = "search"
                
                # Send command to STM32
                if command:
                    self.send_command(command)
                
                time.sleep(0.02)  # 50Hz motor control update rate
                
            except Exception as e:
                print(f"[MOTOR ERROR] {e}")
                time.sleep(0.5)
        
        # Stop motors when exiting
        self.send_command("stop")
        print("[MOTOR] Motor control thread stopped")
    
    def avoid_obstacle(self, distance, face_x, frame_width):
        """Determine motor command to avoid detected obstacle (face)"""
        frame_center = frame_width // 2
        x_offset = face_x - frame_center
        
        # Danger zone distances
        DANGER_DISTANCE = 80.0  # cm - stop if obstacle is this close
        SAFE_DISTANCE = 150.0   # cm - can proceed if obstacle is far enough
        
        print(f"[OBSTACLE] Distance: {distance:.1f}cm | X-offset: {x_offset:+4d}px | Mode: {self.current_mode}")
        
        # If obstacle is very close, stop immediately
        if distance < DANGER_DISTANCE:
            print(f"[OBSTACLE] TOO CLOSE! Stopping.")
            return "stop"
        
        # If obstacle is in the way but not too close, navigate around it
        elif distance < SAFE_DISTANCE:
            # Turn away from the obstacle
            # If obstacle is on the right (+offset), turn left
            # If obstacle is on the left (-offset), turn right
            if x_offset > 0:
                print(f"[OBSTACLE] Avoiding - turning left")
                return "left"
            else:
                print(f"[OBSTACLE] Avoiding - turning right")
                return "right"
        
        # Obstacle is far enough - continue with beacon following if available
        # This allows the motor thread to check beacon on next iteration
        else:
            return None  # Let beacon following take over
    
    def decide_beacon_following(self, distance):
        """Determine motor command based on beacon distance"""
        print(f"[BEACON] Distance: {distance:.2f}m | Mode: {self.current_mode}")
        
        # Move toward beacon until minimum distance reached
        if distance > MIN_BLUETOOTH_DISTANCE:
            return "forward"
        else:
            return "stop"  # Close enough to beacon
    
    def send_command(self, command):
        """Send command string to STM32 via UART"""
        try:
            if self.uart and self.uart.is_open:
                # Send command with newline terminator
                message = f"{command}\n"
                self.uart.write(message.encode('utf-8'))
                print(f"[UART] Sent: {command}")
            else:
                # Fallback: just print if UART not available
                print(f"[UART DISABLED] Would send: {command}")
        except Exception as e:
            print(f"[UART ERROR] {e}")
    
    # ===================== MAIN CONTROL ===================== #
    
    def start(self):
        """Initialize and start all threads"""
        # Initialize hardware
        self.initialize_components()
        
        # Create threads
        bt_thread = threading.Thread(target=self.bluetooth_thread, name="Bluetooth", daemon=True)
        cam_thread = threading.Thread(target=self.camera_thread, name="Camera", daemon=True)
        motor_thread = threading.Thread(target=self.motor_control_thread, name="Motors", daemon=True)
        
        # Start all threads
        bt_thread.start()
        cam_thread.start()
        motor_thread.start()
        
        print("\n" + "="*60)
        print("RC CAR SYSTEM RUNNING")
        print("="*60)
        print("Priority: 1. Avoid Obstacles  2. Follow Beacon  3. Search")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        # Keep main thread alive
        try:
            while True:
                time.sleep(1)
                # Optional: Print periodic status
                with self.data_lock:
                    print(f"[STATUS] Mode: {self.current_mode} | "
                          f"BT: {'Yes' if self.bluetooth_distance else 'No'} | "
                          f"Obstacle: {'Yes' if self.closest_face_distance else 'No'}")
        except KeyboardInterrupt:
            print("\n[STOP] Shutting down...")
            self.stop()
    
    def stop(self):
        """Stop all threads and cleanup"""
        self.running = False
        time.sleep(0.5)  # Give threads time to finish
        
        # Stop camera
        if self.picam2:
            try:
                self.picam2.stop()
                print("[CLEANUP] Camera stopped")
            except:
                pass
        
        # Close UART
        if self.uart and self.uart.is_open:
            try:
                self.uart.close()
                print("[CLEANUP] UART closed")
            except:
                pass
        
        print("[CLEANUP] All systems stopped")

# ===================== ENTRY POINT ===================== #

def main():
    car = RCCarController()
    car.start()

if __name__ == "__main__":
    main()
