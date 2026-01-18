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

# ARDUINO CONNECTION Config
# Raspberry Pi usually assigns /dev/ttyACM0 to the first USB Arduino
ARDUINO_PORT = '/dev/ttyACM0'  
ARDUINO_BAUDRATE = 115200        

# Decision Thresholds
MIN_BLUETOOTH_DISTANCE = 0.5  
DANGER_DISTANCE = 40.0        
SAFE_DISTANCE = 100.0         

# ===================== BLUETOOTH FUNCTIONS ===================== #
# (Kept identical to your original)
def parse_ibeacon(scan_entry):
    for (adtype, desc, value) in scan_entry.getScanData():
        if desc == "Manufacturer":
            raw = value.lower()
            if raw.startswith("4c000215") and len(raw) >= 50:
                uuid = raw[8:40]
                major = int(raw[40:44], 16)
                minor = int(raw[44:48], 16)
                tx_power = int(raw[48:50], 16)
                if tx_power > 127: tx_power -= 256
                return uuid, major, minor, tx_power
    return None

def estimate_distance(rssi: float, tx_power: int, n: float = PATH_LOSS_N):
    if rssi == 0: return None
    ratio_db = tx_power - rssi
    return 10 ** (ratio_db / (10 * n))

# ===================== CAMERA FUNCTIONS ===================== #
def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

# ===================== MAIN CONTROLLER ===================== #

class RCCarController:
    def __init__(self):
        # Shared data
        self.bluetooth_distance = None
        self.bluetooth_rssi = None
        self.closest_face_distance = None
        self.closest_face_x = None
        self.face_frame_width = None
        self.running = True
        
        # Locks & Communication
        self.data_lock = threading.Lock()
        self.arduino = None
        
        # Components
        self.scanner = None
        self.picam2 = None
        self.face_cascade = None
        self.focal_length = None
        self.current_mode = "SEARCHING"
        
        # Command Mapping (Python String -> Arduino Char)
        self.CMD_MAP = {
            "forward": b'F',
            "backward": b'B',
            "left": b'L',
            "right": b'R',
            "stop": b'S',
            "search": b'L' # Default search behavior is spinning left
        }

    def initialize_components(self):
        print("[INIT] Initializing components...")
        
        # 1. Connect to Arduino via USB
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
            time.sleep(2) # CRITICAL: Wait for Arduino to reboot after USB connection
            print(f"[INIT] Arduino connected on {ARDUINO_PORT}")
        except Exception as e:
            print(f"[ERROR] Arduino connection failed: {e}")
            print("  -> Is the USB cable plugged in?")
            print("  -> Did you upload the Arduino code?")

        # 2. Bluetooth
        try:
            self.scanner = Scanner()
            print("[INIT] Bluetooth initialized")
        except Exception as e:
            print(f"[ERROR] Bluetooth failed: {e}")

        # 3. Camera
        try:
            self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            if self.face_cascade.empty(): raise Exception("XML file missing")
            
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
            self.picam2.configure(config)
            self.picam2.start()
            
            self.focal_length = calculate_focal_length(CALIBRATION_DISTANCE_CM, FACE_WIDTH_CM, CALIBRATION_PIX_WIDTH)
            print("[INIT] Camera initialized")
        except Exception as e:
            print(f"[ERROR] Camera failed: {e}")

    # --- THREADS (Bluetooth and Camera logic remains largely same) ---

    def bluetooth_thread(self):
        print("[BT] Started")
        while self.running:
            try:
                if self.scanner is None:
                    time.sleep(1); continue
                
                rssi_vals = []
                tx_vals = []
                
                for _ in range(SAMPLES_PER_READING):
                    if not self.running: break
                    devices = self.scanner.scan(0.4)
                    for dev in devices:
                        data = parse_ibeacon(dev)
                        if data and data[0] == TARGET_UUID:
                            rssi_vals.append(dev.rssi)
                            tx_vals.append(data[3])
                
                if rssi_vals:
                    dist = estimate_distance(sum(rssi_vals)/len(rssi_vals), int(sum(tx_vals)/len(tx_vals)))
                    with self.data_lock: self.bluetooth_distance = dist
                else:
                    with self.data_lock: self.bluetooth_distance = None
                
            except Exception as e: print(f"[BT ERR] {e}"); time.sleep(1)

    def camera_thread(self):
        print("[CAM] Started")
        while self.running:
            try:
                if self.picam2 is None: time.sleep(1); continue
                frame = self.picam2.capture_array()
                if frame is None: continue
                
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.05, 4, (30,30))
                
                if len(faces) > 0:
                    # Sort faces by width (largest = closest)
                    faces = sorted(faces, key=lambda x: x[2], reverse=True)
                    x, y, w, h = faces[0]
                    dist = distance_to_camera(FACE_WIDTH_CM, self.focal_length, w)
                    
                    with self.data_lock:
                        self.closest_face_distance = dist
                        self.closest_face_x = x + w//2
                        self.face_frame_width = frame.shape[1]
                else:
                    with self.data_lock:
                        self.closest_face_distance = None
                
                time.sleep(0.05)
            except Exception as e: print(f"[CAM ERR] {e}"); time.sleep(0.5)

    # --- MOTOR CONTROL ---

    def motor_control_thread(self):
        print("[MOTOR] Started")
        last_command = None

        while self.running:
            with self.data_lock:
                bt_dist = self.bluetooth_distance
                face_dist = self.closest_face_distance
                face_x = self.closest_face_x
                width = self.face_frame_width

            command = "stop" # Default safe state

            # 1. OBSTACLE AVOIDANCE (Priority High)
            if face_dist is not None:
                self.current_mode = "AVOIDING"
                if face_dist < DANGER_DISTANCE:
                    command = "stop"
                elif face_dist < SAFE_DISTANCE:
                    # Turn away from face
                    center = width // 2
                    if face_x > center: command = "left"  # Face is right, turn left
                    else: command = "right"               # Face is left, turn right
                else:
                    # Face seen but far away, fall through to beacon logic
                    pass 

            # 2. BEACON FOLLOWING
            if command == "stop" and bt_dist is not None and self.current_mode != "AVOIDING":
                self.current_mode = "FOLLOWING"
                if bt_dist > MIN_BLUETOOTH_DISTANCE:
                    command = "forward"
                else:
                    command = "stop" # Reached target
            elif command == "stop" and bt_dist is None and self.current_mode != "AVOIDING":
                 self.current_mode = "SEARCHING"
                 command = "search" # Will map to spin left

            # Send command only if it changed (reduces USB traffic)
            if command != last_command:
                self.send_to_arduino(command)
                last_command = command
            
            time.sleep(0.1)

        self.send_to_arduino("stop")

    def send_to_arduino(self, command_str):
        """Maps string command to byte character and sends via USB"""
        if self.arduino and self.arduino.is_open:
            if command_str in self.CMD_MAP:
                byte_cmd = self.CMD_MAP[command_str]
                self.arduino.write(byte_cmd)
                print(f"[ARDUINO] Sent: {command_str} -> {byte_cmd}")
            else:
                print(f"[ARDUINO] Unknown command: {command_str}")

    def start(self):
        self.initialize_components()
        t1 = threading.Thread(target=self.bluetooth_thread, daemon=True)
        t2 = threading.Thread(target=self.camera_thread, daemon=True)
        t3 = threading.Thread(target=self.motor_control_thread, daemon=True)
        t1.start(); t2.start(); t3.start()
        
        print("\n=== ROVER RUNNING ===")
        print("Ctrl+C to stop")
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.running = False
        time.sleep(0.5)
        if self.arduino: self.arduino.close()
        if self.picam2: self.picam2.stop()
        print("System Stopped.")

if __name__ == "__main__":
    RCCarController().start()