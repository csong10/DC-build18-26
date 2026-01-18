# RC Car Threaded Control System

## Overview

This system integrates three separate components into a single threaded application:
1. **Bluetooth Beacon Scanner** - Tracks distance to a Bluetooth beacon
2. **Camera Face Detection** - Detects and tracks faces
3. **STM32 Motor Control** - Sends movement commands via UART

## How Threading Works

### What is Threading?

Threading allows multiple tasks to run **concurrently** (at the same time). Instead of doing one thing, waiting for it to finish, then doing the next thing, threads let your program do many things simultaneously.

**Without Threading:**
```
Scan Bluetooth → Wait → Process Camera → Wait → Move Motors → Repeat
```
(Total cycle time: ~1 second, jerky movement)

**With Threading:**
```
Thread 1: Scan Bluetooth continuously (100ms updates)
Thread 2: Process Camera continuously (50ms updates)  
Thread 3: Control Motors continuously (20ms updates)
All running at the same time!
```
(Smooth, responsive behavior)

### The Three Threads

#### 1. **Bluetooth Thread** (`bluetooth_thread`)
- **What it does:** Continuously scans for your Bluetooth beacon
- **How often:** Every ~500ms (adjustable)
- **Output:** Updates `self.bluetooth_distance` and `self.bluetooth_rssi`
- **Key Code:**
  ```python
  def bluetooth_thread(self):
      while self.running:
          # Scan for beacon multiple times for accuracy
          rssi_values = []
          for _ in range(SAMPLES_PER_READING):
              devices = self.scanner.scan(0.4)
              # ... collect RSSI values
          
          # Calculate average distance
          distance = estimate_distance(avg_rssi, avg_tx_power)
          
          # Thread-safe update of shared data
          with self.data_lock:
              self.bluetooth_distance = distance
  ```

#### 2. **Camera Thread** (`camera_thread`)
- **What it does:** Continuously captures frames and detects faces
- **How often:** Every 50ms (~20 FPS)
- **Output:** Updates `self.closest_face_distance` and `self.closest_face_x`
- **Key Code:**
  ```python
  def camera_thread(self):
      while self.running:
          # Capture frame
          frame = self.picam2.capture_array()
          
          # Detect faces
          faces = self.face_cascade.detectMultiScale(gray, ...)
          
          # Find closest face
          # ... process faces
          
          # Thread-safe update
          with self.data_lock:
              self.closest_face_distance = dist
              self.closest_face_x = x_center
  ```

#### 3. **Motor Control Thread** (`motor_control_thread`)
- **What it does:** Makes decisions and sends commands to STM32
- **How often:** Every 20ms (50 Hz for smooth control)
- **Input:** Reads data from other threads
- **Output:** Sends UART commands like "forward", "left", "right", "stop"
- **Key Code:**
  ```python
  def motor_control_thread(self):
      while self.running:
          # Thread-safe read of all sensor data
          with self.data_lock:
              bt_distance = self.bluetooth_distance
              face_distance = self.closest_face_distance
              face_x = self.closest_face_x
          
          # Decision logic
          if face_distance is not None:
              command = self.decide_face_following(...)
          elif bt_distance is not None:
              command = self.decide_beacon_following(...)
          else:
              command = "search"
          
          # Send to STM32
          self.send_command(command)
  ```

### Thread Synchronization - The Lock

**The Problem:** Multiple threads reading/writing the same variable at the same time can cause data corruption.

**The Solution:** Use a `threading.Lock()` to ensure only one thread accesses shared data at a time.

```python
# In __init__:
self.data_lock = threading.Lock()

# When WRITING shared data:
with self.data_lock:
    self.bluetooth_distance = new_value  # Only this thread can access now

# When READING shared data:
with self.data_lock:
    my_copy = self.bluetooth_distance  # Safe read
```

The `with self.data_lock:` block ensures **mutual exclusion** - only one thread can be inside that block at any moment.

## Control Logic Priority

The system follows this priority order:

1. **Avoid Obstacles** (Highest Priority)
   - If a face (obstacle) is detected, avoid it
   - Stop immediately if obstacle < 80cm away
   - Turn away from obstacle if 80-150cm away
   - Proceed if obstacle > 150cm away

2. **Beacon Following** (Medium Priority)
   - If no obstacles detected, move toward beacon
   - Stop when within 0.5m of beacon

3. **Search Mode** (Lowest Priority)
   - No obstacles, no beacon
   - Execute search pattern (you can customize this)

## Configuration

Edit these values at the top of `rc_car_threaded.py`:

```python
# Bluetooth Beacon
TARGET_UUID = "DC672026BD1866667777674206742067".lower()

# Camera Calibration
FACE_WIDTH_CM = 14.0
CALIBRATION_DISTANCE_CM = 50.0
CALIBRATION_PIX_WIDTH = 140

# UART Communication
UART_PORT = '/dev/ttyAMA0'      # UART port on Raspberry Pi
UART_BAUDRATE = 115200          # Match your STM32 baud rate

# Behavior Thresholds
MIN_BLUETOOTH_DISTANCE = 0.5    # meters - stop when this close to beacon
DANGER_DISTANCE = 80.0          # cm - stop immediately if obstacle this close
SAFE_DISTANCE = 150.0           # cm - obstacle far enough to proceed
```

## UART Commands Sent to STM32

The system sends these string commands (with newline `\n`):

- `"forward\n"` - Move forward
- `"backward\n"` - Move backward
- `"left\n"` - Turn left
- `"right\n"` - Turn right
- `"stop\n"` - Stop all motors
- `"search\n"` - Search pattern (optional - implement in STM32)

### STM32 Code Example

Your STM32 should receive these commands via UART and act accordingly:

```c
// Pseudo-code for STM32
char uart_buffer[32];

void UART_ReceiveCallback(char* data) {
    if (strcmp(data, "forward\n") == 0) {
        move_forward();
    }
    else if (strcmp(data, "backward\n") == 0) {
        move_backward();
    }
    else if (strcmp(data, "left\n") == 0) {
        turn_left();
    }
    else if (strcmp(data, "right\n") == 0) {
        turn_right();
    }
    else if (strcmp(data, "stop\n") == 0) {
        stop_motors();
    }
    else if (strcmp(data, "search\n") == 0) {
        execute_search_pattern();
    }
}
```

## Installation & Setup

### 1. Install Required Libraries

```bash
# System packages
sudo apt-get update
sudo apt-get install python3-opencv python3-picamera2

# Python packages
pip3 install bluepy pyserial numpy
```

### 2. Enable UART on Raspberry Pi

Edit `/boot/config.txt`:
```
enable_uart=1
```

Disable serial console in `/boot/cmdline.txt`:
- Remove `console=serial0,115200`

Reboot:
```bash
sudo reboot
```

### 3. Download Haar Cascade File

```bash
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
```

Place in the same directory as `rc_car_threaded.py`

### 4. Connect STM32 to Raspberry Pi

```
Raspberry Pi GPIO 14 (TXD) → STM32 RX
Raspberry Pi GPIO 15 (RXD) → STM32 TX
Raspberry Pi GND → STM32 GND
```

## Running the System

```bash
sudo python3 rc_car_threaded.py
```

**Note:** `sudo` is required for Bluetooth scanning.

### Expected Output

```
[INIT] Initializing components...
[INIT] UART initialized on /dev/ttyAMA0 at 115200 baud
[INIT] Bluetooth scanner initialized
[INIT] Camera initialized with face detection
[INIT] All components initialized

[BT] Bluetooth thread started
[CAM] Camera thread started
[MOTOR] Motor control thread started

============================================================
RC CAR SYSTEM RUNNING
============================================================
Priority: 1. Avoid Obstacles  2. Follow Beacon  3. Search
Press Ctrl+C to stop
============================================================

[OBSTACLE] Distance: 95.3cm | X-offset: +120px | Mode: AVOIDING_OBSTACLE
[OBSTACLE] Avoiding - turning left
[UART] Sent: left
[BEACON] Distance: 2.50m | Mode: FOLLOWING_BEACON
[UART] Sent: forward
[BEACON] Distance: 1.80m | Mode: FOLLOWING_BEACON
[UART] Sent: forward
...
```

## Troubleshooting

### Bluetooth Not Working
- Make sure beacon app is running with screen ON
- Check UUID matches (no dashes, lowercase)
- Run with `sudo` for BLE permissions

### Camera Not Working
- Check camera is enabled: `sudo raspi-config` → Interface Options → Camera
- Verify Haar cascade XML file exists in same directory
- Check camera connection

### UART Not Working
- Verify UART is enabled in `/boot/config.txt`
- Check baud rate matches STM32 configuration
- Test with minicom: `minicom -D /dev/ttyAMA0 -b 115200`
- Verify wiring (TX↔RX, GND↔GND)

### Threading Issues
- If one component fails, others continue running (by design)
- Check individual thread error messages
- Each thread has error handling to prevent crashes

## Customization Ideas

### Change Control Priorities
Edit the decision logic in `motor_control_thread()`:
```python
# Example: Only follow beacon, ignore obstacles
if bt_distance is not None:
    command = self.decide_beacon_following(bt_distance)
elif face_distance is not None:
    command = "stop"  # Just stop if obstacle detected
```

### Adjust Obstacle Avoidance Behavior
Modify `avoid_obstacle()`:
```python
# Example: More aggressive avoidance
DANGER_DISTANCE = 120.0  # Stop earlier
# Add backward movement
if distance < DANGER_DISTANCE:
    return "backward"  # Back away instead of stopping
```

### Implement Search Pattern
```python
def decide_beacon_following(self, distance):
    if distance > MIN_BLUETOOTH_DISTANCE:
        return "forward"
    else:
        return "stop"
        
# Or add rotation search:
def search_pattern(self):
    """Rotate in place to search for targets"""
    return "rotate-left"  # Slowly spin to scan area
```

### Add Display Thread
```python
def display_thread(self):
    """Show camera feed with annotations"""
    while self.running:
        frame = self.picam2.capture_array()
        # ... draw boxes, distances
        cv2.imshow('RC Car View', frame)
        cv2.waitKey(1)
```

## Performance Tips

1. **Adjust Sleep Times** for different update rates:
   - Faster = more responsive but more CPU usage
   - Slower = less CPU but less responsive

2. **Camera Resolution** - Lower resolution = faster processing:
   ```python
   config = picam2.create_video_configuration(
       main={"format": "RGB888", "size": (320, 240)}  # Half resolution
   )
   ```

3. **Face Detection Parameters**:
   - `scaleFactor=1.1` = faster but less accurate
   - `minNeighbors=3` = more detections but more false positives

## How This Solves Your Problem

**Before (Sequential):**
- Camera blocks Bluetooth → motors wait for camera → everything is slow and jerky
- Can't track beacon while checking for obstacles
- One failure stops everything

**After (Threaded):**
- ✅ All sensors update independently at their own optimal rates
- ✅ Motor control gets fresh data from all sources every 20ms
- ✅ One thread failing doesn't crash the others
- ✅ Smooth, responsive robot behavior
- ✅ Can track beacon while simultaneously watching for obstacles

The key is that **sensor threads collect data** while **motor thread makes decisions** using the most recent data from all sources:
- "Is there an obstacle? Yes → avoid it"
- "No obstacle? Is there a beacon? Yes → go toward it"
- "Neither? Search for beacon"

All happening in parallel for smooth navigation!
