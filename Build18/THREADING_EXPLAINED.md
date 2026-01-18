# RC Car Threading Architecture

## Thread Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         MAIN PROGRAM                             │
│  - Initialize hardware (camera, bluetooth, uart)                │
│  - Create and start 3 threads                                   │
│  - Monitor status and handle shutdown                           │
└─────────────────────────────────────────────────────────────────┘
                                 │
                    ┌────────────┼────────────┐
                    │            │            │
                    ▼            ▼            ▼
        
┌───────────────────┐  ┌──────────────────┐  ┌─────────────────────┐
│ BLUETOOTH THREAD  │  │  CAMERA THREAD   │  │ MOTOR CONTROL THREAD│
│  (100ms cycle)    │  │  (50ms cycle)    │  │    (20ms cycle)     │
├───────────────────┤  ├──────────────────┤  ├─────────────────────┤
│                   │  │                  │  │                     │
│ 1. Scan for       │  │ 1. Capture frame │  │ 1. READ shared data │
│    beacon         │  │                  │  │    with lock:       │
│                   │  │ 2. Convert to    │  │    - bt_distance    │
│ 2. Collect RSSI   │  │    grayscale     │  │    - face_distance  │
│    samples (5x)   │  │                  │  │    - face_x         │
│                   │  │ 3. Detect faces  │  │                     │
│ 3. Calculate avg  │  │                  │  │ 2. DECIDE action:   │
│    RSSI & distance│  │ 4. Find closest  │  │    Priority:        │
│                   │  │    face          │  │    a) Face follow   │
│ 4. WRITE to       │  │                  │  │    b) Beacon follow │
│    shared vars:   │  │ 5. Calculate     │  │    c) Search        │
│    [LOCK]         │  │    distance &    │  │                     │
│    - bt_distance  │  │    position      │  │ 3. SEND command     │
│    - bt_rssi      │  │                  │  │    via UART:        │
│    [UNLOCK]       │  │ 6. WRITE to      │  │    "forward"        │
│                   │  │    shared vars:  │  │    "left"           │
│ 5. Sleep 100ms    │  │    [LOCK]        │  │    "right"          │
│                   │  │    - face_dist   │  │    "backward"       │
│ 6. Repeat         │  │    - face_x      │  │    "stop"           │
│                   │  │    [UNLOCK]      │  │                     │
│                   │  │                  │  │ 4. Sleep 20ms       │
│                   │  │ 7. Sleep 50ms    │  │                     │
│                   │  │                  │  │ 5. Repeat           │
│                   │  │ 8. Repeat        │  │                     │
└───────────────────┘  └──────────────────┘  └─────────────────────┘
         │                      │                       │
         │                      │                       │
         └──────────────────────┼───────────────────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │   SHARED VARIABLES    │
                    │   (Protected by Lock) │
                    ├───────────────────────┤
                    │ • bluetooth_distance  │
                    │ • bluetooth_rssi      │
                    │ • closest_face_dist   │
                    │ • closest_face_x      │
                    │ • face_frame_width    │
                    │ • running (bool)      │
                    └───────────────────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │  UART to STM32        │
                    │  /dev/ttyAMA0         │
                    │  115200 baud          │
                    └───────────────────────┘
```

## Data Flow Example

### Scenario: RC car is moving toward beacon and encounters an obstacle

```
Time 0ms:
  Camera Thread: Captures frame, detects face (obstacle) at x=450, distance=120cm
                 Writes to shared vars (with lock)
  
  Motor Thread:  Reads shared vars (with lock)
                 obstacle_distance = 120cm, obstacle_x = 450
                 120cm < 150cm SAFE_DISTANCE → need to avoid
                 offset = +130 (obstacle on right)
                 Decision: "left" (turn left to avoid)
                 Sends "left\n" via UART
  
  BT Thread:     Scanning... beacon found at 3.2m

Time 20ms:
  Motor Thread:  Reads same obstacle data
                 Still avoiding, sends "left\n" again
  
Time 50ms:
  Camera Thread: New frame captured, obstacle now at x=200 (car turned left!)
                 distance=125cm, still in avoidance zone
                 Writes new data to shared vars
  
  Motor Thread:  Reads NEW obstacle data
                 obstacle_distance = 125cm, obstacle_x = 200
                 offset = -120 (now obstacle on LEFT side)
                 Decision: "right" (turn right to avoid)
                 Sends "right\n"

Time 100ms:
  BT Thread:     Beacon still at 3.2m
                 Writes bt_distance = 3.2
  
  Camera Thread: Obstacle at x=600, distance=160cm
                 160cm > 150cm → safe distance!
                 Writes new data
  
  Motor Thread:  Reads both obstacle AND beacon data
                 Obstacle is far (160cm > SAFE_DISTANCE)
                 Returns None from avoid_obstacle()
                 Falls through to beacon following
                 Decision: "forward" (toward beacon)
                 Sends "forward\n"

Time 150ms:
  Camera Thread: No obstacle detected anymore
                 Writes None values

  Motor Thread:  No obstacle detected
                 Beacon at 3.1m (getting closer!)
                 Decision: "forward"
                 Sends "forward\n"

... continues toward beacon with obstacle avoidance ready!
```

## Why Each Thread Has Different Update Rates

### Bluetooth Thread (100ms = 10 updates/second)
- Bluetooth scanning is SLOW (each scan takes ~400ms)
- Multiple samples needed for accuracy (5 samples)
- Beacon distance changes slowly (you're not teleporting!)
- 10 Hz is plenty for tracking movement

### Camera Thread (50ms = 20 updates/second)  
- Face detection is MODERATELY fast
- 20 FPS is smooth enough to track head movement
- Higher would use too much CPU
- Lower would feel laggy

### Motor Control Thread (20ms = 50 updates/second)
- Needs to be FASTEST for smooth movement
- Reads latest sensor data every 20ms
- Makes decisions and sends commands quickly
- 50 Hz = responsive, smooth robot behavior
- Like a game running at 50 FPS vs 10 FPS!

## The Lock Mechanism Explained

```python
# Without lock (BAD - race condition!):
# Thread 1:                    Thread 2:
self.distance = 100            temp = self.distance  # Reads old value
                               temp = temp + 10
self.distance = 200            self.distance = temp  # Overwrites Thread 1!
# Result: distance = 110 (should be 200!)

# With lock (GOOD):
# Thread 1:                    Thread 2:
with self.data_lock:           # Waits here... can't enter
    self.distance = 100        # Still waiting...
    # do more stuff            # Still waiting...
# Lock released                with self.data_lock:  # Now can enter!
                                   temp = self.distance  # Reads 100
                                   temp = temp + 10
                                   self.distance = temp  # Writes 110
# Result: distance = 110 (correct!)
```

The lock ensures **atomic operations** - each thread gets exclusive access
to shared data while inside the `with self.data_lock:` block.

## Daemon Threads vs Regular Threads

```python
# Daemon thread (daemon=True):
thread = threading.Thread(target=my_function, daemon=True)
thread.start()
# When main program exits, daemon threads are killed immediately

# Regular thread (daemon=False):
thread = threading.Thread(target=my_function, daemon=False)
thread.start()
# When main program exits, it WAITS for this thread to finish naturally

# We use daemon=True so Ctrl+C stops everything immediately!
```

## Thread Safety Best Practices

### ✓ DO:
- Always use locks when reading/writing shared variables
- Keep locked sections SHORT (don't do heavy work inside)
- Use try-except in threads to catch errors
- Set `daemon=True` for automatic cleanup
- Add `time.sleep()` to prevent CPU hogging

### ✗ DON'T:
- Access shared variables without locks
- Hold locks while doing slow operations (like file I/O)
- Create threads inside threads (keep it simple)
- Forget error handling in thread functions
- Use global variables instead of class attributes

## Performance Considerations

```
CPU Usage per Thread (approximate):
┌──────────────────┬──────────┬────────────────┐
│ Thread           │ CPU %    │ Why            │
├──────────────────┼──────────┼────────────────┤
│ Bluetooth        │ 5-10%    │ BLE scanning   │
│ Camera           │ 20-30%   │ Image process  │
│ Motor Control    │ 1-2%     │ Just logic     │
│ Total            │ 25-40%   │ Leaves headroom│
└──────────────────┴──────────┴────────────────┘

On Raspberry Pi 4, this leaves plenty of CPU for the OS and other tasks!
```

## Crash Recovery

Each thread has independent error handling:

```python
def bluetooth_thread(self):
    while self.running:
        try:
            # ... do bluetooth stuff
        except Exception as e:
            print(f"[BT ERROR] {e}")
            time.sleep(1)  # Wait before retry
    # If BT crashes, camera and motor still work!
```

This means:
- Camera crashes → BT and motors keep working
- BT crashes → Camera and motors keep working  
- One failure doesn't bring down the whole system!

## Shutdown Sequence

When you press Ctrl+C:

```
1. KeyboardInterrupt caught in main thread
2. Sets self.running = False
3. All threads see running=False in their while loops
4. Threads exit their loops gracefully
5. Daemon threads are killed
6. Camera stopped
7. UART closed
8. Program exits cleanly
```
