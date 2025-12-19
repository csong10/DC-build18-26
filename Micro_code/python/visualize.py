#!/usr/bin/env python3

import serial
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import re
from collections import deque

# --- CONFIGURATION ---
MAX_LIST_LEN = 500
SERIAL_PORT = "/dev/cu.usbmodem141103" 
BAUD_RATE = 115200 
UPDATE_INTERVAL_MS = 50  

# --- GLOBAL SHARED DATA ---
data_time = deque(maxlen=MAX_LIST_LEN)
data_target = deque(maxlen=MAX_LIST_LEN)
data_left = deque(maxlen=MAX_LIST_LEN)
data_right = deque(maxlen=MAX_LIST_LEN)

start_time = time.time()
serial_conn = None
thread_running = True 
data_lock = threading.Lock()

# Regex to match C code output: "T:450,ML:392,MR:407" (Integers scaled by 100)
# Groups: 1=Target, 2=Left, 3=Right
# Pattern looks for T:[int],ML:[int],MR:[int]
DATA_PATTERN = re.compile(r'T:(\-?\d+),ML:(\-?\d+),MR:(\-?\d+)')

def init_serial():
    global serial_conn
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE}...")
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        serial_conn.flushInput()
        print("Serial connection established successfully.")
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return False

def serial_worker():
    """Background thread to read serial data."""
    global start_time
    print("Background thread started reading serial data...")
    
    while thread_running:
        if serial_conn and serial_conn.is_open:
            try:
                line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    match = DATA_PATTERN.search(line)
                    if match:
                        # Parse integers and divide by 100.0 to restore float precision
                        t_val = float(match.group(1)) / 100.0
                        l_val = float(match.group(2)) / 100.0
                        r_val = float(match.group(3)) / 100.0
                        
                        elapsed = time.time() - start_time
                        
                        with data_lock:
                            data_time.append(elapsed)
                            data_target.append(t_val)
                            data_left.append(l_val)
                            data_right.append(r_val)
            except Exception:
                pass
        else:
            time.sleep(0.1)

def animate(i, ax, lines):
    """Updates the plot lines."""
    line_target, line_left, line_right = lines
    
    with data_lock:
        if not data_time:
            return line_target, line_left, line_right
            
        times = list(data_time)
        targets = list(data_target)
        lefts = list(data_left)
        rights = list(data_right)

    # Update Data
    line_target.set_data(times, targets)
    line_left.set_data(times, lefts)
    line_right.set_data(times, rights)
    
    # Rescale Axes
    ax.set_xlim(times[0], times[-1])
    
    all_y = targets + lefts + rights
    if all_y:
        y_min = min(all_y) - 2
        y_max = max(all_y) + 2
        ax.set_ylim(y_min, y_max)
        
        # Title with current status
        ax.set_title(f"Target: {targets[-1]:.2f} | L: {lefts[-1]:.2f} | R: {rights[-1]:.2f}")
        
    return line_target, line_left, line_right

def main():
    global thread_running, serial_conn
    
    if not init_serial():
        return

    # Start Background Thread
    io_thread = threading.Thread(target=serial_worker, daemon=True)
    io_thread.start()

    # Setup Plot
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Create 3 lines
    line_target, = ax.plot([], [], label='Target', color='orange', linestyle='--', linewidth=2)
    line_left, = ax.plot([], [], label='Left Motor', color='blue', alpha=0.8)
    line_right, = ax.plot([], [], label='Right Motor', color='green', alpha=0.8)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed')
    ax.legend(loc='upper left')
    ax.grid(True)
    
    # Start Animation
    ani = animation.FuncAnimation(
        fig, 
        animate, 
        fargs=(ax, (line_target, line_left, line_right)), 
        interval=UPDATE_INTERVAL_MS, 
        blit=False
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        thread_running = False
        io_thread.join(timeout=1.0)
        if serial_conn:
            serial_conn.close()

if __name__ == "__main__":
    main()