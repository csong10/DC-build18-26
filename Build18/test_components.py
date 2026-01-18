#!/usr/bin/env python3
"""
RC Car Component Testing Script

Use this to test each component individually before running the full threaded system.
Run with: sudo python3 test_components.py
"""

import sys
import time

def test_arduino():
    """Test USB Serial communication with Arduino"""
    print("\n" + "="*60)
    print("TESTING: Arduino USB Connection")
    print("="*60)
    print("Ensure Arduino is plugged into USB and code is uploaded!")
    input("Press Enter to start test...")
    
    try:
        import serial
        # Standard USB Serial Port for Arduino
        PORT = '/dev/ttyACM0' 
        BAUD = 9600
        
        print(f"Connecting to {PORT}...")
        arduino = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2) # Wait for auto-reset
        print("✓ Connected!")
        
        commands = [
            (b'F', "Forward"),
            (b'B', "Backward"),
            (b'L', "Left"),
            (b'R', "Right"),
            (b'S', "Stop")
        ]
        
        print("\nSending test commands...")
        for cmd_byte, desc in commands:
            arduino.write(cmd_byte)
            print(f"  Sent: {desc} ({cmd_byte})")
            time.sleep(1.0) # Run each action for 1 second
            
        arduino.write(b'S') # Ensure stop at end
        arduino.close()
        print("\n✓ Arduino test complete")
        return True
        
    except serial.SerialException as e:
        print(f"✗ ERROR: Could not open {PORT}")
        print("  1. Check USB cable")
        print("  2. Try: ls /dev/tty* (look for ttyACM0 or ttyUSB0)")
        return False
    except ImportError:
        print("✗ ERROR: pyserial not installed (pip3 install pyserial)")
        return False

def test_bluetooth():
    """Test Bluetooth beacon scanning"""
    print("\n" + "="*60)
    print("TESTING: Bluetooth Beacon Scanner")
    print("="*60)
    print("Make sure your beacon app is running with screen ON!")
    input("Press Enter to start test...")
    
    try:
        from bluepy.btle import Scanner
        
        TARGET_UUID = "DC672026BD1866667777674206742067".lower()
        scanner = Scanner()
        
        print("\nScanning for 5 seconds...")
        for i in range(5):
            devices = scanner.scan(1.0)
            found = False
            
            for dev in devices:
                for (adtype, desc, value) in dev.getScanData():
                    if desc == "Manufacturer":
                        print(f"DEBUG Manufacturer: {value}")
                        raw = value.lower()
                        if raw.startswith("4c000215") and len(raw) >= 50:
                            uuid = raw[8:40]
                            if uuid == TARGET_UUID:
                                print(f"✓ Beacon found! RSSI: {dev.rssi} dBm")
                                found = True
            
            if not found:
                print(f"✗ Beacon not found (attempt {i+1}/5)")
        
        print("\n✓ Bluetooth test complete")
        return True
        
    except ImportError:
        print("✗ ERROR: bluepy library not installed")
        print("  Install with: pip3 install bluepy")
        return False
    except Exception as e:
        print(f"✗ ERROR: {e}")
        return False

def test_camera():
    """Test camera and face detection"""
    print("\n" + "="*60)
    print("TESTING: Camera & Face Detection")
    print("="*60)
    print("Position your face in front of the camera")
    input("Press Enter to start test...")
    
    try:
        import cv2
        from picamera2 import Picamera2
        
        # Load face detector
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        if face_cascade.empty():
            print("✗ ERROR: haarcascade_frontalface_default.xml not found")
            print("  Download from: https://github.com/opencv/opencv/tree/master/data/haarcascades")
            return False
        
        # Initialize camera
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        picam2.configure(config)
        picam2.start()
        
        print("\nTesting for 5 seconds...")
        print("Look at the camera - you should see detection messages below")
        
        for i in range(50):  # 5 seconds at 10 FPS
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.05,
                minNeighbors=4,
                minSize=(30, 30)
            )
            
            if len(faces) > 0:
                print(f"✓ Detected {len(faces)} face(s) at frame {i+1}/50")
            else:
                print(f"  No faces detected at frame {i+1}/50")
            
            time.sleep(0.1)
        
        picam2.stop()
        print("\n✓ Camera test complete")
        return True
        
    except ImportError as e:
        print(f"✗ ERROR: Required library not installed: {e}")
        print("  Install with: sudo apt-get install python3-opencv python3-picamera2")
        return False
    except Exception as e:
        print(f"✗ ERROR: {e}")
        return False

def test_threading():
    """Test basic threading functionality"""
    print("\n" + "="*60)
    print("TESTING: Threading Basics")
    print("="*60)
    
    try:
        import threading
        
        results = {'thread1': 0, 'thread2': 0}
        lock = threading.Lock()
        
        def worker1():
            for i in range(5):
                with lock:
                    results['thread1'] += 1
                time.sleep(0.1)
        
        def worker2():
            for i in range(5):
                with lock:
                    results['thread2'] += 1
                time.sleep(0.15)
        
        print("Starting two test threads...")
        t1 = threading.Thread(target=worker1)
        t2 = threading.Thread(target=worker2)
        
        t1.start()
        t2.start()
        
        t1.join()
        t2.join()
        
        print(f"Thread 1 count: {results['thread1']}")
        print(f"Thread 2 count: {results['thread2']}")
        
        if results['thread1'] == 5 and results['thread2'] == 5:
            print("\n✓ Threading test complete - both threads ran successfully!")
            return True
        else:
            print("\n✗ Threading test failed - unexpected results")
            return False
            
    except Exception as e:
        print(f"✗ ERROR: {e}")
        return False

def main():
    """Run all tests or individual test"""
    print("\n" + "="*60)
    print("RC CAR COMPONENT TEST SUITE")
    print("="*60)
    print("\nThis script will help you verify each component works")
    print("before running the full threaded system.\n")
    
    print("Available tests:")
    print("  1. Bluetooth Beacon Scanner")
    print("  2. Camera & Face Detection")
    print("  3. UART Communication")
    print("  4. Threading Basics")
    print("  5. Run ALL tests")
    print("  0. Exit")
    
    choice = input("\nEnter test number: ").strip()
    
    results = {}
    
    if choice == '1':
        results['bluetooth'] = test_bluetooth()
    elif choice == '2':
        results['camera'] = test_camera()
    elif choice == '3':
        results['uart'] = test_arduino()
    elif choice == '4':
        results['threading'] = test_threading()
    elif choice == '5':
        print("\n" + "="*60)
        print("RUNNING ALL TESTS")
        print("="*60)
        results['threading'] = test_threading()
        results['bluetooth'] = test_bluetooth()
        results['camera'] = test_camera()
        results['uart'] = test_arduino()
    elif choice == '0':
        print("Exiting...")
        return
    else:
        print("Invalid choice!")
        return
    
    # Print summary
    if results:
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        for test_name, passed in results.items():
            status = "✓ PASSED" if passed else "✗ FAILED"
            print(f"{test_name.upper()}: {status}")
        
        all_passed = all(results.values())
        if all_passed:
            print("\n🎉 All tests passed! You're ready to run the full system.")
            print("   Run: sudo python3 rc_car_threaded.py")
        else:
            print("\n⚠ Some tests failed. Fix the issues above before running the full system.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
