import time
import cv2
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURATION ---
PROCESS_WIDTH = 400
WIDTH_CM = 50.0   
DISTANCE_CM = 100.0
PIX_WIDTH = 100   # Calibrate this!

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def main():
    print("INITIALIZING: Setting up Picamera2 (Native Mode)...")
    
    # 1. Setup the Camera
    try:
        picam2 = Picamera2()
        # Configure for 640x480 video, formatted as BGR (what OpenCV wants)
        config = picam2.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"}
        )
        picam2.configure(config)
        picam2.start()
        print("SUCCESS: IMX708 is running natively!")
    except Exception as e:
        print(f"ERROR: Could not start Picamera2. {e}")
        print("Ensure 'rpicam-hello' works in the terminal first.")
        return

    # 2. Setup HOG Detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # 3. Calibration (Placeholder)
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print("SYSTEM READY. Press 'q' to quit.")

    # 4. Main Loop
    while True:
        try:
            # CAPTURE: Get the standard 'main' stream as a numpy array
            frame = picam2.capture_array()
        except Exception as e:
            print("Frame drop")
            continue

        if frame is None: continue

        start_time = time.time()

        # Resize for Speed
        height, width = frame.shape[:2]
        scale_factor = PROCESS_WIDTH / float(width)
        new_height = int(height * scale_factor)
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, new_height))

        # Detect
        rects, weights = hog.detectMultiScale(
            frame_small, 
            winStride=(8, 8), 
            padding=(8, 8), 
            scale=1.05
        )

        # Draw
        for (x, y, w, h) in rects:
            big_x = int(x / scale_factor)
            big_y = int(y / scale_factor)
            big_w = int(w / scale_factor)
            big_h = int(h / scale_factor)

            dist = distance_to_camera(WIDTH_CM, focal_length, big_w)

            cv2.rectangle(frame, (big_x, big_y), (big_x + big_w, big_y + big_h), (0, 255, 0), 2)
            cv2.putText(frame, f"{dist:.0f}cm", (big_x, big_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('IMX708 Picamera2', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()