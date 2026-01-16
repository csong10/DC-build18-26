import cv2
import numpy as np
import time

# --- CONSTANTS & CALIBRATION ---
# Distance Calculation Constants
WIDTH_CM = 50.0   # Approx width of a human shoulder/body
DISTANCE_CM = 100.0
PIX_WIDTH = 150   # Measure this again with the HOG detector!

# HOG Speed Configuration
# HOG is heavy on the CPU. We process a smaller image to keep FPS high.
PROCESS_WIDTH = 400 

def get_arducam_pipeline(width=640, height=480, framerate=30):
    """
    Arducam/Libcamera pipeline for RPi 4.
    """
    return (
        f"libcamerasrc ! video/x-raw, width={width}, height={height}, framerate={framerate}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def main():
    print("INITIALIZING: Setting up HOG Person Detector (No TensorFlow)...")
    
    # 1. Initialize HOG Descriptor (The "Edge" Detector)
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # 2. Initialize Arducam
    print("OPENING CAMERA...")
    cap = cv2.VideoCapture(get_arducam_pipeline(), cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("WARNING: GStreamer failed. Trying standard V4L2 index 0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: Could not open camera.")
            return

    # 3. Calibrate Focal Length
    # (This assumes PIX_WIDTH was measured at DISTANCE_CM)
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print(f"Focal Length Calculated: {focal_length:.2f}")
    print("SYSTEM READY. Detecting Bodies & Distance. Press 'q' to quit.")

    while True:
        start_time = time.time()
        
        ret, frame = cap.read()
        if not ret: break

        # 4. Resize for Speed (The trick to making this work on CPU)
        height, width = frame.shape[:2]
        scale_factor = PROCESS_WIDTH / float(width)
        new_height = int(height * scale_factor)
        
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, new_height))

        # 5. Run HOG Detection
        # winStride: (8,8) is faster, (4,4) is more accurate
        # padding: Helps detect people near the edges
        # scale: 1.05 is standard
        rects, weights = hog.detectMultiScale(
            frame_small, 
            winStride=(8, 8), 
            padding=(8, 8), 
            scale=1.05
        )

        # 6. Process Detections
        for (x, y, w, h) in rects:
            # We must scale the box back up to the original frame size
            # because we detected on 'frame_small' but we draw on 'frame'
            big_x = int(x / scale_factor)
            big_y = int(y / scale_factor)
            big_w = int(w / scale_factor)
            big_h = int(h / scale_factor)

            # Calculate Distance
            dist = distance_to_camera(WIDTH_CM, focal_length, big_w)

            # Draw Box
            cv2.rectangle(frame, (big_x, big_y), (big_x + big_w, big_y + big_h), (0, 255, 0), 2)
            
            # Draw Text
            label = f"Person: {dist:.1f}cm"
            cv2.putText(frame, label, (big_x, big_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 7. FPS Counter
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('HOG Distance Detector', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()