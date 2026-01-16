import cv2
import numpy as np
import time

# --- CONSTANTS ---
# HOG works best on smaller images.
PROCESS_WIDTH = 400 

# Distance Calculation Constants (Recalibrate these for the Wide Lens!)
# The IMX708 Wide has a MUCH wider field of view, so 150 pixels will be different.
WIDTH_CM = 50.0   
DISTANCE_CM = 100.0
PIX_WIDTH = 100   # <--- LIKELY LOWER due to wide angle lens. Measure this!

def get_imx708_pipeline(width=640, height=480, framerate=30):
    """
    Specific GStreamer pipeline for IMX708 (Camera Module 3).
    It forces the camera to output NV12 (fast), then converts to BGR for OpenCV.
    """
    return (
        "libcamerasrc ! "
        f"video/x-raw, width={width}, height={height}, framerate={framerate}/1 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink"
    )

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def main():
    print("INITIALIZING: Setting up HOG Person Detector...")
    
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    print("OPENING IMX708 CAMERA (Module 3)...")
    
    # 1. Use the specific IMX708 Pipeline
    cap = cv2.VideoCapture(get_imx708_pipeline(), cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("ERROR: Could not open camera via GStreamer.")
        print("troubleshoot: Run 'sudo apt-get install gstreamer1.0-libcamera'")
        return

    # 2. Calibration (Placeholder)
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print("SYSTEM READY. Press 'q' to quit.")

    while True:
        start_time = time.time()
        
        ret, frame = cap.read()
        if not ret: 
            print("Error: Lost frame from camera.")
            break

        # 3. Resize for Speed
        height, width = frame.shape[:2]
        scale_factor = PROCESS_WIDTH / float(width)
        new_height = int(height * scale_factor)
        frame_small = cv2.resize(frame, (PROCESS_WIDTH, new_height))

        # 4. Run Detection
        rects, weights = hog.detectMultiScale(
            frame_small, 
            winStride=(8, 8), 
            padding=(8, 8), 
            scale=1.05
        )

        # 5. Draw Results
        for (x, y, w, h) in rects:
            big_x = int(x / scale_factor)
            big_y = int(y / scale_factor)
            big_w = int(w / scale_factor)
            big_h = int(h / scale_factor)

            dist = distance_to_camera(WIDTH_CM, focal_length, big_w)

            cv2.rectangle(frame, (big_x, big_y), (big_x + big_w, big_y + big_h), (0, 255, 0), 2)
            cv2.putText(frame, f"Person: {dist:.1f}cm", (big_x, big_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('IMX708 View', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()