import time
import cv2
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURATION ---
# Faces are smaller than bodies, so we can process a slightly better resolution
PROCESS_WIDTH = 400
WIDTH_CM = 14.0   # Average width of a human face (ear to ear)
DISTANCE_CM = 50.0 # Calibrate at 50cm
PIX_WIDTH = 140   # <--- MEASURE THIS AGAIN for the face!

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def main():
    print("INITIALIZING: Setting up Face Detector (Haar Cascade)...")
    
   # NEW WORKING CODE (Looks for the file you just downloaded) 
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Safety check to make sure it loaded
    if face_cascade.empty():
        print("ERROR: Could not find 'haarcascade_frontalface_default.xml'. Did you run the wget command?")
        exit()

    print("INITIALIZING: Setting up IMX708 Camera...")
    try:
        picam2 = Picamera2()
        # We ask for RGB888 this time to fix the "Weird Color" issue manually later
        config = picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        print("SUCCESS: Camera running.")
    except Exception as e:
        print(f"ERROR: Camera failed. {e}")
        return

    # 2. Calibration Placeholder
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print("SYSTEM READY. Look at the camera. Press 'q' to quit.")

    while True:
        try:
            # Capture data
            frame = picam2.capture_array()
        except:
            continue

        if frame is None: continue

        start_time = time.time()

        # 3. FIX COLORS (RGB -> BGR for Display)
        # OpenCV uses BGR. If the feed looked weird, this swap fixes it for your eyes.
        frame_display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 4. GRAYSCALE (For the AI)
        # The AI works faster and better in Black & White
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # 5. Detect Faces
        # scaleFactor: 1.1 = checks for faces 10% larger at each pass
        # minNeighbors: Higher (5) = less false positives, but might miss faces
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        # 6. Draw Results
        for (x, y, w, h) in faces:
            # Calculate Distance
            dist = distance_to_camera(WIDTH_CM, focal_length, w)

            # Draw green box
            cv2.rectangle(frame_display, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Label
            cv2.putText(frame_display, f"Face: {dist:.0f}cm", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # FPS Counter
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame_display, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Face Detection (Grayscale Logic)', frame_display)
        
        if cv2.waitKey(1) == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()