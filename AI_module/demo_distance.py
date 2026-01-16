import time
import cv2
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURATION ---
WIDTH_CM = 14.0    # Avg face width
DISTANCE_CM = 50.0 # Calibration distance
PIX_WIDTH = 140    # Your calibration value

def distance_to_camera(known_width, focal_length, pixel_width):
    if pixel_width == 0: return 0
    return (known_width * focal_length) / pixel_width

def calculate_focal_length(known_dist, known_width, width_pix):
    return (width_pix * known_dist) / known_width

def main():
    print("INITIALIZING: Loading Face Detector...")
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print("ERROR: XML file not found.")
        return

    print("INITIALIZING: Camera...")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Calibration
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print("SYSTEM READY. Tracking Multiple Faces.")

    while True:
        try:
            frame = picam2.capture_array()
        except:
            continue
        if frame is None: continue

        start_time = time.time()
        
        # 1. Colors & Grayscale
        frame_display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # 2. Detect Multiple Faces
        # minNeighbors=4 catches more faces than 5 (but maybe more false positives)
        # scaleFactor=1.05 is slower but catches faces further away
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.05,
            minNeighbors=4,
            minSize=(30, 30)
        )

        # 3. Process All Faces
        detected_objects = []

        for (x, y, w, h) in faces:
            dist = distance_to_camera(WIDTH_CM, focal_length, w)
            # Store data for sorting: (distance, x, y, w, h)
            detected_objects.append((dist, x, y, w, h))

        # 4. Sort by Distance (Closest first)
        detected_objects.sort(key=lambda x: x[0]) 

        # 5. Draw Results
        for i, (dist, x, y, w, h) in enumerate(detected_objects):
            # Logic: If it's the closest face (index 0), make it RED. Others GREEN.
            if i == 0:
                color = (0, 0, 255) # Red (BGR)
                label = f"CLOSEST: {dist:.0f}cm"
            else:
                color = (0, 255, 0) # Green
                label = f"{dist:.0f}cm"

            cv2.rectangle(frame_display, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame_display, label, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Print logic for your Rover (Optional)
        if len(detected_objects) > 0:
            print(f"Faces visible: {len(detected_objects)} | Closest: {detected_objects[0][0]:.1f}cm")

        # FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame_display, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Multi-Face Tracker', frame_display)
        
        if cv2.waitKey(1) == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
