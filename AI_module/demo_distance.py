# Caleb Song
import cv2

# CONSTANTS - CALIBRATION
WIDTH_CM = 14.0 
DISTANCE_CM = 50.0

# The pixel width of your face at that known distance.
PIX_WIDTH = 180  # DEFAULT, CHANGE THIS after testing

# Calculates focal length of camera using F = (P * D) / W
def calculate_focal_length(dist, width, width_pix):
    return (width_pix * dist) / width

# Calculates distance from camera to object D = (W * F) / P
def distance_to_camera(width, length, width_pix):
    return (width * length) / width_pix

def main():
    # Load the pre-trained Haar Cascade face detector (built into OpenCV)
    # This finds the face so we can measure its pixel width
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Initialize Webcam. On Mac, this is usually 0 or 1.
    cap = cv2.VideoCapture(0)

    # Calculate the initial Focal Length based on your calibration constants
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Face detection works better/faster in B&W
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Calculate distance using the triangle similarity formula
            dist_cm = distance_to_camera(WIDTH_CM, focal_length, w)

            label = f"Dist: {dist_cm:.1f} cm | Pixels: {w}"
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            break 

        cv2.imshow('Distance Estimator (Face)', frame)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()