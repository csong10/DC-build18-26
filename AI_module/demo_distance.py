import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

# --- CONSTANTS ---
# Person detection usually requires a different calibration than faces
# because a "person" (full body) is much wider/taller.
# YOU MUST RE-CALIBRATE THIS:
WIDTH_CM = 50.0   # Approx width of a human shoulder/body
DISTANCE_CM = 100.0
PIX_WIDTH = 150   # Measure this again with the new model!

# Path to the COCO model (detects people, cars, dogs, etc.)
MODEL_PATH = "models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"

def get_arducam_pipeline(width=640, height=480, framerate=30):
    """
    Arducam/Libcamera often fails with standard index 0.
    This GStreamer pipeline forces it to work on Pi 4.
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
    # 1. Setup Coral TPU
    interpreter = tflite.Interpreter(
        model_path=MODEL_PATH,
        experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')]
    )
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    h_model = input_details[0]['shape'][1]
    w_model = input_details[0]['shape'][2]

    # 2. Initialize Arducam (Try GStreamer first, fall back to index 0)
    print("Attempting to open Arducam via GStreamer...")
    cap = cv2.VideoCapture(get_arducam_pipeline(), cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("GStreamer failed. Trying standard V4L2 index 0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: Could not open camera. Check connection or libcamera installation.")
            return

    # Calibrate Focal Length (using your constants)
    focal_length = calculate_focal_length(DISTANCE_CM, WIDTH_CM, PIX_WIDTH)
    print("System Ready. Detecting BODIES (Class 0). Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 3. Preprocess
        frame_resized = cv2.resize(frame, (w_model, h_model))
        input_data = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(input_data, axis=0)

        # 4. Inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # 5. Extract Results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]

        im_h, im_w, _ = frame.shape

        for i in range(len(scores)):
            if scores[i] > 0.5:
                # CRITICAL CHANGE: Check for Class 0 (Person)
                # Note: Some models use 0 for Person, others use 1.
                # If this doesn't detect you, try: if classes[i] == 1:
                if int(classes[i]) == 0: 
                    
                    ymin, xmin, ymax, xmax = boxes[i]
                    
                    # Convert to pixels
                    left = int(xmin * im_w)
                    right = int(xmax * im_w)
                    top = int(ymin * im_h)
                    bottom = int(ymax * im_h)
                    
                    pixel_width = right - left
                    
                    # Estimate Distance
                    dist = distance_to_camera(WIDTH_CM, focal_length, pixel_width)

                    # Draw Box & Distance
                    cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
                    cv2.putText(frame, f"Person: {dist:.1f}cm", (left, top-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow('Person Distance Detector', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()