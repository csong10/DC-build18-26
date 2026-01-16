import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time
from flask import Flask, Response

# --- SETUP ---
app = Flask(__name__)

MODEL_PATH = "models/ssd_mobilenet_v2_coco_quant_postprocess.tflite"
CONFIDENCE_THRESHOLD = 0.5
PERSON_CLASS_ID = 0

# Load Model Once
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
h_model = input_details[0]['shape'][1]
w_model = input_details[0]['shape'][2]

def get_arducam_pipeline(width=640, height=480, framerate=30):
    return (
        f"libcamerasrc ! video/x-raw, width={width}, height={height}, framerate={framerate}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )

# --- THE CAMERA LOOP ---
def generate_frames():
    # Open Camera
    cap = cv2.VideoCapture(get_arducam_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret: break

        # --- DETECTION LOGIC START ---
        # 1. Resize for AI
        frame_resized = cv2.resize(frame, (w_model, h_model))
        input_data = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(input_data, axis=0)

        # 2. Run Inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # 3. Get Results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]

        im_h, im_w, _ = frame.shape

        for i in range(len(scores)):
            if scores[i] > CONFIDENCE_THRESHOLD:
                if int(classes[i]) == PERSON_CLASS_ID:
                    ymin, xmin, ymax, xmax = boxes[i]
                    left = int(xmin * im_w)
                    right = int(xmax * im_w)
                    top = int(ymin * im_h)
                    bottom = int(ymax * im_h)

                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    cv2.putText(frame, "PERSON", (left, top-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # --- DETECTION LOGIC END ---

        # Encode the frame as JPEG for the web
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Yield the frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# --- WEB ROUTES ---
@app.route('/')
def index():
    return "<h1>Robot Feed</h1><img src='/video_feed' width='640'>"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Host 0.0.0.0 allows connection from your laptop
    app.run(host='0.0.0.0', port=5000, debug=False)