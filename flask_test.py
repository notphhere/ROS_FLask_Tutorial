from flask import Flask, render_template, request, Response
import cv2
import numpy as np

app = Flask(__name__)

# Global variable to store the latest image
latest_frame = None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# to publish image to flask


def generate_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            _, buffer = cv2.imencode('.jpg', latest_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


# to receive image from flask
@app.route('/upload', methods=['POST'])
def upload():
    global latest_frame
    image = request.files['image']
    image_data = image.read()
    nparr = np.frombuffer(image_data, np.uint8)
    latest_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    return 'Image received'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
