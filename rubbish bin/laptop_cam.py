from flask import Flask, request, Response, render_template
import cv2

app = Flask(__name__)

# taking video live from camera
camera = cv2.VideoCapture(0)


def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route("/")
def hello():
    return render_template('index.html')


@app.route("/video")
def video_feed():
    return render_template('video.html')


@app.route("/video_play")
def video_play():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='localhost', port=5000, debug=True)
