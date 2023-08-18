from flask import Flask, request, Response, render_template
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

# model = YOLO("/home/soton/ros2_ws_foxy/src/opencv_tools/opencv_tools/best.pt")

app = Flask(__name__)
current_frame = None  # Global variable to store the most recent video frame
cv_bridge = CvBridge()  # Create a CvBridge instance


class ImageSubscriber(Node):
    def _init_(self):
        super()._init_('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/i069/camera/color/image_raw', self.listener_callback, 10)

    def listener_callback(self, data):
        global current_frame
        self.get_logger().info('Receiving video frame')
        current_frame = data  # Store the received Image message directly


def generate_video():
    global current_frame
    while True:
        if current_frame is not None:
            # Convert the Image message to an OpenCV image
            current_cv_frame = cv_bridge.imgmsg_to_cv2(current_frame)
            # Perform image processing on current_cv_frame if desired
            # In this example, we just resize the frame to half its original size
            current_cv_frame = cv2.resize(
                current_cv_frame, (0, 0), fx=0.5, fy=0.5)
            # results = model(current_cv_frame ,save=True)

            # annotated_frame = results[0].plot()
            # Convert the processed OpenCV image back to a JPEG format
            _, buffer = cv2.imencode('.jpg', current_cv_frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            print("No current frame available!")


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
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ros_thread = threading.Thread(target=main)
    ros_thread.start()
    print("ROS 2 node running.")
    app.run(host='localhost', port=5000, threaded=True, debug=True)
