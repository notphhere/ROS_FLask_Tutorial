import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response, render_template, request
import threading
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

app = Flask(__name__)
current_frame = None  # Global variable to store the most recent video frame
cv_bridge = CvBridge()  # Create a CvBridge instance

int_data = None
heart_rate = None


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile_location = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        qos_profile_stop = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.msg = Int32()

        self.subscription = self.create_subscription(
            Image, '/i069/camera/color/image_raw', self.listener_callback, 10)
        self._publisher_location = self.create_publisher(
            Int32, '/preset_location', qos_profile_location)
        self._publisher_stop = self.create_publisher(
            Int32, '/i069/robot_stop', qos_profile_stop)
        self._heartrate_subscription = self.create_subscription(
            String, '/heart_rate', self.heartrate_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def heartrate_callback(self, msg):
        global heart_rate
        heart_rate = str(msg.data)

    def timer_callback(self):
        global int_data
        if int_data is not None:

            self.msg.data = int_data
            if self.msg.data < 0:
                self._publisher_stop.publish(self.msg)

                # self.get_logger().info('sending stop/go command')

            else:
                self._publisher_location.publish(self.msg)
                # self.get_logger().info('sending nav command')

        else:
            self.msg.data = -10  # making -10 = none
            # self.get_logger().info('sending no command')

        int_data = None

    def listener_callback(self, data):
        global current_frame
        self.get_logger().info('Receiving video frame')
        current_frame = data  # Store the received Image message directly


def generate_video():
    global current_frame
    while True:
        if current_frame is not None:

            current_cv_frame = cv_bridge.imgmsg_to_cv2(current_frame)
            current_cv_frame = cv2.resize(
                current_cv_frame, (0, 0), fx=0.5, fy=0.5)

            _, buffer = cv2.imencode('.jpg', current_cv_frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            print("No current frame available!")


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/stream')
def stream():
    return render_template('stream.html')


@app.route('/video')
def video():
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/starting', methods=['GET'])
def starting():
    value = -2
    # do ur stuff with the -2
    print("-2")
    return render_template('index.html')


@app.route('/stop', methods=['GET'])
def start():
    value = -1
    # do ur stuff with the -1
    print("-1")
    return render_template('index.html')


@app.route('/home', methods=['GET'])
def home():
    value = 0
    # do ur stuff with the 0
    print("0")
    return render_template('index.html')


@app.route('/ward1', methods=['GET'])
def ward1():
    value = 1
    # do ur stuff with the 1
    print("1")
    return render_template('index.html')


@app.route('/ward2', methods=['GET'])
def ward2():
    value = 2
    # do ur stuff with the 2
    print("2")
    return render_template('index.html')


'''
#placeholder code for receiving data from client
@app.route("/send_data/<int:data>")
def get_data(data):
    global int_data 
    int_data = data
    return "Data sent successfully!"

#placeholder code for sending heart rate
@app.route("/send_heart_rate_string", methods=["POST"] )
def send_heart_rate_string(data):
    # Process the received heart rate string data here
    print("Received heart rate string:", data)
    # You can perform further actions with the heart rate string data
    return "Heart rate string data received successfully!"
'''


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
    app.run(host='localhost', port=5000, threaded=True)
