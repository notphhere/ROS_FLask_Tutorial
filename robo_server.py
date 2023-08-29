import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response, render_template, request
import threading
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import numpy as np

import subprocess

def get_local_ipv4_addresses():
    try:
        # Run the "hostname -I" command and capture its output
        result = subprocess.run(["hostname", "-I"], stdout=subprocess.PIPE, text=True)
        output = result.stdout.strip()
        # Split the output by spaces to get individual IPv4 addresses
        ipv4_addresses = output.split()
        return ipv4_addresses
    except Exception as e:
        print("Error:", e)
        return None

app = Flask(__name__)
current_frame = None  # Global variable to store the most recent video frame
cv_bridge = CvBridge()  # Create a CvBridge instance

int_data = None
heart_rate = None
local_ipv4_addresses = get_local_ipv4_addresses()[0]

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
            CompressedImage, '/i069/camera/color/image_raw/compressed', self.listener_callback, 30)
        self._publisher_location = self.create_publisher(
            Int32, '/preset_location', qos_profile_location)
        self._publisher_stop = self.create_publisher(
            Int32, '/i069/robot_stop', qos_profile_stop)
        self._heartrate_subscription = self.create_subscription(
            String, '/heart_rate', self.heartrate_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def heartrate_callback(self, msg):
        global heart_rate
        if msg.data == '-1':
            heart_rate = "Finger not detected"
        else:
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
        # self.get_logger().info('Receiving video frame')
        current_frame = data.data  # Store the received Image message directly


def generate_video():
    global current_frame
    while True:
        if current_frame is not None:
            # speed up --> decode
            nparr = np.frombuffer(current_frame, np.uint8)
            current_cv_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            current_cv_frame = cv2.resize(current_cv_frame,(0,0),fx=0.5,fy=0.5)
            _, buffer = cv2.imencode('.jpg', current_cv_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            print("No current frame available!")


@app.route('/')
def index():
    # change variable based on traffic
    return render_template('index.html', ip=local_ipv4_addresses)


@app.route('/stream')
def stream():
    return render_template('stream.html', ip=local_ipv4_addresses)


@app.route('/video')
def video():
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/updates')
def updates():
    def generate_updates():
        while True:
            # heart_rate = "charles"
            # can also get heart rate when u run this code
            data = heart_rate
            yield f"data: {data}\n\n"
            time.sleep(1)
    return Response(generate_updates(), content_type='text/event-stream')


@app.route('/starting', methods=['GET'])
def starting():
    value = -2
    # do ur stuff with the -2
    global int_data
    int_data = value
    print("-2")
    return render_template('index.html', ip=local_ipv4_addresses)


@app.route('/stop', methods=['GET'])
def start():
    value = -1
    # do ur stuff with the -1
    global int_data
    int_data = value
    print("-1")
    return render_template('index.html', ip=local_ipv4_addresses)


@app.route('/home', methods=['GET'])
def home():
    value = 0
    # do ur stuff with the 0
    global int_data
    int_data = value
    print("0")
    return render_template('index.html', ip=local_ipv4_addresses)


@app.route('/ward1', methods=['GET'])
def ward1():
    value = 1
    # do ur stuff with the 1
    global int_data
    int_data = value
    print("1")
    return render_template('index.html', ip=local_ipv4_addresses)


@app.route('/ward2', methods=['GET'])
def ward2():
    value = 2
    # do ur stuff with the 2
    global int_data
    int_data = value
    print("2")
    return render_template('index.html', ip=local_ipv4_addresses)


'''
#placeholder code for receiving data from client
@app.route("/send_data/<int:data>")
def get_data(data):
    global int_data 
    int_data = data
    return "Data sent successfully!"

#placeholder code for sending heart rate
    global int_data
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
    app.run(host='0.0.0.0', port=5000, threaded=True)
