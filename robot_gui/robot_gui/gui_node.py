import threading
import time
import os

from flask import Flask, render_template, request, jsonify, Response
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, UInt8
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class RobotGuiNode(Node):
    def __init__(self):
        super().__init__("robot_gui_node")

        self.command_pub = self.create_publisher(String, "/gui_command", 10)

        # Store latest compressed JPEG bytes
        self.latest_frame = None

        self.camera_sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.camera_callback,
            10
        )
        self.battery_sub = self.create_subscription(
            UInt8,
            "/battery_percentage",
            self.battery_callback,
            10
        )


        self.speed_sub = self.create_subscription(
            Twist,
           '/cmd_vel',
            self.speed_callback,
            10
        )

        # New simple detection result
        self.latest_detection = {
            "class_name": "none",
            "confidence": 0.0
        }

        self.latest_speed = {
            "type": "linear",
            "value": 0.0,
            "unit": "m/s"
        }

        self.latest_battery = {
            "percentage": 0
        }

        self.detection_sub = self.create_subscription(
            String,
            "/detected_item/string",
            self.detection_callback,
            10
        )

        template_dir = os.path.join(
            get_package_share_directory("robot_gui"),
            "templates"
        )

        self.app = Flask(
            __name__,
            template_folder=template_dir
        )

        self.setup_routes()

    def camera_callback(self, msg):
        self.latest_frame = bytes(msg.data)

    def speed_callback(self, msg):
        linear_speed = math.sqrt(
            msg.linear.x ** 2 + msg.linear.y ** 2 
        )
            
        angular_speed = abs(msg.angular.z)

        if angular_speed > 0.01 and linear_speed < 0.01:
            self.latest_speed = {
                "type": "angular",
                "value": round(angular_speed, 2),
                "unit": "rad/s"
            }
        else:
            self.latest_speed = {
                "type": "linear",
                "value": round(linear_speed, 2),
                "unit": "m/s"
            }

    def battery_callback(self, msg):
        self.latest_battery = {
            "percentage": int(msg.data)
        }


    def detection_callback(self, msg):
        try:
            class_name, confidence = msg.data.split(",")

            self.latest_detection = {
                "class_name": class_name,
                "confidence": float(confidence)
            }

        except Exception as e:
            self.get_logger().error(f"Failed to parse detection: {e}")

            self.latest_detection = {
                "class_name": "none",
                "confidence": 0.0
            }

    def generate_frames(self):
        while True:
            if self.latest_frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + self.latest_frame +
                    b"\r\n"
                )

            time.sleep(0.03)

    def setup_routes(self):
        @self.app.route("/")
        def home():
            return render_template("index_v3.html")

        @self.app.route("/command", methods=["POST"])
        def command():
            data = request.get_json()
            cmd = data.get("command", "")

            msg = String()
            msg.data = cmd
            self.command_pub.publish(msg)

            self.get_logger().info(f"Published command: {cmd}")

            return jsonify({
                "status": "ok",
                "command": cmd
            })

        @self.app.route("/detections")
        def get_detections():
            return jsonify(self.latest_detection)

        @self.app.route("/video_feed")
        def video_feed():
            return Response(
                self.generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.route("/speed")
        def speed():
            return jsonify(self.latest_speed)

        @self.app.route("/battery")
        def battery():
            return jsonify(self.latest_battery)

    def run_flask(self):
        self.app.run(
            host="0.0.0.0",
            port=5000,
            debug=False,
            use_reloader=False,
            threaded=True
        )


def main(args=None):
    rclpy.init(args=args)

    node = RobotGuiNode()

    flask_thread = threading.Thread(target=node.run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()