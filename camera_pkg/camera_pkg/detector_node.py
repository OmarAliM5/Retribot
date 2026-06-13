#!/usr/bin/env python3

import os
import subprocess
import numpy as np
import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String , Float32
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class CameraDetectorNode(Node):
    def __init__(self):
        super().__init__("camera_detector_node")

        self.image_pub = self.create_publisher(
            CompressedImage,
            "/camera/image/compressed",
            10
        )

        self.detection_pub = self.create_publisher(
            String,
            "/detected_item",
            10
        )

        self.center_error_pub = self.create_publisher(
            Float32,
            "/object_center_error",
            10
        )

        package_share = get_package_share_directory("camera_pkg")
        model_path = os.path.join(package_share, "models", "best.pt")
        self.model = YOLO(model_path)

        self.conf_threshold = 0.50
        self.frame_count = 0
        self.process_every_n_frames = 10

        self.get_logger().info("Starting rpicam-vid camera stream...")

        self.process = subprocess.Popen(
            [
                "rpicam-vid",
                "-t", "0",
                "--width", "640",
                "--height", "480",
                "--codec", "mjpeg",
                "--nopreview",
                "-o", "-"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        self.buffer = bytearray()
        self.timer = self.create_timer(0.01, self.read_frame)

        self.get_logger().info("Camera + detector node started.")
        self.get_logger().info(f"Using model: {model_path}")
        self.get_logger().info("Publishing camera on /camera/image/compressed")
        self.get_logger().info("Publishing detections on /detected_item")

    def read_frame(self):
        chunk = self.process.stdout.read(4096)

        if not chunk:
            return

        self.buffer.extend(chunk)

        start = self.buffer.find(b"\xff\xd8")
        end = self.buffer.find(b"\xff\xd9")

        if start != -1 and end != -1 and end > start:
            jpg = self.buffer[start:end + 2]
            self.buffer = self.buffer[end + 2:]

            self.frame_count += 1

            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            img_msg.format = "jpeg"
            img_msg.data = bytes(jpg)

            self.image_pub.publish(img_msg)

            if self.frame_count % self.process_every_n_frames != 0:
                return

            np_arr = np.frombuffer(jpg, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                return

            results = self.model(frame, imgsz=320, verbose=False)

            best_name = None
            best_conf = 0.0
            best_error = 0.0


            for result in results:
                for box in result.boxes:
                    confidence = float(box.conf[0].item())

                    if confidence < self.conf_threshold:
                        continue

                    class_id = int(box.cls[0].item())
                    class_name = self.model.names[class_id]
                    x_min, _, x_max, _ = box.xyxy[0].tolist()
                    object_center_x = (x_min + x_max) / 2.0
                    image_center_x = frame.shape[1] / 2.0
                    center_error = (object_center_x - image_center_x) / image_center_x
                    center_error = round(center_error, 3)

                    if confidence > best_conf:
                        best_conf = confidence
                        best_name = class_name
                        best_error = center_error

            msg = String()
        
            if best_name is not None:
                msg.data = f"{best_name},{best_conf:.2f}"
                
                error_msg = Float32()
                error_msg.data = float(best_error)
                self.center_error_pub.publish(error_msg)

                self.get_logger().info(f"Detected: {msg.data}")
            else:
                msg.data = "none,0.00"

            self.detection_pub.publish(msg)

    def destroy_node(self):
        if self.process:
            self.process.terminate()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()