#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EKFNode(Node):
    """
    2D EKF for mecanum robot

    State:
        x = [pos_x, pos_y, yaw]^T

    Inputs from encoders:
        body-frame velocities vx_body, vy_body
        yaw rate from encoders or IMU gyro

    Measurements:
        yaw from IMU orientation
    """

    def __init__(self):
        super().__init__('ekf_node')

        # ---------------- Robot params ----------------
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_base', 0.1325)   # نصف الطول
        self.declare_parameter('track_width', 0.1875)  # نصف العرض

        self.declare_parameter('encoder_is_rpm', True)
        self.declare_parameter('rpm_scale', 1.0)

        # signs لو أي عجلة اتجاهها معكوس
        self.declare_parameter('sign_fl', 1.0)
        self.declare_parameter('sign_fr', 1.0)
        self.declare_parameter('sign_bl', 1.0)
        self.declare_parameter('sign_br', 1.0)

        # ---------------- timing ----------------
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_dt', 0.1)

        # ---------------- frames ----------------
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # ---------------- IMU fusion ----------------
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('use_imu_gyro_for_prediction', True)
        self.declare_parameter('gyro_deadband', 0.01)

        # ---------------- process noise ----------------
        self.declare_parameter('q_x', 0.01)
        self.declare_parameter('q_y', 0.01)
        self.declare_parameter('q_yaw', 0.02)

        # ---------------- measurement noise ----------------
        self.declare_parameter('r_yaw', 0.03)

        # ---------------- misc ----------------
        self.declare_parameter('zero_velocity_threshold', 0.001)

        # Load params
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.W = float(self.get_parameter('track_width').value)

        self.encoder_is_rpm = bool(self.get_parameter('encoder_is_rpm').value)
        self.rpm_scale = float(self.get_parameter('rpm_scale').value)

        self.sign_fl = float(self.get_parameter('sign_fl').value)
        self.sign_fr = float(self.get_parameter('sign_fr').value)
        self.sign_bl = float(self.get_parameter('sign_bl').value)
        self.sign_br = float(self.get_parameter('sign_br').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_dt = float(self.get_parameter('max_dt').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        self.use_imu_gyro_for_prediction = bool(
            self.get_parameter('use_imu_gyro_for_prediction').value
        )
        self.gyro_deadband = float(self.get_parameter('gyro_deadband').value)

        self.q_x = float(self.get_parameter('q_x').value)
        self.q_y = float(self.get_parameter('q_y').value)
        self.q_yaw = float(self.get_parameter('q_yaw').value)

        self.r_yaw = float(self.get_parameter('r_yaw').value)

        self.zero_velocity_threshold = float(
            self.get_parameter('zero_velocity_threshold').value
        )

        # ---------------- EKF state ----------------
        # x = [x, y, yaw]
        self.x_hat = np.zeros((3, 1), dtype=float)

        # covariance
        self.P = np.diag([0.05, 0.05, 0.1]).astype(float)

        self.Q = np.diag([self.q_x, self.q_y, self.q_yaw]).astype(float)
        self.R_yaw = np.array([[self.r_yaw]], dtype=float)

        # ---------------- latest sensor values ----------------
        self.has_encoder = False
        self.has_imu = False

        # encoder order from ESP:
        # [FL, FR, BL, BR]
        self.fl = 0.0
        self.fr = 0.0
        self.bl = 0.0
        self.br = 0.0

        self.imu_yaw = 0.0
        self.imu_wz = 0.0

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.wz = 0.0

        self.last_time = None

        # ---------------- ROS ----------------
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            '/encoder_feedback',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            'EKF node started | '
            f'r={self.r:.4f}, L={self.L:.4f}, W={self.W:.4f}, '
            f'use_imu_yaw={self.use_imu_yaw}, '
            f'use_imu_gyro_for_prediction={self.use_imu_gyro_for_prediction}'
        )

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def imu_callback(self, msg: Imu):
        self.imu_yaw = quaternion_to_yaw(msg.orientation)
        self.imu_wz = msg.angular_velocity.z

        if abs(self.imu_wz) < self.gyro_deadband:
            self.imu_wz = 0.0

        self.has_imu = True

    def encoder_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn(
                'encoder_feedback must contain 4 values [FL, FR, BL, BR]'
            )
            return

        self.fl = float(msg.data[0]) * self.sign_fl
        self.fr = float(msg.data[1]) * self.sign_fr
        self.bl = float(msg.data[2]) * self.sign_bl
        self.br = float(msg.data[3]) * self.sign_br

        self.has_encoder = True

    # ---------------------------------------------------------
    # Utility
    # ---------------------------------------------------------
    def rpm_to_rad_s(self, rpm: float) -> float:
        return rpm * 2.0 * math.pi / 60.0

    def mecanum_forward_kinematics(self, fl: float, fr: float, bl: float, br: float):
        """
        Inputs expected as wheel angular speeds in rad/s.
        Returns:
            vx_body [m/s]
            vy_body [m/s]
            wz      [rad/s]
        """

        vx = (self.r / 4.0) * (fl + fr + bl + br)
        vy = (self.r / 4.0) * (-fl + fr + bl - br)
        wz = (self.r / (4.0 * (self.L + self.W))) * (-fl + fr - bl + br)

        if abs(vx) < self.zero_velocity_threshold:
            vx = 0.0
        if abs(vy) < self.zero_velocity_threshold:
            vy = 0.0
        if abs(wz) < self.zero_velocity_threshold:
            wz = 0.0

        return vx, vy, wz

    # ---------------------------------------------------------
    # EKF
    # ---------------------------------------------------------
    def predict(self, dt: float, vx_body: float, vy_body: float, wz_input: float):
        """
        State:
            x, y, yaw

        Motion model:
            x_k+1 = x + (cos(yaw)*vx - sin(yaw)*vy) * dt
            y_k+1 = y + (sin(yaw)*vx + cos(yaw)*vy) * dt
            yaw_k+1 = yaw + wz * dt
        """
        x = float(self.x_hat[0, 0])
        y = float(self.x_hat[1, 0])
        yaw = float(self.x_hat[2, 0])

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        vx_world = cos_yaw * vx_body - sin_yaw * vy_body
        vy_world = sin_yaw * vx_body + cos_yaw * vy_body

        x_pred = x + vx_world * dt
        y_pred = y + vy_world * dt
        yaw_pred = normalize_angle(yaw + wz_input * dt)

        self.x_hat[0, 0] = x_pred
        self.x_hat[1, 0] = y_pred
        self.x_hat[2, 0] = yaw_pred

        # Jacobian F = df/dx
        F = np.array([
            [1.0, 0.0, (-sin_yaw * vx_body - cos_yaw * vy_body) * dt],
            [0.0, 1.0, ( cos_yaw * vx_body - sin_yaw * vy_body) * dt],
            [0.0, 0.0, 1.0]
        ], dtype=float)

        # Simple process noise
        self.P = F @ self.P @ F.T + self.Q

    def update_yaw(self, yaw_meas: float):
        """
        Measurement:
            z = yaw
        """
        H = np.array([[0.0, 0.0, 1.0]], dtype=float)

        z = np.array([[yaw_meas]], dtype=float)
        z_pred = np.array([[self.x_hat[2, 0]]], dtype=float)

        innovation = z - z_pred
        innovation[0, 0] = normalize_angle(float(innovation[0, 0]))

        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x_hat = self.x_hat + K @ innovation
        self.x_hat[2, 0] = normalize_angle(float(self.x_hat[2, 0]))

        I = np.eye(3, dtype=float)
        self.P = (I - K @ H) @ self.P

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        if not self.has_encoder:
            return

        fl = self.fl
        fr = self.fr
        bl = self.bl
        br = self.br

        if self.encoder_is_rpm:
            fl = self.rpm_to_rad_s(fl * self.rpm_scale)
            fr = self.rpm_to_rad_s(fr * self.rpm_scale)
            bl = self.rpm_to_rad_s(bl * self.rpm_scale)
            br = self.rpm_to_rad_s(br * self.rpm_scale)

        vx_body, vy_body, wz_enc = self.mecanum_forward_kinematics(fl, fr, bl, br)

        if self.has_imu and self.use_imu_gyro_for_prediction:
            wz_input = self.imu_wz
        else:
            wz_input = wz_enc

        # 1) Predict
        self.predict(dt, vx_body, vy_body, wz_input)

        # 2) Update with IMU yaw
        if self.has_imu and self.use_imu_yaw:
            self.update_yaw(self.imu_yaw)

        self.vx_body = vx_body
        self.vy_body = vy_body
        self.wz = wz_input

        self.publish_odom(now)

    # ---------------------------------------------------------
    # Publish
    # ---------------------------------------------------------
    def publish_odom(self, now):
        x = float(self.x_hat[0, 0])
        y = float(self.x_hat[1, 0])
        yaw = float(self.x_hat[2, 0])

        q = yaw_to_quaternion(yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = self.vx_body
        odom.twist.twist.linear.y = self.vy_body
        odom.twist.twist.angular.z = self.wz

        # Pose covariance from EKF P
        odom.pose.covariance[0] = float(self.P[0, 0])   # x
        odom.pose.covariance[1] = float(self.P[0, 1])
        odom.pose.covariance[5] = float(self.P[0, 2])

        odom.pose.covariance[6] = float(self.P[1, 0])
        odom.pose.covariance[7] = float(self.P[1, 1])   # y
        odom.pose.covariance[11] = float(self.P[1, 2])

        odom.pose.covariance[30] = float(self.P[2, 0])
        odom.pose.covariance[31] = float(self.P[2, 1])
        odom.pose.covariance[35] = float(self.P[2, 2])  # yaw

        # Twist covariance (simple)
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

        if self.publish_tf and self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = q

            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()