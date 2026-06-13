#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.r = 0.04
        self.lx = 0.175
        self.ly = 0.20
        self.k = self.lx + self.ly

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_offset = None

        self.last_time = self.get_clock().now()
        self.last_rpm = [0, 0, 0, 0]  # Changed to int

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.enc_sub = self.create_subscription(
            Int16MultiArray,
            '/encoder_feedback',
            self.encoder_callback,
            qos
        )

        self.imu_sub = self.create_subscription(
            Vector3,
            '/imu/rpy',
            self.imu_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info('Mecanum Odometry Node Started')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def imu_callback(self, msg):
        raw_yaw = math.radians(msg.z)

        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw
            self.get_logger().info(
                f'Yaw offset set to {math.degrees(self.yaw_offset):.2f} deg'
            )

        self.yaw = self.normalize_angle(raw_yaw - self.yaw_offset)

    def encoder_callback(self, msg):
        if len(msg.data) >= 4:
            self.last_rpm = list(msg.data[:4])

    def rpm_to_rad_s(self, rpm):
        return float(rpm) * 2.0 * math.pi / 60.0

    def yaw_to_quat(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        fl_rpm, fr_rpm, rl_rpm, rr_rpm = self.last_rpm

        w_fl = self.rpm_to_rad_s(fl_rpm)
        w_fr = self.rpm_to_rad_s(fr_rpm)
        w_rl = self.rpm_to_rad_s(rl_rpm)
        w_rr = self.rpm_to_rad_s(rr_rpm)

        vx_body = (self.r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy_body = (self.r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz_enc = (self.r / (4.0 * self.k)) * (-w_fl + w_fr - w_rl + w_rr)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        vx_world = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world = vx_body * sin_yaw + vy_body * cos_yaw

        self.x += vx_world * dt
        self.y += vy_world * dt

        qz, qw = self.yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz_enc

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()