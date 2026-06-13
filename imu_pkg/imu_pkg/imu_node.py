#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
from geometry_msgs.msg import Vector3

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

G_TO_MS2 = 9.80665


def read_word_2c(bus, reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value = -((65536 - value))
    return value


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.pub_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_fused = self.create_publisher(Imu, '/imu/data', 10)
        self.rpy_pub = self.create_publisher(Vector3, '/imu/rpy', 10)

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('lowpass_alpha', 0.8)
        self.declare_parameter('complementary_alpha', 0.98)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)
        self.lowpass_alpha = float(self.get_parameter('lowpass_alpha').value)
        self.comp_alpha = float(self.get_parameter('complementary_alpha').value)

        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.2)

        self.get_logger().info('MPU6050 initialized')
        self.get_logger().info('Keep IMU completely still: calibrating...')

        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.accel_bias_z = 0.0

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.calibrate_imu()

        self.filt_ax = 0.0
        self.filt_ay = 0.0
        self.filt_az = G_TO_MS2

        self.filt_gx = 0.0
        self.filt_gy = 0.0
        self.filt_gz = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.last_time = time.time()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def read_raw_imu(self):
        accel_x_raw = read_word_2c(self.bus, ACCEL_XOUT_H)
        accel_y_raw = read_word_2c(self.bus, ACCEL_XOUT_H + 2)
        accel_z_raw = read_word_2c(self.bus, ACCEL_XOUT_H + 4)

        gyro_x_raw = read_word_2c(self.bus, GYRO_XOUT_H)
        gyro_y_raw = read_word_2c(self.bus, GYRO_XOUT_H + 2)
        gyro_z_raw = read_word_2c(self.bus, GYRO_XOUT_H + 4)

        return accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw

    def calibrate_imu(self):
        accel_scale = 16384.0
        gyro_scale = 131.0

        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0
        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0

        for _ in range(self.calibration_samples):
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw_imu()

            ax = (ax_raw / accel_scale) * G_TO_MS2
            ay = (ay_raw / accel_scale) * G_TO_MS2
            az = (az_raw / accel_scale) * G_TO_MS2

            gx = math.radians(gx_raw / gyro_scale)
            gy = math.radians(gy_raw / gyro_scale)
            gz = math.radians(gz_raw / gyro_scale)

            sum_ax += ax
            sum_ay += ay
            sum_az += az
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz

            time.sleep(0.01)

        avg_gx = sum_gx / self.calibration_samples
        avg_gy = sum_gy / self.calibration_samples
        avg_gz = sum_gz / self.calibration_samples

        # Important:
        # Do NOT subtract accel average on X/Y for tilt estimation,
        # because that removes the gravity component caused by mounting angle.
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.accel_bias_z = 0.0

        self.gyro_bias_x = avg_gx
        self.gyro_bias_y = avg_gy
        self.gyro_bias_z = avg_gz

        self.get_logger().info(
            'Accel bias disabled for tilt estimation; only gyro bias is calibrated.'
        )
        self.get_logger().info(
            f'Gyro bias [rad/s]: x={self.gyro_bias_x:.4f}, y={self.gyro_bias_y:.4f}, z={self.gyro_bias_z:.4f}'
        )
        self.get_logger().info('Calibration complete.')

    def publish_imu(self):
        try:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            if dt <= 0.0 or dt > 0.1:
                dt = 1.0 / self.publish_rate

            accel_scale = 16384.0
            gyro_scale = 131.0

            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw_imu()

            ax = (ax_raw / accel_scale) * G_TO_MS2
            ay = (ay_raw / accel_scale) * G_TO_MS2
            az = (az_raw / accel_scale) * G_TO_MS2

            gx = math.radians(gx_raw / gyro_scale)
            gy = math.radians(gy_raw / gyro_scale)
            gz = math.radians(gz_raw / gyro_scale)

            ax -= self.accel_bias_x
            ay -= self.accel_bias_y
            az -= self.accel_bias_z

            gx -= self.gyro_bias_x
            gy -= self.gyro_bias_y
            gz -= self.gyro_bias_z

            a = self.lowpass_alpha
            self.filt_ax = a * self.filt_ax + (1.0 - a) * ax
            self.filt_ay = a * self.filt_ay + (1.0 - a) * ay
            self.filt_az = a * self.filt_az + (1.0 - a) * az

            self.filt_gx = a * self.filt_gx + (1.0 - a) * gx
            self.filt_gy = a * self.filt_gy + (1.0 - a) * gy
            self.filt_gz = a * self.filt_gz + (1.0 - a) * gz

            roll_acc = math.atan2(self.filt_ay, self.filt_az)
            pitch_acc = math.atan2(
                -self.filt_ax,
                math.sqrt(self.filt_ay * self.filt_ay + self.filt_az * self.filt_az)
            )

            roll_gyro = self.roll + self.filt_gx * dt
            pitch_gyro = self.pitch + self.filt_gy * dt
            yaw_gyro = self.yaw + self.filt_gz * dt

            ca = self.comp_alpha
            self.roll = ca * roll_gyro + (1.0 - ca) * roll_acc
            self.pitch = ca * pitch_gyro + (1.0 - ca) * pitch_acc
            self.yaw = yaw_gyro

            qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)

            stamp = self.get_clock().now().to_msg()

            raw_msg = Imu()
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = self.frame_id
            raw_msg.orientation.x = 0.0
            raw_msg.orientation.y = 0.0
            raw_msg.orientation.z = 0.0
            raw_msg.orientation.w = 0.0
            raw_msg.orientation_covariance[0] = -1.0
            raw_msg.angular_velocity.x = self.filt_gx
            raw_msg.angular_velocity.y = self.filt_gy
            raw_msg.angular_velocity.z = self.filt_gz
            raw_msg.linear_acceleration.x = self.filt_ax
            raw_msg.linear_acceleration.y = self.filt_ay
            raw_msg.linear_acceleration.z = self.filt_az
            raw_msg.angular_velocity_covariance[0] = 0.02
            raw_msg.angular_velocity_covariance[4] = 0.02
            raw_msg.angular_velocity_covariance[8] = 0.02
            raw_msg.linear_acceleration_covariance[0] = 0.04
            raw_msg.linear_acceleration_covariance[4] = 0.04
            raw_msg.linear_acceleration_covariance[8] = 0.04

            fused_msg = Imu()
            fused_msg.header.stamp = stamp
            fused_msg.header.frame_id = self.frame_id
            fused_msg.orientation.x = qx
            fused_msg.orientation.y = qy
            fused_msg.orientation.z = qz
            fused_msg.orientation.w = qw
            fused_msg.angular_velocity.x = self.filt_gx
            fused_msg.angular_velocity.y = self.filt_gy
            fused_msg.angular_velocity.z = self.filt_gz
            fused_msg.linear_acceleration.x = self.filt_ax
            fused_msg.linear_acceleration.y = self.filt_ay
            fused_msg.linear_acceleration.z = self.filt_az

            fused_msg.orientation_covariance[0] = 0.02
            fused_msg.orientation_covariance[4] = 0.02
            fused_msg.orientation_covariance[8] = 0.05

            fused_msg.angular_velocity_covariance[0] = 0.02
            fused_msg.angular_velocity_covariance[4] = 0.02
            fused_msg.angular_velocity_covariance[8] = 0.02

            fused_msg.linear_acceleration_covariance[0] = 0.04
            fused_msg.linear_acceleration_covariance[4] = 0.04
            fused_msg.linear_acceleration_covariance[8] = 0.04

            rpy_msg = Vector3()
            rpy_msg.x = math.degrees(self.roll)
            rpy_msg.y = math.degrees(self.pitch)
            rpy_msg.z = math.degrees(self.yaw)

            self.rpy_pub.publish(rpy_msg)

            self.pub_raw.publish(raw_msg)
            self.pub_fused.publish(fused_msg)

        except Exception as e:
            self.get_logger().error(f'publish_imu error: {e}')

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()