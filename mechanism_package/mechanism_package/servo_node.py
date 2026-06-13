#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

# Pi 5 GPIO imports
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
import gpiozero

# Force gpiozero to use the lgpio factory for Raspberry Pi 5 compatibility
gpiozero.Device.pin_factory = LGPIOFactory()

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')

        # Initialize the servos on GPIO 12 and 13
        # Note: min_pulse_width and max_pulse_width depend on your specific servos.
        # 0.5ms (0.0005) to 2.5ms (0.0025) is standard for 0 to 180 degree rotation.
        self.gripper_servo = AngularServo(
            12, min_angle=0, max_angle=180, 
            min_pulse_width=0.0005, max_pulse_width=0.0025
        )
        self.arm_servo = AngularServo(
            13, min_angle=0, max_angle=180, 
            min_pulse_width=0.0005, max_pulse_width=0.0025
        )

        # Create subscriptions
        self.gripper_sub = self.create_subscription(
            UInt8,
            '/gripper_angle',
            self.gripper_callback,
            10
        )
        self.arm_sub = self.create_subscription(
            UInt8,
            '/arm_angle',
            self.arm_callback,
            10
        )

        self.get_logger().info('Servo Controller Node started. Listening to /gripper_angle and /arm_angle.')

    def gripper_callback(self, msg):
        # A UInt8 message goes from 0-255. We clamp it to 0-180 for the servo.
        angle = max(0, min(180, msg.data))
        self.gripper_servo.angle = angle
        self.get_logger().info(f'Gripper moved to {angle}°')

    def arm_callback(self, msg):
        angle = max(0, min(180, msg.data))
        self.arm_servo.angle = angle
        self.get_logger().info(f'Arm moved to {angle}°')

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down servo node...')
    finally:
        # Detach servos on shutdown to prevent jitter
        node.gripper_servo.detach()
        node.arm_servo.detach()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()