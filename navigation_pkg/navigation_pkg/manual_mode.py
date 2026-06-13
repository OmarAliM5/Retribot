import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ManualMode(Node):
    def __init__(self):
        super().__init__('manual_mode_node')

        self.gui_sub = self.create_subscription(
            String,
            '/gui_command',
            self.gui_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.manual_active = False

        self.linear_speed = 0.5
        self.side_speed = 0.5
        self.angular_speed = 0.8

        self.get_logger().info('Manual mode node started.')

    def gui_callback(self, msg):
        command = msg.data.lower()

        twist = Twist()

        if command == 'manual':
            self.manual_active = True
            self.get_logger().info('Manual mode activated.')
            return

        if command == 'auto':
            self.manual_active = False
            self.publish_stop()
            self.get_logger().info('Auto mode activated. Manual disabled.')
            return

        if command == 'stop':
            self.publish_stop()
            self.get_logger().info('Robot stopped.')
            return

        if not self.manual_active:
            return

        if command == 'forward':
            twist.linear.x = self.linear_speed

        elif command == 'backward':
            twist.linear.x = -self.linear_speed

        elif command == 'left':
            twist.linear.y = self.side_speed

        elif command == 'right':
            twist.linear.y = -self.side_speed

        elif command == 'forward_left':
            twist.linear.x = self.linear_speed
            twist.linear.y = self.side_speed

        elif command == 'forward_right':
            twist.linear.x = self.linear_speed
            twist.linear.y = -self.side_speed

        elif command == 'backward_left':
            twist.linear.x = -self.linear_speed
            twist.linear.y = self.side_speed

        elif command == 'backward_right':
            twist.linear.x = -self.linear_speed
            twist.linear.y = -self.side_speed

        elif command == 'rotate_left':
            twist.angular.z = self.angular_speed

        elif command == 'rotate_right':
            twist.angular.z = -self.angular_speed

        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        self.cmd_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ManualMode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()