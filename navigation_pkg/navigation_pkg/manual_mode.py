import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
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

        # Listen to the obstacle avoidance node to know when it is active
        self.obs_avoid_sub = self.create_subscription(
            Bool,
            '/obstacle_avoidance',
            self.obs_avoid_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.manual_active = False
        self.is_avoiding = False

        self.linear_speed = 0.5
        self.side_speed = 0.5
        self.angular_speed = 0.8

        self.get_logger().info('Manual mode node started.')

    def obs_avoid_callback(self, msg):
        self.is_avoiding = msg.data

    def gui_callback(self, msg):
        command = msg.data.lower()
        twist = Twist()

        # 1. Safety First: Always allow the robot to be stopped completely
        if command == 'stop' or command == 'stop_manual':
            self.publish_stop()
            self.get_logger().info('Robot stopped.')
            return

        # 2. Sequence Lockout: Block all other commands if actively avoiding an obstacle
        if self.is_avoiding:
            self.get_logger().warn(f'Obstacle avoidance active. Ignoring manual command: {command}')
            return

        # 3. Mode Switching
        if command == 'manual':
            self.manual_active = True
            self.get_logger().info('Manual mode activated.')
            return

        if command == 'auto':
            self.manual_active = False
            self.publish_stop()
            self.get_logger().info('Auto mode activated. Manual disabled.')
            return

        # 4. Movement Execution
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