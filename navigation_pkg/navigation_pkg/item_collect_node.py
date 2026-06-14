import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist

class ItemCollect(Node):
    def __init__(self):
        super().__init__('item_collect_node')

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

        self.item_collect_active = False

        self.linear_speed = 0.5
        self.side_speed = 0.5
        self.angular_speed = 0.8

        self.get_logger().info('Item collect node started.')

    def gui_callback(self, msg):
        command = msg.data.lower()

        twist = Twist()

        if command == 'item_collect':
            self.item_collect_active = True
            self.get_logger().info('Item collect mode activated.')
            return

        if command == 'auto':
            self.item_collect_active = False
            self.publish_stop()
            self.get_logger().info('Auto mode activated. Item collect disabled.')
            return

        if command == 'stop':
            self.publish_stop()
            self.get_logger().info('Robot stopped.')
            return

        if not self.item_collect_active:
            return

        if command == 'stop_item_collect':
            self.publish_stop()
            return

        elif command == 'forward':
