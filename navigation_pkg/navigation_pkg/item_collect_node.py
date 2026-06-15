import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray, UInt8
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import time
import math


def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ItemCollect(Node):
    def __init__(self):
        super().__init__('item_collect_node')

        self.gui_sub = self.create_subscription(
            String,
            '/gui_command',
            self.gui_callback,
            10
        )

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.item_detected_sub = self.create_subscription(
            Vector3,
            '/detected_item/vector',
            self.item_detected_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_angle_pub = self.create_publisher(UInt8, '/arm_angle', 10)
        self.gripper_angle_pub = self.create_publisher(UInt8, '/gripper_angle', 10)
        self.collecting_pub = self.create_publisher(Bool, '/collecting_item', 10)
        
        self.ToF_range_sub = self.create_subscription(
            Float32MultiArray,
            'tof_distances',
            self.ToF_range_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None
        self.yaw = 0.0
        self.save_pose = None
        self.save_yaw = 0.0
        self.ToF_range = 0.0

        self.currently_collecting = False
        self.object_center_error = 0.0
        self.object_num = 0
        self.state = 0
        self.linear_speed = 0.2
        self.side_speed = -0.05
        self.Kp = -0.1
        self.start = False
        self.wait_start = None
        self.get_logger().info('Item collect node started.')

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = get_yaw(self.pose.orientation)

    def ToF_range_callback(self, msg):
        if len(msg.data) > 0:
            self.ToF_range = min(msg.data)

    def gui_callback(self, msg):
        command = msg.data.lower()

        if command == 'auto':
            self.start = True
            # Removed the True publish from here. It should only broadcast True when an item is SEEN.
            self.get_logger().info('Auto mode activated. Item collect armed.')
            return

        if command == 'stop' or command == 'manual':
            self.start = False
            self.collecting_pub.publish(Bool(data=False))
            self.get_logger().info('Item collect stopped.')
            return

    def item_detected_callback(self, msg):
        if (not self.currently_collecting) and (msg.x >=1.0) and (msg.y >= 0.6) and self.start:
            self.start = True
            self.currently_collecting = True
            self.object_num = int(msg.x)
            self.object_center_error = (msg.z)
            # Broadcast that we are actively collecting so path follower and obstacle avoidance pause
            self.collecting_pub.publish(Bool(data=True)) 
            self.get_logger().info('Item detected! Taking over robot control.')
            
        elif self.currently_collecting and (msg.x ==self.object_num) and (msg.y >= 0.6) and self.start:
            self.object_center_error = (msg.z)

    def control_loop(self):
        if self.wait_start is None:
            self.wait_start = time.time()

        if self.pose is None:
            return

        if not (self.start and self.currently_collecting):
            return

        if self.state == 0:
            self.save_pose = self.pose
            self.save_yaw = self.yaw
            self.state = 1

        elif self.state == 1:
            if abs(self.object_center_error) > 0.1:
                cmd = Twist()
                cmd.angular.z = -0.1 * (1 if self.object_center_error > 0 else -1)
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.state = 2

        elif self.state == 2:
            if self.ToF_range >= 310:
                cmd = Twist()
                cmd.linear.x = self.linear_speed
                self.cmd_pub.publish(cmd)
            elif self.ToF_range < 270:
                cmd = Twist()
                cmd.linear.x = -self.linear_speed
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
                self.state = 3
                self.wait_start = time.time()

        elif self.state == 3:
            self.arm_angle_pub.publish(UInt8(data=150))
            self.gripper_angle_pub.publish(UInt8(data=100))
            if time.time() - self.wait_start > 2.0:
                self.state = 4
                self.wait_start = time.time()
                
        elif self.state == 4:
            self.arm_angle_pub.publish(UInt8(data=150))
            self.gripper_angle_pub.publish(UInt8(data=160))
            if time.time() - self.wait_start > 2.0:
                self.state = 5
                self.wait_start = time.time()
                
        elif self.state == 5:
            self.arm_angle_pub.publish(UInt8(data=0))
            self.gripper_angle_pub.publish(UInt8(data=160))
            if time.time() - self.wait_start > 2.0:
                self.state = 6
                self.wait_start = time.time()
                
        elif self.state == 6:
            self.arm_angle_pub.publish(UInt8(data=0))
            self.gripper_angle_pub.publish(UInt8(data=0))
            if time.time() - self.wait_start > 2.0:
                self.state = 0
                self.currently_collecting = False
                self.object_num = 0
                self.object_center_error = 0.0
                # Collection finished, release control back to the path follower
                self.collecting_pub.publish(Bool(data=False))
                self.get_logger().info('Item collected and reset to search for next item.')


def main(args=None):
    rclpy.init(args=args)
    node = ItemCollect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()