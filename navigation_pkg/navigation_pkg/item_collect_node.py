# Author: Mark George Makram
# ID: 22P0060

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray, UInt8
from geometry_msgs.msg import Twist, Vector3, Pose2D
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
        
        # New publisher for the PID target
        self.target_pub = self.create_publisher(Pose2D, '/target_pose', 10)
        
        self.ToF_range_sub = self.create_subscription(
            Float32MultiArray,
            'tof_distances',
            self.ToF_range_callback,
            10
        )

        # New subscriber to check if PID backup is complete
        self.pid_status_sub = self.create_subscription(
            Bool, 
            '/goal_reached', 
            self.status_callback, 
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None
        self.yaw = 0.0
        self.save_pose = None
        self.save_yaw = 0.0
        self.ToF_range = 1000.0 # Initialize high so it doesn't trigger 0 falsely

        self.currently_collecting = False
        self.object_center_error = 0.0
        self.object_num = 0
        self.state = 0
        self.linear_speed = 0.02
        self.side_speed = -0.0
        self.Kp = -0.1
        self.start = False
        self.wait_start = None
        
        self.goal_reached = False
        self.target_published = False

        self.get_logger().info('Item collect node started.')
    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = get_yaw(self.pose.orientation)

    def ToF_range_callback(self, msg):
        if len(msg.data) > 0:
            self.ToF_range = min(msg.data)

    def status_callback(self, msg):
        self.goal_reached = msg.data

    def gui_callback(self, msg):
        command = msg.data.lower()

        if command == 'auto':
            self.start = True
            self.get_logger().info('Auto mode activated. Item collect armed.')
            return

        if command == 'stop' or command == 'manual':
            self.start = False
            self.collecting_pub.publish(Bool(data=False))
            self.get_logger().info('Item collect stopped.')
            return

    def item_detected_callback(self, msg):
        if (not self.currently_collecting) and (msg.x >=1.0) and (msg.y >= 0.5) and self.start:
            self.start = True
            self.currently_collecting = True
            self.object_num = int(msg.x)
            self.object_center_error = (msg.z)
            self.collecting_pub.publish(Bool(data=True)) 
            self.get_logger().info('Item detected! Taking over robot control.')
            
        elif self.currently_collecting and (msg.x ==self.object_num) and (msg.y >= 0.5) and self.start:
            self.object_center_error = (msg.z)

    def control_loop(self):
        if self.wait_start is None:
            self.wait_start = time.time()

        if self.pose is None:
            return

        if not (self.start and self.currently_collecting):
            return

        if self.state == 0:
            self.get_logger().info(f'State 0: Saving initial pose (X: {self.pose.position.x:.2f}, Y: {self.pose.position.y:.2f})')
            self.save_pose = self.pose
            self.save_yaw = self.yaw
            self.state = 1

        elif self.state == 1:
            if abs(self.object_center_error) > 0.06:
                self.get_logger().info(f'State 1: Aligning with item. Current error: {self.object_center_error:.3f}')
                cmd = Twist()
                cmd.angular.z = (-0.15*self.object_center_error) - 0.05*(1 if self.object_center_error>0 else -1 )
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info('State 1: Alignment complete. Transitioning to forward approach.')
                self.cmd_pub.publish(Twist())
                self.state = 2

        elif self.state == 2:
            # Move forward until ToF reads exactly 0
            if self.ToF_range > 15.0:
                self.get_logger().info(f'State 2: Moving forward towards item. Current ToF range: {self.ToF_range:.2f}')
                cmd = Twist()
                cmd.linear.x = self.linear_speed
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info('State 2: ToF range reached 0.0. Stopping forward movement.')
                self.cmd_pub.publish(Twist()) # Stop moving forward
                self.state = 3
                self.target_published = False
                self.goal_reached = False

        elif self.state == 3:
            # Publish a target 30 cm backward using the PID controller
            if not self.target_published:
                backup_target = Pose2D()
                backup_target.x = self.pose.position.x - 0.22 * math.cos(self.yaw)
                backup_target.y = self.pose.position.y - 0.22 * math.sin(self.yaw)
                backup_target.theta = self.yaw
                
                self.target_pub.publish(backup_target)
                self.target_published = True
                self.get_logger().info('ToF read 0. Backing up 30 cm via PID target.')
                
            elif self.goal_reached:
                self.state = 4
                self.wait_start = time.time()

        elif self.state == 4:
            self.arm_angle_pub.publish(UInt8(data=160))
            self.gripper_angle_pub.publish(UInt8(data=60))
            if time.time() - self.wait_start > 2.0:
                self.state = 5
                self.wait_start = time.time()

        elif self.state == 5:
            self.arm_angle_pub.publish(UInt8(data=165))
            self.gripper_angle_pub.publish(UInt8(data=60))
            if time.time() - self.wait_start > 1.0:
                self.state = 6
                self.wait_start = time.time()

        elif self.state == 6:
            self.arm_angle_pub.publish(UInt8(data=165))
            self.gripper_angle_pub.publish(UInt8(data=130))
            if time.time() - self.wait_start > 2.0:
                self.state = 7
                self.wait_start = time.time()
                
        elif self.state == 7:
            self.arm_angle_pub.publish(UInt8(data=20))
            self.gripper_angle_pub.publish(UInt8(data=130))
            if time.time() - self.wait_start > 3.0:
                self.state = 8
                self.wait_start = time.time()
                
        elif self.state == 8:
            self.arm_angle_pub.publish(UInt8(data=20))
            self.gripper_angle_pub.publish(UInt8(data=60))
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