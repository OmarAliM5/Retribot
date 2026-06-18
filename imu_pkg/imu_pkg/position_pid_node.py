#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class PositionPIDNode(Node):
    def __init__(self):
        super().__init__('position_pid_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        
        # State tracking for the goal_reached topic
        self.target_ever_set = False 
        self.is_reached = False
        
        # Track obstacle avoidance state to prevent accidental manual overrides
        self.is_avoiding = False

        self.kp_dist = 0.8
        self.kp_yaw = 0.4
        self.kp_final_yaw = 0.8

        # --- SPEED LIMITS REDUCED HERE ---
        self.max_v = 0.04  # Reduced from 0.12
        self.max_w = 0.12  # Reduced from 0.20

        self.pos_tolerance = 0.04
        self.yaw_tolerance = math.radians(5.0)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for goal_reached status
        self.status_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.target_sub = self.create_subscription(
            Pose2D,
            '/target_pose',
            self.target_callback,
            10
        )

        self.start_stop_sub = self.create_subscription(String, "/gui_command", self.start_stop_callback, 10)
        
        # Subscription to know when an obstacle avoidance sequence is running
        self.obs_avoid_sub = self.create_subscription(Bool, '/obstacle_avoidance', self.obs_avoid_callback, 10)
        
        self.collecting_sub = self.create_subscription(Bool, '/collecting_item', self.collecting_callback, 10)
        self.is_collecting = False


        self.last_gui_command = None  # Track the last GUI command to avoid redundant processing
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Position PID Node Started')

    def obs_avoid_callback(self, msg):
        self.is_avoiding = msg.data

    def start_stop_callback(self, msg):
        self.last_gui_command = msg.data.lower()
        if self.last_gui_command == "manual":
            # Safety override: Do not abort PID if obstacle avoidance needs it to maneuver
            if self.is_avoiding:
                self.get_logger().warn("Obstacle avoidance active. Ignoring manual mode switch in PID node.")
                return
                
            self.get_logger().info("Manual mode activated. Stopping PID control.")
            self.stop_robot()
            self.has_target = False  # Clear any active target when switching to manual
        elif self.last_gui_command == "auto":
            self.get_logger().info("Auto mode activated. PID control enabled.")
        elif self.last_gui_command == "stop":
            self.get_logger().info("Stop command received. Stopping robot and PID control.")
            self.stop_robot()
            self.has_target = False  # Clear any active target when stopping
            self.is_reached = True  # Indicate that the goal is reached when stopping

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, max_value):
        return max(min(value, max_value), -max_value)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    def collecting_callback(self, msg):
        # If transitioning from not collecting to collecting
        if msg.data == True and not self.is_collecting:
            self.get_logger().info("Item collection taking over. Canceling current path target.")
            self.stop_robot()
            self.has_target = False  # This stops the PID control loop from driving forward
            
            self.is_reached = False

        self.is_collecting = msg.data
        

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta
        self.has_target = True
        
        # Reset states when a new target arrives
        self.target_ever_set = True
        self.is_reached = False

        self.get_logger().info(
            f'New target: x={self.target_x:.2f}, '
            f'y={self.target_y:.2f}, yaw={math.degrees(self.target_yaw):.1f} deg'
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        # Spam the current status at the timer frequency (20 Hz)
        if self.target_ever_set:
            status_msg = Bool()
            status_msg.data = self.is_reached
            self.status_pub.publish(status_msg)

        # If there is no active target, exit the loop before sending motor commands
        if not self.has_target:
            return

        dx_world = self.target_x - self.x
        dy_world = self.target_y - self.y

        distance = math.sqrt(dx_world * dx_world + dy_world * dy_world)
        final_yaw_error = self.normalize_angle(self.target_yaw - self.yaw)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        ex_body = cos_yaw * dx_world + sin_yaw * dy_world
        ey_body = -sin_yaw * dx_world + cos_yaw * dy_world

        cmd = Twist()

        if distance > self.pos_tolerance:
            vx = self.clamp(self.kp_dist * ex_body, self.max_v)
            vy = self.clamp(self.kp_dist * ey_body, self.max_v)
            
            # --- IN-MOTION TURNING LIMIT REDUCED HERE ---
            wz = self.clamp(self.kp_yaw * final_yaw_error, 0.10) # Reduced from 0.15

            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz

        else:
            if abs(final_yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.clamp(
                    self.kp_final_yaw * final_yaw_error,
                    self.max_w
                )
            else:
                self.stop_robot()
                self.has_target = False
                
                # Mark as reached so the next timer loops spam True
                self.is_reached = True 
                
                self.get_logger().info('Target reached')
                return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()