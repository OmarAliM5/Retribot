# Author: Mark George Makram
# ID: 22P0060

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Bool
import math

def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MotionSequence(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obs_sub = self.create_subscription(Bool, '/obstacle_detected', self.obs_callback, 10)
        self.pid_status_sub = self.create_subscription(Bool, '/goal_reached', self.status_callback, 10)
        self.start_stop_sub = self.create_subscription(String, "/gui_command", self.start_stop_callback, 10)
        self.obs_avoid_sub = self.create_subscription(Bool, '/obstacle_avoidance', self.obs_avoidance_callback, 10)
        
        self.target_pub = self.create_publisher(Pose2D, '/target_pose', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None
        self.yaw = 0.0
        self.start = False
        self.obstacle_avoidance = False
        self.obstacle_detected = False
        
        # State machine initialization
        self.goal_reached = True
        self.state = 0

        # Initial pose tracking for relative movements
        self.has_initial_pose = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.save_state = 0
        self.save_x=0.0
        self.save_y=0.0
        self.save_yaw=0.0

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = get_yaw(self.pose.orientation)

    def obs_callback(self, msg):
        self.obstacle_detected = msg.data

    def status_callback(self, msg):
        self.goal_reached = msg.data

    def obs_avoidance_callback(self, msg):
        self.obstacle_avoidance = msg.data
        self.state = self.save_state

    def start_stop_callback(self, msg):
        try:
            command = msg.data.lower()
            if command == "auto":
                self.get_logger().info("Path following armed.")
                self.start = True
                self.goal_reached = True  # Allow the sequence to start immediately
            elif (command == "stop"):
                self.get_logger().info("Stopping path following.")
                self.state = self.save_state
                self.start = False
                self.goal_reached = True
            elif command == "manual" and not(self.obstacle_avoidance or self.obstacle_detected):
                self.get_logger().info("Manual mode activated. Stopping path following.")
                self.start = False
                self.save_x = self.pose.position.x
                self.save_y = self.pose.position.y
                self.save_yaw = self.yaw
                self.state = 99

                
        except Exception as e:
            self.get_logger().error(f"Error in start_stop_callback: {e}")

    def get_absolute_target(self, rel_pose):
        """
        Transforms a relative Pose2D into global Odometry coordinates 
        based on the robot's initial starting position.
        """
        abs_x = self.initial_x + (rel_pose.x * math.cos(self.initial_yaw)) - (rel_pose.y * math.sin(self.initial_yaw))
        abs_y = self.initial_y + (rel_pose.x * math.sin(self.initial_yaw)) + (rel_pose.y * math.cos(self.initial_yaw))
        abs_yaw = self.initial_yaw + rel_pose.theta

        # Keep yaw normalized between -pi and pi
        abs_yaw = math.atan2(math.sin(abs_yaw), math.cos(abs_yaw))
        
        return Pose2D(x=abs_x, y=abs_y, theta=abs_yaw)

    def publish_target(self, global_target):
        self.target_pub.publish(global_target)
        self.goal_reached = False 
        self.get_logger().info(f"Published Global Target -> X:{global_target.x:.2f}, Y:{global_target.y:.2f}, Yaw:{math.degrees(global_target.theta):.1f}°")

    def control_loop(self):
        if self.pose is None:
            return

        if self.obstacle_avoidance or self.obstacle_detected or not self.start:
            return
            
        # Lock in the origin coordinate frame exactly when the sequence starts
        if not self.has_initial_pose:
            self.initial_x = self.pose.position.x
            self.initial_y = self.pose.position.y
            self.initial_yaw = self.yaw
            self.has_initial_pose = True
            self.get_logger().info(f"Locked Initial Origin -> X:{self.initial_x:.2f}, Y:{self.initial_y:.2f}, Yaw:{math.degrees(self.initial_yaw):.1f}°")
        
        # Define the path RELATIVE to the starting origin
        rel_targets = [
            Pose2D(x=1.0, y=0.0, theta=0.0),    # State 0: 1 meter directly in front of the start point
            Pose2D(x=1.0, y=0.0, theta=-1.57),  # State 1: Pivot -90 degrees
            Pose2D(x=1.0, y=-1.0, theta=-1.57), # State 2: 1 meter right of the start point
            Pose2D(x=1.0, y=-1.0, theta=-3.14)  # State 3: Pivot another -90 degrees
        ]

        if self.state == 0 and self.goal_reached:
            target = self.get_absolute_target(rel_targets[0])
            self.publish_target(target)
            self.save_state = 0
            self.state = 1
            self.get_logger().info("State 0 complete.")

        elif self.state == 1 and self.goal_reached:
            target = self.get_absolute_target(rel_targets[1])
            self.publish_target(target)
            self.save_state = 1
            self.state = 2
            self.get_logger().info("State 1 complete.")

        elif self.state == 2 and self.goal_reached:
            target = self.get_absolute_target(rel_targets[2])
            self.publish_target(target)
            self.save_state = 2
            self.state = 3
            self.get_logger().info("State 2 complete.")

        elif self.state == 3 and self.goal_reached:
            target = self.get_absolute_target(rel_targets[3])
            self.publish_target(target)
            self.save_state = 3
            self.state = 4
            self.get_logger().info("State 3 complete.")

        elif self.state == 4 and self.goal_reached:
            self.get_logger().info("Path sequence complete!")
            self.save_state = 4
            self.state = 5
            self.get_logger().info("state 4 complete.")

        elif self.state == 99 :
            self.publish_target(Pose2D(x=self.save_x, y=self.save_y, theta=self.save_yaw))
            self.get_logger().info("Resuming from manual mode.")
            self.state = self.save_state
            self.save_state = 99

def main():
    rclpy.init()
    node = MotionSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()