# Author: Mark George Makram
# ID: 22P0060

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Bool
import math
import time

def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MotionSequence(Node):
    def __init__(self):
        super().__init__('mini_path_node')

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
        self.last_target_time = 0.0

        # Initial pose tracking for relative movements
        self.has_initial_pose = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.save_state = 0
        self.save_x = 0.0
        self.save_y = 0.0
        self.save_yaw = 0.0

        # Generate the Single Sweep Path
        self.rel_targets = self.generate_path()

    def generate_path(self):
        targets = []
        
        # Single sweep logic
        targets.append(Pose2D(x=1.5, y=0.0, theta=0.0))                   # Go 1.5m in X
        targets.append(Pose2D(x=1.5, y=0.0, theta=-math.pi/2))            # Rotate -90 deg
        targets.append(Pose2D(x=1.5, y=-0.5, theta=-math.pi/2))           # Move 0.5m in -Y
        targets.append(Pose2D(x=1.5, y=-0.5, theta=-math.pi))             # Rotate -90 deg (to -180)
        targets.append(Pose2D(x=0.0, y=-0.5, theta=-math.pi))             # Move 1.5m in -X
        
        # Return to Origin Sequence
        targets.append(Pose2D(x=0.0, y=-0.5, theta=math.pi/2))            # Rotate to face positive Y (home)
        targets.append(Pose2D(x=0.0, y=0.0, theta=math.pi/2))             # Move to Y=0
        targets.append(Pose2D(x=0.0, y=0.0, theta=0.0))                   # Re-align to exactly 0.0 rad
                
        return targets

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = get_yaw(self.pose.orientation)

    def obs_callback(self, msg):
        self.obstacle_detected = msg.data

    def status_callback(self, msg):
        # Anti-race condition: Ignore lingering True states right after publishing a target
        if time.time() - self.last_target_time < 0.5:
            return
        self.goal_reached = msg.data

    def obs_avoidance_callback(self, msg):
        if self.obstacle_avoidance == True and msg.data == False:
            self.state = self.save_state
        self.obstacle_avoidance = msg.data

    def start_stop_callback(self, msg):
        try:
            command = msg.data.lower()
            if command == "auto":
                # FIX: Ensure we do not reset the path if we are resuming from manual mode (State 99)
                if self.state >= len(self.rel_targets) and self.state != 99:
                    self.get_logger().info("Previous path complete. Restarting sequence from the beginning.")
                    self.state = 0
                    self.save_state = 0
                
                self.get_logger().info("Path following armed.")
                self.start = True
                self.goal_reached = True  
                
            elif (command == "stop"):
                self.get_logger().info("Stopping path following.")
                self.state = self.save_state
                self.start = False
                self.goal_reached = True
                
            elif command == "manual" and not(self.obstacle_avoidance or self.obstacle_detected):
                self.get_logger().info("Manual mode activated. Stopping path following.")
                self.start = False
                
                # Save exact coordinates to drive back to when Auto is pressed again
                self.save_x = self.pose.position.x
                self.save_y = self.pose.position.y
                self.save_yaw = self.yaw
                self.state = 99
                
        except Exception as e:
            self.get_logger().error(f"Error in start_stop_callback: {e}")

    def publish_target(self, global_target):
        self.target_pub.publish(global_target)
        self.goal_reached = False 
        self.last_target_time = time.time()
        self.get_logger().info(f"Published Global Target -> X:{global_target.x:.2f}, Y:{global_target.y:.2f}, Yaw:{math.degrees(global_target.theta):.1f}°")

    def control_loop(self):
        if self.pose is None:
            return

        if self.obstacle_avoidance or self.obstacle_detected or not self.start:
            return
            
        if not self.has_initial_pose:
            self.initial_x = self.pose.position.x
            self.initial_y = self.pose.position.y
            self.initial_yaw = self.yaw
            self.has_initial_pose = True
            self.get_logger().info(f"Locked Initial Origin -> X:{self.initial_x:.2f}, Y:{self.initial_y:.2f}, Yaw:{math.degrees(self.initial_yaw):.1f}°")
        
        # Execute active path
        if self.state < len(self.rel_targets) and self.goal_reached:
            target = self.get_absolute_target(self.rel_targets[self.state])
            self.publish_target(target)
            self.save_state = self.state
            self.state += 1
            self.get_logger().info(f"State {self.save_state} complete.")

        # Path sequence finished
        elif self.state == len(self.rel_targets) and self.goal_reached:
            self.get_logger().info("Mini path sequence complete! Waiting for AUTO command to restart.")
            self.save_state = self.state
            self.state += 1 

        # Resume from manual mode
        elif self.state == 99:
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