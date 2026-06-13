import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String  # Added String import
import math
import time

def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MotionSequence(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obs_sub = self.create_subscription(Bool, '/obstacle_detected', self.obs_callback, 10)
        self.pid_status_sub = self.create_subscription(Bool, '/goal_reached', self.status_callback, 10)
        
        # ADDED: Subscription for GUI commands
        self.start_stop_sub = self.create_subscription(String, "/gui_command", self.start_stop_callback, 10)

        self.target_pub = self.create_publisher(Pose2D, '/target_pose', 10)
        self.obs_avoid_pub = self.create_publisher(Bool, '/obstacle_avoidance', 10)

        self.pose = None
        self.yaw = 0.0
        self.obstacle_detected = False
        self.pid_goal_reached = True 

        # ADDED: Start flag controlled by GUI
        self.start = False

        self.state = 0

        self.timer = self.create_timer(0.1, self.sequence_loop)
        self.wait_start = None

    # ADDED: Callback to handle Start/Stop from GUI
    def start_stop_callback(self, msg):
        try:
            command = msg.data.lower()
            if command == "auto":
                self.get_logger().info("Obstacle avoidance armed.")
                self.start = True
            elif (command == "stop") or (command == "manual"):
                self.get_logger().info("Stopping obstacle avoidance.")
                self.start = False
                self.state = 0  # Reset state machine so it doesn't resume mid-maneuver
        except Exception as e:
            self.get_logger().error(f"Error in start_stop_callback: {e}")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = get_yaw(self.pose.orientation)

    def obs_callback(self, msg):
        self.obstacle_detected = msg.data

    def status_callback(self, msg):
        self.pid_goal_reached = msg.data

    def publish_target(self, x, y, yaw):
        target = Pose2D()
        target.x = x
        target.y = y
        target.theta = math.atan2(math.sin(yaw), math.cos(yaw))
        self.target_pub.publish(target)
        self.pid_goal_reached = False 
        self.get_logger().info(f"Sent PID Target: X:{x:.2f}, Y:{y:.2f}, Yaw:{math.degrees(target.theta):.1f}°")

    def sequence_loop(self):
        if self.wait_start is None:
            self.wait_start = time.time()

        if self.pose is None:
            return

        # ADDED: If GUI hasn't started auto mode, broadcast False and do nothing
        if not self.start:
            self.obs_avoid_pub.publish(Bool(data=False))
            return

        # Continuously broadcast our status so the Line Follower doesn't interrupt!
        is_avoiding = (self.state != 0)
        self.obs_avoid_pub.publish(Bool(data=is_avoiding))

        # 0 -> Wait for obstacle
        if self.state == 0 and self.obstacle_detected:
            self.state = 1
            self.wait_start = time.time()
            self.get_logger().info("Obstacle detected! Starting avoidance sequence.")

        elif self.state == 1:
            if (time.time() - self.wait_start >=3.0) and self.obstacle_detected:
                self.get_logger().info("Obstacle still present after 3 seconds. Proceeding with avoidance.")
                self.state = 2
            elif (time.time() - self.wait_start >=3.0) and (not self.obstacle_detected):
                self.get_logger().info("Obstacle cleared before 3 seconds. Resetting sequence.")
                self.state = 0
                
        # 2 -> Rotate 90 degrees
        elif self.state == 2 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            turn_angle = -1.57
            target_yaw = self.yaw + turn_angle
            
            self.publish_target(current_x, current_y, target_yaw)
            self.state = 3

        # 3 -> Move forward 0.65m in the new direction
        elif self.state == 3 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            target_x = current_x + (0.65 * math.cos(self.yaw))
            target_y = current_y + (0.65* math.sin(self.yaw))
            
            self.publish_target(target_x, target_y, self.yaw)
            self.state = 4

        # 4 -> Rotate back to original heading
        elif self.state == 4 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            turn_angle = 1.57
            target_yaw = self.yaw + turn_angle
            
            self.publish_target(current_x, current_y, target_yaw)
            self.state = 5

        # 5 -> Move forward 0.4m past the obstacle
        elif self.state == 5 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            target_x = current_x + (0.4 * math.cos(self.yaw))
            target_y = current_y + (0.4 * math.sin(self.yaw))
            
            self.publish_target(target_x, target_y, self.yaw)
            self.state = 6

        # 6 -> Reset Sequence
        elif self.state == 6 and self.pid_goal_reached:
            self.state = 0
            self.get_logger().info("Avoidance Sequence Complete!")
            
def main(args=None):
    rclpy.init(args=args)
    node = MotionSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()