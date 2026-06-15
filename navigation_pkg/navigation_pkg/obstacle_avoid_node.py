import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String
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
        self.start_stop_sub = self.create_subscription(String, "/gui_command", self.start_stop_callback, 10)

        self.target_pub = self.create_publisher(Pose2D, '/target_pose', 10)
        self.obs_avoid_pub = self.create_publisher(Bool, '/obstacle_avoidance', 10)

        self.pose = None
        self.yaw = 0.0
        self.obstacle_detected = False
        self.pid_goal_reached = True 
        self.start = False
        self.state = 0

        self.timer = self.create_timer(0.1, self.sequence_loop)
        self.wait_start = None

    def start_stop_callback(self, msg):
        try:
            command = msg.data.lower()
            if command == "auto":
                self.get_logger().info("Obstacle avoidance armed.")
                self.start = True
            elif (command == "stop"):
                self.get_logger().info("Stopping obstacle avoidance.")
                self.start = False
                self.state = 0  
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

        if not self.start:
            self.obs_avoid_pub.publish(Bool(data=False))
            return

        is_avoiding = (self.state != 0)
        self.obs_avoid_pub.publish(Bool(data=is_avoiding))

        # 0 -> Wait for obstacle
        if self.state == 0 and self.obstacle_detected:
            self.state = 1
            self.wait_start = time.time()
            self.get_logger().info("Obstacle detected! Halting robot.")
            
            # IMMEDIATELY halt the robot by feeding its current position to the PID controller
            self.publish_target(self.pose.position.x, self.pose.position.y, self.yaw)

        elif self.state == 1:
            if (time.time() - self.wait_start >= 5.0) and self.obstacle_detected:
                self.get_logger().info("Obstacle still present. Proceeding to strafe right.")
                self.state = 2
            elif (time.time() - self.wait_start >= 5.0) and (not self.obstacle_detected):
                self.get_logger().info("Obstacle cleared before 5 seconds. Resetting sequence.")
                self.state = 0
                
        # 2 -> Strafe Right 0.65m (Local Y = -0.65)
        # REMOVED: `and self.pid_goal_reached` to prevent deadlocking against interrupted path targets
        elif self.state == 2:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            target_x = current_x + (0.65 * math.sin(self.yaw))
            target_y = current_y - (0.65 * math.cos(self.yaw))
            
            self.publish_target(target_x, target_y, self.yaw)
            self.state = 3

        # 3 -> Move Forward 0.65m (Local X = +0.65)
        elif self.state == 3 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            target_x = current_x + (0.65 * math.cos(self.yaw))
            target_y = current_y + (0.65 * math.sin(self.yaw))
            
            self.publish_target(target_x, target_y, self.yaw)
            self.state = 4

        # 4 -> Strafe Left 0.65m (Local Y = +0.65)
        elif self.state == 4 and self.pid_goal_reached:
            current_x = self.pose.position.x
            current_y = self.pose.position.y
            
            target_x = current_x - (0.65 * math.sin(self.yaw))
            target_y = current_y + (0.65 * math.cos(self.yaw))
            
            self.publish_target(target_x, target_y, self.yaw)
            self.state = 5

        # 5 -> Reset Sequence
        elif self.state == 5 and self.pid_goal_reached:
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