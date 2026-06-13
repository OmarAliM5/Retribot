import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Bool

class LineFollower(Node):
        
    def __init__(self):
        super().__init__('line_follower_node')  

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_stop_sub = self.create_subscription(String, "/gui_command", self.start_stop_callback, 10)
        self.lane_info_sub = self.create_subscription(Vector3, '/lane_detection/lane_info', self.lane_callback, 10)
        
        self.obstacle_detected_sub = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_detected_callback, 10)
        self.obstacle_avoidance_sub = self.create_subscription(Bool, '/obstacle_avoidance', self.obstacle_avoidance_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)

        self.error = 0.0
        self.confidence = 0.0
        self.start = False
        self.obstacle_avoidance = False
        self.obstacle_detected = False
    
    def start_stop_callback(self, msg):
        try:
            command = msg.data.lower()
            if command == "auto":
                self.get_logger().info("Starting line following.")
                self.start = True
          
            #elif command == "stop":
             #   self.get_logger().info("Stopping line following.")
              #  self.start = False
                
                # THE FIX: Publish the stop command immediately right here
               # stop_cmd = Twist()
                #self.pub.publish(stop_cmd)
          
            else:
                self.get_logger().warn(f"Unknown command received: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error in start_stop_callback: {e}")

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = msg.data

    def obstacle_avoidance_callback(self, msg):
        self.obstacle_avoidance = msg.data

    def lane_callback(self, msg):
        self.error = msg.x
        self.confidence = msg.y
        
    def control_loop(self):
        # 1. If we are avoiding an obstacle, DO NOTHING! Let the PID node drive.
        if self.obstacle_avoidance or self.obstacle_detected:
            return 

        # 2. If the user pressed "Auto", do line following
        if self.start:
            cmd = Twist()
            Kp = 0.3
            
            if self.confidence > 0.4:
                cmd.linear.x = 0.02
                cmd.angular.z = -Kp * self.error  
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()