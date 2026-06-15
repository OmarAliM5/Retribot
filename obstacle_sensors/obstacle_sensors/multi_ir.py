#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalInputDevice

class MultiIRSensorNode(Node):
    def __init__(self):
        super().__init__('multi_ir_node')
        
        # The list of GPIO pins your sensors are connected to
        self.ir_pins = [4, 14, 15, 23]
        
        self.sensors = {}
        self.publishers_ = {}
        
        # Loop through the pins to initialize them dynamically
        for pin in self.ir_pins:
            # 1. pull_up=False ensures the pin rests at 0V and waits for a HIGH signal
            self.sensors[pin] = DigitalInputDevice(pin, pull_up=False)
            
            # 2. Create a publisher for each pin (e.g., topic: '/ir_sensor_14')
            topic_name = f'ir_sensor_{pin}'
            self.publishers_[pin] = self.create_publisher(Bool, topic_name, 10)
            
            self.get_logger().info(f'Initialized Active-High IR sensor on GPIO {pin} -> /{topic_name}')
        
        # Check all sensors at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Loop through the dictionary and publish the state of each sensor
        for pin, sensor in self.sensors.items():
            msg = Bool()
            
            # Active-High: sensor outputs 1 (True) when intercepted
            is_intercepted =not bool(sensor.value)
            
            msg.data = is_intercepted
            self.publishers_[pin].publish(msg)
            
            # Optional: Only log when a sensor is actively intercepted
            if is_intercepted:
                self.get_logger().debug(f'Sensor {pin} Intercepted!')

def main(args=None):
    rclpy.init(args=args)
    node = MultiIRSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()