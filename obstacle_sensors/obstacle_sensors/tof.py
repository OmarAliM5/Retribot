#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import board
import busio
import adafruit_vl53l0x

class VL53L0XNode(Node):
    def __init__(self):
        super().__init__('vl53l0x_node')
        
        # Parameters
        self.declare_parameter('offset_mm', 70)
        self.declare_parameter('i2c_address', 0x29) 
        
        # New parameter: threshold in millimeters (default: 200mm / 20cm)
        self.declare_parameter('threshold_mm', 200)
        
        # Publisher is now a Bool
        self.publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)
        
        target_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.initialize_sensor(target_address)
        except Exception as e:
            self.get_logger().error(f'Initialization Error: {e}')
            return

        self.timer = self.create_timer(0.1, self.timer_callback)

    def initialize_sensor(self, target_address):
        """Safely connects to the sensor and changes its address if needed."""
        import time
        default_address = 0x29
        
        # Try a few times in case the I2C bus is temporarily hung from an abrupt shutdown
        for attempt in range(3):
            try:
                # SCENARIO A: No custom address requested
                if target_address == default_address:
                    self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=default_address)
                    self.get_logger().info(f'VL53L0X Initialized at default address {hex(default_address)}.')
                    return

                # SCENARIO B: Custom address requested
                # 1. Warm Boot: Try the target address FIRST
                try:
                    self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=target_address)
                    self.get_logger().info(f'Sensor was already at target address {hex(target_address)}.')
                    return
                except Exception:
                    pass # Move on to check the default address
                
                # 2. Cold Boot: If target address failed, it must be at the default address
                self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=default_address)
                self.sensor.set_address(target_address)
                self.get_logger().info(f'Successfully changed I2C address to {hex(target_address)}.')
                return

            except Exception as e:
                self.get_logger().warn(f'Init attempt {attempt + 1} failed: {e}. Retrying...')
                time.sleep(0.5)
                
        self.get_logger().error(f'Could not initialize sensor at {hex(default_address)} or {hex(target_address)}.')
        raise RuntimeError("Failed to find expected ID register values. Check wiring or power cycle the sensor.")

    def timer_callback(self):
        try:
            offset = self.get_parameter('offset_mm').get_parameter_value().integer_value
            threshold = self.get_parameter('threshold_mm').get_parameter_value().integer_value
            
            raw_distance = self.sensor.range
            corrected_distance = max(0, raw_distance - offset)
            
            # Logic: True if something is closer than the threshold
            is_close = corrected_distance < threshold
                
            msg = Bool()
            msg.data = is_close
            self.publisher_.publish(msg)
            
            self.get_logger().debug(f'Dist: {corrected_distance}mm | Threshold: {threshold}mm | Output: {is_close}')
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VL53L0XNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()