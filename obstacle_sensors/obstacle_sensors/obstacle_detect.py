#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
import time

# ToF imports
import board
import busio
import adafruit_vl53l0x

# GPIO imports
from gpiozero import DigitalInputDevice, DigitalOutputDevice


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        
        # --- 1. ToF Parameters & Setup ---
        self.declare_parameter('offset_mm', 70)
        self.declare_parameter('threshold_mm', 100)
        self.declare_parameter('required_detections', 4) # Added parameter for consecutive hits
        
        self.consecutive_detections = 0 # Counter for consecutive detections

        # Unique addresses for the 3 sensors (default is 0x29)
        self.tof_addresses = [0x2A, 0x2B, 0x2C]
        self.xshut_pins = [17, 27, 22]
        self.tof_sensors = []
        self.xshut_devices = []
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.initialize_tof_sensors()
        except Exception as e:
            self.get_logger().error(f'ToF I2C Bus Error: {e}')

        # --- 2. IR Sensor Setup ---
        self.ir_pins = [4, 14, 15, 23]
        self.ir_sensors = {}
        for pin in self.ir_pins:
            self.ir_sensors[pin] = DigitalInputDevice(pin, pull_up=False)
            self.get_logger().info(f'Initialized Active-High IR sensor on GPIO {pin}')

        # --- 3. Publishers & Timer ---
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        # New publisher for the raw ToF measurements
        self.distance_pub = self.create_publisher(Float32MultiArray, '/tof_distances', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def initialize_tof_sensors(self):
        """Wakes up ToF sensors sequentially to assign unique I2C addresses."""
        default_address = 0x29

        # Step 1: Initialize all X-SHUT pins and pull them LOW to turn off all sensors
        for pin in self.xshut_pins:
            xshut = DigitalOutputDevice(pin)
            xshut.off()  # Pull LOW to reset/disable sensor
            self.xshut_devices.append(xshut)
        
        # Brief pause to ensure all sensors have powered down
        time.sleep(0.1)
        self.get_logger().info('All ToF sensors reset via X-SHUT.')

        # Step 2: Wake them up one by one and reassign addresses
        for i, xshut in enumerate(self.xshut_devices):
            target_address = self.tof_addresses[i]
            
            # Turn on this specific sensor
            xshut.on()
            time.sleep(0.1) # Wait for the sensor to boot
            
            try:
                # The sensor wakes up at the default address
                sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=default_address)
                # Change it to our target address
                sensor.set_address(target_address)
                self.tof_sensors.append(sensor)
                self.get_logger().info(f'ToF {i+1} initialized on X-SHUT pin {self.xshut_pins[i]} at address {hex(target_address)}.')
            except ValueError:
                self.get_logger().error(f'Failed to find ToF {i+1} at default address during initialization.')
            except Exception as e:
                self.get_logger().error(f'Error initializing ToF {i+1}: {e}')

    def timer_callback(self):
        current_raw_detection = False
        tof_measurements = []

        # --- Check ToF Sensors ---
        offset = self.get_parameter('offset_mm').get_parameter_value().integer_value
        threshold = self.get_parameter('threshold_mm').get_parameter_value().integer_value
        
        for i, sensor in enumerate(self.tof_sensors):
            try:
                raw_distance = sensor.range
                corrected_distance = max(0, raw_distance - offset)
                
                # Append the measurement as a float
                tof_measurements.append(float(corrected_distance))
                
                if corrected_distance < threshold:
                    current_raw_detection = True
            except Exception as e:
                self.get_logger().error(f'Error reading ToF sensor {i+1}: {e}')
                # Append -1.0 so the array always has 3 elements, indicating a failed read
                tof_measurements.append(-1.0) 

        # --- Check IR Sensors ---
        for pin, sensor in self.ir_sensors.items():
            is_intercepted = not bool(sensor.value)
            if is_intercepted:
                current_raw_detection = True
                self.get_logger().debug(f'IR Sensor {pin} Intercepted!')

        # --- Apply Filtering Logic ---
        if current_raw_detection:
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0
            
        required_hits = self.get_parameter('required_detections').get_parameter_value().integer_value
        filtered_obstacle_detected = (self.consecutive_detections >= required_hits)

        # --- Publish the Data ---
        
        # 1. Publish the combined, filtered boolean state
        obs_msg = Bool()
        obs_msg.data = filtered_obstacle_detected
        self.obstacle_pub.publish(obs_msg)
        
        # 2. Publish the ToF distances array
        dist_msg = Float32MultiArray()
        dist_msg.data = tof_measurements
        self.distance_pub.publish(dist_msg)
        
        # Optional overall log
        if filtered_obstacle_detected:
            self.get_logger().debug(f'Obstacle verified! ({self.consecutive_detections} consecutive hits)')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()