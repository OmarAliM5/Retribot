#!/usr/bin/env python3

import os
import signal
import subprocess
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager_node')

        package_share = get_package_share_directory('process_manager_pkg')
        config_file = os.path.join(package_share, 'config', 'managed_nodes.yaml')

        self.processes = {}
        self.commands = self.load_config(config_file)

        self.cmd_sub = self.create_subscription(
            String,
            '/process_manager/command',
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/process_manager/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Process Manager Started')
        self.get_logger().info(f'Config: {config_file}')
        self.get_logger().info(f'Nodes: {list(self.commands.keys())}')

        startup_order = ['micro_ros', 'imu', 'ekf_node', 'position_pid', 'mecanum']
        for name in startup_order:
            if name in self.commands:
                self.start_process(name)

    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data.get('nodes', {})
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {e}')
            return {}

    def command_callback(self, msg):
        cmd = msg.data.strip().split()

        if not cmd:
            return

        action = cmd[0].lower()

        if action == 'start_all':
            order = ['micro_ros', 'imu', 'ekf_node', 'position_pid', 'mecanum']
            for name in order:
                if name in self.commands:
                    self.start_process(name)
            return

        if action == 'stop_all':
            order = ['mecanum', 'position_pid', 'ekf_node', 'imu']
            for name in order:
                if name in self.commands:
                    self.stop_process(name)
            return

        if action == 'status':
            self.publish_status()
            return

        if len(cmd) < 2:
            self.get_logger().warn('use: start <name> / stop <name> / status / start_all / stop_all')
            return

        name = cmd[1]

        if name not in self.commands:
            self.get_logger().warn(f'Unknown: {name}')
            return

        if action == 'start':
            self.start_process(name)
        elif action == 'stop':
            self.stop_process(name)
        elif action == 'restart':
            self.stop_process(name)
            self.start_process(name)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def start_process(self, name):
        if name in self.processes and self.processes[name].poll() is None:
            self.get_logger().info(f'{name} already running')
            return

        command = self.commands[name]

        full_cmd = (
            "bash -lc "
            f"\"source /opt/ros/jazzy/setup.bash && "
            f"source ~/ws2/install/setup.bash && "
            f"{command}\""
        )

        try:
            proc = subprocess.Popen(
                full_cmd,
                shell=True,
                preexec_fn=os.setsid
            )
            self.processes[name] = proc
            self.get_logger().info(f'Started {name}: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to start {name}: {e}')

    def stop_process(self, name):
        if name not in self.processes or self.processes[name].poll() is not None:
            self.get_logger().info(f'{name} not running')
            return

        try:
            os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
            self.processes[name].wait(timeout=3.0)
            self.get_logger().info(f'Stopped {name}')
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.processes[name].pid), signal.SIGKILL)
            self.get_logger().warn(f'Force killed {name}')
        except Exception as e:
            self.get_logger().error(f'Failed to stop {name}: {e}')

    def publish_status(self):
        msg = String()
        status = []

        for name in self.commands.keys():
            if name in self.processes and self.processes[name].poll() is None:
                status.append(f'{name}:RUNNING')
            else:
                status.append(f'{name}:STOPPED')

        msg.data = ' | '.join(status)
        self.status_pub.publish(msg)

    def destroy_node(self):
        stop_order = ['mecanum', 'position_pid', 'ekf_node', 'imu']
        for name in stop_order:
            if name in self.commands:
                self.stop_process(name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProcessManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()