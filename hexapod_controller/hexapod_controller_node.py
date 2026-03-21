import os
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from importlib import resources

import controller
from controller import HexapodController, HexapodKernel, HexapodInterface


class HexapodControllerNode(Node):

    def __init__(self):
        super().__init__('hexapod_controller_node')

        # Use default config inside Hexapod-Controller. We can always specify a new config file as parameter
        default_config_path = resources.files('controller.config') / 'config.yml'
        self.declare_parameter('config_path', str(default_config_path))
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('node_rate', 50.0)  # Hz - ROS2 node publishing rate

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        node_rate = self.get_parameter('node_rate').get_parameter_value().double_value

        # Load config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Create the controller
        kernel = HexapodKernel(port=serial_port, baud=serial_baud)
        interface = HexapodInterface(kernel, config)
        self._controller = HexapodController(interface, config)

        # Create timer for control loop
        controller_rate = config['control']['update_rate']
        self._controller_dt = 1.0 / controller_rate
        self._node_dt = 1.0 / node_rate
        self._last_frame = time.perf_counter()
        self.create_timer(self._controller_dt, self._timer_cb)

        ros_distro = os.environ.get("ROS_DISTRO")
        self.get_logger().info('HexapodControllerNode started')
        self.get_logger().info(f'Controller version: {controller.__version__}')
        self.get_logger().info(f'ROS distro        : {ros_distro}')


    def _timer_cb(self):

        now = time.perf_counter()
        dt = now - self._last_frame
        self._last_frame = now

        ok = self._controller.update(dt)

        if not ok:
            self.get_logger().warn('Controller update failed', throttle_duration_sec=1.0)

        # Warn if over budget
        elapsed = time.perf_counter() - now
        if elapsed > self._controller_dt:
            self.get_logger().warn(
                f'Frame over budget: {elapsed * 1000:.1f}ms > {self._controller_dt * 1000:.1f}ms',
                throttle_duration_sec=1.0,
            )


def main(args=None):

    rclpy.init(args=args)
    node = HexapodControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
