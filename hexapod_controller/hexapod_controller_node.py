import os
import time
import yaml
import math
from importlib import resources

import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import TransformStamped, Twist, Pose
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

import controller
from controller import HexapodController, HexapodKernel, HexapodInterface


class HexapodControllerNode(Node):

    def __init__(self):
        super().__init__('hexapod_controller_node')

        # Use default config inside Hexapod-Controller. We can always specify a new config file as parameter
        default_config_path = resources.files('controller') / 'config' / 'config.yml'
        self.declare_parameter('config_path', str(default_config_path))
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('node_rate', 20.0)  # Hz - ROS2 node publishing rate

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        node_rate = self.get_parameter('node_rate').get_parameter_value().double_value

        # Load config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self._leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]
        self._joint_names = [f'leg_{i+1}_{joint}' for i, leg in enumerate(self._leg_names) for joint in ['coxa', 'femur', 'tibia']]

        # Create the controller
        kernel = HexapodKernel(port=serial_port, baud=serial_baud)
        interface = HexapodInterface(kernel, config)
        self._controller = HexapodController(interface, config, verbose=False)  # we will add logging here

        # Publishers
        self._state_pub = self.create_publisher(String, '/hexapod/state', 10)
        self._joints_pub = self.create_publisher(JointState, '/hexapod/joint_values', 10)
        self._odom_pub = self.create_publisher(Odometry, '/hexapod/odom', 10)
        self._voltage_pub = self.create_publisher(Float32, '/hexapod/battery_voltage', 10)
        self._current_pub = self.create_publisher(Float32, '/hexapod/current_draw', 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, '/hexapod/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Pose, '/hexapod/cmd_pose', self._cmd_pose_cb, 10)

        # Create timer for control loop
        controller_rate = config['control']['update_rate']
        self._controller_dt = 1.0 / controller_rate
        self._last_frame = time.perf_counter()
        self.create_timer(self._controller_dt, self._timer_cb)

        self._node_dt = 1.0 / node_rate
        self.create_timer(self._node_dt, self._publish_cb)

        ros_distro = os.environ.get("ROS_DISTRO")
        self.get_logger().info('HexapodControllerNode started')
        self.get_logger().info(f'Controller version: {controller.__version__}')
        self.get_logger().info(f'ROS distro        : {ros_distro}')
        self.get_logger().info(f'Controller rate   : {controller_rate:.0f} Hz')
        self.get_logger().info(f'Node rate         : {node_rate} Hz')

    def emergency_stop(self):
        self._controller.emergency_stop()

    def _publish_cb(self):
        status = self._controller.get_status()
        self._publish_state(status)
        self._publish_joints(status)
        self._publish_odometry(status)
        self._publish_power(status)

    def _publish_state(self, status: dict):
        msg = String()
        msg.data = status.get('state', 'UNKNOWN')
        self._state_pub.publish(msg)

    def _publish_joints(self, status: dict):
        joint_values = status.get('joint_values', {})
        if not joint_values:
            return

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [
            angle
            for leg in self._leg_names
            for i, angle in enumerate([
                -math.radians(joint_values.get(leg, [0.0, 0.0, 0.0])[0]),  # coxa inverted
                math.radians(joint_values.get(leg, [0.0, 0.0, 0.0])[1]),  # femur unchanged
                math.radians(joint_values.get(leg, [0.0, 0.0, 0.0])[2]),  # tibia unchanged
            ])
        ]

        self._joints_pub.publish(msg)

    def _publish_odometry(self, status: dict):
        """
        Build and publish the odom -> base TF and nav_msgs/Odometry message.

        Frame convention
        ----------------
        The controller works in its own body frame where the robot's forward
        direction is +x.  The URDF/RViz world has the robot facing +y, so the
        controller frame is rotated +90deg (pi/2) around z relative to the URDF
        frame. Concretely:

            controller +x  =  URDF +y   (forward)
            controller +y  =  URDF −x   (right)

        To express the controller's heading in the URDF/world frame we apply a
        −pi/2 yaw offset when composing the quaternion:

            urdf_yaw = controller_yaw − pi/2
        """
        odom = status['odometry']

        # body_position is the effective total (sequencer + user offset), in mm.
        x = status['body_position'][0] / 1000.0 + odom['x'] / 1000.0  # mm -> m
        y = status['body_position'][1] / 1000.0 + odom['y'] / 1000.0
        z = status['body_position'][2] / 1000.0

        # body_orientation is in degrees (see get_status); convert to radians.
        roll  = math.radians(status['body_orientation'][0])
        pitch = math.radians(status['body_orientation'][1])

        # odom['yaw'] is already in radians; body_orientation[2] is in degrees.
        # Combine first, then apply the −pi/2 URDF frame offset.
        # Do NOT call math.radians() again on the combined value — it is already
        # in radians at this point.
        yaw = odom['yaw'] + math.radians(status['body_orientation'][2])
        yaw_urdf = yaw - math.pi / 2   # controller frame -> URDF/world frame

        q = quaternion_from_euler(roll, pitch, yaw_urdf)

        stamp = self.get_clock().now().to_msg()

        # TF: odom -> base
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(tf_msg)

        # nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        vx = status['linear_velocity'][0] / 1000.0  # mm/s -> m/s
        vy = status['linear_velocity'][1] / 1000.0
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = math.radians(status['angular_velocity'])

        self._odom_pub.publish(odom_msg)

    def _publish_power(self, status: dict):
        voltage_msg = Float32()
        voltage_msg.data = float(status['battery_voltage'])
        self._voltage_pub.publish(voltage_msg)

        current_msg = Float32()
        current_msg.data = float(status['current_draw'])
        self._current_pub.publish(current_msg)

    def _cmd_vel_cb(self, msg: Twist):
        self._controller.set_linear_velocity(msg.linear.x, msg.linear.y, msg.linear.z)
        self._controller.set_angular_velocity(msg.angular.z)

    def _cmd_pose_cb(self, msg: Pose):
        """
        Receive a body-pose command in the URDF/ROS frame and forward it to
        the controller in the controller's body frame.

        Frame rotation (URDF -> controller)
        ------------------------------------
        The URDF frame is rotated +pi/2 around z relative to the controller
        frame (controller +x = URDF +y, controller +y = URDF −x).  To convert
        an incoming URDF-frame vector to the controller frame we apply a −pi/2
        rotation around z:

            ctrl_x =  urdf_y
            ctrl_y = −urdf_x

        For yaw the same rotation applies:

            ctrl_yaw = urdf_yaw + pi/2

        This is the inverse of the −pi/2 offset applied in
        _publish_odometry when broadcasting TF.

        Units
        -----
        The ROS Pose message uses metres; the controller expects millimetres.
        Orientation arrives as a quaternion and is converted to Euler angles
        (degrees) before being forwarded.
        """
        # Convert position: metres -> mm, then rotate URDF -> controller frame.
        urdf_x = msg.position.x * 1000.0   # m -> mm
        urdf_y = msg.position.y * 1000.0
        z = msg.position.z * 1000.0   # z axis is shared between both frames

        ctrl_x = urdf_y    # URDF +y  ->  controller +x (forward)
        ctrl_y = -urdf_x    # URDF +x  ->  controller −y (left)

        # Convert orientation: quaternion -> Euler, then rotate yaw.
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        self._controller.set_body_position(ctrl_x, ctrl_y, z)
        self._controller.set_body_orientation(
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

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
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()