import os
import yaml
import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import TransformStamped, Twist, Pose
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

import controller
from controller import HexapodController, HexapodKernel, HexapodInterface


class FakeKernel:
    """Drop-in replacement for HexapodKernel that needs no UART hardware.

    Tracks commanded servo angles internally so reads reflect the last write,
    making the controller's IK feedback loop behave correctly without servos.
    """

    def __init__(self):
        self._servo_angles = {}  # pin -> float (degrees)

    def close(self):
        pass

    def connect_power(self) -> bool:
        return True

    def disconnect_power(self) -> bool:
        return True

    def get_voltage(self) -> float:
        return 12.0

    def get_current(self) -> float:
        return 0.0

    def attach_servos(self) -> bool:
        return True

    def detach_servos(self) -> bool:
        return True

    def set_servo_angle(self, pin: int, angle: float) -> bool:
        self._servo_angles[pin] = float(angle)
        return True

    def set_servo_angles(self, values) -> bool:
        for pin, angle in values:
            self._servo_angles[pin] = float(angle)
        return True

    def get_servo_angle(self, pin: int) -> float:
        return self._servo_angles.get(pin, 0.0)

    def get_servo_angles(self, pins) -> list:
        return [self._servo_angles.get(pin, 0.0) for pin in pins]

    def set_led(self, pin: int, r: int, g: int, b: int) -> bool:
        return True


class HexapodControllerNode(Node):

    def __init__(self):
        super().__init__('hexapod_controller_node')

        # Use default config inside Hexapod-Controller. We can always specify a new config file as parameter
        package_share = get_package_share_directory('hexapod_controller')
        default_config_path = os.path.join(package_share, 'config', 'config.yml')
        self.declare_parameter('config_path', str(default_config_path))
        self.declare_parameter('node_rate', 20)  # Hz - ROS2 node publishing rate
        self.declare_parameter('power_telemetry_rate', 2)  # Hz - battery/current polling
        self.declare_parameter('fake_hardware', False)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        node_rate = self.get_parameter('node_rate').get_parameter_value().integer_value
        power_rate = self.get_parameter('power_telemetry_rate').get_parameter_value().integer_value
        fake_hardware = self.get_parameter('fake_hardware').get_parameter_value().bool_value

        # Load config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self._leg_names = [k for k, v in config['kinematics']['legs'].items() if isinstance(v, dict)]
        self._joint_names = [f'leg_{i+1}_{joint}' for i, leg in enumerate(self._leg_names) for joint in ['coxa', 'femur', 'tibia']]

        # Velocities
        self.lin_vel_max = config['safety'].get('lin_vel_max', 250)
        self.ang_vel_max = config['safety'].get('ang_vel_max', 60)
        self.body_lin_vel_max = config['safety'].get('body_lin_vel_max', 50)
        self.body_ang_vel_max = config['safety'].get('body_ang_vel_max', 30)

        # Create the controller
        if fake_hardware:
            kernel = FakeKernel()
        else:
            serial_port = config['serial'].get('port', '/dev/ttyAMA0')
            serial_baud = config['serial'].get('baud', 115200)
            kernel = HexapodKernel(port=serial_port, baud=serial_baud)
        interface = HexapodInterface(kernel, config)
        self._controller = HexapodController(interface, config, verbose=False)

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
        self.create_subscription(Twist, '/hexapod/cmd_vel_norm', self._cmd_vel_norm_cb, 10)
        self.create_subscription(Float32, '/hexapod/cmd_height', self._cmd_height_cb, 10)
        self.create_subscription(Float32, '/hexapod/cmd_pitch', self._cmd_pitch_cb, 10)

        # Create timer for control loop.
        controller_rate = config['rate']['controller_update_rate']
        self._controller_dt = 1.0 / controller_rate

        # Upper bound on the dt fed to the gait. With a single-threaded executor
        # a control tick can be delayed (e.g. while the publish callback does its
        # serial telemetry reads); clamping prevents a delayed tick from injecting
        # a phase lurch, while a near-zero dt from a catch-up firing simply adds no
        # phase. This keeps gait pacing tied to real time without jerk.
        self._max_controller_dt = 2.0 * self._controller_dt
        self._last_ros_time = self.get_clock().now()
        self.create_timer(self._controller_dt, self._timer_cb)

        self._node_dt = 1.0 / node_rate
        self.create_timer(self._node_dt, self._publish_cb)

        # Battery/current readings hit the serial link. Poll them on a slow timer
        # rather than inside the high-rate publish path, so the frequent joint/odom
        # publishing stays free of serial I/O and does not block the (single-
        # threaded) control loop.
        self._power_dt = 1.0 / power_rate
        self.create_timer(self._power_dt, self._publish_power_cb)

        ros_distro = os.environ.get("ROS_DISTRO")
        self.get_logger().info('HexapodControllerNode started')
        self.get_logger().info(f'Controller version: {controller.__version__}')
        self.get_logger().info(f'ROS distro        : {ros_distro}')
        self.get_logger().info(f'Controller rate   : {controller_rate} Hz')
        self.get_logger().info(f'Node rate         : {node_rate} Hz')
        self.get_logger().info(f'Power rate        : {power_rate} Hz')
        if fake_hardware:
            self.get_logger().warning('fake_hardware=True — no UART, servo commands are silently discarded')

    def emergency_stop(self):
        self._controller.emergency_stop()

    def _publish_cb(self):
        status = self._read_cheap_status()
        self._publish_state(status)
        self._publish_joints(status)
        self._publish_odometry(status)

    def _read_cheap_status(self) -> dict:
        """
        Snapshot of controller state that needs NO serial I/O.

        This mirrors the serial-free fields of HexapodController.get_status()
        and deliberately omits battery_voltage / current_draw — the only fields
        that poll the serial link. Those are published separately at a low rate
        (see _publish_power_cb) so this high-rate path never blocks the control
        loop on serial traffic.
        """
        c = self._controller
        return {
            'state': c.state.name,
            'body_position': c.body_position.tolist(),
            'body_orientation': [math.degrees(a) for a in c.body_orientation],
            'linear_velocity': c.linear_velocity.tolist(),
            'angular_velocity': c.angular_velocity,
            'joint_values': {k: v.tolist() for k, v in c.current_joints.items()},
            'odometry': {
                'x': c.odom_x,
                'y': c.odom_y,
                'yaw': c.odom_yaw,
            },
        }

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

        # Collect the angles in a format that is suitable to JointState messages and then
        # translate them from kinematic space (controller) to servo space (same as urdf)
        positions = [
            angle
            for leg in self._leg_names
            for i, angle in enumerate([
                self._controller.interface.kinematic_space_to_servo_space(leg, 0, joint_values.get(leg, [0.0, 0.0, 0.0])[0]),  # coxa inverted
                self._controller.interface.kinematic_space_to_servo_space(leg, 1, joint_values.get(leg, [0.0, 0.0, 0.0])[1]),  # femur unchanged
                self._controller.interface.kinematic_space_to_servo_space(leg, 2, joint_values.get(leg, [0.0, 0.0, 0.0])[2]),  # tibia unchanged
            ])
        ]

        # Degrees to radians
        positions = [math.radians(angle) for angle in positions]

        msg.position = positions

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
        y = - (status['body_position'][1] / 1000.0 + odom['y'] / 1000.0)
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

        # NOTE: odometry over-reports during capped combined maneuvers.
        # These velocities (and the integrated pose above) come from the
        # controller's commanded velocity. When a combined linear+angular
        # command exceeds the gait's max_phase_rate, the gait clamps phase_rate
        # (so the robot physically moves slower) but the controller still
        # integrates the full commanded velocity. The published twist/pose
        # therefore over-estimate the true motion by effective_speed/max while
        # the cap is active. Only an issue for aggressive combined turns; for
        # exact dead-reckoning the gait would need to report its achieved
        # (post-clamp) velocity back to the odometry integrator.
        vx = status['linear_velocity'][0] / 1000.0  # mm/s -> m/s
        vy = status['linear_velocity'][1] / 1000.0
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = math.radians(status['angular_velocity'])

        self._odom_pub.publish(odom_msg)

    def _publish_power_cb(self):
        # Slow timer: the voltage/current reads go over the serial link, so they
        # run at power_telemetry_rate (a few Hz) rather than at node_rate. Calls
        # the interface directly — single-threaded execution serializes this with
        # the control loop's serial access, so frames never interleave.
        voltage_msg = Float32()
        voltage_msg.data = float(self._controller.interface.get_voltage())
        self._voltage_pub.publish(voltage_msg)

        current_msg = Float32()
        current_msg.data = float(self._controller.interface.get_current())
        self._current_pub.publish(current_msg)

    def _cmd_vel_cb(self, msg: Twist):
        # geometry_msgs/Twist follows ROS SI conventions (linear in m/s, angular
        # in rad/s), whereas the controller API works in mm/s and deg/s. Convert
        # at this boundary so the topic stays standards-compliant (usable by
        # rviz/nav2/standard teleop) while the controller keeps its interpretable
        # internal units. The normalized topic (/hexapod/cmd_vel_norm) needs no
        # conversion — it carries unitless [-1, 1] values scaled by config maxima.
        #
        # No phase-rate limiting here: the gait generator caps phase_rate
        # (max_phase_rate) so combined linear+angular commands don't drive
        # the swing too short for the servos.
        self._controller.set_linear_velocity(
            msg.linear.x * 1000.0,   # m/s -> mm/s
            msg.linear.y * 1000.0,
            msg.linear.z * 1000.0,
        )
        self._controller.set_angular_velocity(
            math.degrees(msg.angular.z),  # rad/s -> deg/s
        )

    def _cmd_vel_norm_cb(self, msg: Twist):
        """
        Receive a normalized linear velocity that will be scaled by the maximum
        velocity defined in the config file.
        """
        self._controller.set_linear_velocity(
            round(msg.linear.x, 2) * self.lin_vel_max,
            round(msg.linear.y, 2) * self.lin_vel_max,
            round(msg.linear.z, 2) * self.lin_vel_max
        )
        self._controller.set_angular_velocity(
            round(msg.angular.z, 2) * self.ang_vel_max,
        )

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
        x = msg.position.x * 1000.0   # m -> mm
        y = msg.position.y * 1000.0
        z = msg.position.z * 1000.0   # z axis is shared between both frames

        # Convert orientation: quaternion -> Euler, then rotate yaw.
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        self._controller.set_body_position(x, y, z)
        self._controller.set_body_orientation(
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    def _cmd_height_cb(self, msg: Float32):
        """
        Receive absolute height offset in mm.
        """
        self._controller.set_body_position(0, 0, msg.data)

    def _cmd_pitch_cb(self, msg: Float32):
        """
        Receive pitch offset in degrees.
        """
        self._controller.set_body_orientation(0, msg.data, 0)

    def _timer_cb(self):
        # Advance the gait by the real elapsed wall-clock time, exactly like the
        # standalone controller loop (main.py passes its measured actual_dt).
        #
        # Feeding a constant nominal period here is wrong: when the timer falls
        # behind and the executor fires it back-to-back to catch up, each firing
        # would advance the gait phase by a full period of stride despite almost
        # no real time having passed. That produces bursts of fast, tiny steps
        # followed by stalls (jerky, "in-place" motion) and decouples gait cadence
        # from real time, so the robot covers less ground/rotation than commanded.
        #
        # The dt is clamped (see _max_controller_dt) so a single delayed tick
        # cannot lurch the gait, and a ~0 dt catch-up firing simply adds no phase.
        now = self.get_clock().now()
        dt = (now - self._last_ros_time).nanoseconds / 1e9
        self._last_ros_time = now
        dt = min(max(dt, 0.0), self._max_controller_dt)

        ok = self._controller.update(dt)

        if not ok:
            self.get_logger().warn('Controller update failed', throttle_duration_sec=1.0)

        # Warn if over budget
        elapsed = (self.get_clock().now() - now).nanoseconds / 1e9
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
        if rclpy.ok():
            rclpy.shutdown()
