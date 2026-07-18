import os
import math

import yaml
import rclpy
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32, Bool
from geometry_msgs.msg import TransformStamped, Twist, Pose
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from hexapod import connect, GaitId


GAITS = {'tripod': GaitId.TRIPOD, 'wave': GaitId.WAVE, 'ripple': GaitId.RIPPLE}


class HexapodControllerNode(Node):
    """
    ROS2 front end for the Servo2040 that provisions the board, 
    streams velocity and body-pose setpoints, keeps the command watchdog 
    alive with heartbeats and republishes telemetry
    """

    def __init__(self):
        super().__init__('hexapod_controller_node')

        package_share = get_package_share_directory('hexapod_controller')
        default_config_path = os.path.join(package_share, 'config', 'config.yml')
        self.declare_parameter('config_path', str(default_config_path))
        self.declare_parameter('port', '')             # overrides serial.port from the config when set
        self.declare_parameter('update_rate', 50)      # Hz, single serial loop (command write + telemetry read)
        self.declare_parameter('reply_timeout', 0.05)  # s, per-query wait, a lost reply costs this much instead of stalling the loop
        self.declare_parameter('enable_on_start', False)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        reply_timeout = self.get_parameter('reply_timeout').get_parameter_value().double_value
        enable_on_start = self.get_parameter('enable_on_start').get_parameter_value().bool_value

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Joint names follow the board's leg-major order (coxa, femur, tibia per leg)
        leg_names = list(config['kinematics']['mounts'].keys())
        self._joint_names = [f'leg_{i + 1}_{joint}'
                             for i in range(len(leg_names))
                             for joint in ('coxa', 'femur', 'tibia')]

        self.lin_vel_max = config['safety'].get('lin_vel_max', 300)
        self.ang_vel_max = config['safety'].get('ang_vel_max', 60)

        # Open the serial link and provision the board from the config
        self.get_logger().info('Connecting to the Servo2040 and provisioning...')
        self._bot = connect(config=config, port=(port or None), timeout=reply_timeout)
        self.get_logger().info('Provisioning complete')
        if enable_on_start:
            self._bot.enable()
            self.get_logger().info('Sent ENABLE, the board is running its startup sequence')
        else:
            self.get_logger().info('Robot in OFF state, send True on /hexapod/enable to stand up')

        # Desired setpoints and dirty flags (sent only when they change)
        self._lin = [0.0, 0.0]   # vx, vy mm/s
        self._wz = 0.0           # deg/s
        self._pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # dx, dy, dz mm, roll, pitch, yaw deg
        self._vel_dirty = False
        self._pose_dirty = False

        # Publishers
        self._state_pub = self.create_publisher(String, '/hexapod/state', 10)
        self._joints_pub = self.create_publisher(JointState, '/hexapod/joint_values', 10)
        self._odom_pub = self.create_publisher(Odometry, '/hexapod/odom', 10)
        self._voltage_pub = self.create_publisher(Float32, '/hexapod/battery_voltage', 10)
        self._current_pub = self.create_publisher(Float32, '/hexapod/current_draw', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Bool, '/hexapod/enable', self._enable_cb, 10)
        self.create_subscription(Twist, '/hexapod/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Twist, '/hexapod/cmd_vel_norm', self._cmd_vel_norm_cb, 10)
        self.create_subscription(Pose, '/hexapod/cmd_pose', self._cmd_pose_cb, 10)
        self.create_subscription(Float32, '/hexapod/cmd_height', self._cmd_height_cb, 10)
        self.create_subscription(Float32, '/hexapod/cmd_pitch', self._cmd_pitch_cb, 10)
        self.create_subscription(String, '/hexapod/cmd_gait', self._cmd_gait_cb, 10)

        # One serial loop: a single thread owns the port, writing the command or
        # heartbeat then reading telemetry in a fixed order, so the cadence stays
        # regular instead of three timers competing and bursting when the link is slow
        self.create_timer(1.0 / max(update_rate, 1), self._update_cb)

        self.get_logger().info('HexapodControllerNode started')

        ros_distro = os.environ.get("ROS_DISTRO")
        # self.get_logger().info(f'Controller version: {hexapod.__version__}')
        self.get_logger().info(f'ROS distro        : {ros_distro}')
        self.get_logger().info(f'Update rate       : {update_rate} Hz')     # Hz, single serial loop (command + telemetry)

    # One serial loop: write the pending command or heartbeat, then read telemetry

    def _update_cb(self):
        # Command path first, so the board gets the freshest setpoint and the
        # watchdog is pet even if the telemetry reads below fail
        try:
            if self._vel_dirty:
                self._bot.set_velocity(self._lin[0], self._lin[1], self._wz)
                self._vel_dirty = False
            else:
                self._bot.heartbeat()   # keep the board's command watchdog happy
            if self._pose_dirty:
                self._bot.set_body_pose(*self._pose)
                self._pose_dirty = False
        except Exception as exc:
            self.get_logger().warn(f'send failed: {exc}', throttle_duration_sec=1.0)

        # Telemetry path: two serial round-trips, then publish. get_telemetry
        # already carries voltage and current, so there is no separate power query
        try:
            tel = self._bot.get_telemetry()
            joints = self._bot.get_joints()
        except Exception as exc:
            self.get_logger().warn(f'telemetry failed: {exc}', throttle_duration_sec=1.0)
            return

        self._state_pub.publish(String(data=tel.state.name))
        self._publish_joints(joints)
        self._publish_odometry(tel)
        self._voltage_pub.publish(Float32(data=float(tel.voltage)))
        self._current_pub.publish(Float32(data=float(tel.current)))

    def _publish_joints(self, joints):
        # Joints are servo-space degrees, leg-major (coxa, femur, tibia), converted to radians for JointState
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [math.radians(a) for a in joints]
        self._joints_pub.publish(msg)

    def _publish_odometry(self, tel):
        # board odom is forward = +x while the URDF world faces +y, so rotate yaw by -pi/2 (odom_yaw arrives in degrees)
        x = tel.odom_x / 1000.0          # mm to m
        y = -(tel.odom_y / 1000.0)
        z = 0.0
        yaw_urdf = math.radians(tel.odom_yaw) - math.pi / 2.0
        q = quaternion_from_euler(0.0, 0.0, yaw_urdf)
        stamp = self.get_clock().now().to_msg()

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

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self._odom_pub.publish(odom_msg)

    # Command callbacks (store the desired setpoint, the sender timer forwards it)

    def _enable_cb(self, msg: Bool):
        # True stands the robot up (enable), False sits it down (shutdown). The
        # board ignores the request from a state where it does not apply
        if msg.data:
            self._bot.enable()
        else:
            self._bot.shutdown()

    def _cmd_vel_cb(self, msg: Twist):
        # geometry_msgs/Twist is SI (m/s, rad/s), the board API is mm/s and deg/s
        self._lin = [msg.linear.x * 1000.0, msg.linear.y * 1000.0]
        self._wz = math.degrees(msg.angular.z)
        self._vel_dirty = True

    def _cmd_vel_norm_cb(self, msg: Twist):
        self._lin = [round(msg.linear.x, 2) * self.lin_vel_max,
                     round(msg.linear.y, 2) * self.lin_vel_max]
        self._wz = round(msg.angular.z, 2) * self.ang_vel_max
        self._vel_dirty = True

    def _cmd_pose_cb(self, msg: Pose):
        # Convert URDF to board frame (ctrl_x = urdf_y, ctrl_y = -urdf_x)
        x = msg.position.y * 1000.0
        y = -msg.position.x * 1000.0
        z = msg.position.z * 1000.0
        roll, pitch, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._pose = [x, y, z, math.degrees(roll), math.degrees(pitch),
                      math.degrees(yaw) + 90.0]
        self._pose_dirty = True

    def _cmd_height_cb(self, msg: Float32):
        self._pose[2] = msg.data
        self._pose_dirty = True

    def _cmd_pitch_cb(self, msg: Float32):
        self._pose[4] = msg.data
        self._pose_dirty = True

    def _cmd_gait_cb(self, msg: String):
        gait = GAITS.get(msg.data.lower())
        if gait is None:
            self.get_logger().warn(f'unknown gait: {msg.data}')
            return
        self._bot.set_gait(gait)

    def shutdown(self):
        try:
            self._bot.set_velocity(0.0, 0.0, 0.0)
            self._bot.shutdown()
            self._bot.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HexapodControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
