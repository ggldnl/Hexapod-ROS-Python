#!/usr/bin/env python3
"""
Grab a single frame off a sensor_msgs/Image topic and write it to disk.

Deliberately has no cv_bridge, numpy or OpenCV dependency, so it runs on a bare
ROS install: colour frames go out as binary PPM and mono as PGM, both of which
eog, GIMP and ffmpeg open directly.

    ros2 run camera_ros camera_node          # in one terminal
    ./grab_frame.py                          # in another
    ./grab_frame.py --topic /camera/image_raw --out shot.ppm
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class FrameGrabber(Node):

    def __init__(self, topic, out_path, timeout):
        super().__init__('frame_grabber')
        self.out_path = out_path
        self.saved = False

        # Camera drivers commonly publish best effort, a reliable subscription
        # would simply never match and the node would look like it hung
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(f'Waiting for a frame on {topic} (timeout {timeout}s)...')

    def _on_image(self, msg):
        if self.saved:
            return

        enc = msg.encoding.lower()
        if enc in ('rgb8', 'bgr8'):
            magic, channels = b'P6', 3
        elif enc in ('mono8', '8uc1'):
            magic, channels = b'P5', 1
        else:
            self.get_logger().error(
                f'unsupported encoding {msg.encoding!r}, '
                'relaunch the camera node with -p format:=RGB888')
            self.saved = True
            return

        expected = msg.width * channels
        rows = []
        for r in range(msg.height):
            # step counts the padding the driver may add, the PPM row must not
            start = r * msg.step
            row = bytes(msg.data[start:start + expected])
            if len(row) != expected:
                self.get_logger().error(f'short row {r}: got {len(row)}, want {expected}')
                self.saved = True
                return
            # PPM is RGB, so bgr8 needs the channel order swapped
            if enc == 'bgr8':
                row = bytes(b for i in range(0, len(row), 3)
                            for b in (row[i + 2], row[i + 1], row[i]))
            rows.append(row)

        with open(self.out_path, 'wb') as f:
            f.write(b'%s\n%d %d\n255\n' % (magic, msg.width, msg.height))
            for row in rows:
                f.write(row)

        self.get_logger().info(
            f'Wrote {self.out_path}: {msg.width}x{msg.height} {msg.encoding}')
        self.saved = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--out', default='frame.ppm')
    parser.add_argument('--timeout', type=float, default=15.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv[:1] + ros_args)
    node = FrameGrabber(args.topic, args.out, args.timeout)

    elapsed = 0.0
    try:
        while rclpy.ok() and not node.saved and elapsed < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed += 0.1
    except KeyboardInterrupt:
        pass

    ok = node.saved
    if not ok:
        node.get_logger().error(
            f'no frame within {args.timeout}s, check: ros2 topic hz {args.topic}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
