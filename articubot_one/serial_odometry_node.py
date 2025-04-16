#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class SerialOdometryNode(Node):
    def __init__(self):
        super().__init__('serial_odometry_node')

        # === Serial port setup ===
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        time.sleep(2)  # Give time for ESP32 to reset

        # === ROS publisher setup ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # === Robot parameters ===
        self.wheel_base = 0.15  # meters between left/right wheels
        self.last_left = 0.0
        self.last_right = 0.0
        self.last_time = self.get_clock().now()

        # === Timer to read and publish odometry ===
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line.startswith("ODOM:"):
                return

            # Parse line: ODOM:12.3,12.5
            parts = line[5:].split(',')
            if len(parts) != 2:
                return

            left = float(parts[0])
            right = float(parts[1])

            # Calculate average distance moved
            distance_l = left - self.last_left
            distance_r = right - self.last_right
            distance = (distance_l + distance_r) / 2.0

            # Calculate velocity
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt == 0:
                return

            linear_velocity = distance / dt
            angular_velocity = (distance_r - distance_l) / (self.wheel_base * dt)

            # Update odometry state (we'll track only x for simplicity)
            self.publish_odometry(linear_velocity, angular_velocity, distance, dt)

            # Update memory
            self.last_left = left
            self.last_right = right
            self.last_time = now

        except Exception as e:
            self.get_logger().warn(f"Failed to read from serial: {e}")

    def publish_odometry(self, linear, angular, distance, dt):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # For now, just simulate forward movement along x
        msg.pose.pose.position.x += distance
        msg.pose.pose.orientation = Quaternion()  # No rotation for now

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular

        self.odom_pub.publish(msg)

        # TF transform (odom → base_link)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
