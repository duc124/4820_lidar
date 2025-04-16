#!/usr/bin/env python3
# This script reads LiDAR data from a ROS 2 topic and sends it over serial (USB) to an ESP32.
# It filters the data to only include the front 180 degrees and formats it for transmission.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import serial

class LidarFront180(Node):

    def __init__(self):
        super().__init__('lidar_front_180')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Connect to serial port
        try:
            self.serial_conn = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info("Serial connected on /dev/ttyUSB1")
        except serial.SerialException as e:
            self.serial_conn = None
            self.get_logger().error(f"Serial connection failed: {e}")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter to front ±90° → -π/2 to +π/2 radians
        front_mask = (angles >= -np.pi/2) & (angles <= np.pi/2)
        ranges = ranges[front_mask]
        angles = angles[front_mask]

        # Filter out invalid (inf/NaN)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return  # No usable data

        # Build message: angle(deg),distance(cm);...
        data_points = []
        for angle, distance in zip(angles, ranges):
            angle_deg = int(np.degrees(angle))        # -90 to +90
            distance_cm = int(distance * 100)
            data_points.append(f"{angle_deg},{distance_cm}")

        message = ";".join(data_points) + "\n"
        print(f"Sending {len(data_points)} points")
        print("Serial message:", message[:100], "...")  # print just the first 100 chars

        # Send over serial
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(message.encode())
                self.serial_conn.flush()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFront180()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()