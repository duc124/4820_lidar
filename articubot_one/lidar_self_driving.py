#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import serial
from collections import deque
from time import time


class SimpleKalman:
    def __init__(self, q=0.1, r=0.5):
        self.q = q
        self.r = r
        self.x = 0
        self.p = 1

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x


class LidarSelfDriving(Node):
    def __init__(self):
        super().__init__('lidar_self_driving')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Self-driving with LiDAR")

        self.serial_port = '/dev/ttyUSB1'
        self.serial_conn = self.try_connect_serial()

        self.robot_diameter = 0.5
        self.safe_distance = 0.6
        self.robot_safe_distance = self.robot_diameter + 0.2
        self.forward_safe_distance = np.sqrt(self.safe_distance**2 + (self.robot_safe_distance / 2)**2)

        self.cmd_interval = 0.2
        self.last_cmd_time = time()

        self.scan_history = deque(maxlen=3)
        self.flicker_threshold = 2
        self.flicker_counters = {k: 0 for k in [
            'left', 'slight_left', 'front_left', 'front_right', 'slight_right', 'right']}
        self.use_kalman = True
        self.kalmans = {k: SimpleKalman() for k in self.flicker_counters}

    def try_connect_serial(self):
        try:
            conn = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info(f"Serial connected on {self.serial_port}")
            return conn
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return None

    def scan_callback(self, msg):
        now = time()
        if now - self.last_cmd_time < self.cmd_interval:
            return

        if not self.serial_conn or not self.serial_conn.is_open:
            self.serial_conn = self.try_connect_serial()
            if not self.serial_conn or not self.serial_conn.is_open:
                return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        ranges[ranges < 0.15] = 0.0

        sectors = {
            'left': (-2 * np.pi / 3, -np.pi / 2),
            'slight_left': (-5 * np.pi / 6, -2 * np.pi / 3),
            'front_left': (-np.pi, -5 * np.pi / 6),
            'front_right': (5 * np.pi / 6, np.pi),
            'slight_right': (2 * np.pi / 3, 5 * np.pi / 6),
            'right': (np.pi / 2, 2 * np.pi / 3),
        }

        sector_flags = {}
        debug_lines = []

        for name, (start, end) in sectors.items():
            mask = (angles >= start) & (angles <= end)
            sector_ranges = ranges[mask]
            sector_angles = angles[mask]

            valid_mask = sector_ranges > 0.0
            sector_ranges = sector_ranges[valid_mask]
            sector_angles = sector_angles[valid_mask]

            obstacle = True
            min_dist = 0.0
            angle_deg = int(np.degrees((start + end) / 2))

            if len(sector_ranges) > 0:
                min_dist = float(np.min(sector_ranges))
                if self.use_kalman:
                    min_dist = self.kalmans[name].update(min_dist)
                obstacle = False
                for r, a in zip(sector_ranges, sector_angles):
                    try:
                        if name in ['slight_left', 'slight_right', 'left', 'right']:
                            threshold = abs(self.robot_safe_distance / np.cos(a - np.pi / 2))
                        else:
                            threshold = self.forward_safe_distance
                        if r < threshold:
                            obstacle = True
                            break
                    except ZeroDivisionError:
                        continue
                angle_deg = int(np.degrees(np.mean(sector_angles)))

            if obstacle:
                self.flicker_counters[name] = min(self.flicker_threshold, self.flicker_counters[name] + 1)
            else:
                self.flicker_counters[name] = max(0, self.flicker_counters[name] - 1)

            flag = self.flicker_counters[name] >= self.flicker_threshold
            sector_flags[name] = flag
            status = "!" if flag else "OK"
            debug_lines.append(f"{name:14s} | angle: {angle_deg:>4}\u00b0 | min_dist: {min_dist * 100:>5.0f} cm | {status}")

        # Command decision
        if not sector_flags['front_left'] and not sector_flags['front_right']:
            if (sector_flags['left'] or sector_flags['slight_left']) and (sector_flags['right'] or sector_flags['slight_right']):
                cmd = 'S'
            elif (sector_flags['left'] or sector_flags['slight_left']):
                cmd = 'SR'
            elif (sector_flags['right'] or sector_flags['slight_right']):
                cmd = 'SL'
            else:
                cmd = 'F'
        else:
            if not sector_flags['slight_left']:
                cmd = 'SL'
            elif not sector_flags['slight_right']:
                cmd = 'SR'
            elif not sector_flags['left']:
                cmd = 'HL'
            elif not sector_flags['right']:
                cmd = 'HR'
            else:
                cmd = 'S'

        self.get_logger().info("\n" + "\n".join(debug_lines) + f"\n? selected command: {cmd}")
        self.send_command(cmd)
        self.last_cmd_time = now

    def send_command(self, cmd):
        try:
            self.serial_conn.write((cmd + '\n').encode())
            self.get_logger().info(f"Sent to ESP32: {cmd}")
        except serial.SerialException as e:
            self.get_logger().error(f"Send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfDriving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()