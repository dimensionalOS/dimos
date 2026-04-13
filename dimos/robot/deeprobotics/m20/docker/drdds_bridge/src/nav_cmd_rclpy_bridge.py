#!/usr/bin/env python3
"""TCP→ROS2 /NAV_CMD bridge for AOS.

Listens on a TCP port for velocity commands from NOS and publishes
them to /NAV_CMD via rclpy. This is the proven path that worked in
the mac_bridge — rclpy publishers match with basic_server's subscriber.

Protocol: 12-byte binary messages (3 x float32: x_vel, y_vel, yaw_vel)

Usage on AOS:
    source /opt/ros/foxy/setup.bash
    export PYTHONPATH=/opt/drdds/lib/python3.8/site-packages:$PYTHONPATH
    python3 nav_cmd_rclpy_bridge.py [port]  (default: 9740)
"""

import socket
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from drdds.msg import NavCmd

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9740
WATCHDOG_TIMEOUT = 0.5  # zero velocity after 500ms no command


class NavCmdBridge(Node):
    def __init__(self):
        super().__init__("nav_cmd_bridge")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pub = self.create_publisher(NavCmd, "/NAV_CMD", qos)
        self._last_cmd_time = time.monotonic()
        self._lock = threading.Lock()
        self._latest = (0.0, 0.0, 0.0)
        self.get_logger().info(f"NavCmd publisher created on /NAV_CMD")

    def publish_vel(self, x, y, yaw):
        msg = NavCmd()
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        msg.header.frame_id = 0
        msg.data.x_vel = float(x)
        msg.data.y_vel = float(y)
        msg.data.yaw_vel = float(yaw)
        self._pub.publish(msg)
        with self._lock:
            self._latest = (x, y, yaw)
            self._last_cmd_time = time.monotonic()

    def watchdog_check(self):
        with self._lock:
            if time.monotonic() - self._last_cmd_time > WATCHDOG_TIMEOUT:
                if self._latest != (0.0, 0.0, 0.0):
                    self._latest = (0.0, 0.0, 0.0)
                    msg = NavCmd()
                    now = self.get_clock().now().to_msg()
                    msg.header.stamp.sec = now.sec
                    msg.header.stamp.nanosec = now.nanosec
                    msg.data.x_vel = 0.0
                    msg.data.y_vel = 0.0
                    msg.data.yaw_vel = 0.0
                    self._pub.publish(msg)


def tcp_server(node):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(1)
    server.settimeout(1.0)
    node.get_logger().info(f"Listening on port {PORT}")

    msg_count = 0
    while rclpy.ok():
        # Watchdog
        node.watchdog_check()

        try:
            client, addr = server.accept()
        except socket.timeout:
            continue

        node.get_logger().info(f"Client connected from {addr[0]}")
        client.settimeout(1.0)

        while rclpy.ok():
            node.watchdog_check()
            try:
                packet = _recv_packet(client)
            except socket.timeout:
                continue
            except ConnectionError:
                break
            if packet is None:
                break

            x, y, yaw = packet
            node.publish_vel(x, y, yaw)
            msg_count += 1

            if msg_count % 100 == 1:
                node.get_logger().info(f"#{msg_count} x={x:.3f} y={y:.3f} yaw={yaw:.3f}")

        client.close()
        node.get_logger().info(f"Client disconnected ({msg_count} msgs total)")
        # Zero on disconnect
        node.publish_vel(0.0, 0.0, 0.0)

    server.close()


def _recv_packet(client):
    data = b""
    while len(data) < 12:
        chunk = client.recv(12 - len(data))
        if not chunk:
            if len(data) == 0:
                return None
            raise ConnectionError("partial packet")
        data += chunk
    return struct.unpack("fff", data)


def main():
    rclpy.init()
    node = NavCmdBridge()

    # Spin in background for DDS discovery
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        tcp_server(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
