#!/usr/bin/env python3
"""Launch dimos M20 on NOS -- direct ROS2 mode (no bridge).

Runs inside the Humble Docker container on NOS. Uses M20ROSSensors
for direct rclpy access to /ODOM, /ALIGNED_POINTS, /IMU, /NAV_CMD.

Access from Mac:
  Web UI:  http://10.21.41.1:7779/command-center  (via AOS WiFi NAT)
  Rerun:   rerun --connect rerun+http://10.21.41.1:9876/proxy
"""

import signal
import sys
import time

from dimos.core.global_config import global_config

global_config.update(viewer_backend="rerun-web")

from dimos.core.blueprints import autoconnect
from dimos.robot.deeprobotics.m20.blueprints.smart.m20_smart import m20_smart
from dimos.robot.deeprobotics.m20.connection import m20_connection

# Direct ROS2 mode: no bridge_host, robot_ip is AOS eth0 (shared L2 with NOS)
AOS_ETH0 = "10.21.33.103"

bp = autoconnect(
    m20_smart,
    m20_connection(ip=AOS_ETH0, enable_ros=True),
).global_config(
    robot_ip=AOS_ETH0,
    n_dask_workers=2,  # NOS has 4 cores total; keep headroom for RT processes
)


def main():
    print("Starting dimos M20 on NOS (direct ROS2 mode)...")
    coordinator = bp.build()
    print("dimos running!")
    print("  Web UI:  http://0.0.0.0:7779/command-center")
    print("  Rerun:   rerun+http://0.0.0.0:9876/proxy")
    print("  Press Ctrl+C to stop")

    def shutdown(signum, frame):
        print("\nShutting down...")
        coordinator.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
