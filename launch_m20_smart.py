#!/usr/bin/env python3
"""Launch M20 Smart blueprint with Mac Bridge.

Autonomous navigation + live mapping via TCP bridge to GOS.

Usage:
    python launch_m20_smart.py
"""

import signal
import sys
import time

# viewer_backend must be set BEFORE importing blueprints — the m20_minimal
# match statement evaluates at import time.
from dimos.core.global_config import global_config

global_config.update(viewer_backend="rerun")

from dimos.core.blueprints import autoconnect
from dimos.robot.deeprobotics.m20.blueprints.smart.m20_smart import m20_smart
from dimos.robot.deeprobotics.m20.connection import m20_connection

# Override m20_connection to use Mac Bridge
bp = autoconnect(
    m20_smart,
    m20_connection(bridge_host="10.21.41.1", ip="10.21.41.1"),
).global_config(robot_ip="10.21.41.1")


def main():
    print("Building M20 Smart blueprint (Mac Bridge)...")
    coordinator = bp.build()
    print("Blueprint running!")
    print("  - Rerun Viewer: open Rerun app")
    print("  - Web UI:       http://localhost:7779")
    print("  - Press Ctrl+C to stop")

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
