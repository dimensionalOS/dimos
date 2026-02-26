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

import dask

from dimos.core.global_config import global_config

global_config.update(viewer_backend="rerun-web")

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.deeprobotics.m20.blueprints.basic.m20_minimal import m20_minimal
from dimos.robot.deeprobotics.m20.connection import m20_connection

# Direct ROS2 mode: no bridge_host, robot_ip is AOS eth0 (shared L2 with NOS)
AOS_ETH0 = "10.21.33.103"

# NOS-tuned blueprint: VoxelGridMapper on CPU without CUDA is expensive.
# - voxel_size=0.2: 8x fewer voxels than 0.1 (cubic), fine for outdoor patrol
# - publish_interval=1.0: Lesh recommended 1Hz instead of every-frame (~10Hz)
# - resolution=0.2: match costmap grid to voxel size (16x smaller grid than default 0.05)
# - n_dask_workers=2: 2 workers for parallelism (less hitchy Rerun visualization).
#   terminate=False in Dask config prevents worker kills that destroy Rerun server.
# - memory_limit=7GB: per-worker limit. 2x7=14GB on 15GB system — tight but
#   sshd is OOM-protected and 4GB swap absorbs pressure spikes.
bp = autoconnect(
    m20_minimal,
    voxel_mapper(voxel_size=0.2, publish_interval=1.0),
    cost_mapper(config=HeightCostConfig(max_height=1.5, resolution=0.2)),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
    m20_connection(ip=AOS_ETH0, enable_ros=True),
).global_config(
    robot_ip=AOS_ETH0,
    robot_model="deeprobotics_m20",
    robot_width=0.3,
    robot_rotation_diameter=0.6,
    n_dask_workers=2,
    memory_limit="7GB",
)


def main():
    # Dask memory management: spill/pause but NEVER terminate workers.
    # Terminating kills the Rerun gRPC server (runs in-worker, doesn't respawn).
    # With sshd OOM-protected and 4GB swap, it's safer to swap than lose Rerun.
    dask.config.set({
        "distributed.worker.memory.target": 0.7,
        "distributed.worker.memory.spill": 0.8,
        "distributed.worker.memory.pause": 0.9,
        "distributed.worker.memory.terminate": False,
    })

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
