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

# Disable default rerun bridge from m20_minimal — we configure our own below
# with memory_limit and visual_override for the NOS environment.
global_config.update(viewer_backend="none")

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.deeprobotics.m20.blueprints.basic.m20_minimal import m20_minimal
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.visualization.rerun.bridge import rerun_bridge

# Direct ROS2 mode: no bridge_host, robot_ip is AOS eth0 (shared L2 with NOS)
AOS_ETH0 = "10.21.33.103"

# NOS-tuned config notes:
# - voxel_size=0.1: Fine enough for indoor navigation (doors are ~0.8m wide).
#   Lesh: "0.2 is nuts, it will think doors are not passable."
# - publish_interval=0.5: Publish voxel grid at 2Hz (not every lidar frame ~10Hz).
#   Keeps compute manageable on RK3588 CPU. Lesh: "do 1hz or 0.5hz even."
# - algo="height_cost": Gradient-based terrain slope analysis. Produces continuous
#   costs (0-100) based on terrain steepness — much better for tight spaces than
#   binary free/occupied. Needs ignore_noise tuned to voxel resolution.
# - ignore_noise=0.1: Match to voxel_size — 1-voxel height jumps are quantization
#   noise, not real obstacles. Default 0.05 was too sensitive at 0.2 voxels.
# - can_climb=0.2: 20cm height change per cell = cost 100 (lethal). M20 can step
#   over ~15cm, so 20cm is a reasonable lethal threshold.
# - smoothing=1.5: More gap-filling between sparse ground observations.
# - memory_limit="512MB": Caps Rerun recording memory. Won't retain full history
#   but draws current state. Prevents Rerun from eating all Dask worker memory.
#   Lesh: "limit it to something small and it won't remember the history for you
#   but it will draw the current state."
# - global_map visual_override: Render voxels as 3D boxes (not flat sprites).
#   Lesh: 'has this "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1,
#   mode="boxes") line in rerun bridge.'
bp = autoconnect(
    m20_minimal,
    voxel_mapper(voxel_size=0.05, publish_interval=1.0, max_height=0.7),
    cost_mapper(config=HeightCostConfig(
        max_height=0.7, resolution=0.05, ignore_noise=0.05, can_climb=0.25,
        smoothing=5.0,
    )),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
    m20_connection(ip=AOS_ETH0, enable_ros=True),
    rerun_bridge(
        viewer_mode="web",
        memory_limit="512MB",
        pubsubs=[LCM(autoconf=True)],
        visual_override={
            "world/global_map": lambda grid: grid.to_rerun(
                voxel_size=0.05, mode="boxes",
            ),
            "world/navigation_costmap": lambda grid: grid.to_rerun(
                colormap="Accent",
                z_offset=0.015,
                opacity=0.2,
                background="#484981",
            ),
        },
    ),
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
