# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Replay a Mid-360 recording through alfred-nav's voxel map and MLS planner, on the GPU or the CPU.

    DIMOS_LOG_LEVEL=DEBUG python dimos/mapping/gpu_mls/demo_nav_replay.py <mem2.db> [--cpu | --cpu-surfaces] [--seek S] [--duration S]

The raw lidar stream is placed by an odometry stream turned into tf, as Point-LIO does live.
"""

from __future__ import annotations

import argparse
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.memory.replay_module import replay_module
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative

# alfred-nav's settings (robot/diy/alfred/blueprints/_nav.py, alfred_nav.py, embodiment/alfred.py).
VOXEL_SIZE_METERS = 0.08
MAX_RANGE_METERS = 15.0
ROBOT_HEIGHT_METERS = 1.86
BOUNDARY_CLEARANCE_METERS = 0.20
STEP_THRESHOLD_METERS = 0.08


LIDAR_STREAM = "livox_lidar"
ODOMETRY_STREAM = "pointlio_odometry_corrected"
# Deploying sends the class by its import path, so it lives at module level;
# a worker importing this file gets a port-less one that adds its ports from
# the dataset config.
Replay = replay_module(
    sys.argv[1] if __name__ == "__main__" and len(sys.argv) > 1 else "",
    topics=f"{LIDAR_STREAM},{ODOMETRY_STREAM}",
)


class OdometryTf(Module):
    """Publish each odometry pose as its frame -> child_frame tf edge."""

    odometry: In[Odometry]
    tf: Out[TFMessage]

    async def handle_odometry(self, msg: Odometry) -> None:
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=msg.position,
                    rotation=msg.orientation,
                    frame_id=msg.frame_id,
                    child_frame_id=msg.child_frame_id,
                    ts=msg.ts,
                )
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("db")
    parser.add_argument("--cpu", action="store_true", help="ray tracer and surfaces on the CPU")
    parser.add_argument(
        "--cpu-surfaces", action="store_true", help="GPU ray tracer, surfaces on the CPU"
    )
    parser.add_argument("--sensor-frame", default="mid360_link")
    parser.add_argument("--seek", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    gpu = not args.cpu
    blueprint = autoconnect(
        Replay.blueprint(
            dataset=args.db,
            topics=f"{LIDAR_STREAM},{ODOMETRY_STREAM}",
            seek=args.seek,
            duration=args.duration,
        ),
        OdometryTf.blueprint().remappings([(OdometryTf, "odometry", ODOMETRY_STREAM)]),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=MAX_RANGE_METERS,
            global_emit_every=50,
            gpu=gpu,
        ).remappings([(RayTracingVoxelMap, "lidar", LIDAR_STREAM)]),
        MLSPlannerNative.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            robot_height=ROBOT_HEIGHT_METERS,
            boundary_clearance_m=BOUNDARY_CLEARANCE_METERS,
            step_threshold_m=STEP_THRESHOLD_METERS,
            step_penalty_weight=1.0,
            base_frame=args.sensor_frame,
            gpu_surfaces=gpu and not args.cpu_surfaces,
            gpu_boundary_distance=gpu and not args.cpu_surfaces,
        ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    )
    ModuleCoordinator.build(blueprint).loop()


if __name__ == "__main__":
    main()
