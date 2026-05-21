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

"""
Spins up a blueprint with PGO, the KITTI-360 playback module, and a
TopicCounter module that subscribes to every PGO output. Reports per-topic
message counts and a one-line verdict.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.playback import (
    Kitti360PlaybackModule,
)
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO


class TopicCounterModule(Module):
    """Subscribes to every PGO output stream and counts arrivals per topic."""

    corrected_odometry: In[Odometry]
    global_map: In[PointCloud2]
    corrected_tf: In[Odometry]
    pose_graph: In[Graph3D]
    loop_closure_event: In[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._counts: dict[str, int] = {
            "corrected_odometry": 0,
            "global_map": 0,
            "corrected_tf": 0,
            "pose_graph": 0,
            "loop_closure_event": 0,
        }

    @rpc
    def start(self) -> None:
        super().start()
        for stream_name in self._counts:
            stream = getattr(self, stream_name)
            self.register_disposable(Disposable(stream.subscribe(self._make_counter(stream_name))))

    def _make_counter(self, name: str) -> Any:
        def _on_message(_message: Any) -> None:
            self._counts[name] += 1

        return _on_message

    @rpc
    def counts(self) -> dict[str, int]:
        return dict(self._counts)


def main() -> None:
    parser = argparse.ArgumentParser(description="PGO liveness probe via DimOS modules")
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=2)
    parser.add_argument("--num-scans", type=int, default=200)
    parser.add_argument(
        "--loop-search-radius-m",
        type=float,
        default=4.0,
        help="m; default 4.0 matches groundtruth radius",
    )
    parser.add_argument("--publish-interval-sec", type=float, default=0.02)
    parser.add_argument("--drain-sec", type=float, default=5.0)
    parser.add_argument("--poll-interval-sec", type=float, default=0.5)
    args = parser.parse_args()

    playback_blueprint = Kitti360PlaybackModule.blueprint(
        kitti360_root=str(args.kitti360_root),
        sequence_id=args.sequence,
        max_scans=args.num_scans,
        publish_interval_sec=args.publish_interval_sec,
    )
    pgo_blueprint = PGO.blueprint(
        loop_search_radius=args.loop_search_radius_m,
    )
    counter_blueprint = TopicCounterModule.blueprint()

    blueprint = autoconnect(playback_blueprint, pgo_blueprint, counter_blueprint)
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        playback = coordinator.get_instance(Kitti360PlaybackModule)
        counter = coordinator.get_instance(TopicCounterModule)
        while not playback.is_finished():
            time.sleep(args.poll_interval_sec)
        playback_error = playback.playback_error()
        if playback_error is not None:
            raise RuntimeError(f"Kitti360 playback aborted: {playback_error}")
        time.sleep(args.drain_sec)
        counts = counter.counts()
    finally:
        coordinator.stop()

    print("\n=== PGO topic message counts ===")
    for name in (
        "corrected_odometry",
        "global_map",
        "corrected_tf",
        "pose_graph",
        "loop_closure_event",
    ):
        print(f"  {name:<24} {counts.get(name, 0):>6}")

    print("\nverdict:")
    if counts.get("pose_graph", 0) == 0:
        print("  ⚠ no pose graph — PGO never promoted a keyframe. Check --key_pose_delta_*.")
    elif counts.get("loop_closure_event", 0) == 0:
        print(
            "  ⚠ graph builds, no loop closure events — try wider --loop-search-radius "
            "or lower --scan-context-match-threshold."
        )
    else:
        print("  ✓ all topics firing — PGO is alive end-to-end.")


if __name__ == "__main__":
    main()
