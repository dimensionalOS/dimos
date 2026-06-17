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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""End-to-end test for RayTracingVoxelMap using the synthetic clearing scene.

Spins up three modules — a synthetic lidar source, the Rust ray tracer,
and a global-map collector — feeds the floor/wall/box/person sequence
through, then scores the published DynamicCloud frames against two
penalties:

    forget_box   : per missing box voxel per frame (the static obstacle
                   should never disappear from the published map after
                   it's been confirmed)
    ghost_person : per stale voxel sitting in the person plane (x=PERSON_X,
                   z above the floor zone where wall returns sweep through)
                   that doesn't belong to the current person position —
                   i.e. the ray tracer didn't clear the person's previous
                   footprint when the person moved.

Lower is better. Always prints the score; --rerun adds a live visualization
of both the input lidar and the published global map side by side.
"""

from __future__ import annotations

import argparse
import threading
import time
from typing import Any

import numpy as np
import pytest
import rerun as rr

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.mapping.ray_tracing.demo_clearing_scene import (
    CLASS_COLORS,
    PERSON_HALF_WIDTH,
    PERSON_X,
    VOXEL_SIZE,
    Frame,
    _box_visible_face_points,
    _classify_points,
    synthetic_scene,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# A ray from sensor (0,0,1) to a wall point at (WALL_X=6, y_w, z_w) crosses
# the person plane (x=PERSON_X=3) at y = y_w/2 and z = 0.5*z_w + 0.5.
# The wall y range is [-3, 3] and z range is [0, 2.5], so the cells at
# the person column that the wall actually sweeps are bounded:
#
#     y_at_person_plane ∈ [-1.5,  1.5]   (voxel index [-15, 14])
#     z_at_person_plane ∈ [ 0.5, 1.75]   (voxel index [  5, 17])
#
# Cells outside that bounding box can't be cleared by wall returns —
# scene limitation, not a ray-tracer bug — so we don't count them as
# ghosts.
_GHOST_CHECK_MIN_Z_VOXEL = 5
_GHOST_CHECK_MAX_Z_VOXEL = 17
_GHOST_CHECK_MIN_Y_VOXEL = -15
_GHOST_CHECK_MAX_Y_VOXEL = 14

# A voxel needs `1 - min_health + 1` hits to become confirmed and survive
# occlusion. Default config is min_health=-1, max_health=1 → 2 hits, so
# the first two frames are warmup for the box-presence check.
_BOX_WARMUP_FRAMES = 2


def _voxel_key(x: float, y: float, z: float, voxel_size: float) -> tuple[int, int, int]:
    return (
        int(np.floor(x / voxel_size)),
        int(np.floor(y / voxel_size)),
        int(np.floor(z / voxel_size)),
    )


def _expected_box_voxel_keys(voxel_size: float) -> set[tuple[int, int, int]]:
    return {_voxel_key(x, y, z, voxel_size) for x, y, z in _box_visible_face_points()}


def _person_voxel_y_range(person_y: float, voxel_size: float) -> tuple[int, int]:
    return (
        int(np.floor((person_y - PERSON_HALF_WIDTH) / voxel_size)),
        int(np.floor((person_y + PERSON_HALF_WIDTH - 1e-9) / voxel_size)),
    )


class SyntheticLidarSource(Module):
    """Publishes the synthetic-scene PointCloud2 + Odometry pair per frame."""

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, num_frames: int = 30, frame_dt: float = 0.1, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._num_frames = num_frames
        self._frame_dt = frame_dt
        self._stop = threading.Event()
        self._done = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="synthetic-lidar"
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        super().stop()

    @rpc
    def wait_done(self, timeout: float = 60.0) -> bool:
        return self._done.wait(timeout)

    def _publish_loop(self) -> None:
        for frame in synthetic_scene(num_frames=self._num_frames, frame_dt=self._frame_dt):
            if self._stop.is_set():
                return
            cloud = PointCloud2.from_numpy(
                points=frame.points,
                frame_id="world",
                timestamp=frame.timestamp_s,
            )
            ox, oy, oz = (float(v) for v in frame.sensor_origin)
            odom = Odometry(
                ts=frame.timestamp_s,
                frame_id="world",
                child_frame_id="sensor",
                pose=Pose(ox, oy, oz),
            )
            self.lidar.publish(cloud)
            self.odometry.publish(odom)
            if not self._stop.wait(self._frame_dt):
                continue
            return
        self._done.set()


class GlobalMapCollector(Module):
    """Subscribes to global_map and stores every frame for later inspection."""

    global_map: In[DynamicCloud]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._collected: list[DynamicCloud] = []
        self._unsub = None

    @rpc
    def build(self) -> None:
        super().build()
        self._unsub = self.global_map.subscribe(self._on_msg)

    @rpc
    def stop(self) -> None:
        if self._unsub is not None:
            self._unsub()
            self._unsub = None
        super().stop()

    def _on_msg(self, msg: DynamicCloud) -> None:
        with self._lock:
            self._collected.append(msg)

    @rpc
    def get_frame_count(self) -> int:
        with self._lock:
            return len(self._collected)

    @rpc
    def get_all_frames(self) -> list[DynamicCloud]:
        with self._lock:
            return list(self._collected)


def compute_loss(
    collected: list[DynamicCloud],
    expected_frames: list[Frame],
    voxel_size: float,
) -> dict:
    box_keys = _expected_box_voxel_keys(voxel_size)
    person_x_voxel = int(np.floor(PERSON_X / voxel_size))

    # Match by rounded ts (microsecond resolution); the Rust side writes
    # the input PointCloud2 stamp through to the published DynamicCloud,
    # and the Python `DynamicCloud.ts` is that same value re-decoded.
    by_ts: dict[int, DynamicCloud] = {round((out.ts or 0.0) * 1_000_000): out for out in collected}

    forget_box = 0
    ghost_person = 0
    matched = 0

    for frame in expected_frames:
        ts_us = round(frame.timestamp_s * 1_000_000)
        out = by_ts.get(ts_us)
        if out is None:
            continue
        matched += 1
        out_keys = {(int(v[0]), int(v[1]), int(v[2])) for v in out.voxels}

        if frame.index >= _BOX_WARMUP_FRAMES:
            forget_box += len(box_keys - out_keys)

        if frame.person_y is not None:
            y_lo, y_hi = _person_voxel_y_range(frame.person_y, voxel_size)
            for vx, vy, vz in out_keys:
                if vx != person_x_voxel:
                    continue
                if vz < _GHOST_CHECK_MIN_Z_VOXEL or vz > _GHOST_CHECK_MAX_Z_VOXEL:
                    continue
                if vy < _GHOST_CHECK_MIN_Y_VOXEL or vy > _GHOST_CHECK_MAX_Y_VOXEL:
                    continue
                if vy < y_lo or vy > y_hi:
                    ghost_person += 1

    return {
        "score": float(forget_box + ghost_person),
        "forget_box": forget_box,
        "ghost_person": ghost_person,
        "matched_frames": matched,
        "expected_frames": len(expected_frames),
        "received_frames": len(collected),
        "box_voxel_count": len(box_keys),
    }


def run(
    num_frames: int = 30,
    frame_dt: float = 0.1,
    use_rerun: bool = False,
    voxel_size: float = VOXEL_SIZE,
    settle_secs: float = 1.5,
) -> dict:
    """Spin up the modules, feed the scene, score the output."""
    coord = ModuleCoordinator()
    coord.start()
    collected: list[DynamicCloud] = []
    try:
        source = coord.deploy(
            SyntheticLidarSource,
            num_frames=num_frames,
            frame_dt=frame_dt,
        )
        ray_tracer = coord.deploy(
            RayTracingVoxelMap,
            voxel_size=voxel_size,
            auto_build=True,  # always rebuild — picks up source changes between runs
        )
        collector = coord.deploy(GlobalMapCollector)

        # Wire ports to LCM topics explicitly — `.connect()` doesn't always
        # propagate transports through to In ports in deployed worker
        # modules, and the NativeModule binary needs the topic names anyway.
        source.lidar.transport = LCMTransport("/test_lidar", PointCloud2)
        source.odometry.transport = LCMTransport("/test_odometry", Odometry)
        ray_tracer.lidar.transport = LCMTransport("/test_lidar", PointCloud2)
        ray_tracer.odometry.transport = LCMTransport("/test_odometry", Odometry)
        ray_tracer.global_map.transport = LCMTransport("/test_global_map", DynamicCloud)
        collector.global_map.transport = LCMTransport("/test_global_map", DynamicCloud)

        ray_tracer.build()
        collector.build()

        ray_tracer.start()
        collector.start()
        # Give the Rust binary a moment to bind LCM subscriptions, otherwise
        # the first lidar frames are sent into the void.
        time.sleep(0.5)
        source.start()

        source.wait_done(timeout=num_frames * frame_dt + 30.0)
        # Let the ray tracer finish processing trailing frames.
        time.sleep(settle_secs)
        collected = collector.get_all_frames()

        source.stop()
        collector.stop()
        ray_tracer.stop()
    finally:
        coord.stop()

    expected = list(synthetic_scene(num_frames=num_frames, frame_dt=frame_dt))
    loss = compute_loss(collected, expected, voxel_size)

    print()
    print(f"score                  : {loss['score']:.0f}   (lower is better)")
    print(
        f"  forget_box           : {loss['forget_box']}  / target box voxels = {loss['box_voxel_count']}"
    )
    print(f"  ghost_person         : {loss['ghost_person']}")
    print(f"  matched frames       : {loss['matched_frames']} / {loss['expected_frames']} expected")
    print(f"  received frames      : {loss['received_frames']}")
    print()

    if use_rerun:
        _visualize(collected, expected, voxel_size)

    return loss


def _visualize(
    collected: list[DynamicCloud],
    expected_frames: list[Frame],
    voxel_size: float,
) -> None:
    """Stream input + output side by side, color-coded so the ray tracer's
    state is visually distinct from the sensor returns.

    Color scheme (intentionally non-overlapping):
        input/by_class : floor=gray, wall=blue, person=red, box=orange
                         — what the synthetic lidar emits this frame.
        output/map     : bright magenta — every voxel the ray tracer
                         currently holds. Sits on top of the input,
                         slightly smaller radius so the input class
                         colors stay visible beneath.
        output/box     : bright green — published voxels that fall inside
                         the box AABB. If the green stays solid through
                         the whole walk, the ray tracer is preserving
                         the static obstacle correctly.
    """
    rr.init("ray_tracing_clearing_test", spawn=True)
    time.sleep(1.0)

    box_min = np.array([4.0, 0.3, 0.0], dtype=np.float32)  # mirrors BOX_X/Y/Z
    box_max = np.array([4.5, 1.1, 0.5], dtype=np.float32)

    by_ts = {round((out.ts or 0.0) * 1_000_000): out for out in collected}
    for frame in expected_frames:
        rr.set_time("time", duration=frame.timestamp_s)

        # ---- input, colored by surface class
        classes = _classify_points(frame.points, frame.person_y)
        rr.log(
            "input/by_class",
            rr.Points3D(
                positions=frame.points,
                colors=CLASS_COLORS[classes],
                radii=voxel_size / 2,
            ),
        )

        # ---- output, two layers in distinct solid colors
        ts_us = round(frame.timestamp_s * 1_000_000)
        out = by_ts.get(ts_us)
        if out is None or len(out) == 0:
            # Clear stale points so the entity disappears on frames where
            # we didn't receive output. (Logging an empty Points3D works.)
            rr.log("output/map", rr.Points3D([]))
            rr.log("output/box", rr.Points3D([]))
            time.sleep(0.05)
            continue

        world = out.world_positions()
        in_box = np.all((world >= box_min) & (world <= box_max), axis=1)

        # Everything the tracer publishes — bright magenta. Includes box
        # voxels too, so the pink consistently represents "is this in the
        # published map" regardless of class.
        rr.log(
            "output/map",
            rr.Points3D(
                positions=world,
                colors=np.array([[255, 0, 200]], dtype=np.uint8),
                radii=voxel_size / 2 * 0.55,
            ),
        )
        # Box subset — small bright-green dot sitting inside the pink
        # output voxel, so you can see at a glance whether the static
        # obstacle is being preserved across occlusion. Drawn smaller
        # than the pink so the pink shows around it.
        rr.log(
            "output/box",
            rr.Points3D(
                positions=world[in_box],
                colors=np.array([[60, 255, 80]], dtype=np.uint8),
                radii=voxel_size / 2 * 0.25,
            ),
        )
        time.sleep(0.05)


@pytest.mark.slow
def test_ray_tracing_clearing():
    loss = run(num_frames=20, frame_dt=0.1, use_rerun=False)
    # Observed on a clean run: forget_box ≈ 15, ghost_person ≈ 78 over
    # 19 matched frames. Thresholds are 3-4× the observed values — meant
    # to flag outright regressions (ray tracer eats the box, never clears,
    # etc.) without being flaky on timing jitter.
    assert loss["matched_frames"] >= 15, f"too few matched frames: {loss}"
    assert loss["forget_box"] < 80, f"too many missing box voxels: {loss}"
    assert loss["ghost_person"] < 300, f"too many ghost person voxels: {loss}"


def main():
    parser = argparse.ArgumentParser(description="End-to-end test for RayTracingVoxelMap")
    parser.add_argument("--rerun", action="store_true", help="visualize input + output in Rerun")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--voxel-size", type=float, default=VOXEL_SIZE)
    args = parser.parse_args()
    run(
        num_frames=args.frames,
        frame_dt=args.dt,
        use_rerun=args.rerun,
        voxel_size=args.voxel_size,
    )


if __name__ == "__main__":
    main()
