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

"""Integration smoke test: the native multicam cuVSLAM module on a short Spot snippet.

A three-module blueprint: a publisher module replays the LFS snippet (the three
landscape cameras with their registered depth), CuvslamOdometry runs the dimSLAM
binary in multicam mode, and a sink collects the odometry it publishes. The test
asserts poses flow and agree with the robot's own recorded odometry. cuVSLAM's
Multisensor depth path has no CPU fallback (use_gpu=False aborts in CUDA), so runs
are not bit-reproducible and the assertions are bounds, not exact poses.
"""

import time
from typing import Any

import numpy as np
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted

# The front pair is mounted portrait and cuVSLAM requires one resolution across the
# rig, so the native module runs the three landscape cameras.
CAMERAS = ["left", "right", "back"]
OPTICAL_FRAMES = [f"{camera}_camera_optical" for camera in CAMERAS]
MIN_POSES = 12
MAX_FRAME_JUMP_M = 1.0
POSE_DEADLINE_S = 60.0


class _NearestWalker:
    """Walk a timestamp-ordered stream, returning the observation nearest each query."""

    def __init__(self, stream: Any) -> None:
        self._iterator = iter(stream)
        self._head = next(self._iterator, None)
        self._lookahead = next(self._iterator, None)

    def at(self, stamp: float) -> Any:
        while (
            self._head is not None
            and self._lookahead is not None
            and abs(self._lookahead.ts - stamp) < abs(self._head.ts - stamp)
        ):
            self._head = self._lookahead
            self._lookahead = next(self._iterator, None)
        return self._head


def open_snippet() -> SqliteStore:
    store = SqliteStore(path=str(get_data("spot_multicam_short.db")), must_exist=True)
    store.start()
    return store


class SpotSnippetPublisher(Module):
    """Replays the Spot snippet onto the camera topics the tracker consumes."""

    image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    @rpc
    def replay(self, settle_s: float = 2.0, tick_s: float = 0.1) -> int:
        """Publish mount tf + intrinsics, then every frame set; returns the tick count."""
        store = open_snippet()
        gray_infos: dict[str, CameraInfo] = {}
        for observation in store.stream("grayscale_info", CameraInfo):
            gray_infos.setdefault(observation.data.frame_id, observation.data)
            if len(gray_infos) >= 5:
                break
        depth_infos: dict[str, CameraInfo] = {}
        for observation in store.stream("depth_info", CameraInfo):
            depth_infos.setdefault(observation.data.frame_id, observation.data)
            if len(depth_infos) >= 5:
                break
        mount: dict[str, Any] = {}
        for tf_observation in store.stream("tf", TFMessage):
            for transform in tf_observation.data.transforms:
                if transform.frame_id == "base_link":
                    mount.setdefault(transform.child_frame_id, transform)
            if len(mount) >= 5:
                break

        def publish_static(stamp: float) -> None:
            for transform in mount.values():
                transform.ts = stamp
            self.tf.publish(TFMessage(*mount.values()))
            for frame in OPTICAL_FRAMES:
                gray_infos[frame].ts = stamp
                self.camera_info.publish(gray_infos[frame])
                depth_infos[frame].ts = stamp
                self.depth_camera_info.publish(depth_infos[frame])

        for _ in range(5):
            publish_static(time.time())
            time.sleep(settle_s / 5)

        gray_walkers = {
            camera: _NearestWalker(store.stream(f"grayscale_image_{camera}", Image))
            for camera in CAMERAS[1:]
        }
        depth_walkers = {
            camera: _NearestWalker(store.stream(f"depth_image_{camera}", Image))
            for camera in CAMERAS
        }
        ticks = 0
        for frame_observation in store.stream(f"grayscale_image_{CAMERAS[0]}", Image):
            stamp = frame_observation.ts
            publish_static(stamp)
            self.image.publish(frame_observation.data)
            for camera in CAMERAS[1:]:
                gray = gray_walkers[camera].at(stamp)
                if gray is not None:
                    self.image.publish(gray.data)
            for camera in CAMERAS:
                depth = depth_walkers[camera].at(stamp)
                if depth is not None:
                    self.depth_image.publish(depth.data)
            ticks += 1
            time.sleep(tick_s)
        return ticks


class OdometrySink(Module):
    """Collects every odometry pose the tracker publishes."""

    odometry: In[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        from reactivex.disposable import Disposable

        self._poses: list[list[float]] = []
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._collect, self.odometry))
        )

    def _collect(self, message: Odometry) -> None:
        position = message.pose.position
        self._poses.append(
            [float(message.ts or 0.0), float(position.x), float(position.y), float(position.z)]
        )

    @rpc
    def poses(self) -> list[list[float]]:
        return list(getattr(self, "_poses", []))


def recorded_odometry_path(start: float, end: float) -> float:
    store = open_snippet()
    points = [
        [o.data.pose.position.x, o.data.pose.position.y, o.data.pose.position.z]
        for o in store.stream("odometry", Odometry)
        if start <= o.ts <= end
    ]
    return float(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1).sum())


def test_spot_multicam_snippet_tracks() -> None:
    get_data("spot_multicam_short.db")  # fetch LFS before the worker needs it
    blueprint = autoconnect(
        SpotSnippetPublisher.blueprint(),
        CuvslamOdometry.blueprint(
            camera_mode="multicam",
            camera_frames=OPTICAL_FRAMES,
            depth_camera_frames=OPTICAL_FRAMES,
            enable_slam=False,
        ),
        OdometrySink.blueprint(),
    ).global_config(n_workers=2)

    coordinator = ModuleCoordinator.build(blueprint)
    try:
        publisher = coordinator.get_instance(SpotSnippetPublisher)
        sink = coordinator.get_instance(OdometrySink)
        ticks = publisher.replay()
        deadline = time.time() + POSE_DEADLINE_S
        poses = sink.poses()
        while len(poses) < MIN_POSES and time.time() < deadline:
            time.sleep(0.5)
            poses = sink.poses()
    finally:
        coordinator.stop()

    assert ticks >= 20, f"snippet unexpectedly short: {ticks} ticks"
    trajectory = np.asarray(poses, dtype=float)
    assert len(trajectory) >= MIN_POSES, f"only {len(trajectory)} poses published"
    assert np.isfinite(trajectory).all()

    jumps = np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1)
    assert jumps.max() < MAX_FRAME_JUMP_M, f"teleport in native replay: {jumps.max():.2f} m"

    path = float(np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum())
    reference = recorded_odometry_path(trajectory[0, 0], trajectory[-1, 0])
    assert reference > 0.5, "snippet window should contain real motion"
    assert 0.3 * reference < path < 3.0 * reference, (
        f"native cuVSLAM path {path:.2f} m vs odometry {reference:.2f} m"
    )
