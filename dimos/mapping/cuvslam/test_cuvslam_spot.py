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

Boots the dimSLAM binary through ``CuvslamOdometry`` (``camera_mode="multicam"``, the
three landscape cameras with their registered depth anchoring scale), feeds it 4 s of
recorded frames over LCM, and checks the odometry it publishes flows and agrees with
the robot's own recorded odometry. cuVSLAM's Multisensor depth path has no CPU
fallback (use_gpu=False aborts in CUDA), so runs are not bit-reproducible and the
assertions are bounds, not exact poses.
"""

import time
from typing import Any

import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted

# The front pair is mounted portrait and cuVSLAM requires one resolution across the
# rig, so the native module runs the three landscape cameras.
CAMERAS = ["left", "right", "back"]
OPTICAL_FRAMES = [f"{camera}_camera_optical" for camera in CAMERAS]
TOPIC = "/test_cuvslam_spot"
MIN_POSES = 12
MAX_FRAME_JUMP_M = 1.0
SETTLE_S = 2.0
POSE_DEADLINE_S = 60.0


class NearestWalker:
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


def first_per_frame(store: SqliteStore, stream: str) -> dict[str, CameraInfo]:
    found: dict[str, CameraInfo] = {}
    for observation in store.stream(stream, CameraInfo):
        found.setdefault(observation.data.frame_id, observation.data)
        if len(found) >= 5:
            break
    return found


def mount_edges(store: SqliteStore) -> list[Any]:
    edges = {}
    for observation in store.stream("tf", TFMessage):
        for transform in observation.data.transforms:
            if transform.frame_id == "base_link":
                edges[transform.child_frame_id] = transform
        if len(edges) >= 5:
            break
    return list(edges.values())


def recorded_odometry_path(store: SqliteStore, start: float, end: float) -> float:
    points = [
        [o.data.pose.position.x, o.data.pose.position.y, o.data.pose.position.z]
        for o in store.stream("odometry", Odometry)
        if start <= o.ts <= end
    ]
    return float(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1).sum())


def test_spot_multicam_snippet_tracks() -> None:
    store = SqliteStore(path=str(get_data("spot_multicam_short.db")), must_exist=True)
    store.start()
    gray_infos = first_per_frame(store, "grayscale_info")
    depth_infos = first_per_frame(store, "depth_info")
    mount = mount_edges(store)

    module = CuvslamOdometry(
        camera_mode="multicam",
        camera_frames=OPTICAL_FRAMES,
        depth_camera_frames=OPTICAL_FRAMES,
        enable_slam=False,
    )
    transports: dict[str, LCMTransport[Any]] = {
        "image": LCMTransport(f"{TOPIC}/image", Image),
        "depth_image": LCMTransport(f"{TOPIC}/depth_image", Image),
        "camera_info": LCMTransport(f"{TOPIC}/camera_info", CameraInfo),
        "depth_camera_info": LCMTransport(f"{TOPIC}/depth_camera_info", CameraInfo),
        "imu": LCMTransport(f"{TOPIC}/imu", Imu),
        "imu_info": LCMTransport(f"{TOPIC}/imu_info", ImuInfo),
        "tf": LCMTransport(f"{TOPIC}/tf", TFMessage),
        "odometry": LCMTransport(f"{TOPIC}/odometry", Odometry),
        "corrected_odometry": LCMTransport(f"{TOPIC}/corrected_odometry", Odometry),
    }
    for name, transport in transports.items():
        getattr(module, name).transport = transport

    poses: list[list[float]] = []
    transports["odometry"].subscribe(
        lambda message: poses.append(
            [
                float(message.ts or 0.0),
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ]
        )
    )

    def publish_static(stamp: float) -> None:
        for transform in mount:
            transform.ts = stamp
        transports["tf"].broadcast(None, TFMessage(*mount))
        for frame in OPTICAL_FRAMES:
            info = gray_infos[frame]
            info.ts = stamp
            transports["camera_info"].broadcast(None, info)
            depth_info = depth_infos[frame]
            depth_info.ts = stamp
            transports["depth_camera_info"].broadcast(None, depth_info)

    try:
        module.build()  # runs build_command (nix build of the pinned dimSLAM) if needed
        module.start()
        for _ in range(5):
            publish_static(time.time())
            time.sleep(SETTLE_S / 5)

        gray_walkers = {
            c: NearestWalker(store.stream(f"grayscale_image_{c}", Image)) for c in CAMERAS[1:]
        }
        depth_walkers = {c: NearestWalker(store.stream(f"depth_image_{c}", Image)) for c in CAMERAS}
        first_stamp = None
        ticks = 0
        for observation in store.stream(f"grayscale_image_{CAMERAS[0]}", Image):
            stamp = observation.ts
            if first_stamp is None:
                first_stamp = stamp
            publish_static(stamp)
            transports["image"].broadcast(None, observation.data)
            for camera in CAMERAS[1:]:
                gray = gray_walkers[camera].at(stamp)
                if gray is not None:
                    transports["image"].broadcast(None, gray.data)
            for camera in CAMERAS:
                depth = depth_walkers[camera].at(stamp)
                if depth is not None:
                    transports["depth_image"].broadcast(None, depth.data)
            ticks += 1
            time.sleep(0.1)

        deadline = time.time() + POSE_DEADLINE_S
        while len(poses) < MIN_POSES and time.time() < deadline:
            time.sleep(0.5)
    finally:
        module.stop()
        for transport in transports.values():
            transport.stop()

    assert ticks >= 20, f"snippet unexpectedly short: {ticks} ticks"
    trajectory = np.asarray(poses, dtype=float)
    assert len(trajectory) >= MIN_POSES, f"only {len(trajectory)} poses published"
    assert np.isfinite(trajectory).all()

    jumps = np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1)
    assert jumps.max() < MAX_FRAME_JUMP_M, f"teleport in native replay: {jumps.max():.2f} m"

    path = float(np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum())
    reference = recorded_odometry_path(store, trajectory[0, 0], trajectory[-1, 0])
    assert reference > 0.5, "snippet window should contain real motion"
    assert 0.3 * reference < path < 3.0 * reference, (
        f"native cuVSLAM path {path:.2f} m vs odometry {reference:.2f} m"
    )
