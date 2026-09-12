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

"""Serving the timeline replay to the viewer: the index (stamps, heights, orbit
positions) and the camera frame nearest a time. Mixed into MemoryWorldModule;
the replay streams themselves are built in module.py (_ensure_replay)."""

from __future__ import annotations

from collections import OrderedDict
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.teleop.memory_world.replay import VoxelReplay, frame_positions, stamped_positions
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Height ramp stops (RGB), floor to ceiling: purple, blue, cyan, light green,
# over the 5th-95th percentile of height so a few stray returns far above or
# below the building do not flatten everything else into one shade. Yellow,
# orange, red and white stay for the heat map and the answer markers.
HEIGHT_COLOR_STOPS = np.array(
    [
        [110.0, 30.0, 170.0],
        [40.0, 90.0, 235.0],
        [40.0, 200.0, 230.0],
        [150.0, 240.0, 150.0],
    ]
)


class ReplayServing:
    """Needs, from the module: ``config``, ``_ensure_store``, ``_replay_if_ready``,
    ``_replay_index``, ``_replay_frames``, ``_tf_tree``, ``_frame_pose_at``,
    ``_camera_frame``, ``_camera_hfov``, ``_encode_jpeg``."""

    config: Any
    _replay_index: dict[str, Any] | None
    _replay_lock: Any
    _replay_frames: OrderedDict[float, tuple[bytes, dict[str, Any]]]

    if TYPE_CHECKING:

        def _ensure_store(self) -> Any: ...
        def _replay_if_ready(self) -> tuple[VoxelReplay, dict[str, Any]]: ...
        def _tf_tree(self) -> Any: ...
        def _frame_pose_at(self, frame: str, ts: float) -> Any: ...
        def _camera_frame(self) -> str: ...
        def _camera_hfov(self) -> float: ...
        def _camera_pose_of(self, obs: Any) -> Any: ...
        @staticmethod
        def _encode_jpeg(img: Any, max_size: int, quality: int) -> bytes: ...

    def _replay_index_json(self) -> dict[str, Any]:
        return self._replay_if_ready()[1]

    def _build_replay_index_json(self, replay: VoxelReplay) -> dict[str, Any]:
        images = self._ensure_store().streams[self.config.image_stream_name]
        payload = replay.index.to_json()
        # Frame stamps let the viewer ask for exact frames, so its cache hits.
        payload["frames"] = [round(float(obs.ts), 4) for obs in images]
        payload["hfov_deg"] = self._camera_hfov()
        # The viewer colours replayed voxels itself, on the static map's ramp.
        final = replay.final_keyframe().data.points_f32()
        z = final[:, 2] if len(final) else np.zeros(1)
        low = float(np.percentile(z, self.config.height_ramp_low_percentile))
        high = float(np.percentile(z, self.config.height_ramp_high_percentile))
        payload["height"] = {"floor": low, "span": max(high - low, 1e-3)}
        payload["colors"] = (HEIGHT_COLOR_STOPS / 255.0).round(4).tolist()
        payload["orbit"] = self._orbit_positions(replay.index.scan_ts)
        return payload

    def _effective_orbit_frame(self) -> str:
        """The frame actually orbited: the configured one, or the camera when tf lacks it.

        Everything that starts from where the robot is -- the orbit, the route's start
        pose -- has to agree on this, or Navigate 404s on a recording the viewer is
        happily orbiting.
        """
        tree = self._tf_tree()
        frame = self.config.orbit_frame
        if tree is not None and frame not in tree.frames:
            logger.warning("orbit frame %r not in tf; using the camera instead", frame)
            frame = self._camera_frame()
        return frame

    def _orbit_positions(self, stamps: np.ndarray) -> dict[str, Any]:
        """Where the orbit frame was at each replay scan, for the viewer to circle."""
        tree = self._tf_tree()
        frame = self._effective_orbit_frame()
        if tree is not None:
            positions = frame_positions(stamps, lambda ts: self._frame_pose_at(frame, ts))
        else:  # no tf: the pose stamped on the lidar scans is all there is
            # Row order, like build_replay_streams: sorting here would pair scan n's
            # pose with a different scan's voxels.
            scans = self._ensure_store().streams[self.config.lidar_stream_name]
            positions = stamped_positions(scans)[: len(stamps)]
        return {"frame": frame, "positions": positions}

    def _replay_frame(self, ts: float) -> tuple[bytes, dict[str, Any]] | None:
        """JPEG and camera pose of the image nearest *ts*, in a small LRU."""
        images = self._ensure_store().streams[self.config.image_stream_name]
        candidates = list(images.at(ts, tolerance=0.25))
        if not candidates:
            return None
        obs = min(candidates, key=lambda o: abs(float(o.ts) - ts))
        # Not obs.id: an mcap numbers each windowed read from 0. Not a ROUNDED stamp
        # either -- at 4 decimals two frames a tenth of a millisecond apart share a bucket
        # and the second one is served the first one's JPEG and pose. The stamp is
        # deterministic for a given observation, so the exact float is already a stable
        # key and rounding bought nothing.
        key = float(obs.ts)
        cached = self._replay_frames.get(key)
        if cached is not None:
            self._replay_frames.move_to_end(key)
            return cached
        jpeg = self._encode_jpeg(
            obs.data, self.config.replay_frame_max_size, self.config.replay_frame_jpeg_quality
        )
        meta: dict[str, Any] = {"ts": round(float(obs.ts), 4), "hfov_deg": self._camera_hfov()}
        camera = self._camera_pose_of(obs)
        if camera is not None:
            meta.update(
                position=[float(v) for v in camera[:3, 3]],
                forward=[float(v) for v in camera[:3, 2]],
                up=[float(v) for v in -camera[:3, 1]],
            )
        self._replay_frames[key] = (jpeg, meta)
        while len(self._replay_frames) > 600:
            self._replay_frames.popitem(last=False)
        return jpeg, meta
