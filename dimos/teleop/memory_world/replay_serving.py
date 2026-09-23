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
nothing builds the replay streams any more; see replay.py."""

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
# orange, red and white stay for the answer markers.
HEIGHT_COLOR_STOPS = np.array(
    [
        [110.0, 30.0, 170.0],
        [40.0, 90.0, 235.0],
        [40.0, 200.0, 230.0],
        [150.0, 240.0, 150.0],
    ]
)


# How often to ask tf where the robot was, when building the PATH rather than the orbit.
# The orbit is one position per step of whatever the viewer seeks over; the path is the
# free corridor a route is planned across, and its density has to be a property of the
# PATH, not of how often a mapper happened to publish. Measured on roscon: one position
# per map snapshot is 119 samples over 659.7 m -- a median gap of 5.54 m, 109 of 118 gaps
# over 2 m -- which is a dotted line, not a corridor. At half a second it is ~1,840
# samples and a third of a metre.
PATH_SAMPLE_S = 0.5
# Samples closer together than this are dropped: a parked robot otherwise spends the
# corridor on one pose, which is the same failure `_recording_start_candidates` guards
# against at the other end.
PATH_MIN_STEP_M = 0.05


class ReplayServing:
    """Needs, from the module: ``config``, ``_ensure_store``, ``_replay_if_ready``,
    ``_replay_index``, ``_replay_frames``, ``_sharp_frames``, ``_tf_tree``,
    ``_frame_pose_at``, ``_camera_frame``, ``_camera_hfov``, ``_encode_jpeg``."""

    config: Any
    _replay_index: dict[str, Any] | None
    _replay_lock: Any
    _replay_frames: OrderedDict[float, tuple[bytes, dict[str, Any]]]
    _sharp_frames: OrderedDict[tuple[float, int, int], tuple[bytes, dict[str, Any]]]

    if TYPE_CHECKING:

        def _ensure_store(self) -> Any: ...
        def _replay_if_ready(self) -> tuple[VoxelReplay, dict[str, Any]]: ...
        def _replay_read(self, fn: Any, *args: Any) -> Any: ...
        def _map_timeline(self) -> tuple[str, list[float]] | None: ...
        def _tf_tree(self) -> Any: ...
        def _frame_pose_at(self, frame: str, ts: float) -> Any: ...
        def _camera_frame(self) -> str: ...
        def _camera_hfov(self) -> float: ...
        def _camera_pose_of(self, obs: Any) -> Any: ...
        @staticmethod
        def _encode_jpeg(img: Any, max_size: int, quality: int) -> bytes: ...

    def _timeline_index_json(self) -> dict[str, Any]:
        """What the viewer seeks over: the map's own messages where it has several,
        and the ray-traced replay otherwise.

        The map stream wins because it is already there. A recording whose mapper
        published as it went carries the map at every moment of it, so the timeline is a
        read. Replay streams are the same picture reconstructed scan by scan; nothing
        writes them any more, so they are only ever read off a recording made before that.
        """
        found = self._map_timeline_index_json()
        return found if found is not None else self._replay_index_json()

    def _map_timeline_index_json(self) -> dict[str, Any] | None:
        """The seek index over the map stream's own messages, or None if it has none.

        Deliberately without `height` and `colors`: those exist so the viewer can colour
        replayed voxels itself on the static map's ramp, and a snapshot arrives coloured
        by the server on that same ramp (`_map_ramp`). Nothing to match.
        """
        return self._replay_read(self._map_timeline_index)

    def _map_timeline_index(self) -> dict[str, Any] | None:
        """`_map_timeline_index_json` under the store lock, like every read of it."""
        found = self._map_timeline()
        if found is None:
            return None
        _, stamps = found
        # `.order_by("ts")`, and for the reason `_build_replay_index_json` gives: the
        # viewer binary-searches this list and a stream iterates in write order.
        images = self._ensure_store().streams[self.config.image_stream_name].order_by("ts")
        return {
            "mode": "map_snapshots",
            "voxel_size": float(self.config.voxel_size),
            # Every snapshot is its own keyframe: there are no diffs to apply between
            # them, so the viewer's segment machinery maps one-to-one onto them.
            "scans": stamps,
            "keyframes": [{"scan": i, "ts": ts} for i, ts in enumerate(stamps)],
            "frames": [float(obs.ts) for obs in images],
            "hfov_deg": self._camera_hfov(),
            # Two different questions, and on this index they have two different answers:
            # `orbit` is one position per SNAPSHOT because the viewer indexes it by step,
            # `path` is the corridor at its own cadence because the planner walks it.
            "orbit": self._orbit_positions(np.asarray(stamps, dtype=np.float64)),
            "path": self._robot_path(self._effective_orbit_frame(), stamps[0], stamps[-1]),
        }

    def _replay_index_json(self) -> dict[str, Any]:
        return self._replay_if_ready()[1]

    def _build_replay_index_json(self, replay: VoxelReplay) -> dict[str, Any]:
        # `.order_by("ts")`, like every other read of an image stream in this package: a
        # stream iterates in the order it was WRITTEN, and the viewer binary-searches this
        # list. Measured on stamps inserted 1.0, 2.0, 0.5, 3.0, 2.5: scrubbing to 0.5 --
        # a stamp that is in the list -- found nothing and blanked the camera frame.
        images = self._ensure_store().streams[self.config.image_stream_name].order_by("ts")
        payload = replay.index.to_json()
        # Frame stamps let the viewer ask for exact frames, so its cache hits.
        # EXACT stamps. The comment above is the contract -- the viewer picks a `t` out of
        # this list and asks for it by value -- and rounding here broke it at the source:
        # two frames a tenth of a millisecond apart arrived as the same number, so the
        # viewer could not name the second one however exact the route's key became.
        # Keying the cache exactly was only half the fix.
        payload["frames"] = [float(obs.ts) for obs in images]
        payload["hfov_deg"] = self._camera_hfov()
        # The viewer colours replayed voxels itself, on the static map's ramp -- so the
        # ramp has to be measured on the voxels it will actually colour. The keyframe is
        # the UNFILTERED cloud; the replay draws only the z slab (`_grid_indices`), and
        # the static map is cut to the same band in world_cache. Measured on a keyframe
        # holding strays at -4.0 and 9.0 with a slab of -0.5..3.0: the ramp went out as
        # floor -2.8 span 9.7 where the voxels on screen wanted floor 0.1 span 1.8 --
        # 2.9 m of offset and five times the span, so every replayed voxel was the wrong
        # colour against a static map that had the band applied.
        final = replay.final_keyframe().data.points_f32()
        if len(final):
            inside = final[(final[:, 2] >= replay.z_min) & (final[:, 2] <= replay.z_max)]
            final = inside if len(inside) else final
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
        if tree is not None and not tree.has_frame(frame):
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
            # TIME order, because the builder wrote them in time order: position n
            # has to be the pose of the scan whose voxels are frame n. This read was row
            # order to match a builder that was also row order, and when the builder was
            # put right the two stopped agreeing -- measured, on scans written 1, 2, 0.5,
            # 3, 2.5, seeking to 2.0 orbited [5, 0, 0] instead of [20, 0, 0].
            scans = self._ensure_store().streams[self.config.lidar_stream_name].order_by("ts")
            positions = stamped_positions(scans)[: len(stamps)]
            # ...and PADDED to the scan count, which the tf branch above always satisfies.
            # Truncating alone let this come back shorter when only some scans carry a
            # pose, so the viewer's `orbitPositions[scan]` ran off the end and
            # `_robot_end_pose()` read `positions[-1]` as "where the robot ended" when it
            # was really wherever the last POSED scan was, many scans earlier.
            if positions and len(positions) < len(stamps):
                positions = positions + [positions[-1]] * (len(stamps) - len(positions))
        return {"frame": frame, "positions": positions}

    def _robot_path(self, frame: str, first: float, last: float) -> dict[str, Any]:
        """Where *frame* went between two stamps, sampled at the PATH's own cadence.

        Separate from `_orbit_positions` on purpose. The orbit is indexed BY STEP -- the
        viewer does `positions[scan]` -- so it must stay one position per step whatever
        that costs the geometry. The planner wants the opposite: every place the robot
        demonstrably was, close enough together that the space between two of them is
        space it actually crossed.
        """
        if last <= first:
            return {"frame": frame, "positions": []}
        stamps = np.arange(first, last, PATH_SAMPLE_S, dtype=np.float64)
        walked = frame_positions(stamps, lambda ts: self._frame_pose_at(frame, ts))
        kept: list[list[float]] = []
        for where in walked:
            if (
                kept
                and float(np.linalg.norm(np.asarray(where) - np.asarray(kept[-1])))
                < PATH_MIN_STEP_M
            ):
                continue
            kept.append(where)
        logger.info(
            "robot path: %d samples over %.1f s of %r (the orbit has one per step)",
            len(kept),
            last - first,
            frame,
        )
        return {"frame": frame, "positions": kept}

    def _replay_frame(
        self, ts: float, max_size: int | None = None
    ) -> tuple[bytes, dict[str, Any]] | None:
        """JPEG and camera pose of the image nearest *ts*, in a small LRU.

        *max_size* larger than the scrub size serves the SHARP re-encode a viewer standing
        in front of a capture-pose marker asks for, out of its own small cache. The two
        never share an entry: one stamp has two encodings, and a cache keyed on the stamp
        alone handed whichever arrived first to both callers.
        """
        scrub_size = int(self.config.replay_frame_max_size)
        size = scrub_size if max_size is None else int(max_size)
        sharp = size > scrub_size
        quality = int(
            self.config.marker_sharp_jpeg_quality
            if sharp
            else self.config.replay_frame_jpeg_quality
        )

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
        stamp = float(obs.ts)
        cache: OrderedDict[Any, tuple[bytes, dict[str, Any]]] = (
            self._sharp_frames if sharp else self._replay_frames
        )
        key: Any = (stamp, size, quality) if sharp else stamp
        limit = int(self.config.marker_sharp_cache_size) if sharp else 600

        cached = cache.get(key)
        if cached is not None:
            cache.move_to_end(key)
            return cached
        jpeg = self._encode_jpeg(obs.data, size, quality)
        meta: dict[str, Any] = {"ts": stamp, "hfov_deg": self._camera_hfov()}
        camera = self._camera_pose_of(obs)
        if camera is not None:
            meta.update(
                position=[float(v) for v in camera[:3, 3]],
                forward=[float(v) for v in camera[:3, 2]],
                up=[float(v) for v in -camera[:3, 1]],
            )
        cache[key] = (jpeg, meta)
        while len(cache) > limit:
            cache.popitem(last=False)
        return jpeg, meta
