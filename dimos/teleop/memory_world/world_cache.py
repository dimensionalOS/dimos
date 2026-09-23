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

"""Building the cached world payloads a viewer is sent on connect: the voxel cloud, the
capture poses and their thumbnails -- and, when the recording's map stream holds more
than one message, the map as it stood at any moment of it. Mixed into
MemoryWorldModule, which owns the caches these fill and the lock order that protects
them."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.teleop.memory_world.visual_search import body_style_quaternion
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# What `global_map_smooth` names its output: `<in>_smoothed`.
SMOOTHED_SUFFIX = "_smoothed"


def _sibling_map_stream(name: str) -> str:
    """The other half of a `<raw>` / `<raw>_smoothed` pair, given either half."""
    return (
        name[: -len(SMOOTHED_SUFFIX)] if name.endswith(SMOOTHED_SUFFIX) else name + SMOOTHED_SUFFIX
    )


class WorldCache:
    """The builders behind the payloads every viewer gets on connect.

    Class attributes, not `__init__` state: module.py owns the constructor and sits on
    the repo's 75 KB per-file ceiling. `_reopen_store` clears both.
    """

    # () means "looked, and this recording has no map timeline"; None means "not looked".
    _map_timeline_cache: tuple[str, list[float]] | tuple[()] | None = None
    _map_ramp_cache: tuple[float, float] | None = None

    def _global_map_cloud(self) -> tuple[str, np.ndarray] | None:
        """The ray-traced global map stored in the recording, when it has one.

        One PointCloud2 holding the finished voxel set. Built ahead of time from a
        deskewed, registered lidar stream, so it is the whole map with every scan's
        rays already applied -- strictly better than the replay's final keyframe (which
        is the same idea but only as far as the replay got) and far better than a plain
        accumulation, which keeps every reflection and every person who walked past.

        The CONFIGURED stream is tried first and always wins where it exists, so
        `--memoryworldmodule.global-map-stream-name` names the map and nothing overrules
        it. Its `<raw>`/`<raw>_smoothed` sibling is the fallback -- a recording carries
        one, the other, or both, and the default asks for the smoothed one because
        `global_map_smooth`'s closing fills the doorway-sized holes and the speckle
        between two sweeps while leaving the map's outline alone. Both are tried THROUGH
        the reads below rather than matched on the name, so a stream that is declared but
        empty, unreadable or unplaceable by tf falls through to its sibling instead of
        losing the map. Returns the name it read beside the cloud, so the log says which.
        """
        configured = self.config.global_map_stream_name
        if not configured:
            return None
        for name in (configured, _sibling_map_stream(configured)):
            xyz = self._named_map_cloud(name)
            if xyz is not None:
                return name, xyz
        return None

    def _named_map_cloud(self, name: str, at: float | None = None) -> np.ndarray | None:
        """One map stream read and placed in `world_frame`, or None if it cannot be.

        None rather than a raise on every way of being unusable -- absent, declared but
        empty, or in a frame tf cannot place -- because both callers have somewhere else
        to go, and the point of returning None is that they get there.

        *at* reads the last message written at or before that stamp rather than the last
        one in the stream: a mapper publishes its map as it grows, so the same stream
        that gives the finished world gives every earlier state of it. `before` is
        exclusive, hence the millisecond -- the stamp asked for is one this stream wrote.
        """
        store = self._ensure_store()
        if name not in store.list_streams():
            return None
        try:
            stream = store.streams[name].order_by("ts")
            latest = stream.last() if at is None else stream.before(at + 1e-3).last()
        except LookupError:  # declared but empty
            return None
        xyz = latest.data.points_f32()
        if xyz is None or len(xyz) == 0:
            return None
        xyz = np.asarray(xyz, dtype=np.float32)
        # The cloud says which frame it is in and it is NOT safe to assume that is ours.
        # Everything else in the world -- the capture poses, the trajectory, the planner's
        # map -- is in `world_frame`, so a map written in any other frame has to be moved
        # into it or it sits somewhere else entirely while looking perfectly reasonable.
        frame = str(getattr(latest.data, "frame_id", "") or "").lstrip("/")
        if frame and frame != self.config.world_frame:
            matrix = self._frame_pose_at(frame, float(latest.ts))
            if matrix is None:
                # Refuse rather than place it wrongly: the caller falls back to a source
                # whose frame is known.
                logger.warning(
                    "%s is in %r and tf cannot place that in %r; ignoring it",
                    name,
                    frame,
                    self.config.world_frame,
                )
                return None
            logger.info("%s is in %r; moving it into %r", name, frame, self.config.world_frame)
            xyz = (np.asarray(matrix) @ np.c_[xyz, np.ones(len(xyz))].T).T[:, :3]
        return np.ascontiguousarray(xyz, dtype=np.float32)

    def _map_height_mask(self, found: np.ndarray) -> np.ndarray:
        """Points that are finite on EVERY axis and inside the configured height band."""
        z = found[:, 2]
        low = self.config.map_z_min if self.config.map_z_min is not None else -np.inf
        high = self.config.map_z_max if self.config.map_z_max is not None else np.inf
        return np.isfinite(found).all(axis=1) & (z >= low) & (z <= high)

    # ---- the map's own timeline ----------------------------------------------

    def _map_timeline(self) -> tuple[str, list[float]] | None:
        """The map stream's messages, when it holds more than one: the timeline itself.

        A mapper publishes the map AS IT GROWS -- roscon_setup.mcap carries 119 of them,
        one every 7.8 s, 812 KB at the start and 50.9 MB at the end -- so the recording
        already knows what the map looked like at any moment and scrubbing is a read, not
        a build. This is why the ray-traced `voxel_diff`/`voxel_keyframe` replay is not
        needed on such a recording: it reconstructs, scan by scan and in half an hour,
        something the file was handed for free.

        The same two candidates as the drawn world, in the same order, so the timeline
        scrubs the map the viewer is looking at. Stamps only -- the clouds stay on disk.
        """
        if self._map_timeline_cache is not None:
            return self._map_timeline_cache or None
        configured = self.config.global_map_stream_name
        store = self._ensure_store()
        names = store.list_streams() if configured else []
        for name in (configured, _sibling_map_stream(configured)) if configured else ():
            if name not in names:
                continue
            stamps = sorted(float(obs.ts) for obs in store.streams[name].order_by("ts"))
            if len(stamps) > 1:
                logger.info("map timeline: %d %s messages to scrub through", len(stamps), name)
                self._map_timeline_cache = (name, stamps)
                return self._map_timeline_cache
        self._map_timeline_cache = ()  # asked and answered: not every recording has one
        return None

    def _map_snapshot(self, number: int) -> tuple[dict[str, Any], bytes] | None:
        """The map as it stood at snapshot *number*, packed like the static cloud.

        Coloured on the FINISHED map's height ramp rather than its own: a ramp taken
        from each snapshot would recolour the whole world at every step of the scrub,
        because the early ones hold a fraction of the building's height range.
        """
        found = self._map_timeline()
        if found is None:
            return None
        name, stamps = found
        if not 0 <= number < len(stamps):
            return None
        xyz = self._named_map_cloud(name, at=stamps[number])
        if xyz is None or xyz.size == 0:
            return None
        keep = self._map_height_mask(xyz)
        if not keep.any():
            return None
        xyz = xyz[keep]
        held = int(xyz.shape[0])
        cap = self.config.map_timeline_max_points
        if cap > 0 and xyz.shape[0] > cap:
            xyz = xyz[:: xyz.shape[0] // cap + 1]
        positions = np.ascontiguousarray(xyz.astype(np.float32))
        # `points` is what the message HELD and `n` what was sent: the late snapshots are
        # strided to the cap and the early ones are not, so comparing `n` across a scrub
        # says less than it looks like it does.
        header = {
            **self._cloud_header(positions),
            "snapshot": number,
            "ts": stamps[number],
            "points": held,
        }
        return header, positions.tobytes() + self._height_colors(
            positions, self._map_ramp()
        ).tobytes()

    def _map_ramp(self) -> tuple[float, float] | None:
        """The height ramp of the finished map, measured once and kept."""
        if self._map_ramp_cache is None:
            found = self._map_timeline()
            xyz = self._named_map_cloud(found[0]) if found else None
            if xyz is None or xyz.size == 0:
                return None
            inside = xyz[self._map_height_mask(xyz)]
            z = (inside if len(inside) else xyz)[:, 2]
            self._map_ramp_cache = (
                float(np.percentile(z, self.config.height_ramp_low_percentile)),
                float(np.percentile(z, self.config.height_ramp_high_percentile)),
            )
        return self._map_ramp_cache

    def _build_voxel_cloud_from_lidar(self) -> tuple[dict[str, Any], bytes] | None:
        """The voxel map, packed for the wire.

        Preference, best first: a ``global_map`` stream written into the recording; the
        final keyframe of ray-traced replay streams it already carries; a plain
        accumulation of the scans. The first two are ray-traced -- every scan cleared the
        voxels its rays passed through, so what is left is what the last look at each
        place saw, and windows, people and reflections do not pile up the way they do in
        an accumulation. The cloud is height-coloured so the user gets depth cues without
        true RGB.
        """

        def from_replay() -> np.ndarray | None:
            try:
                replay = self._ensure_replay()
                found = self._replay_read(lambda: replay.final_keyframe().data.points_f32())
                if found is None or found.size == 0:  # scans tf could not place
                    raise RuntimeError("the replay's final keyframe is empty")
                logger.info("voxel cloud from the ray-traced replay: %d voxels", len(found))
                return found
            except Exception:
                if self._stopping.is_set():
                    raise
                # No seekable replay (this recording never had one, or too few scans):
                # the map still shows.
                logger.info("no replay streams for the cloud; accumulating the scans instead")
                return None

        drawn_from: list[str] = []  # the stream the drawn cloud came from, if it was one

        def from_global_map() -> np.ndarray | None:
            # Contained like `from_replay`, and for the same reason. This source is
            # matched by NAME alone -- it never goes through detect_streams, so nothing
            # checks its message type -- and a recording whose `global_map` is an
            # OccupancyGrid (an ordinary ROS name for a 2-D map) raised straight out of
            # the loop into the handler below. That returned None for the WHOLE build, so
            # the replay and the accumulated scans were never tried and the viewer got
            # "world load failed" with a usable map sitting in the recording.
            # No `if self._stopping.is_set(): raise` here, unlike from_replay, which
            # keeps it because it opens a whole index while a stop may be in flight.
            # Copying the clause turned a bad-data failure into a total one whenever a
            # stop happened to be in flight: the re-raise is caught
            # by this function's own trailing handler, which returns None for the WHOLE
            # build and skips the very fall-through the guard was added to provide.
            #
            # Nor is one needed: `_replay_read` here is a plain stream read that never
            # looks at `_stopping`, so no cancellation is raised in this function to
            # propagate. The sources that DO watch it raise it themselves -- from_replay
            # when it is reached, which is only when this one yields nothing.
            try:
                found = self._replay_read(self._global_map_cloud)
            except Exception:
                logger.exception(
                    "the %s stream could not be read as a cloud; trying the next source",
                    self.config.global_map_stream_name,
                )
                return None
            if found is None:
                return None
            name, xyz = found
            logger.info("voxel cloud from the %s stream: %d voxels", name, len(xyz))
            drawn_from.append(name)
            return xyz

        try:
            # Best first, and each source is tried THROUGH the height filter before the
            # next is given up on: a global map whose points all sit outside
            # map_z_min/max is as useless as an absent one, and committing to it there
            # left the viewer with "world load failed" while the scans it could have
            # accumulated were sitting in the recording.
            xyz: np.ndarray | None = None
            for source in (
                from_global_map,
                from_replay,
                lambda: self._replay_read(self._accumulated_cloud),
            ):
                found = source()
                if found is None or found.size == 0:
                    continue
                z = found[:, 2]
                # Finite on EVERY axis, not just inside the height band on z. The height
                # test alone let a NaN or an infinity through in x or y -- and the
                # `global_map` source is a raw PointCloud2 written by somebody else's
                # registration pipeline, which is exactly where a few degenerate points
                # come from. One of them made `_cloud_header`'s min/max NaN, which every
                # viewer then received as the world's bounds, and `np.histogram2d` raised
                # `supplied range of [nan, nan] is not finite` -- after `_cached_cloud`
                # had already been assigned, so every later build failed identically and
                # the recording was permanently unviewable. The cloud itself was fine.
                finite = np.isfinite(found).all(axis=1)
                keep = self._map_height_mask(found)
                if not finite.all():
                    logger.warning(
                        "dropped %d point(s) with a non-finite coordinate", int((~finite).sum())
                    )
                logger.info(
                    "cloud z spans %.2f..%.2f; keeping %d of %d voxels",
                    float(z[finite].min()) if finite.any() else float("nan"),
                    float(z[finite].max()) if finite.any() else float("nan"),
                    int(keep.sum()),
                    len(z),
                )
                if not keep.any():
                    logger.warning(
                        "every voxel was outside the height band; trying the next source"
                    )
                    continue
                xyz = found[keep]
                break
            if xyz is None or xyz.size == 0:
                return None
            # The whole map, unstrided, for planning -- and NOT always the one on screen.
            self._map_xyz = self._planning_map(xyz, drawn_from[0] if drawn_from else None)
            if xyz.shape[0] > self.config.max_points:
                stride = xyz.shape[0] // self.config.max_points + 1
                xyz = xyz[::stride]

            positions = np.ascontiguousarray(xyz.astype(np.float32))
            # Lidar has no RGB, so always height-colour.
            rgb = self._height_colors(positions)
            header = self._cloud_header(positions)
            payload = positions.tobytes() + rgb.tobytes()
            logger.info("built voxel cloud: n=%d", positions.shape[0])
            return header, payload
        except Exception:
            logger.exception("voxel-from-lidar build failed")
            return None

    def _planning_map(self, drawn: np.ndarray, source: str | None) -> np.ndarray:
        """The map the route planner walks over, which is not always the one on screen.

        A morphological closing is the right thing to LOOK at and the wrong thing to plan
        over. Filling a gap narrower than the structuring element seals the space between
        a floor and the shelf above it, and standable surface is exactly what the MLS
        planner reads out of that space -- so the map that looks more solid offers fewer
        places to stand. Measured on grocery.db, same planner and same 25 early poses:
        `global_map` 308,080 surface cells and a 42.93 m route to a basket from the 8th,
        `global_map_smoothed` 271,037 cells and NO route from any of them.

        So a smoothed stream is drawn and its raw sibling is planned over. With no raw
        sibling to read the drawn cloud is still the best map there is, and a closed map
        plans worse than an open one but far better than none.
        """
        if source and source.endswith(SMOOTHED_SUFFIX):
            raw = self._named_map_cloud(_sibling_map_stream(source))
            if raw is not None:
                kept = raw[self._map_height_mask(raw)]
                if kept.size:
                    logger.info(
                        "planning over the %s stream instead: %d voxels",
                        _sibling_map_stream(source),
                        len(kept),
                    )
                    return np.ascontiguousarray(kept.astype(np.float32))
                logger.warning("the raw map is empty after filtering; planning over the drawn one")
        return np.ascontiguousarray(drawn.astype(np.float32))

    def _build_image_poses(self) -> tuple[tuple[dict[str, Any], bytes], list[bytes]]:
        """Sample N capture poses and JPEG thumbnails from the color_image stream.

        Returns ((header, packed_pose_payload), list_of_jpegs).
        Pose payload: ``N*12 bytes float32`` xyz, then ``N*16 bytes float32`` quat.
        Thumbnails are sent as separate MSG_IMAGE_THUMBNAIL frames so each
        decode happens lazily on the client.
        """
        try:
            store = self._ensure_store()
            stream = store.streams[self.config.image_stream_name]
            first, last = stream.first(), stream.last()
            span = max(float(last.ts) - float(first.ts), 1e-3)
            n = max(2, int(self.config.n_image_markers))
            interval = span / n
            max_size = int(self.config.thumbnail_max_size)
            quality = int(self.config.thumbnail_jpeg_quality)

            positions: list[tuple[float, float, float]] = []
            quats: list[tuple[float, float, float, float]] = []
            timestamps: list[float] = []
            ids: list[int] = []  # marker ids, unique per marker
            thumbnails: list[bytes] = []

            # One indexed read per marker rather than a pass over every frame:
            # on an mcap a full pass decompresses every image chunk (minutes),
            # while a read from a stamp touches only the chunk that holds it.
            def sampled() -> Any:
                # `after(ts).limit(1)` is a FILTER, not a cursor: when the recording holds
                # fewer frames than markers asked for, every remaining probe lands on the
                # same last frame. With n=200 over a 3-frame stream that published 200
                # markers, 197 of them the same picture in the same place, and ran 200
                # JPEG encodes to do it. One marker per frame, however many probes hit it.
                seen: set[float] = set()
                for k in range(n):
                    found = stream.after(float(first.ts) + k * interval - 1e-6).limit(1).to_list()
                    if found and float(found[0].ts) not in seen:
                        seen.add(float(found[0].ts))
                        yield found[0]

            for k, obs in enumerate(sampled()):
                optical = self._camera_pose_of(obs)
                if optical is None:
                    continue
                # Markers stand where the camera was and face the way it looked.
                positions.append(tuple(float(v) for v in optical[:3, 3]))  # type: ignore[arg-type]
                quats.append(body_style_quaternion(optical))
                timestamps.append(float(obs.ts))
                ids.append(k)  # unique per marker: an mcap's observation ids are window-local

                try:
                    thumbnails.append(self._encode_jpeg(obs.data, max_size, quality))
                except Exception:
                    logger.exception("thumbnail encode failed at ts=%s", obs.ts)
                    thumbnails.append(b"")

                if len(positions) >= n:
                    break

            pos_arr = np.asarray(positions, dtype=np.float32)
            quat_arr = np.asarray(quats, dtype=np.float32)
            header = {
                "n": int(pos_arr.shape[0]),
                "timestamps": timestamps,
                "ids": ids,
            }
            payload = pos_arr.tobytes() + quat_arr.tobytes()
            logger.info("built %d image-pose markers + thumbnails", header["n"])
            return (header, payload), thumbnails
        except Exception:
            logger.exception("failed to build image poses")
            return ({"n": 0, "timestamps": [], "ids": []}, b""), []
