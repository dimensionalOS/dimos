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

"""Building the cached world payloads a viewer is sent on connect: the voxel cloud
from the ray-traced replay, the capture poses and their thumbnails, and the top-down
map rendered from the same cloud. Mixed into MemoryWorldModule, which owns the caches
these fill and the lock order that protects them."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from dimos.teleop.memory_world.visual_search import body_style_quaternion
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WorldCache:
    """The three builders behind the payloads every viewer gets on connect."""

    def _global_map_cloud(self) -> np.ndarray | None:
        """The ray-traced global map stored in the recording, when it has one.

        One PointCloud2 holding the finished voxel set. Built ahead of time from a
        deskewed, registered lidar stream, so it is the whole map with every scan's
        rays already applied -- strictly better than the replay's final keyframe (which
        is the same idea but only as far as the replay got) and far better than a plain
        accumulation, which keeps every reflection and every person who walked past.
        """
        name = self.config.global_map_stream_name
        if not name:
            return None
        store = self._ensure_store()
        if name not in store.list_streams():
            return None
        try:
            latest = store.streams[name].last()
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

    def _build_voxel_cloud_from_lidar(self) -> tuple[dict[str, Any], bytes] | None:
        """The voxel map, packed for the wire.

        Preference, best first: a ``global_map`` stream written into the recording; the
        ray-traced replay's final keyframe; a plain accumulation of the scans. The first
        two are ray-traced -- every scan cleared the voxels its rays passed through, so
        what is left is what the last look at each place saw, and windows, people and
        reflections do not pile up the way they do in an accumulation. The cloud is
        height-coloured so the user gets depth cues without true RGB.
        """

        def from_replay() -> np.ndarray | None:
            if not self.config.build_replay_on_start:
                return None
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
                # No seekable replay (a failed build, too few scans): the map still shows.
                logger.exception("no replay for the cloud; accumulating the scans instead")
                return None

        def from_global_map() -> np.ndarray | None:
            # Contained like `from_replay`, and for the same reason. This source is
            # matched by NAME alone -- it never goes through detect_streams, so nothing
            # checks its message type -- and a recording whose `global_map` is an
            # OccupancyGrid (an ordinary ROS name for a 2-D map) raised straight out of
            # the loop into the handler below. That returned None for the WHOLE build, so
            # the replay and the accumulated scans were never tried and the viewer got
            # "world load failed" with a usable map sitting in the recording.
            # No `if self._stopping.is_set(): raise` here, unlike from_replay. That
            # re-raise belongs there because _ensure_replay is a long build that WATCHES
            # _stopping and raises its own cancellation; this is one quick stream read
            # that never looks at it. Copying the clause turned a bad-data failure into a
            # total one whenever a stop happened to be in flight: the re-raise is caught
            # by this function's own trailing handler, which returns None for the WHOLE
            # build and skips the very fall-through the guard was added to provide. A
            # genuine cancellation still propagates -- from_replay is next in the chain
            # and raises it there.
            try:
                found = self._replay_read(self._global_map_cloud)
            except Exception:
                logger.exception(
                    "the %s stream could not be read as a cloud; trying the next source",
                    self.config.global_map_stream_name,
                )
                return None
            if found is not None:
                logger.info(
                    "voxel cloud from the %s stream: %d voxels",
                    self.config.global_map_stream_name,
                    len(found),
                )
            return found

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
                low = self.config.map_z_min if self.config.map_z_min is not None else -np.inf
                high = self.config.map_z_max if self.config.map_z_max is not None else np.inf
                keep = (z >= low) & (z <= high)
                logger.info(
                    "cloud z spans %.2f..%.2f; keeping %d of %d voxels",
                    float(z.min()),
                    float(z.max()),
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
            self._map_xyz = np.ascontiguousarray(
                xyz.astype(np.float32)
            )  # the whole map, for planning
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
            source_ids: list[int] = []  # the store's own, which analyze_memory names
            source_ids_are_real = not self.config.store_path.endswith(".mcap")
            thumbnails: list[bytes] = []

            # One indexed read per marker rather than a pass over every frame:
            # on an mcap a full pass decompresses every image chunk (minutes),
            # while a read from a stamp touches only the chunk that holds it.
            def sampled() -> Any:
                # `after(ts).limit(1)` is a FILTER, not a cursor: when the recording holds
                # fewer frames than markers asked for, every remaining probe lands on the
                # same last frame. With n=200 over a 3-frame stream that published 200
                # markers, 197 of them the same picture in the same place, and ran 200
                # JPEG encodes to do it -- and because module.py keys `sources` by id, the
                # duplicates then collapsed and analyze_memory's ids snapped to the last
                # marker of each run. One marker per frame, however many probes hit it.
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
                # An mcap numbers each windowed read from zero, so its ids would name
                # the wrong frames; only a db's are the ids analyze_memory means.
                source_ids.append(int(getattr(obs, "id", 0)) if source_ids_are_real else -1)

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
                "source_ids": source_ids,  # what analyze_memory's observation_ids mean
            }
            payload = pos_arr.tobytes() + quat_arr.tobytes()
            logger.info("built %d image-pose markers + thumbnails", header["n"])
            return (header, payload), thumbnails
        except Exception:
            logger.exception("failed to build image poses")
            return ({"n": 0, "timestamps": [], "ids": []}, b""), []

    def _build_top_down_map(
        self, cloud: tuple[dict[str, Any], bytes]
    ) -> tuple[dict[str, Any], bytes] | None:
        """Render a top-down density map from the same point cloud shown in VR.

        Used for two things on the client: a GTA-style HUD minimap and a
        ground-pasted texture (so the user sees walls "drawn" on the floor).
        """
        cloud_header, cloud_payload = cloud
        n = int(cloud_header.get("n", 0))
        xyz = np.frombuffer(cloud_payload, dtype=np.float32, count=n * 3).reshape(n, 3)
        if xyz.size == 0:
            return None

        z = xyz[:, 2]
        low = float(np.percentile(z, self.config.map_z_low_percentile))
        high = float(np.percentile(z, self.config.map_z_high_percentile))
        m = (z >= low) & (z <= high)
        xy = xyz[m, :2]
        if xy.size == 0:
            xy = xyz[:, :2]

        x_min, x_max = float(xy[:, 0].min()), float(xy[:, 0].max())
        y_min, y_max = float(xy[:, 1].min()), float(xy[:, 1].max())
        cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
        half = max(x_max - x_min, y_max - y_min) / 2 * 1.05
        x_min, x_max, y_min, y_max = cx - half, cx + half, cy - half, cy + half

        size = int(self.config.map_image_size)
        hist, _, _ = np.histogram2d(
            xy[:, 0], xy[:, 1], bins=size, range=[[x_min, x_max], [y_min, y_max]]
        )
        density_scale = max(float(np.percentile(hist, 99)), 1.0)
        norm = np.clip(hist / density_scale, 0.0, 1.0)
        gray = (norm.T * 255).astype(np.uint8)
        gray = np.flipud(gray)
        # Light cyan walls on dark navy background — matches the world theme.
        rgb = np.zeros((size, size, 3), dtype=np.uint8)
        rgb[..., 0] = (gray.astype(np.uint16) * 76 // 255).astype(np.uint8)
        rgb[..., 1] = (gray.astype(np.uint16) * 217 // 255).astype(np.uint8)
        rgb[..., 2] = (gray.astype(np.uint16) * 255 // 255).astype(np.uint8)
        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if not ok:
            return None
        header = {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "width_px": size,
            "height_px": size,
        }
        logger.info("built top-down map: %dx%d bounds=%s", size, size, header)
        return header, buf.tobytes()
