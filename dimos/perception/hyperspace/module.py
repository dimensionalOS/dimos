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

"""Hyperspace: a voxel map where every voxel carries a SigLIP 2 embedding.

Listens to SigLIP 2 patch grids, places each patch in 3D via the depth
camera + TF, and accumulates per-voxel mean embeddings. Text queries score
every voxel by cosine similarity: heatmap renders, cluster coordinates, a
"nearby" heading, and a scored voxel cloud logged to Rerun.
"""

from __future__ import annotations

from collections import deque
import math
import threading
import time
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.mapping.voxels.keys import KEY_OFFSET, pack_indices
from dimos.models.embedding.base import PatchEmbeddings
from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.hyperspace.cluster import VoxelCluster, top_clusters
from dimos.perception.hyperspace.projection import project_patches_to_world
from dimos.perception.hyperspace.query import DEFAULT_BACKGROUND_PROMPTS, score_query
from dimos.perception.hyperspace.render import (
    VIEWS,
    normalize_scores,
    render_nearby,
    render_view,
    scores_to_colors,
)
from dimos.perception.hyperspace.voxel_map import (
    Aggregate,
    EmbeddingVoxelMap,
    keys_near_mask,
)
from dimos.protocol.tf.tf import TF
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

#: Stream-time seconds a lidar cloud may wait for its world transform.
_LIDAR_PENDING_S = 12.0
#: Stream-time seconds a camera frame may wait for its world transform.
_FRAME_PENDING_S = 20.0


class HyperspaceModuleConfig(ModuleConfig):
    voxel_size_m: float = 0.1
    #: Fold pending voxel contributions into means at this period.
    reduce_interval_s: float = 5.0
    world_frame: str = "world"
    robot_frame: str = "base_link"
    #: Take every Nth depth pixel when projecting patches to 3D.
    depth_stride: int = 4
    min_depth_m: float = 0.3
    max_depth_m: float = 6.0
    depth_match_tolerance_s: float = 0.05
    #: Nearest-stamp tolerance; world odometry in recordings can gap ~1 s.
    tf_tolerance_s: float = 1.0
    #: TF history depth in stream time. Replay pipelines lag the tf feed by
    #: several seconds, so keep well more than the default 10 s.
    tf_buffer_s: float = 120.0
    #: Text tower for queries; must match the SigLIP 2 module's model.
    model_name: str = "google/siglip2-base-patch16-256"
    device: str | None = None
    nearby_radius_m: float = 3.0
    nearby_bin_deg: float = 5.0
    rerun_entity: str = "world/hyperspace"
    #: Generic prompts a query is contrasted against; empty list disables.
    background_prompts: list[str] = list(DEFAULT_BACKGROUND_PROMPTS)
    #: Rounds of 6-neighbor score averaging to suppress speckle.
    score_smooth_iterations: int = 2
    #: Per-voxel aggregation of frame contributions. The robust modes
    #: (median/medoid) keep a few samples per voxel and suppress single-frame
    #: outliers that give the mean false positives.
    aggregate: Aggregate = "median"
    #: Frame samples kept per voxel for the robust aggregation modes.
    max_samples_per_voxel: int = 8
    #: Maintain a ray-traced lidar voxel map and drop embedding voxels with no
    #: solid lidar geometry nearby (carves ghosts and dynamic objects).
    use_lidar_clean_map: bool = True
    lidar_max_range_m: float = 30.0
    #: Chebyshev slack (in voxels) when matching embedding voxels to lidar
    #: voxels; absorbs depth-vs-lidar discretization offsets.
    lidar_clean_dilation: int = 1


class HyperspaceModule(Module):
    """Voxel-embedding map with text-query skills over SigLIP 2 patches."""

    config: HyperspaceModuleConfig

    patch_embeddings: In[PatchEmbeddings]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    tf: In[TFMessage]
    lidar: In[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.map: EmbeddingVoxelMap | None = None
        self._map_lock = threading.Lock()
        self._color_info: CameraInfo | None = None
        self._depth_info: CameraInfo | None = None
        self._frames_in = 0
        self._frames_used = 0
        self._points_inserted = 0
        self._last_skip_reason = ""
        #: (frame ts, voxel keys the frame contributed to) — provenance for
        #: mapping query matches back to source images.
        self._frame_log: list[tuple[float, NDArray[np.int64]]] = []
        self._ray_mapper: VoxelRayMapper | None = None
        #: The Rust mapper is single-borrow (RefCell): concurrent add_frame /
        #: voxel_count from the lidar and RPC threads raises, so serialize.
        self._ray_lock = threading.Lock()
        #: Clouds whose world tf hasn't arrived yet; drained on later arrivals.
        self._lidar_pending: deque[PointCloud2] = deque()
        #: Camera frames waiting for their world tf, same pattern as lidar.
        #: Drained from both the ingest and reduce threads, hence the lock.
        self._frame_pending: deque[tuple[PatchEmbeddings, Image]] = deque()
        self._frame_pending_lock = threading.Lock()
        self._lidar_frames = 0
        self._lidar_skipped = 0
        #: Timestamp of the newest processed frame — the map's "now". Replayed
        #: data carries recording-epoch stamps, so never use wall clock.
        self._last_ts: float | None = None
        model_kwargs: dict[str, Any] = {"model_name": self.config.model_name}
        if self.config.device is not None:
            model_kwargs["device"] = self.config.device
        self._text_model = SigLIP2Model(**model_kwargs)

    # ------------------------------------------------------------------ ingest

    def _ensure_map(self, dim: int) -> EmbeddingVoxelMap:
        with self._map_lock:
            if self.map is None:
                self.map = EmbeddingVoxelMap(
                    voxel_size=self.config.voxel_size_m,
                    dim=dim,
                    aggregate=self.config.aggregate,
                    max_samples=self.config.max_samples_per_voxel,
                )
            return self.map

    def _ingest(self, pair: tuple[PatchEmbeddings, ...]) -> None:
        patches = pair[0]
        depth = cast("Image | None", pair[1])  # None when no frame matched
        self._frames_in += 1
        if depth is None:
            self._last_skip_reason = "no depth frame within tolerance"
            return
        self._last_ts = patches.ts
        # The tf feed can trail the frame stream by seconds; frames whose
        # transform isn't buffered yet wait and retry instead of dropping.
        self._frame_pending.append((patches, depth))
        self._drain_pending_frames(now_ts=patches.ts)

    def _drain_pending_frames(self, now_ts: float) -> None:
        with self._frame_pending_lock:
            while self._frame_pending:
                patches, depth = self._frame_pending[0]
                outcome = self._try_process_frame(patches, depth)
                if outcome == "tf_not_ready":
                    if now_ts - patches.ts > _FRAME_PENDING_S:
                        self._frame_pending.popleft()
                        continue
                    return
                self._frame_pending.popleft()

    def _try_process_frame(self, patches: PatchEmbeddings, depth: Image) -> str:
        color_info, depth_info = self._color_info, self._depth_info
        if color_info is None or depth_info is None:
            self._last_skip_reason = "camera_info not yet received"
            return "tf_not_ready"

        ts = patches.ts
        tol = self.config.tf_tolerance_s
        color_from_depth = self.tfbuffer.get(color_info.frame_id, depth.frame_id, ts, tol)
        world_from_depth = self.tfbuffer.get(self.config.world_frame, depth.frame_id, ts, tol)
        if color_from_depth is None or world_from_depth is None:
            self._last_skip_reason = (
                f"tf unavailable ({color_info.frame_id}<-{depth.frame_id}: "
                f"{color_from_depth is not None}, "
                f"{self.config.world_frame}<-{depth.frame_id}: {world_from_depth is not None})"
            )
            return "tf_not_ready"

        points, patch_idx = project_patches_to_world(
            patches,
            depth,
            depth_info,
            color_info,
            color_from_depth.to_matrix(),
            world_from_depth.to_matrix(),
            stride=self.config.depth_stride,
            min_depth_m=self.config.min_depth_m,
            max_depth_m=self.config.max_depth_m,
        )
        if points.shape[0] == 0:
            self._last_skip_reason = "no valid depth points"
            return "skipped"

        flat = np.asarray(patches.to_numpy(), dtype=np.float32).reshape(-1, patches.dim)
        keys = self._ensure_map(patches.dim).insert_points(points, flat, patch_indices=patch_idx)
        self._frame_log.append((ts, keys))
        self._frames_used += 1
        self._points_inserted += int(points.shape[0])
        return "used"

    # ------------------------------------------------------------------ lidar

    def _on_lidar(self, cloud: PointCloud2) -> None:
        """Feed point-lio lidar clouds into the ray-traced clean map.

        The world tf for a cloud's timestamp usually arrives after the cloud
        itself, so clouds queue until their transform shows up; ones still
        unresolved after ``_LIDAR_PENDING_S`` of stream time are dropped.
        """
        mapper = self._ray_mapper
        if mapper is None:
            return
        self._lidar_pending.append(cloud)
        while self._lidar_pending:
            head = self._lidar_pending[0]
            world_from_lidar = self.tfbuffer.get(
                self.config.world_frame, head.frame_id, head.ts, self.config.tf_tolerance_s
            )
            if world_from_lidar is None:
                if cloud.ts - head.ts > _LIDAR_PENDING_S:
                    self._lidar_pending.popleft()
                    self._lidar_skipped += 1
                    continue
                return
            self._lidar_pending.popleft()
            matrix = world_from_lidar.to_matrix()
            points = head.points_f32() @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(
                np.float32
            )
            with self._ray_lock:
                mapper.add_frame(
                    points, (float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]))
                )
            self._lidar_frames += 1

    def _lidar_clean_mask(self, voxel_map: EmbeddingVoxelMap) -> NDArray[np.bool_] | None:
        """Which embedding voxels have solid lidar geometry nearby; None = no filter."""
        mapper = self._ray_mapper
        if mapper is None:
            return None
        with self._ray_lock:
            if mapper.voxel_count() == 0:
                return None
            centers = mapper.global_map()
        lidar_keys = self._pack_lidar_keys(centers)
        return keys_near_mask(
            voxel_map.voxel_keys(), lidar_keys, radius=self.config.lidar_clean_dilation
        )

    def _pack_lidar_keys(self, centers: NDArray[np.float32]) -> NDArray[np.int64]:
        idx = np.floor(centers / self.config.voxel_size_m).astype(np.int64) + KEY_OFFSET
        return np.unique(pack_indices(idx))

    def _lidar_voxel_count(self) -> int:
        if self._ray_mapper is None:
            return 0
        with self._ray_lock:
            return int(self._ray_mapper.voxel_count())

    # ------------------------------------------------------------------ queries

    def _require_map(self) -> EmbeddingVoxelMap:
        with self._map_lock:
            if self.map is None or self.map.reduce() == 0:
                raise RuntimeError("hyperspace map is empty — nothing ingested yet")
            return self.map

    def _query_scores(self, query: str) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """(centers (N, 3), scores (N,)) for a text query — see score_query().

        Smoothing runs on the full map (it needs complete neighborhoods),
        then voxels with no nearby lidar geometry are dropped.
        """
        voxel_map = self._require_map()
        centers, scores = score_query(
            voxel_map,
            self._text_model,
            query,
            background_prompts=self.config.background_prompts,
            smooth_iterations=self.config.score_smooth_iterations,
        )
        mask = self._lidar_clean_mask(voxel_map)
        if mask is not None:
            centers, scores = centers[mask], scores[mask]
        return centers, scores

    @rpc
    def save_map(self, path: str) -> str:
        """Persist the voxel map plus lidar clean-map keys and frame provenance.

        The extras let offline tools apply the same lidar filtering and map
        query matches back to the recording frames that produced them.
        """
        voxel_map = self._require_map()
        extras: dict[str, Any] = {}
        mapper = self._ray_mapper
        if mapper is not None:
            with self._ray_lock:
                centers = mapper.global_map() if mapper.voxel_count() else None
            if centers is not None:
                extras["lidar_keys"] = self._pack_lidar_keys(centers)
                extras["lidar_dilation"] = np.int64(self.config.lidar_clean_dilation)
        frame_log = list(self._frame_log)
        if frame_log:
            extras["frame_ts"] = np.array([ts for ts, _ in frame_log])
            extras["frame_keys"] = np.concatenate([keys for _, keys in frame_log])
            sizes = [keys.size for _, keys in frame_log]
            extras["frame_key_offsets"] = np.concatenate([[0], np.cumsum(sizes)])
        voxel_map.save(path, extras=extras)
        return path

    @rpc
    def stats(self) -> dict[str, Any]:
        """Ingestion counters and map size, for monitoring and tests."""
        with self._map_lock:
            voxel_count = self.map.reduce() if self.map is not None else 0
        tf_lag: dict[str, float] = {}
        if self._last_ts is not None and self._tf is not None:
            for key, buffer in self._tf.buffers.items():
                newest = buffer.last()
                if key[0] in (self.config.world_frame, "base_link") and newest is not None:
                    tf_lag[f"{key[0]}->{key[1]}"] = round(self._last_ts - newest.ts, 3)
        return {
            "frames_in": self._frames_in,
            "frames_used": self._frames_used,
            "points_inserted": self._points_inserted,
            "voxels": voxel_count,
            "lidar_frames": self._lidar_frames,
            "lidar_skipped": self._lidar_skipped,
            "lidar_voxels": self._lidar_voxel_count(),
            "last_skip_reason": self._last_skip_reason,
            "tf_lag_s": tf_lag,
        }

    @skill
    def query_heatmap(self, query: str, view: str = "top") -> Image:
        """Render a heatmap of how well each map voxel matches a text query.

        view is one of "top", "side", "front" or "isometric". Red voxels match
        the query strongly, gray voxels don't.
        """
        if view not in VIEWS:
            raise ValueError(f"view must be one of {VIEWS}, got {view!r}")
        centers, scores = self._query_scores(query)
        img = render_view(centers, scores, view=view, voxel_size=self.config.voxel_size_m)
        return Image(data=img, format=ImageFormat.RGB, frame_id=view, ts=time.time())

    @skill
    def query_clusters(self, query: str, top_k: int = 5) -> list[VoxelCluster]:
        """World-frame coordinates of the top clusters matching a text query.

        Returns up to top_k dicts with x, y, z (meters, world frame), score
        and voxel count, hottest first.
        """
        centers, scores = self._query_scores(query)
        return top_clusters(centers, scores, voxel_size=self.config.voxel_size_m, top_k=top_k)

    # ------------------------------------------------------------------ nearby

    def _robot_pose(self) -> tuple[NDArray[np.float64], float]:
        """Latest (position (3,), yaw) of the robot in the world frame."""
        if self._last_ts is None:
            raise RuntimeError("no frames processed yet — robot pose unknown")
        pose = self.tfbuffer.get(
            self.config.world_frame, self.config.robot_frame, self._last_ts, 60.0
        )
        if pose is None:
            raise RuntimeError(
                f"no {self.config.world_frame}<-{self.config.robot_frame} transform available"
            )
        matrix = pose.to_matrix()
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return matrix[:3, 3], yaw

    def _nearby_scores(
        self, query: str
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float64], float]:
        centers, scores = self._query_scores(query)
        position, yaw = self._robot_pose()
        dist = np.hypot(centers[:, 0] - position[0], centers[:, 1] - position[1])
        inside = (dist <= self.config.nearby_radius_m) & (dist > 0.3)
        return centers[inside], scores[inside], position, yaw

    @skill
    def nearby(self, query: str) -> dict[str, Any]:
        """Which way to turn toward the best match for a query within 3 m.

        Scores the voxels inside a cylinder around the robot, bins them into
        5-degree azimuth sectors relative to the robot's heading, and reports
        the turn toward the best sector: e.g. relative_deg=180 means the match
        is behind the robot.
        """
        centers, scores, position, yaw = self._nearby_scores(query)
        if centers.shape[0] == 0:
            raise RuntimeError(f"no voxels within {self.config.nearby_radius_m} m of the robot")

        azimuth = np.arctan2(centers[:, 1] - position[1], centers[:, 0] - position[0])
        relative = np.degrees(azimuth - yaw)
        relative = (relative + 180.0) % 360.0 - 180.0
        bin_deg = self.config.nearby_bin_deg
        bins = np.floor((relative + 180.0) / bin_deg).astype(np.int64)
        n_bins = round(360.0 / bin_deg)
        bins = np.clip(bins, 0, n_bins - 1)
        bin_scores = np.full(n_bins, -np.inf, dtype=np.float32)
        np.maximum.at(bin_scores, bins, scores)

        best_bin = int(np.argmax(bin_scores))
        relative_deg = float(best_bin * bin_deg - 180.0 + bin_deg / 2)
        if abs(relative_deg) <= bin_deg:
            direction = "ahead"
        elif abs(relative_deg) >= 180.0 - bin_deg:
            direction = "behind"
        elif relative_deg > 0:
            direction = f"turn left {abs(relative_deg):.0f} degrees"
        else:
            direction = f"turn right {abs(relative_deg):.0f} degrees"
        return {
            "relative_deg": relative_deg,
            "direction": direction,
            "best_score": float(bin_scores[best_bin]),
            "voxels_considered": int(centers.shape[0]),
            "robot_xy": [float(position[0]), float(position[1])],
            "robot_yaw_deg": math.degrees(yaw),
        }

    @skill
    def nearby_heatmap(self, query: str) -> Image:
        """Top-down heatmap of the 3 m query cylinder with a heading arrow."""
        heading = self.nearby(query)
        centers, scores, position, yaw = self._nearby_scores(query)
        img = render_nearby(
            centers,
            scores,
            robot_xy=(float(position[0]), float(position[1])),
            robot_yaw_rad=yaw,
            heading_rad=yaw + math.radians(heading["relative_deg"]),
            radius_m=self.config.nearby_radius_m,
            voxel_size=self.config.voxel_size_m,
        )
        return Image(data=img, format=ImageFormat.RGB, frame_id="nearby", ts=time.time())

    # ------------------------------------------------------------------ rerun

    @rpc
    def log_query_rerun(self, query: str, save_path: str | None = None) -> str:
        """Log the scored voxel cloud for a query to Rerun.

        With save_path, writes a standalone .rrd containing one frame of the
        heatmap voxel cloud and returns the path; otherwise logs to the
        default active Rerun recording.
        """
        import rerun as rr

        centers, scores = self._query_scores(query)
        colors = scores_to_colors(normalize_scores(scores))
        points = rr.Points3D(positions=centers, colors=colors, radii=self.config.voxel_size_m / 2)
        entity = f"{self.config.rerun_entity}/{query.replace(' ', '_')}"
        if save_path:
            recording = rr.RecordingStream(application_id="hyperspace")
            recording.save(save_path)
            recording.log(entity, points, static=True)
            recording.flush()
            return save_path
        rr.log(entity, points, static=True)
        return entity

    # ------------------------------------------------------------------ lifecycle

    @rpc
    def start(self) -> None:
        super().start()

        # Deep TF history: lookups happen at frame timestamps that trail the
        # live tf feed, and the default 10 s window prunes past them.
        self._tf = TF(self.tf, buffer_size=self.config.tf_buffer_s)

        self.register_disposable(
            Disposable(self.camera_info.subscribe(lambda info: setattr(self, "_color_info", info)))
        )
        self.register_disposable(
            Disposable(
                self.depth_camera_info.subscribe(lambda info: setattr(self, "_depth_info", info))
            )
        )

        if self.config.use_lidar_clean_map:
            self._ray_mapper = VoxelRayMapper(
                voxel_size=self.config.voxel_size_m,
                max_range=self.config.lidar_max_range_m,
            )
            self.register_disposable(
                backpressure(self.lidar.pure_observable()).subscribe(self._on_lidar)
            )

        paired = align_timestamped(
            backpressure(self.patch_embeddings.pure_observable()),
            self.depth_image.pure_observable(),
            buffer_size=20.0,
            match_tolerance=self.config.depth_match_tolerance_s,
        )
        self.register_disposable(paired.subscribe(self._ingest))

        def _periodic_reduce(_: Any) -> None:
            # Retry frames that were waiting on tf when the stream went quiet.
            if self._last_ts is not None:
                self._drain_pending_frames(now_ts=self._last_ts)
            with self._map_lock:
                if self.map is not None:
                    voxels = self.map.reduce()
                    logger.info(
                        "hyperspace: %d voxels, %d/%d frames used, %d points",
                        voxels,
                        self._frames_used,
                        self._frames_in,
                        self._points_inserted,
                    )

        self.register_disposable(
            rx.interval(self.config.reduce_interval_s).subscribe(_periodic_reduce)
        )

    @rpc
    def stop(self) -> None:
        self._text_model.stop()
        super().stop()
