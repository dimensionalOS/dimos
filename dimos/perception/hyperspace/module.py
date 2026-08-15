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
from dimos.models.embedding.base import PatchEmbeddings
from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
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
from dimos.perception.hyperspace.voxel_map import EmbeddingVoxelMap
from dimos.protocol.tf.tf import TF
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


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


class HyperspaceModule(Module):
    """Voxel-embedding map with text-query skills over SigLIP 2 patches."""

    config: HyperspaceModuleConfig

    patch_embeddings: In[PatchEmbeddings]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    tf: In[TFMessage]

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
                self.map = EmbeddingVoxelMap(voxel_size=self.config.voxel_size_m, dim=dim)
            return self.map

    def _ingest(self, pair: tuple[PatchEmbeddings, ...]) -> None:
        patches = pair[0]
        depth = cast("Image | None", pair[1])  # None when no frame matched
        self._frames_in += 1
        if depth is None:
            self._last_skip_reason = "no depth frame within tolerance"
            return
        color_info, depth_info = self._color_info, self._depth_info
        if color_info is None or depth_info is None:
            self._last_skip_reason = "camera_info not yet received"
            return

        ts = patches.ts
        self._last_ts = ts
        tol = self.config.tf_tolerance_s
        color_from_depth = self.tfbuffer.get(color_info.frame_id, depth.frame_id, ts, tol)
        world_from_depth = self.tfbuffer.get(self.config.world_frame, depth.frame_id, ts, tol)
        if color_from_depth is None or world_from_depth is None:
            self._last_skip_reason = (
                f"tf unavailable ({color_info.frame_id}<-{depth.frame_id}: "
                f"{color_from_depth is not None}, "
                f"{self.config.world_frame}<-{depth.frame_id}: {world_from_depth is not None})"
            )
            return

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
            return

        flat = np.asarray(patches.to_numpy(), dtype=np.float32).reshape(-1, patches.dim)
        self._ensure_map(patches.dim).insert_points(points, flat, patch_indices=patch_idx)
        self._frames_used += 1
        self._points_inserted += int(points.shape[0])

    # ------------------------------------------------------------------ queries

    def _require_map(self) -> EmbeddingVoxelMap:
        with self._map_lock:
            if self.map is None or self.map.reduce() == 0:
                raise RuntimeError("hyperspace map is empty — nothing ingested yet")
            return self.map

    def _query_scores(self, query: str) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """(centers (N, 3), scores (N,)) for a text query — see score_query()."""
        return score_query(
            self._require_map(),
            self._text_model,
            query,
            background_prompts=self.config.background_prompts,
            smooth_iterations=self.config.score_smooth_iterations,
        )

    @rpc
    def save_map(self, path: str) -> str:
        """Persist the reduced voxel map to an .npz for offline queries."""
        self._require_map().save(path)
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

        paired = align_timestamped(
            backpressure(self.patch_embeddings.pure_observable()),
            self.depth_image.pure_observable(),
            buffer_size=20.0,
            match_tolerance=self.config.depth_match_tolerance_s,
        )
        self.register_disposable(paired.subscribe(self._ingest))

        def _periodic_reduce(_: Any) -> None:
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
