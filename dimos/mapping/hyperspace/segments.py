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

"""Frames in, labelled segments out, into a memory store.

Shared by the live ``HyperspaceSegments`` module and the offline CLI. Each
segment becomes one ``hyperspace_segments`` record carrying its label, mask,
flatness and the label's SigLIP2 text embedding, so a query can match the
segment by meaning without the segmenter's vocabulary getting in the way.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs, segmenter as seg
from dimos.mapping.hyperspace.ingest import TF_STREAM, intrinsics_of
from dimos.models.embedding.base import Embedding
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

logger = setup_logger()

SEGMENT_STREAM = "hyperspace_segments"


@dataclass
class SegmentIngestConfig:
    # Never segment frames closer together than this (s).
    min_frame_interval_s: float = 0.5
    # Depth beyond this (m) is a hole: RealSense 65535 mm sentinels and glitches.
    max_depth_m: float = 10.0
    depth_max_dt: float = 0.05
    depth_history: int = 64
    # Stride of the labelled points published as the semantic map.
    map_stride: int = 6


@dataclass
class SegmentedFrame:
    ts: float
    camera_frame: str
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32] | None
    result: seg.FrameSegments
    overlay: NDArray[np.uint8]


class SegmentIngestor:
    """Segment frames, gate the structural classes by flatness, write one
    record per segment into ``store``."""

    def __init__(
        self,
        store: Store | None,
        segmenter: seg.SegFormerSegmenter,
        config: SegmentIngestConfig,
        embed_text: Callable[[str], NDArray[np.float32]] | None = None,
        lookup: Callable[[str, str, float], NDArray[np.float64] | None] | None = None,
    ) -> None:
        self.store = store
        self.segmenter = segmenter
        self.config = config
        self.embed_text = embed_text
        self.lookup = lookup
        self.intrinsics: dict[str, hs.Intrinsics] = {}
        self.depths: deque[tuple[float, str, NDArray[np.float32]]] = deque(
            maxlen=config.depth_history
        )
        self.segments: Stream[Any] | None = store.stream(SEGMENT_STREAM, dict) if store else None
        self.tf_stream: Stream[TFMessage] | None = (
            store.stream(TF_STREAM, TFMessage) if store else None
        )
        self.text_embeddings: dict[str, NDArray[np.float32]] = {}
        self.last_segmented = -np.inf
        self.stats = {"images": 0, "segmented": 0, "segments": 0, "without_depth": 0}

    def add_camera_info(self, info: CameraInfo) -> None:
        self.intrinsics[info.frame_id] = intrinsics_of(info)

    def add_tf(self, msg: TFMessage, ts: float | None = None) -> None:
        if self.tf_stream is None:
            return
        stamps = [float(t.ts) for t in msg.transforms if getattr(t, "ts", None)]
        self.tf_stream.append(
            msg, ts=ts if ts is not None else (max(stamps) if stamps else time.time())
        )

    def add_depth(self, image: Image) -> None:
        depth = np.asarray(image.as_numpy())
        metres = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
        metres[(metres > self.config.max_depth_m) | ~np.isfinite(metres)] = 0.0
        self.depths.append((float(image.ts), image.frame_id, metres))

    def add_image(self, image: Image) -> SegmentedFrame | None:
        """Segment the frame if it is due. Returns what was found, or None if skipped."""
        self.stats["images"] += 1
        ts = float(image.ts)
        if ts - self.last_segmented < self.config.min_frame_interval_s:
            return None
        self.last_segmented = ts
        rgb = seg.image_rgb(image)
        depth = self._paired_depth(image.frame_id, ts)
        camera = self.intrinsics.get(image.frame_id)
        if depth is None:
            self.stats["without_depth"] += 1
        started = time.monotonic()
        result = seg.segment_frame(self.segmenter, rgb, depth, camera)
        if self.stats["segmented"] == 0:
            logger.info(f"hyperspace segments: first frame took {time.monotonic() - started:.2f}s")
        self.stats["segmented"] += 1
        self.stats["segments"] += len(result.segments)
        frame = SegmentedFrame(
            ts=ts,
            camera_frame=image.frame_id,
            rgb=rgb,
            depth=depth,
            result=result,
            overlay=seg.overlay(rgb, result.labels, self.segmenter.colors),
        )
        self._write(frame)
        if self.stats["segmented"] in (1, 10) or self.stats["segmented"] % 50 == 0:
            names = [s.name for s in result.segments[:6]]
            logger.info(
                f"hyperspace segments: {self.stats}, demoted {result.demoted_fraction:.0%}, top {names}"
            )
        return frame

    def semantic_points(
        self, frame: SegmentedFrame, world_frame: str
    ) -> tuple[NDArray[np.float32], NDArray[np.float32]] | None:
        """Labelled depth points of the frame in ``world_frame``: (xyz, label)."""
        camera = self.intrinsics.get(frame.camera_frame)
        if frame.depth is None or camera is None or self.lookup is None:
            return None
        world_from_camera = self.lookup(world_frame, frame.camera_frame, frame.ts)
        if world_from_camera is None:
            return None
        stride = max(self.config.map_stride, 1)
        strided = hs.Intrinsics(
            width=camera.width // stride,
            height=camera.height // stride,
            fx=camera.fx / stride,
            fy=camera.fy / stride,
            cx=camera.cx / stride,
            cy=camera.cy / stride,
        )
        points = seg.points_from_depth(frame.depth[::stride, ::stride], strided)
        labels = frame.result.labels[::stride, ::stride]
        keep = np.isfinite(points[..., 2]) & (labels != seg.UNSURE)
        xyz = points[keep]
        homogeneous = np.concatenate([xyz, np.ones((len(xyz), 1))], axis=1)
        world = (world_from_camera @ homogeneous.T).T[:, :3]
        return world.astype(np.float32), labels[keep].astype(np.float32)

    def _paired_depth(self, camera_frame: str, ts: float) -> NDArray[np.float32] | None:
        best = None
        for depth_ts, depth_frame, metres in self.depths:
            dt = abs(depth_ts - ts)
            if dt <= self.config.depth_max_dt and (best is None or dt < best[0]):
                best = (dt, depth_frame, metres)
        if best is None:
            return None
        _, depth_frame, metres = best
        color = self.intrinsics.get(camera_frame)
        depth_intrinsics = self.intrinsics.get(depth_frame)
        if color is None or depth_intrinsics is None:
            return None
        if depth_frame == camera_frame:
            return metres
        if self.lookup is None:
            return None
        color_from_depth = self.lookup(camera_frame, depth_frame, ts)
        if color_from_depth is None:
            return None
        return hs.reproject_depth(metres, depth_intrinsics, color, color_from_depth)

    def _text_embedding(self, name: str) -> Embedding | None:
        if self.embed_text is None:
            return None
        vector = self.text_embeddings.get(name)
        if vector is None:
            vector = self.text_embeddings[name] = np.asarray(
                self.embed_text(name), dtype=np.float32
            )
        return Embedding(vector=vector)

    def _write(self, frame: SegmentedFrame) -> None:
        if self.segments is None:
            return
        height, width = frame.result.labels.shape
        for segment in frame.result.segments:
            self.segments.append(
                seg.segment_record(
                    segment,
                    camera_frame=frame.camera_frame,
                    ts=frame.ts,
                    width=width,
                    height=height,
                ),
                ts=frame.ts,
                tags={"camera_frame": frame.camera_frame, "name": segment.name},
                embedding=self._text_embedding(segment.name),
            )
