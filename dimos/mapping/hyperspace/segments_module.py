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

"""``HyperspaceSegments``: the noun channel of hyperspace as a memory writer.

Listens to the same camera ports as ``HyperspacePatches``. Every
``min_frame_interval_s`` it labels the frame with a segmenter (SegFormer on
ADE20K by default), keeps structural labels (wall, floor, ceiling, door,
stairs) only where the depth is flat, and writes one ``hyperspace_segments``
record per segment with the label's SigLIP2 text embedding. It publishes the
overlay, the flat mask and the labelled points of the current frame so the
result can be watched live in Rerun.
"""

from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.hyperspace import segmenter as seg
from dimos.mapping.hyperspace.embedder import SIGLIP2_MODEL_NAME, SigLIP2Patches
from dimos.mapping.hyperspace.ingest import transform_to_matrix
from dimos.mapping.hyperspace.module import open_store_with_retry, pick_device
from dimos.mapping.hyperspace.segments import SegmentedFrame, SegmentIngestConfig, SegmentIngestor
from dimos.memory.module import MemoryModule, MemoryModuleConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class HyperspaceSegmentsConfig(MemoryModuleConfig):
    """Which segmenter, how often, and how strict the flatness gate is."""

    segmenter_name: str = seg.SEGFORMER_MODEL_NAME
    # SigLIP2 text tower for the per-segment label embedding; "" = no embedding.
    model_name: str = SIGLIP2_MODEL_NAME
    device: str = "auto"
    world_frame: str = "odom"
    min_frame_interval_s: float = 0.5
    max_depth_m: float = 10.0
    depth_max_dt: float = 0.05
    map_stride: int = 6
    min_area: int = 200
    # Flat-plane filter: plane residual limits (m) over the two window sizes (px).
    flat_rms_max: float = 0.015
    flat_max_deviation: float = 0.04
    flat_windows: tuple[int, int] = (9, 21)
    # Both limits grow with depth squared at this rate (stereo noise); 0 = fixed.
    flat_noise_per_m2: float = 0.003


class HyperspaceSegments(MemoryModule):
    """Labels frames, keeps structural labels only on flat depth, writes segments to memory."""

    config: HyperspaceSegmentsConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    tf: In[TFMessage]

    segment_overlay: Out[Image]
    flat_mask: Out[Image]
    semantic_map: Out[PointCloud2]

    ingestor: SegmentIngestor | None = None

    @rpc
    def start(self) -> None:
        # Never "mps" from "auto": Metal asserts inside forkserver workers on macOS.
        device = pick_device(self.config.device, allow_mps=False)
        logger.info(f"hyperspace segments: loading {self.config.segmenter_name} on {device}")
        segmenter = seg.SegFormerSegmenter(
            seg.SegmenterConfig(
                model_name=self.config.segmenter_name,
                device=device,
                flatness=seg.FlatnessConfig(
                    windows=tuple(self.config.flat_windows),
                    rms_max=self.config.flat_rms_max,
                    max_deviation=self.config.flat_max_deviation,
                    noise_per_m2=self.config.flat_noise_per_m2,
                ),
                min_area=self.config.min_area,
            )
        )
        embed_text = None
        if self.config.model_name:
            self.model = self.register_disposable(
                SigLIP2Patches(model_name=self.config.model_name, device="cpu", towers="text")
            )
            self.model.start()
            embed_text = lambda text: self.model.embed_text_array(text)[0]  # noqa: E731
        open_store_with_retry(self)
        self.ingestor = SegmentIngestor(
            self.store,
            segmenter,
            SegmentIngestConfig(
                min_frame_interval_s=self.config.min_frame_interval_s,
                max_depth_m=self.config.max_depth_m,
                depth_max_dt=self.config.depth_max_dt,
                map_stride=self.config.map_stride,
            ),
            embed_text=embed_text,
            lookup=self._lookup,
        )
        logger.info(f"hyperspace segments: ready, db {self.config.db_path}")
        super().start()

    def _lookup(self, target: str, source: str, ts: float) -> NDArray[np.float64] | None:
        try:
            transform = self.tfbuffer.get(target, source, ts, warn=False)
        except (TypeError, RuntimeError):
            return None
        return None if transform is None else transform_to_matrix(transform)

    async def handle_camera_info(self, info: CameraInfo) -> None:
        if self.ingestor is not None:
            self.ingestor.add_camera_info(info)

    async def handle_depth_camera_info(self, info: CameraInfo) -> None:
        if self.ingestor is not None:
            self.ingestor.add_camera_info(info)

    async def handle_tf(self, msg: TFMessage) -> None:
        if self.ingestor is not None:
            self.ingestor.add_tf(msg)

    async def handle_depth_image(self, image: Image) -> None:
        if self.ingestor is not None:
            self.ingestor.add_depth(image)

    async def handle_color_image(self, image: Image) -> None:
        if self.ingestor is None:
            return
        # SegFormer-B2 takes ~2.5 s per frame on a CPU; latest-only dispatch
        # drops the frames that arrive meanwhile.
        frame = await asyncio.get_running_loop().run_in_executor(
            None, self.ingestor.add_image, image
        )
        if frame is not None:
            self.publish(frame)

    def publish(self, frame: SegmentedFrame) -> None:
        self.segment_overlay.publish(
            Image.from_numpy(
                frame.overlay, format=ImageFormat.RGB, frame_id=frame.camera_frame, ts=frame.ts
            )
        )
        self.flat_mask.publish(
            Image.from_numpy(
                (frame.result.flat * 255).astype(np.uint8),
                format=ImageFormat.GRAY,
                frame_id=frame.camera_frame,
                ts=frame.ts,
            )
        )
        points = (
            self.ingestor.semantic_points(frame, self.config.world_frame) if self.ingestor else None
        )
        if points is not None:
            xyz, labels = points
            self.semantic_map.publish(
                PointCloud2.from_numpy(
                    xyz, frame_id=self.config.world_frame, timestamp=time.time(), intensities=labels
                )
            )

    @rpc
    def ingest_stats(self) -> dict[str, int]:
        return dict(self.ingestor.stats) if self.ingestor is not None else {}

    @rpc
    def class_names(self) -> dict[int, str]:
        return dict(self.ingestor.segmenter.names) if self.ingestor is not None else {}
