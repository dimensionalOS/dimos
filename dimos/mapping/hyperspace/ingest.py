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

"""Frames in, keyframes and patch embeddings out, into a memory store.

Shared by the live ``HyperspacePatches`` module and the offline CLI so both
keep exactly the same frames and write exactly the same records.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.models.embedding.base import Embedding
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

    from dimos.mapping.hyperspace.embedder import SigLIP2Patches
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

logger = setup_logger()

KEYFRAME_STREAM = "hyperspace_keyframes"
PATCH_STREAM = "hyperspace_patches"
TF_STREAM = "tf"

# 4x4 target_from_source at a time, or None when the lookup fails.
TransformLookup = "Callable[[str, str, float], NDArray[np.float64] | None]"


def intrinsics_of(info: CameraInfo) -> hs.Intrinsics:
    matrix = np.asarray(info.get_K_matrix(), dtype=float).reshape(3, 3)
    return hs.Intrinsics(
        width=int(info.width),
        height=int(info.height),
        fx=float(matrix[0, 0]),
        fy=float(matrix[1, 1]),
        cx=float(matrix[0, 2]),
        cy=float(matrix[1, 2]),
    )


def transform_to_matrix(transform: Any) -> NDArray[np.float64]:
    t, q = transform.translation, transform.rotation
    return hs.transform_matrix(np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w]))


@dataclass
class IngestConfig:
    gate: hs.KeyframeGateConfig
    motion_reference_frame: str = "odom"
    # Never embed frames closer together than this (s). 0.2 = 5 Hz.
    min_frame_interval_s: float = 0.2
    # Depth beyond this (m) is a hole: RealSense 65535 mm sentinels and glitches.
    max_depth_m: float = 10.0
    depth_max_dt: float = 0.05
    depth_history: int = 64
    depth_thumbnail_stride: int = 4
    # The keyframe's cell grid: what patch_depth is measured on and what the
    # query pools the members' scores onto. None = the model's own grid for a
    # single fixed-resolution checkpoint (the original layout), else 24x24.
    cell_grid: tuple[int, int] | None = None
    # Fill the depth holes from the colour frame before measuring patch depth
    # (dimos.perception.depth2depth). Stereo returns nothing off glass, shiny
    # floors and dark shelves, and reads *through* a freezer door -- so a patch
    # in front of one is placed metres too far. "" = raw sensor depth only.
    depth2depth_model: str = ""


def grids_of(model: Any, image: Image) -> list[tuple[NDArray[np.float32], tuple[int, int]]]:
    """One ``(grid, (rows, cols))`` per ensemble member, from an ensemble, a
    single SigLIP2Patches, or any object with ``embed_patches`` and
    ``patches_per_side`` (the test stubs)."""
    if hasattr(model, "embed_grids"):
        grids = model.embed_grids(image)
        return grids if isinstance(grids, list) and grids and isinstance(grids[0], tuple) else grids
    side = int(model.patches_per_side)
    return [(model.embed_patches(image)[0], (side, side))]


class PatchIngestor:
    """Gate frames, embed the survivors, pair them with depth, write keyframes
    and one vector per patch into ``store``."""

    def __init__(
        self,
        store: Store,
        model: SigLIP2Patches,
        config: IngestConfig,
        lookup: Callable[[str, str, float], NDArray[np.float64] | None] | None = None,
    ) -> None:
        self.store = store
        self.model = model
        self.config = config
        # Ensemble bookkeeping, written with every keyframe so the query side
        # can load the matching text towers: short tags and the full specs.
        self.members: list[str] = list(getattr(model, "tags", []))
        self.member_specs: list[str] = list(getattr(model, "specs", []))
        # target_from_source(target_frame, source_frame, ts) for the motion gate
        # and depth-to-colour alignment. None = no tf available (both skipped).
        self.lookup = lookup
        self.buffer = hs.RollingBuffer(config.gate)
        self.intrinsics: dict[str, hs.Intrinsics] = {}
        self.depths: deque[tuple[float, str, NDArray[np.float32]]] = deque(
            maxlen=config.depth_history
        )
        # Built on the first colour frame: loading the model costs seconds and
        # an ingest without depth2depth should never pay for it.
        self.fuser: Any = None
        self.keyframes: Stream[Any] = store.stream(KEYFRAME_STREAM, dict)
        self.patches: Stream[Any] = store.stream(PATCH_STREAM, dict)
        self.tf_stream: Stream[TFMessage] = store.stream(TF_STREAM, TFMessage)
        self.last_embedded = -np.inf
        self.stats = {"images": 0, "gated": 0, "embedded": 0, "kept": 0, "kept_without_depth": 0}

    def add_camera_info(self, info: CameraInfo) -> None:
        self.intrinsics[info.frame_id] = intrinsics_of(info)

    def add_tf(self, msg: TFMessage, ts: float | None = None) -> None:
        """Record tf so the query side can place keyframes at query time."""
        stamps = [float(t.ts) for t in msg.transforms if getattr(t, "ts", None)]
        self.tf_stream.append(
            msg, ts=ts if ts is not None else (max(stamps) if stamps else time.time())
        )

    def add_depth(self, image: Image) -> None:
        depth = np.asarray(image.as_numpy())
        metres = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
        metres[(metres > self.config.max_depth_m) | ~np.isfinite(metres)] = 0.0
        self.depths.append((float(image.ts), image.frame_id, metres))

    def _speeds(self, camera_frame: str, ts: float) -> tuple[float, float] | None:
        gate = self.config.gate
        if self.lookup is None or (
            gate.max_angular_velocity is None and gate.max_linear_velocity is None
        ):
            return None
        before = self.lookup(self.config.motion_reference_frame, camera_frame, ts - 0.05)
        after = self.lookup(self.config.motion_reference_frame, camera_frame, ts + 0.05)
        if before is None or after is None:
            return None
        delta = np.linalg.inv(before) @ after
        angle = float(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1.0, 1.0)))
        return angle / 0.1, float(np.linalg.norm(delta[:3, 3])) / 0.1

    def add_image(self, image: Image) -> bool:
        """Returns True when this frame produced a keyframe."""
        self.stats["images"] += 1
        if self.stats["images"] in (1, 50) or self.stats["images"] % 250 == 0:
            logger.info(f"hyperspace ingest: {self.stats}")
        ts = float(image.ts)
        if ts - self.last_embedded < self.config.min_frame_interval_s:
            return False
        rgb = np.asarray(image.to_rgb().data)
        speeds = self._speeds(image.frame_id, ts)
        if hs.quality_gate(self.config.gate, rgb, speeds) is not None:
            self.stats["gated"] += 1
            return False
        started = time.monotonic()
        grids = grids_of(self.model, image)
        grid = grids[0][0]
        if self.stats["embedded"] == 0:
            logger.info(f"hyperspace ingest: first embed took {time.monotonic() - started:.2f}s")
        self.last_embedded = ts
        self.stats["embedded"] += 1
        quality = 1.0 if speeds is None else 1.0 / (1.0 + speeds[0] + 0.25 * speeds[1])
        # Pair depth now, while its frame is still in the short depth history:
        # the buffer judges this frame ~5 embedded frames later.
        depth = self._paired_depth(image.frame_id, ts)
        if depth is not None and self.config.depth2depth_model:
            depth = self._fused_depth(rgb, depth)
        kept = self.buffer.push(
            hs.BufferedFrame(
                ts=ts,
                grid=grid.astype(np.float16),
                quality=quality,
                payload=(image.frame_id, depth, grids),
            )
        )
        if kept is None:
            return False
        self._write_keyframe(kept)
        return True

    def flush(self) -> int:
        kept = self.buffer.flush()
        for frame in kept:
            self._write_keyframe(frame)
        return len(kept)

    def _fused_depth(
        self, rgb: NDArray[np.uint8], depth: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        """Depth with its holes filled from the colour frame. Shapes must match:
        the pairing has already put the depth on the colour camera's grid."""
        from dimos.perception.depth2depth.fusion import Depth2Depth, FuseConfig

        if self.fuser is None:
            self.fuser = Depth2Depth(
                config=FuseConfig(far_m=self.config.max_depth_m),
                model_name=self.config.depth2depth_model,
            )
            self.fuser.start()
            logger.info(f"hyperspace ingest: depth2depth on {self.fuser.device}")
        if rgb.shape[:2] != depth.shape[:2]:
            return depth
        return self.fuser.fuse(rgb, depth).fused

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

    def cell_grid(
        self, grids: list[tuple[NDArray[np.float32], tuple[int, int]]]
    ) -> tuple[int, int]:
        if self.config.cell_grid is not None:
            return self.config.cell_grid
        # One fixed grid: keep the model's own layout, so the vector index's
        # patch ids and the keyframe cells are the same thing.
        return grids[0][1] if len(grids) == 1 else (24, 24)

    def _write_keyframe(self, kept: hs.BufferedFrame) -> None:
        camera_frame, depth, grids = kept.payload
        color = self.intrinsics.get(camera_frame)
        if color is None:
            logger.warning(f"hyperspace: no camera_info for {camera_frame!r} yet; keyframe dropped")
            return
        rows, cols = self.cell_grid(grids)
        if depth is None:
            self.stats["kept_without_depth"] += 1
            patch_depth = np.full(rows * cols, np.nan, dtype=np.float32)
            thumbnail = np.zeros((0, 0), dtype=np.uint16)
        else:
            patch_depth = hs.per_patch_depth(depth, rows, cols)
            stride = max(self.config.depth_thumbnail_stride, 1)
            thumbnail = np.clip(depth[::stride, ::stride] * 1000.0, 0, 65535).astype(np.uint16)
        payload = {
            "camera_frame": camera_frame,
            "ts": kept.ts,
            "rows": rows,
            "cols": cols,
            "intrinsics": vars(color),
            # The primary member's grid, as before; ``grids`` carries every
            # member (primary first) with its own shape when there is more
            # than one, or when the one grid is not the cell grid.
            "grid": kept.grid,
            "patch_depth": patch_depth,
            "thumbnail_mm": thumbnail,
            "thumbnail_stride": self.config.depth_thumbnail_stride,
        }
        shapes = [shape for _, shape in grids]
        if self.member_specs:
            payload["members"] = self.members
            payload["member_specs"] = self.member_specs
        if len(grids) > 1 or shapes[0] != (rows, cols):
            payload["grids"] = [grid.astype(np.float16) for grid, _ in grids]
            payload["grid_shapes"] = [list(shape) for shape in shapes]
            payload.setdefault("members", [f"member{i}" for i in range(len(grids))])
        keyframe = self.keyframes.append(payload, ts=kept.ts, tags={"camera_frame": camera_frame})
        # The vector index holds the primary member only, indexed on its own
        # grid (``grid_shapes[0]``): a nearest-neighbour hook for single-grid
        # tools, not what the ensemble query reads.
        for index in range(len(kept.grid)):
            self.patches.append(
                {"keyframe": keyframe.id, "patch": index},
                ts=kept.ts,
                tags={"keyframe": keyframe.id},
                embedding=Embedding(vector=kept.grid[index].astype(np.float32), timestamp=kept.ts),
            )
        self.stats["kept"] += 1
        logger.info(
            f"hyperspace keyframe {keyframe.id} at {kept.ts:.2f} "
            f"({self.stats['kept']} kept of {self.stats['embedded']} embedded, "
            f"{self.stats['images']} seen)"
        )
