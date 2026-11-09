# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Chunked mesh rendering shared by live viewers and recording replay."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.message_utils import mesh as mesh_utils
from dimos.messages.dimos_msgs.msg import TriangleMesh
from dimos.visualization.rerun.bridge import RerunEntry

Key = tuple[int, int, int]

MESH_ENTITY = "world/mesh"


class MeshColours:
    """Rerun entries for each mesh chunk, turbo by height over the mesh's own range.

    Once the range moves ``recolour_m``, every chunk is recoloured (colours only).
    ``z_range`` pins it. Works as a keyed rerun bridge renderer and in replays.
    """

    @staticmethod
    def message_key(message: TriangleMesh) -> Key:
        return int(message.key[0]), int(message.key[1]), int(message.key[2])

    def __init__(
        self,
        alpha: float = 0.4,
        z_range: tuple[float, float] | None = None,
        recolour_m: float = 0.5,
        static: bool = True,
    ) -> None:
        self.alpha = alpha
        self.z_range = z_range
        self.recolour_m = recolour_m
        self.static = static
        self._heights: dict[Key, np.ndarray] = {}
        self._range: tuple[float, float] | None = None

    def __call__(self, msg: TriangleMesh) -> list[RerunEntry]:
        out = [
            RerunEntry(
                _chunk_path((int(msg.key[0]), int(msg.key[1]), int(msg.key[2]))),
                mesh_utils.to_rerun(msg),
                self.static,
            )
        ]
        if len(msg.faces) == 0:
            self._heights.pop((int(msg.key[0]), int(msg.key[1]), int(msg.key[2])), None)
            return out
        self._heights[(int(msg.key[0]), int(msg.key[1]), int(msg.key[2]))] = msg.vertices.reshape(
            -1, 3
        )[:, 2].copy()
        lo, hi = self.z_range or np.percentile(
            np.concatenate(list(self._heights.values())), [2, 98]
        )
        keys = [(int(msg.key[0]), int(msg.key[1]), int(msg.key[2]))]
        if (
            self._range is None
            or max(abs(lo - self._range[0]), abs(hi - self._range[1])) > self.recolour_m
        ):
            self._range = (float(lo), float(hi))
            keys = list(self._heights)
        return out + [
            RerunEntry(_chunk_path(k), self._colours(self._heights[k]), self.static) for k in keys
        ]

    def _colours(self, z: np.ndarray) -> Any:
        import matplotlib
        import rerun as rr

        assert self._range is not None
        lo, hi = self._range
        t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
        colors = (matplotlib.colormaps["turbo"](t)[:, :3] * 255).astype(np.uint8)
        alpha = int(self.alpha * 255)
        return rr.Mesh3D.from_fields(vertex_colors=colors, albedo_factor=[255, 255, 255, alpha])


def _chunk_path(key: Key) -> str:
    return f"{MESH_ENTITY}/{key[0]}_{key[1]}_{key[2]}"
