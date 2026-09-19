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

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import _json, reference
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import draw_overlays
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


def depth_data(node: Any, ctx: EncodeContext) -> tuple[render.DepthRaster, EncodeContext]:
    selected = ctx.select(node.source)
    key = ("pick_depth", _json(reference(node, ctx)))
    if key not in ctx.cache:
        ctx.cache[key] = render.depth_raster(
            selected.points,
            render.as_view(node.view),
            fov_deg=node.fov_deg,
            size=node.size,
            max_depth=node.max_depth,
            point_size_m=node.point_size_m,
        )
    return ctx.cache[key], selected


@dataclass(frozen=True)
class DepthView:
    """A perspective depth render of the cloud from ``view``, in this build's
    form (image or text)."""

    view: tuple[float, float, float, float, float]  # x, y, z, yaw_deg, pitch_deg
    fov_deg: float = render.DEFAULT_FOV_DEG
    size: tuple[int, int] = render.DEFAULT_DEPTH_SIZE
    max_depth: float | None = render.DEFAULT_MAX_DEPTH_M
    point_size_m: float | None = render.DEFAULT_POINT_SIZE_M
    source: Any = None
    overlays: tuple[Any, ...] = ()

    LEGEND = (
        "DepthView(view=(x, y, z, yaw_deg, pitch_deg), fov_deg=90, size=(w, h), max_depth=None, "
        "point_size_m=None) -> a perspective render from the view. Every return in front of the "
        "view is drawn unless max_depth caps it, each as wide as the gap to its nearest neighbour "
        "so surfaces close at the cloud's own resolution; point_size_m fixes one width for all. "
        "point_size_m in the result is the [smallest, largest] width drawn, in metres. "
        "covered_pixels counts pixels with a return; colour gives near_m, far_m, the near, far and "
        "no-return colours, sampled stops with depths, and the formula depth_m = near_m * "
        "(far_m / near_m) ** f, f from 0 at colour_near to 1 at colour_far on a log scale."
    )

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        original_ctx = ctx
        raster, ctx = depth_data(self, ctx)
        pose = render.as_view(self.view)
        depth, widths = raster.depth, raster.widths
        near, far = render.colour_range(depth, self.max_depth)
        out: dict[str, Any] = {
            "handler": "DepthView",
            "view_ref": reference(self, original_ctx),
            "view": asdict(pose),
            "width": self.size[0],
            "height": self.size[1],
            "fov_deg": self.fov_deg,
            "max_depth_m": self.max_depth,
            "point_size_m": list(widths) if widths is not None else None,
            "covered_pixels": int(np.isfinite(depth).sum()),
            "colour": render.colour_scale(near, far),
        }
        if render.FORM == "image":
            with ctx.artifact("depth.png") as (staging, path):
                render.depth_png(depth, staging, max_depth=self.max_depth)
                if self.overlays:
                    forward, right, up = pose.axes()
                    origin = np.array([pose.x, pose.y, pose.z])
                    focal = (self.size[0] / 2) / math.tan(math.radians(self.fov_deg) / 2)

                    def project(point: np.ndarray) -> tuple[float, float] | None:
                        relative = point - origin
                        distance = float(relative @ forward)
                        if distance <= 0.05 or (
                            self.max_depth is not None and distance > self.max_depth
                        ):
                            return None
                        return (
                            float(relative @ right) / distance * focal + self.size[0] / 2,
                            -float(relative @ up) / distance * focal + self.size[1] / 2,
                        )

                    out["overlays"] = draw_overlays(
                        staging, self.overlays, original_ctx, project, view_ref=out["view_ref"]
                    )
            out["image"] = str(path)
        else:
            out["ascii"] = render.depth_ascii(depth, max_depth=self.max_depth)
            out["ascii_formula"] = "depth_m = near_m * (far_m / near_m) ** (digit / 9)"
        return out
