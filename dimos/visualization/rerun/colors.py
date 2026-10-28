# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register a colormap as AnnotationContext so Rerun resolves colors viewer-side."""
    from matplotlib import colormaps
    import numpy as np
    import rerun as rr

    lut = (colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=lut[i].tolist())
                )
                for i in range(256)
            ]
        ),
        static=True,
    )


def occupancy_lut(
    colormap: str | None,
    opacity: float,
    background: str | None,
) -> np.ndarray:
    """Build a 102-entry RGBA LUT for grid values -1..100. One-time cost per config."""
    bg_rgb = np.array([0, 0, 0], dtype=np.float32)
    if background is not None:
        bg = background.lstrip("#")
        bg_rgb = np.array([int(bg[i : i + 2], 16) for i in (0, 2, 4)], dtype=np.float32)

    # LUT: index 0 = grid value -1 (unknown), index 1 = grid value 0 (free), index 2..101 = grid values 1..100
    lut = np.zeros((102, 4), dtype=np.uint8)

    # Unknown (-1) → index 0: black
    lut[0] = [0, 0, 0, 255]

    if colormap is not None:
        from matplotlib import colormaps

        cmap = colormaps[colormap]
        # Free (0) → index 1
        fg = np.array(cmap(0.0)[:3]) * 255
        blended = fg * opacity + bg_rgb * (1 - opacity)
        lut[1, :3] = blended.astype(np.uint8)
        lut[1, 3] = 255

        # Occupied (1..100) → index 2..101
        for cost in range(1, 101):
            cost_norm = 0.5 + (cost / 100) * 0.5
            fg = np.array(cmap(cost_norm)[:3]) * 255
            blended = fg * opacity + bg_rgb * (1 - opacity)
            lut[cost + 1, :3] = blended.astype(np.uint8)
            lut[cost + 1, 3] = 255
    else:
        # Foxglove-style: free = #484981, occupied = gradient to black
        fg_free = np.array([72, 73, 129], dtype=np.float32)
        blended_free = fg_free * opacity + bg_rgb * (1 - opacity)
        lut[1, :3] = blended_free.astype(np.uint8)
        lut[1, 3] = 255

        for cost in range(1, 101):
            factor = 1 - cost / 100
            fg = np.array([72 * factor, 73 * factor, 129 * factor], dtype=np.float32)
            blended = fg * opacity + bg_rgb * (1 - opacity)
            lut[cost + 1, :3] = blended.astype(np.uint8)
            lut[cost + 1, 3] = 255

    return lut
