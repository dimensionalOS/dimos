# Copyright 2025 Dimensional Inc.
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

"""Fast stateful image generator with visual features for encoding tests."""

from typing import Literal, TypedDict, Union

import numpy as np
from numpy.typing import NDArray


class CircleObject(TypedDict):
    """Type definition for circle objects."""

    type: Literal["circle"]
    x: float
    y: float
    vx: float
    vy: float
    radius: int
    color: NDArray[np.float32]


class RectObject(TypedDict):
    """Type definition for rectangle objects."""

    type: Literal["rect"]
    x: float
    y: float
    vx: float
    vy: float
    width: int
    height: int
    color: NDArray[np.float32]


Object = Union[CircleObject, RectObject]


class FastImageGenerator:
    """
    Stateful image generator that creates images with visual features
    suitable for testing image/video encoding at 30+ FPS.

    Features generated:
    - Moving geometric shapes (tests motion vectors)
    - Color gradients (tests gradient compression)
    - Sharp edges and corners (tests edge preservation)
    - Textured regions (tests detail retention)
    - Smooth regions (tests flat area compression)
    - High contrast boundaries (tests blocking artifacts)
    """

    def __init__(self, width: int = 1280, height: int = 720) -> None:
        """Initialize the generator with pre-computed elements."""
        self.width = width
        self.height = height
        self.frame_count = 0
        self.objects: list[Object] = []

        # Pre-allocate the main canvas
        self.canvas = np.zeros((height, width, 3), dtype=np.float32)

        # Pre-compute coordinate grids for fast gradient generation
        self.x_grid, self.y_grid = np.meshgrid(
            np.linspace(0, 1, width, dtype=np.float32), np.linspace(0, 1, height, dtype=np.float32)
        )

        # Pre-compute base gradient patterns
        self._init_gradients()

        # Initialize moving objects with their properties
        self._init_moving_objects()

        # Pre-compute static texture pattern
        self._init_texture()

        # Pre-allocate shape masks for reuse
        self._init_shape_masks()

    def _init_gradients(self) -> None:
        """Pre-compute gradient patterns."""
        # Diagonal gradient
        self.diag_gradient = (self.x_grid + self.y_grid) * 0.5

        # Radial gradient from center
        cx, cy = 0.5, 0.5
        self.radial_gradient = np.sqrt((self.x_grid - cx) ** 2 + (self.y_grid - cy) ** 2)
        self.radial_gradient = np.clip(1.0 - self.radial_gradient * 1.5, 0, 1)

        # Horizontal and vertical gradients
        self.h_gradient = self.x_grid
        self.v_gradient = self.y_grid

    def _init_moving_objects(self) -> None:
        """Initialize properties of moving objects."""
        self.objects = [
            {
                "type": "circle",
                "x": 0.2,
                "y": 0.3,
                "vx": 0.002,
                "vy": 0.003,
                "radius": 60,
                "color": np.array([255, 100, 100], dtype=np.float32),
            },
            {
                "type": "rect",
                "x": 0.7,
                "y": 0.6,
                "vx": -0.003,
                "vy": 0.002,
                "width": 100,
                "height": 80,
                "color": np.array([100, 255, 100], dtype=np.float32),
            },
            {
                "type": "circle",
                "x": 0.5,
                "y": 0.5,
                "vx": 0.004,
                "vy": -0.002,
                "radius": 40,
                "color": np.array([100, 100, 255], dtype=np.float32),
            },
        ]

    def _init_texture(self) -> None:
        """Pre-compute a texture pattern."""
        # Create a simple checkerboard pattern at lower resolution
        checker_size = 20
        checker_h = self.height // checker_size
        checker_w = self.width // checker_size

        # Create small checkerboard
        checker = np.indices((checker_h, checker_w)).sum(axis=0) % 2

        # Upscale using repeat (fast)
        self.texture = np.repeat(np.repeat(checker, checker_size, axis=0), checker_size, axis=1)
        self.texture = self.texture[: self.height, : self.width].astype(np.float32) * 30

    def _init_shape_masks(self) -> None:
        """Pre-allocate reusable masks for shapes."""
        # Pre-allocate a mask array
        self.temp_mask = np.zeros((self.height, self.width), dtype=np.float32)

        # Pre-compute indices for the entire image
        self.y_indices, self.x_indices = np.indices((self.height, self.width))

    def _draw_circle_fast(self, cx: int, cy: int, radius: int, color: NDArray[np.float32]) -> None:
        """Draw a circle using vectorized operations - optimized version without anti-aliasing."""
        # Compute bounding box to minimize calculations
        y1 = max(0, cy - radius - 1)
        y2 = min(self.height, cy + radius + 2)
        x1 = max(0, cx - radius - 1)
        x2 = min(self.width, cx + radius + 2)

        # Work only on the bounding box region
        if y1 < y2 and x1 < x2:
            y_local, x_local = np.ogrid[y1:y2, x1:x2]
            dist_sq = (x_local - cx) ** 2 + (y_local - cy) ** 2
            mask = dist_sq <= radius**2
            self.canvas[y1:y2, x1:x2][mask] = color

    def _draw_rect_fast(self, x: int, y: int, w: int, h: int, color: NDArray[np.float32]) -> None:
        """Draw a rectangle using slicing."""
        # Clip to canvas boundaries
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(self.width, x + w)
        y2 = min(self.height, y + h)

        if x1 < x2 and y1 < y2:
            self.canvas[y1:y2, x1:x2] = color

    def _update_objects(self) -> None:
        """Update positions of moving objects."""
        for obj in self.objects:
            # Update position
            obj["x"] += obj["vx"]
            obj["y"] += obj["vy"]

            # Bounce off edges
            if obj["type"] == "circle":
                r = obj["radius"] / self.width
                if obj["x"] - r <= 0 or obj["x"] + r >= 1:
                    obj["vx"] *= -1
                    obj["x"] = np.clip(obj["x"], r, 1 - r)

                r = obj["radius"] / self.height
                if obj["y"] - r <= 0 or obj["y"] + r >= 1:
                    obj["vy"] *= -1
                    obj["y"] = np.clip(obj["y"], r, 1 - r)

            elif obj["type"] == "rect":
                w = obj["width"] / self.width
                h = obj["height"] / self.height
                if obj["x"] <= 0 or obj["x"] + w >= 1:
                    obj["vx"] *= -1
                    obj["x"] = np.clip(obj["x"], 0, 1 - w)

                if obj["y"] <= 0 or obj["y"] + h >= 1:
                    obj["vy"] *= -1
                    obj["y"] = np.clip(obj["y"], 0, 1 - h)

    def generate_frame(self) -> NDArray[np.uint8]:
        """
        Generate a single frame with visual features - optimized for 30+ FPS.

        Returns:
            numpy array of shape (height, width, 3) with uint8 values
        """
        # Fast gradient background - use only one gradient per frame
        if self.frame_count % 2 == 0:
            base_gradient = self.h_gradient
        else:
            base_gradient = self.v_gradient

        # Simple color mapping
        self.canvas[:, :, 0] = base_gradient * 150 + 50
        self.canvas[:, :, 1] = base_gradient * 120 + 70
        self.canvas[:, :, 2] = (1 - base_gradient) * 140 + 60

        # Add texture in corner - simplified without per-channel scaling
        tex_size = self.height // 3
        self.canvas[:tex_size, :tex_size] += self.texture[:tex_size, :tex_size, np.newaxis]

        # Add test pattern bars - vectorized
        bar_width = 50
        bar_start = self.width // 3
        for i in range(3):  # Reduced from 5 to 3 bars
            x1 = bar_start + i * bar_width * 2
            x2 = min(x1 + bar_width, self.width)
            if x1 < self.width:
                color_val = 180 + i * 30
                self.canvas[self.height // 2 :, x1:x2] = color_val

        # Update and draw only 2 moving objects (reduced from 3)
        self._update_objects()

        # Draw only first 2 objects for speed
        for obj in self.objects[:2]:
            if obj["type"] == "circle":
                cx = int(obj["x"] * self.width)
                cy = int(obj["y"] * self.height)
                self._draw_circle_fast(cx, cy, obj["radius"], obj["color"])
            elif obj["type"] == "rect":
                x = int(obj["x"] * self.width)
                y = int(obj["y"] * self.height)
                self._draw_rect_fast(x, y, obj["width"], obj["height"], obj["color"])

        # Simple horizontal lines pattern (faster than sine wave)
        line_y = int(self.height * 0.8)
        line_spacing = 10
        for i in range(0, 5):
            y = line_y + i * line_spacing
            if y < self.height:
                self.canvas[y : y + 2, :] = [255, 200, 100]

        # Increment frame counter
        self.frame_count += 1

        # Direct conversion to uint8 (already in valid range)
        return self.canvas.astype(np.uint8)

    def reset(self) -> None:
        """Reset the generator to initial state."""
        self.frame_count = 0
        self._init_moving_objects()


# Convenience function for backward compatibility
_generator: FastImageGenerator | None = None


def random_image(width: int, height: int) -> NDArray[np.uint8]:
    """
    Generate an image with visual features suitable for encoding tests.
    Maintains state for efficient stream generation.

    Args:
        width: Image width in pixels
        height: Image height in pixels

    Returns:
        numpy array of shape (height, width, 3) with uint8 values
    """
    global _generator

    # Initialize or reinitialize if dimensions changed
    if _generator is None or _generator.width != width or _generator.height != height:
        _generator = FastImageGenerator(width, height)

    return _generator.generate_frame()
