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

from __future__ import annotations

import hashlib

from dimos_lcm.foxglove_msgs import Color as LCMColor  # type: ignore[import-untyped]


class Color(LCMColor):  # type: ignore[misc]
    """Color with convenience methods."""

    @classmethod
    def from_string(cls, name: str, alpha: float = 0.2, brightness: float = 1.0) -> Color:
        """Generate a consistent color from a string using hash function.

        Args:
            name: String to generate color from
            alpha: Transparency value (0.0-1.0)
            brightness: Brightness multiplier (0.0-2.0). Values > 1.0 lighten towards white.

        Returns:
            Color instance with deterministic RGB values
        """
        # Hash the string to get consistent values
        hash_obj = hashlib.md5(name.encode())
        hash_bytes = hash_obj.digest()

        # Use first 3 bytes for RGB (0-255)
        r = hash_bytes[0] / 255.0
        g = hash_bytes[1] / 255.0
        b = hash_bytes[2] / 255.0

        # Apply brightness adjustment
        # If brightness > 1.0, mix with white to lighten
        if brightness > 1.0:
            mix_factor = brightness - 1.0  # 0.0 to 1.0
            r = r + (1.0 - r) * mix_factor
            g = g + (1.0 - g) * mix_factor
            b = b + (1.0 - b) * mix_factor
        else:
            # If brightness < 1.0, darken by scaling
            r *= brightness
            g *= brightness
            b *= brightness

        # Create and return color instance
        color = cls()
        color.r = min(1.0, r)
        color.g = min(1.0, g)
        color.b = min(1.0, b)
        color.a = alpha
        return color
