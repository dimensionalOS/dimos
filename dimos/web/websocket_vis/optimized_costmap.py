#!/usr/bin/env python3

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

# Copyright 2025 Dimensional Inc.

import base64
import hashlib
import time
from typing import Any
import zlib

import numpy as np


class OptimizedCostmapEncoder:
    """Handles optimized encoding of costmaps with delta compression."""

    def __init__(self, chunk_size: int = 64) -> None:
        self.chunk_size = chunk_size
        self.last_full_grid: np.ndarray | None = None  # type: ignore[type-arg]
        self.last_full_sent_time: float = 0  # Track when last full update was sent
        self.chunk_hashes: dict[tuple[int, int], str] = {}
        self.full_update_interval = 3.0  # Send full update every 3 seconds

    def encode_costmap(self, grid: np.ndarray, force_full: bool = False) -> dict[str, Any]:  # type: ignore[type-arg]
        """Encode a costmap grid with optimizations.

        Args:
            grid: The costmap grid as numpy array
            force_full: Force sending a full update

        Returns:
            Encoded costmap data
        """
        current_time = time.time()

        # Determine if we need a full update
        send_full = (
            force_full
            or self.last_full_grid is None
            or self.last_full_grid.shape != grid.shape
            or (current_time - self.last_full_sent_time) > self.full_update_interval
        )

        if send_full:
            return self._encode_full(grid, current_time)
        else:
            return self._encode_delta(grid, current_time)

    def _encode_full(self, grid: np.ndarray, current_time: float) -> dict[str, Any]:  # type: ignore[type-arg]
        height, width = grid.shape

        # Convert to uint8 for better compression (costmap values are -1 to 100)
        # Map -1 to 255 for unknown cells
        grid_uint8 = grid.astype(np.int16)
        grid_uint8[grid_uint8 == -1] = 255
        grid_uint8 = grid_uint8.astype(np.uint8)

        # Compress the data
        compressed = zlib.compress(grid_uint8.tobytes(), level=6)

        # Base64 encode
        encoded = base64.b64encode(compressed).decode("ascii")

        # Update state
        self.last_full_grid = grid.copy()
        self.last_full_sent_time = current_time
        self._update_chunk_hashes(grid)

        return {
            "update_type": "full",
            "shape": [height, width],
            "dtype": "u8",  # uint8
            "compressed": True,
            "compression": "zlib",
            "data": encoded,
        }

    def _encode_delta(self, grid: np.ndarray, current_time: float) -> dict[str, Any]:  # type: ignore[type-arg]
        height, width = grid.shape
        changed_chunks = []

        # Divide grid into chunks and check for changes
        for y in range(0, height, self.chunk_size):
            for x in range(0, width, self.chunk_size):
                # Get chunk bounds
                y_end = min(y + self.chunk_size, height)
                x_end = min(x + self.chunk_size, width)

                # Extract chunk
                chunk = grid[y:y_end, x:x_end]

                # Compute hash of chunk
                chunk_hash = hashlib.md5(chunk.tobytes()).hexdigest()
                chunk_key = (y, x)

                # Check if chunk has changed
                if chunk_key not in self.chunk_hashes or self.chunk_hashes[chunk_key] != chunk_hash:
                    # Chunk has changed, encode it
                    chunk_uint8 = chunk.astype(np.int16)
                    chunk_uint8[chunk_uint8 == -1] = 255
                    chunk_uint8 = chunk_uint8.astype(np.uint8)

                    # Compress chunk
                    compressed = zlib.compress(chunk_uint8.tobytes(), level=6)
                    encoded = base64.b64encode(compressed).decode("ascii")

                    changed_chunks.append(
                        {"pos": [y, x], "size": [y_end - y, x_end - x], "data": encoded}
                    )

                    # Update hash
                    self.chunk_hashes[chunk_key] = chunk_hash

        # Update state - only update the grid, not the timer
        self.last_full_grid = grid.copy()

        # If too many chunks changed, send full update instead
        total_chunks = ((height + self.chunk_size - 1) // self.chunk_size) * (
            (width + self.chunk_size - 1) // self.chunk_size
        )

        if len(changed_chunks) > total_chunks * 0.5:
            # More than 50% changed, send full update
            return self._encode_full(grid, current_time)

        return {
            "update_type": "delta",
            "shape": [height, width],
            "dtype": "u8",
            "compressed": True,
            "compression": "zlib",
            "chunks": changed_chunks,
        }

    def _update_chunk_hashes(self, grid: np.ndarray) -> None:  # type: ignore[type-arg]
        """Update all chunk hashes for the grid."""
        self.chunk_hashes.clear()
        height, width = grid.shape

        for y in range(0, height, self.chunk_size):
            for x in range(0, width, self.chunk_size):
                y_end = min(y + self.chunk_size, height)
                x_end = min(x + self.chunk_size, width)
                chunk = grid[y:y_end, x:x_end]
                chunk_hash = hashlib.md5(chunk.tobytes()).hexdigest()
                self.chunk_hashes[(y, x)] = chunk_hash
