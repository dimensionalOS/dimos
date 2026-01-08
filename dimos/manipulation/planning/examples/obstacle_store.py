#!/usr/bin/env python3
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

"""
Obstacle Store - JSON-based obstacle persistence for multi-process sharing.

Allows obstacle tester and planning tester to share obstacle state through
a simple JSON file, enabling simultaneous operation from separate terminals.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.manipulation.planning.spec import Obstacle

# Default store location
DEFAULT_STORE_PATH = Path("/tmp/dimos_obstacles.json")


@dataclass
class ObstacleData:
    """Serializable obstacle data."""

    name: str
    obstacle_type: str  # "BOX", "SPHERE", "CYLINDER"
    pose: list[list[float]]  # 4x4 matrix as nested lists
    dimensions: list[float]
    color: list[float]  # RGBA

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> ObstacleData:
        """Create from dictionary."""
        return cls(
            name=data["name"],
            obstacle_type=data["obstacle_type"],
            pose=data["pose"],
            dimensions=data["dimensions"],
            color=data["color"],
        )

    @classmethod
    def from_obstacle(cls, obstacle: Obstacle) -> ObstacleData:
        """Create from Obstacle spec."""
        from dimos.manipulation.planning.spec import Obstacle, ObstacleType

        return cls(
            name=obstacle.name,
            obstacle_type=obstacle.obstacle_type.name,
            pose=obstacle.pose.tolist(),
            dimensions=list(obstacle.dimensions),
            color=list(obstacle.color),
        )

    def to_obstacle(self) -> Obstacle:
        """Convert to Obstacle spec."""
        from dimos.manipulation.planning.spec import Obstacle, ObstacleType

        return Obstacle(
            name=self.name,
            obstacle_type=ObstacleType[self.obstacle_type],
            pose=np.array(self.pose),
            dimensions=tuple(self.dimensions),
            color=tuple(self.color),
        )


class ObstacleStore:
    """JSON-based obstacle store for multi-process sharing.

    This class provides:
    - Persistent storage of obstacles to a JSON file
    - File watching for automatic sync between processes
    - Thread-safe operations

    Usage:
        # Process 1: Obstacle Tester
        store = ObstacleStore()
        store.add_obstacle(obstacle)
        store.save()  # Writes to file

        # Process 2: Planning Tester
        store = ObstacleStore()
        store.start_watching(on_change=my_callback)  # Auto-sync on changes
    """

    def __init__(self, store_path: Path | str | None = None):
        """Initialize the store.

        Args:
            store_path: Path to JSON file. Defaults to /tmp/dimos_obstacles.json
        """
        self._path = Path(store_path) if store_path else DEFAULT_STORE_PATH
        self._obstacles: dict[str, ObstacleData] = {}
        self._lock = threading.Lock()
        self._watch_thread: threading.Thread | None = None
        self._watching = False
        self._last_mtime: float = 0
        self._on_change: Callable[[dict[str, ObstacleData]], None] | None = None

        # Load existing data if file exists and is non-empty
        if self._path.exists() and self._path.stat().st_size > 0:
            self.load()

    @property
    def path(self) -> Path:
        """Get store file path."""
        return self._path

    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Add or update an obstacle.

        Args:
            obstacle: Obstacle to add
        """
        with self._lock:
            self._obstacles[obstacle.name] = ObstacleData.from_obstacle(obstacle)

    def remove_obstacle(self, name: str) -> bool:
        """Remove an obstacle.

        Args:
            name: Obstacle name

        Returns:
            True if removed, False if not found
        """
        with self._lock:
            if name in self._obstacles:
                del self._obstacles[name]
                return True
            return False

    def update_pose(self, name: str, pose: np.ndarray) -> bool:
        """Update obstacle pose.

        Args:
            name: Obstacle name
            pose: New 4x4 pose matrix

        Returns:
            True if updated, False if not found
        """
        with self._lock:
            if name in self._obstacles:
                self._obstacles[name].pose = pose.tolist()
                return True
            return False

    def clear(self) -> None:
        """Clear all obstacles."""
        with self._lock:
            self._obstacles.clear()

    def get_obstacles(self) -> dict[str, ObstacleData]:
        """Get all obstacles (copy)."""
        with self._lock:
            return dict(self._obstacles)

    def get_obstacle(self, name: str) -> ObstacleData | None:
        """Get a specific obstacle."""
        with self._lock:
            return self._obstacles.get(name)

    def save(self) -> None:
        """Save obstacles to file."""
        with self._lock:
            data = {
                "version": 1,
                "obstacles": {name: obs.to_dict() for name, obs in self._obstacles.items()},
            }
            # Write to temp file first, then rename (atomic)
            temp_path = self._path.with_suffix(".tmp")
            with open(temp_path, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(temp_path, self._path)
            self._last_mtime = self._path.stat().st_mtime

    def load(self) -> bool:
        """Load obstacles from file.

        Returns:
            True if loaded successfully, False otherwise
        """
        if not self._path.exists():
            return False

        try:
            with self._lock:
                with open(self._path) as f:
                    data = json.load(f)

                self._obstacles.clear()
                for name, obs_data in data.get("obstacles", {}).items():
                    self._obstacles[name] = ObstacleData.from_dict(obs_data)

                self._last_mtime = self._path.stat().st_mtime
            return True
        except (json.JSONDecodeError, KeyError, OSError) as e:
            print(f"Warning: Failed to load obstacle store: {e}")
            return False

    def start_watching(
        self,
        on_change: Callable[[dict[str, ObstacleData]], None],
        interval: float = 0.5,
    ) -> None:
        """Start watching file for changes.

        Args:
            on_change: Callback when file changes. Receives dict of obstacles.
            interval: Poll interval in seconds
        """
        if self._watching:
            return

        self._on_change = on_change
        self._watching = True
        self._watch_thread = threading.Thread(
            target=self._watch_loop,
            args=(interval,),
            daemon=True,
        )
        self._watch_thread.start()

    def stop_watching(self) -> None:
        """Stop watching for changes."""
        self._watching = False
        if self._watch_thread:
            self._watch_thread.join(timeout=1.0)
            self._watch_thread = None

    def _watch_loop(self, interval: float) -> None:
        """Watch loop for file changes."""
        while self._watching:
            try:
                if self._path.exists():
                    mtime = self._path.stat().st_mtime
                    if mtime > self._last_mtime:
                        self.load()
                        if self._on_change:
                            self._on_change(self.get_obstacles())
            except OSError:
                pass  # File might be being written
            time.sleep(interval)

    def __len__(self) -> int:
        """Get number of obstacles."""
        with self._lock:
            return len(self._obstacles)

    def __contains__(self, name: str) -> bool:
        """Check if obstacle exists."""
        with self._lock:
            return name in self._obstacles
