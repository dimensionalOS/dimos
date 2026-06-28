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

"""LeRobot v3.0 dataset reader — read-back / summary from `meta/`.

Mirror of ``_LeRobotV3Writer``: reads ``info.json`` + the episodes parquet.
New read-side features (e.g. logging episodes to Rerun) belong here as methods.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.learning.dataprep.core import summarize_lengths
from dimos.learning.dataprep.formats.lerobot.writer import CHUNK, EPISODES_DIR, FILE, META_DIR

_META_COLS = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


class _LeRobotReader:
    """Read-only view over a built LeRobot v3.0 dataset.

    One instance per dataset root; lazily reads ``meta/info.json`` and the
    episodes parquet on demand.
    """

    def __init__(self, path: Path) -> None:
        self.root = Path(path)
        self.info = json.loads((self.root / META_DIR / "info.json").read_text())

    def _features(self) -> tuple[dict[str, Any], dict[str, Any]]:
        observation: dict[str, Any] = {}
        action: dict[str, Any] = {}
        for name, feat in self.info.get("features", {}).items():
            if name in _META_COLS:
                continue
            entry = {"shape": feat.get("shape"), "dtype": feat.get("dtype")}
            if name.startswith("observation"):
                observation[name] = entry
            elif name.startswith("action"):
                action[name] = entry
        return observation, action

    def _episode_lengths(self) -> list[int]:
        import pyarrow.parquet as pq

        ep_file = self.root / META_DIR / EPISODES_DIR / CHUNK / f"{FILE}.parquet"
        if not ep_file.exists():
            return []
        lengths: list[int] = pq.read_table(ep_file, columns=["length"]).column("length").to_pylist()
        return lengths

    def summary(self) -> dict[str, Any]:
        """Observation/action features (shape + dtype), episode/frame counts."""
        observation, action = self._features()
        return {
            "format": "lerobot",
            "version": self.info.get("codebase_version"),
            "path": str(self.root),
            "episodes": self.info.get("total_episodes"),
            "frames": self.info.get("total_frames"),
            "fps": self.info.get("fps"),
            "robot": self.info.get("robot_type"),
            "observation": observation,
            "action": action,
            "episode_lengths": summarize_lengths(self._episode_lengths()),
            "shapes_uniform": True,  # LeRobot declares one global feature schema
            "has_stats": (self.root / META_DIR / "stats.json").exists(),
        }


def inspect(path: Path) -> dict[str, Any]:
    """Summarize a LeRobot v3.0 dataset. Thin driver over ``_LeRobotReader``."""
    return _LeRobotReader(path).summary()
