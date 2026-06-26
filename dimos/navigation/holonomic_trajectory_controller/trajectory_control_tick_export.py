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

"""JSONL export for trajectory control tick logs.

Each line is one UTF-8 JSON object. Normative field names and units are
documented in ``trajectory_control_tick_jsonl.md`` in this package.
"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

from dimos.navigation.holonomic_trajectory_controller.trajectory_control_tick_log import TrajectoryControlTick

TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION = 1


def trajectory_control_tick_to_jsonl_dict(tick: TrajectoryControlTick) -> dict[str, Any]:
    """Map one tick to a JSON-ready dict (stable key order: schema, then dataclass fields)."""
    return {"schema_version": TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION} | asdict(tick)


class JsonlTrajectoryControlTickSink:
    """Append control ticks directly to a JSONL file during live runs."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")

    def append(self, tick: TrajectoryControlTick) -> None:
        self._file.write(
            json.dumps(
                trajectory_control_tick_to_jsonl_dict(tick),
                separators=(",", ":"),
                allow_nan=False,
            )
        )
        self._file.write("\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> JsonlTrajectoryControlTickSink:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


__all__ = [
    "TRAJECTORY_CONTROL_TICK_JSONL_SCHEMA_VERSION",
    "JsonlTrajectoryControlTickSink",
    "trajectory_control_tick_to_jsonl_dict",
]
