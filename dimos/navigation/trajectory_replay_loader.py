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

"""Load Unitree-style timed odom replays for calibration and regression (921 P5-3).

``LegacyPickleStore`` (exposed as ``TimedSensorReplay``) already resolves relative
names through :func:`dimos.utils.data.get_data`, which may pull Git LFS. This
module adds a stable entry point and a **shrink-wrapped** in-tree pickle directory
so default Linux CI can exercise replay wiring without LFS or network.

See ``docs/development/large_file_management.md`` for where optional LFS-backed
replay tests run.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any, Literal

from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.utils.testing.replay import TimedSensorReplay


def trajectory_odom_replay_mini_fixture_dir() -> Path:
    """Absolute path to the tiny in-repo odom replay (three ``*.pickle`` frames)."""
    return Path(__file__).resolve().parent / "fixtures" / "trajectory_odom_replay_mini"


def open_trajectory_odom_replay(
    source: Literal["fixture"] | str | Path,
    *,
    autocast: Callable[[Any], Any] | None = None,
) -> LegacyPickleStore[Any]:
    """Open a timed odom pickle replay compatible with ``ReplayConnection`` datasets.

    Args:
        source: ``"fixture"`` uses :func:`trajectory_odom_replay_mini_fixture_dir`
            (no ``get_data``, safe for default CI). A :class:`~pathlib.Path` must be
            absolute and point at a directory of ``NNN.pickle`` files. Any non-absolute
            string (for example ``"unitree_office_walk/odom"``) is passed to
            ``TimedSensorReplay``, which resolves via ``get_data`` and may require
            Git LFS.
        autocast: Optional transform after load, commonly ``Odometry.from_msg`` for
            Unitree WebRTC dict payloads.

    Returns:
        ``TimedSensorReplay`` alias of ``LegacyPickleStore`` for iterate/stream APIs.
    """
    if source == "fixture":
        return TimedSensorReplay(trajectory_odom_replay_mini_fixture_dir(), autocast=autocast)
    return TimedSensorReplay(source, autocast=autocast)
