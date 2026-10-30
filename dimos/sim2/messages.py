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

"""Snapshot messages between physics and sensor workers.

Robot IO uses standard DimOS messages on ordinary module ports. Only the full
MuJoCo integration state, which sensor workers restore into their own copy of
the model, has no DimOS message type and travels as this pickled dataclass.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class FrameMetadata:
    """Where a snapshot sits in the run.

    Attributes:
        sequence: Publication counter; a reader ignores anything it already has.
        episode_id: Advances on reset and on resume after a pause. Pending sensor
            work from an older episode is discarded.
        physics_tick: Steps taken since the world was built.
        sim_time: MuJoCo time in seconds.
    """

    sequence: int
    episode_id: int
    physics_tick: int
    sim_time: float


@dataclass(frozen=True)
class StateFrame:
    """MuJoCo integration state after one step.

    Attributes:
        metadata: Position of this frame in the run.
        values: ``state`` is the ``mjSTATE_INTEGRATION`` vector; ``wall_time`` is
            the Unix time the frame corresponds to, as a one-element array.
    """

    metadata: FrameMetadata
    values: dict[str, NDArray[np.float64]]
    model_path: str | None = None


@dataclass(frozen=True)
class EnvironmentStep:
    """An upstream result, preserving the environment's observation contract."""

    metadata: FrameMetadata
    observations: dict[str, Any]
    reward: float
    done: bool
    info: dict[str, Any]
