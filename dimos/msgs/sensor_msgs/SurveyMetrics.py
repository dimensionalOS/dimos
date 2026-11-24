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

from dataclasses import dataclass, field

from dimos.types.timestamped import Timestamped


@dataclass(frozen=True)
class SurveySample(Timestamped):
    """A measurement ending at robot-clock ``ts``, with explicit units in metric names.

    ``duration_s`` covers the measurement window, not transport delay. Clock
    uncertainty is retained separately. Missing values are omitted, never zeroed.
    Source identifies the probe target or individual viewer/camera.
    """

    ts: float
    source: str
    values: dict[str, float]
    duration_s: float = 0.0
    time_uncertainty_s: float = 0.0
    labels: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class SurveyMetrics(Timestamped):
    ts: float
    interface: str
    samples: list[SurveySample]
