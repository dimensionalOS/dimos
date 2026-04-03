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

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image


class GuardrailState(str, Enum):
    INIT = "init"
    PASS = "pass"
    CLAMP = "clamp"
    STOP_LATCHED = "stop_latched"
    SENSOR_DEGRADED = "sensor_degraded"


@dataclass(frozen=True)
class GuardrailHealth:
    has_previous_frame: bool
    image_fresh: bool
    cmd_fresh: bool
    risk_fresh: bool
    low_texture: bool = False
    occluded: bool = False


@dataclass
class GuardrailDecision:
    """Policy result consumed by the guardrail worker.

    publish_immediately requests an immediate worker-side publish on the next
    loop iteration. It does not bypass command freshness checks.
    """

    state: GuardrailState
    cmd_vel: Twist
    reason: str
    risk_score: float = 0.0
    publish_immediately: bool = False


class GuardrailPolicy(Protocol):
    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        """Evaluate the latest frame pair and command."""


class PassThroughGuardrailPolicy:
    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        return GuardrailDecision(
            state=GuardrailState.PASS,
            cmd_vel=incoming_cmd_vel,
            reason="pass_through",
            risk_score=0.0,
            publish_immediately=False,
        )
