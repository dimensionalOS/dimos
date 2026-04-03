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


@dataclass
class GuardrailDecision:
    state: GuardrailState
    cmd_vel: Twist
    reason: str


class GuardrailPolicy(Protocol):
    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
    ) -> GuardrailDecision:
        """Evaluate the latest frame pair and command."""
