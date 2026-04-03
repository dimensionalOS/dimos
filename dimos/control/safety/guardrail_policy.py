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