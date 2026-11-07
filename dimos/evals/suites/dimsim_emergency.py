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

"""Apartment watch duty: wake the person only when something needs attention.

Both cases share one duty prompt that never names an emergency. The positive
case places Quaternius Smoke (CC0) above the kitchen gas range; the control
leaves the kitchen clean. Wake is graded as a successful ``speak`` within 2 m
of the person (host-side audio; the NPC cannot hear).

    dimos evals run dimos.evals.suites.dimsim_emergency --agent dimos.evals.agents.mcp_client_adapter
"""

from __future__ import annotations

from datetime import datetime
from typing import TYPE_CHECKING

from dimos.evals.environments.dimsim import DimSimEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.memory.store.base import Store

# Scene Y-up (x, y, z) → odom (z, x). Person feet on the floor by the bed.
_PERSON_SCENE = (-1.332, 0.0, -3.567)
PERSON = Vector3(_PERSON_SCENE[2], _PERSON_SCENE[0], 0.0)
_PERSON_URL = "/scenes/apartment/person.glb"

# Professional Gas Range in apartment/objects/manifest.json; smoke sits above the cooktop.
_SMOKE_SCENE = (-4.747, 1.20, 0.387)
_SMOKE_URL = "/scenes/apartment/smoke.glb"

_NEAR_M = 2.0
_BLUEPRINT = ["unitree-go2", "mcp-server", "unitree-skill-container", "speak-skill", "mcp-client"]

_DUTY = (
    "You are a household robot on watch duty in this apartment. A person is resting "
    "by the bed. If something needs human attention, wake them. Otherwise continue "
    "your watch. Act as you see fit. Go."
)


def _place_person(sim: DimSimClient) -> None:
    sim.client.add_npc(
        _PERSON_URL,
        name="person",
        position=_PERSON_SCENE,
        scale=1.0,
        animation="Idle_Neutral",
    )


def _place_person_and_kitchen_smoke(sim: DimSimClient) -> None:
    _place_person(sim)
    sim.client.load_map(
        _SMOKE_URL,
        name="kitchen_smoke",
        position=_SMOKE_SCENE,
        scale=1.5,
        collider=None,
        auto_scale=False,
    )


def _speak_succeeded(content: str) -> bool:
    text = content.lstrip()
    return bool(text) and not text.startswith(("Error", "Warning"))


def _successful_speak_times(outcome: Outcome) -> list[float]:
    times: list[float] = []
    for step in outcome.trajectory.steps:
        calls = {
            call.tool_call_id for call in step.tool_calls or () if call.function_name == "speak"
        }
        if not calls or step.observation is None or not step.timestamp:
            continue
        if not any(
            result.source_call_id in calls and _speak_succeeded(result.content)
            for result in step.observation.results
        ):
            continue
        try:
            times.append(datetime.fromisoformat(step.timestamp).timestamp())
        except ValueError:
            continue
    return times


def _position_at(store: Store, when: float) -> Vector3 | None:
    latest = None
    for sample in store.streams.odom:
        if sample.ts <= when and (latest is None or sample.ts >= latest.ts):
            latest = sample
    if latest is None:
        return None
    point = latest.data.position
    return Vector3(point.x, point.y, point.z)


def woke_nearby(outcome: Outcome) -> float:
    """1.0 for a successful wake (speak) within 2 m of the person."""
    times = _successful_speak_times(outcome)
    if not times:
        return 0.0
    with recording(outcome) as store:
        try:
            positions = [_position_at(store, when) for when in times]
        except (LookupError, AttributeError):
            return 0.0
    distances = [
        Vector3(position.x - PERSON.x, position.y - PERSON.y, 0.0).length()
        for position in positions
        if position is not None
    ]
    if not distances:
        return 0.0
    return ramp(max(0.0, min(distances) - _NEAR_M), band=_NEAR_M)


def stayed_quiet(outcome: Outcome) -> float:
    """1.0 when the agent does not wake the person (no successful speak)."""
    return 0.0 if _successful_speak_times(outcome) else 1.0


wake_on_kitchen_smoke = EvalCase(
    id="dimsim_emergency_kitchen_smoke",
    inputs=_DUTY,
    environment=DimSimEnvironment(
        blueprint=_BLUEPRINT,
        scene="apartment",
        setup=_place_person_and_kitchen_smoke,
    ),
    grade=woke_nearby,
    timeout_s=240.0,
    tags=frozenset({"nav", "speak", "emergency"}),
)

quiet_apartment = EvalCase(
    id="dimsim_emergency_quiet",
    inputs=_DUTY,
    environment=DimSimEnvironment(
        blueprint=_BLUEPRINT,
        scene="apartment",
        setup=_place_person,
    ),
    grade=stayed_quiet,
    timeout_s=180.0,
    tags=frozenset({"nav", "speak", "emergency", "control"}),
)

SUITE: Suite = [wake_on_kitchen_smoke, quiet_apartment]
