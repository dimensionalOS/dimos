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


"""What can be done to any simulation, as typed actions a robot sends as one WorldAction.

Entities are robot ids or object names; a robot's joint is "<entity>/<joint>" on the wire.
"""

from dataclasses import dataclass
import math
from typing import Any

import cbor2
from dimos_lcm.world_msgs import (
    CborKeyValueCommand as CborKeyValueCommandMessage,
    JointCommand as JointCommandMessage,
    WorldAction,
)

from dimos.sim2.world_observations import WorldMirror, WorldWriter


@dataclass(frozen=True)
class JointCommand:
    """Drive joints through their actuators; each actuator takes the one of these it is built for.

    With kp and kd it is a motor command instead, as a real robot's motors take one:
    effort = kp * (position - q) + kd * (velocity - dq) + effort.
    """

    entity: str
    names: tuple[str, ...]
    position: tuple[float, ...] | None = None
    velocity: tuple[float, ...] | None = None
    effort: tuple[float, ...] | None = None
    kp: tuple[float, ...] | None = None
    kd: tuple[float, ...] | None = None


@dataclass(frozen=True)
class CborKeyValueCommand:
    """Set one key of an entity's CBOR data; what it does is up to the sim, which observes it back."""

    entity: str
    key: str
    value: Any


Action = JointCommand | CborKeyValueCommand

_FIELDS = ("position", "velocity", "effort", "kp", "kd")


def world_action(mirror: WorldMirror, *actions: Action) -> WorldAction:
    """The actions as a WorldAction, addressed by the ids the mirror's world gave its entities."""
    with mirror.lock:
        joints = [
            JointCommandMessage(
                entity_id=mirror.ids[f"{action.entity}/{name}"],
                **{
                    f: math.nan if getattr(action, f) is None else float(getattr(action, f)[i])
                    for f in _FIELDS
                },
            )
            for action in actions
            if isinstance(action, JointCommand)
            for i, name in enumerate(action.names)
        ]
        cbor = [
            _cbor_command(mirror.ids[action.entity], action.key, action.value)
            for action in actions
            if isinstance(action, CborKeyValueCommand)
        ]
        frame, ts = mirror.frame, mirror.ts
    return WorldAction(
        ts=ts,
        frame=frame,
        joint_commands_length=len(joints),
        joint_commands=joints,
        cbor_key_value_commands_length=len(cbor),
        cbor_key_value_commands=cbor,
    )


def _cbor_command(entity: int, key: str, value: Any) -> CborKeyValueCommandMessage:
    encoded = cbor2.dumps(value)
    return CborKeyValueCommandMessage(
        entity_id=entity, key=key, value_length=len(encoded), value=encoded
    )


def read_actions(message: WorldAction, world: WorldWriter) -> list[Action]:
    """A WorldAction's typed actions, by the paths the backend's writer gave its entities' ids: one joint command per run of an entity's joints."""
    actions: list[Action] = []
    for command in message.joint_commands:
        entity, name = world.paths[command.entity_id].split("/", 1)
        values = {f: getattr(command, f) for f in _FIELDS}
        given = {f: None if math.isnan(v) else (v,) for f, v in values.items()}
        previous = actions[-1] if actions else None
        if (
            isinstance(previous, JointCommand)
            and previous.entity == entity
            and all((getattr(previous, f) is None) == (given[f] is None) for f in _FIELDS)
        ):
            actions[-1] = JointCommand(
                entity,
                (*previous.names, name),
                **{
                    f: None if given[f] is None else getattr(previous, f) + given[f]
                    for f in _FIELDS
                },
            )
        else:
            actions.append(JointCommand(entity, (name,), **given))
    for item in message.cbor_key_value_commands:
        actions.append(
            CborKeyValueCommand(
                world.paths[item.entity_id], item.key, cbor2.loads(bytes(item.value))
            )
        )
    return actions
