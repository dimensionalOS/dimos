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


"""What can be done to any simulation, as typed actions: a robot sends them as a WorldAction, the Simulation hands them to its backend.

Entities are robot ids or object names; a robot's joint is "<entity>/<joint>" on the wire.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Any, Literal

from dimos_lcm.world_msgs import Change, WorldAction
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.sim2.spec import Vector
from dimos.sim2.world_change import (
    EVENTS,
    PAUSED,
    WORLD,
    WorldMirror,
    WorldWriter,
    add_entity,
    cbor_update,
    connection_update,
    delete_entity,
    disconnection_update,
    entity_update,
    event_update,
    group_update,
    joint_targets,
    pose_update,
    read_change,
    read_group_payloads,
    tag_update,
    velocity_update,
)


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
class SetJointPositions:
    """Teleport joints, at rest: reset an arm, open a drawer."""

    entity: str
    names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class SetPose:
    """Teleport an entity's root, at rest unless a twist is given."""

    entity: str
    pose: Pose
    twist: Twist | None = None


@dataclass(frozen=True)
class ApplyWrench:
    """A world-frame force and torque on the entity's root, acting at point (in its frame) for duration sim seconds.

    By default held until replaced; zero clears it, and a duration of 0 is a single physics step.
    """

    entity: str
    force: Vector
    torque: Vector = (0.0, 0.0, 0.0)
    point: Vector = (0.0, 0.0, 0.0)
    duration: float = math.inf


@dataclass(frozen=True)
class SetVelocity:
    """Drive an entity's base like a mobile base, held until replaced: linear x/y and angular z in its own heading frame.

    Gravity and contacts keep acting, so it slides along walls and pushes what it walks into; zero stops it.
    """

    entity: str
    twist: Twist


@dataclass(frozen=True)
class Spawn:
    """Add a primitive object: size as MuJoCo gives it (box half extents, sphere radius, cylinder/capsule radius and half length).

    Dynamic when it has mass, fixed in place otherwise.
    """

    entity: str
    shape: Literal["box", "sphere", "cylinder", "capsule"]
    size: tuple[float, ...]
    pose: Pose
    mass: float = 0.0
    rgba: tuple[float, float, float, float] = (0.6, 0.6, 0.6, 1.0)


@dataclass(frozen=True)
class Remove:
    """Take a spawned object out of the world."""

    entity: str


@dataclass(frozen=True)
class Interact:
    """Do one of an entity's named actions (open a door, press a button), as the world defines them."""

    entity: str
    action: str


@dataclass(frozen=True)
class Attach:
    """Carry an entity with another (a robot or object, at its body or root), keeping their pose to each other, until Detach."""

    entity: str
    to: str
    body: str | None = None


@dataclass(frozen=True)
class Detach:
    """Let a carried entity go where it is."""

    entity: str


@dataclass(frozen=True)
class Pause:
    pass


@dataclass(frozen=True)
class Resume:
    pass


@dataclass(frozen=True)
class Step:
    """Advance a paused simulation."""

    steps: int = 1


@dataclass(frozen=True)
class Reset:
    """Back to the initial state, at sim time zero."""


Action = (
    JointCommand
    | SetJointPositions
    | SetPose
    | ApplyWrench
    | SetVelocity
    | Spawn
    | Remove
    | Interact
    | Attach
    | Detach
    | Pause
    | Resume
    | Step
    | Reset
)


# A base's held drive command, as SetVelocity gives it: [vx, vy, wz] in its heading frame.
BASE_VELOCITY = "base_velocity"
# An entity's spawn description, as Spawn gives it.
SPAWN = "spawn"


def world_action(mirror: WorldMirror, *actions: Action) -> WorldAction:
    """The actions as a WorldAction, addressed by the ids the mirror's world gave its entities."""
    with mirror.lock:
        changes = [change for action in actions for change in _encode(mirror, action)]
        frame, ts = mirror.frame, round(mirror.time * 1e9)
    return WorldAction(ts=ts, frame=frame, actions_length=len(changes), actions=changes)


def _encode(mirror: WorldMirror, action: Action) -> list[Change]:
    ids = mirror.ids
    match action:
        case JointCommand(entity, names, position, velocity, effort, kp, kd):
            n = len(names)
            columns = [position, velocity, kp, kd, effort]
            rows = np.column_stack(
                [np.full(n, np.nan) if c is None else np.asarray(c, float) for c in columns]
            )
            group = ids.get(f"{entity}/joints")
            members = [ids[f"{entity}/{name}"] for name in names]
            order = mirror.groups.get(group, []) if group is not None else []
            # Every one of the entity's joints: one group update, its rows in the group's order.
            if sorted(order) == sorted(members):
                rows = rows[[members.index(member) for member in order]]
                return [group_update(group, Change.JOINT_TARGET_SET, joint_targets(rows))]  # type: ignore[arg-type]
            return [
                entity_update(member, (Change.JOINT_TARGET_SET, joint_targets(row[None])))
                for member, row in zip(members, rows, strict=True)
            ]
        case SetJointPositions(entity, names, positions):
            return [
                entity_update(
                    ids[f"{entity}/{name}"],
                    (Change.JOINT_POSITION_SET, joint_targets(np.array([[q, 0.0, 0.0]]))),
                )
                for name, q in zip(names, positions, strict=True)
            ]
        case SetPose(entity, pose, twist):
            twist = twist or Twist()
            return [
                entity_update(
                    ids[entity],
                    pose_update([*pose.position, *pose.orientation]),
                    velocity_update([*twist.linear, *twist.angular]),
                )
            ]
        case ApplyWrench(entity, force, torque, point, duration):
            wrench = np.array([*force, *torque, *point, duration])
            payload = b"\0" * 8 + wrench.astype(">f8").tobytes()
            return [entity_update(ids[entity], (Change.WRENCH, payload))]
        case SetVelocity(entity, twist):
            drive = [twist.linear.x, twist.linear.y, twist.angular.z]
            return [entity_update(ids[entity], cbor_update(BASE_VELOCITY, drive))]
        case Spawn(entity, shape, size, pose, mass, rgba):
            description = {"shape": shape, "size": list(size), "mass": mass, "rgba": list(rgba)}
            return [
                add_entity(
                    0,
                    entity,
                    (
                        cbor_update(SPAWN, description),
                        pose_update([*pose.position, *pose.orientation]),
                    ),
                )
            ]
        case Remove(entity):
            return [delete_entity(ids[entity])]
        case Interact(entity, name):
            return [
                entity_update(
                    ids[entity], event_update(EVENTS["events/interact"], {"action": name})
                )
            ]
        case Attach(entity, to, body):
            return [entity_update(ids[entity], connection_update(ids[to], {"carried_by": body}))]
        case Detach(entity):
            return [entity_update(ids[entity], disconnection_update(0))]
        case Pause() | Resume():
            return [entity_update(WORLD, tag_update(PAUSED, isinstance(action, Pause)))]
        case Step(steps):
            return [entity_update(WORLD, event_update(EVENTS["events/step"], {"steps": steps}))]
        case Reset():
            return [entity_update(WORLD, event_update(EVENTS["events/reset"]))]
    raise TypeError(f"not an action: {action!r}")


def read_actions(message: WorldAction, world: WorldWriter) -> list[Action]:
    """A WorldAction's typed actions, by the paths the backend's writer gave its entities' ids."""
    actions: list[Action] = []
    for change in message.actions:
        for action in _decode(change, world):
            previous = actions[-1] if actions else None
            # A joint at a time on the wire, one command per entity here.
            if (
                isinstance(action, JointCommand)
                and isinstance(previous, JointCommand)
                and _mergeable(previous, action)
            ):
                actions[-1] = _merged(previous, action)
            else:
                actions.append(action)
    return actions


_JOINT_FIELDS = ("position", "velocity", "effort", "kp", "kd")


def _mergeable(a: JointCommand, b: JointCommand) -> bool:
    return a.entity == b.entity and all(
        (getattr(a, f) is None) == (getattr(b, f) is None) for f in _JOINT_FIELDS
    )


def _merged(a: JointCommand, b: JointCommand) -> JointCommand:
    fields = {
        f: None if getattr(a, f) is None else getattr(a, f) + getattr(b, f) for f in _JOINT_FIELDS
    }
    return JointCommand(a.entity, a.names + b.names, **fields)


def _optional(column: np.ndarray) -> tuple[float, ...] | None:
    return None if np.isnan(column).all() else tuple(map(float, column))


def _joint_command(entity: str, names: Sequence[str], rows: np.ndarray) -> JointCommand:
    position, velocity, kp, kd, effort = (_optional(rows[:, i]) for i in range(5))
    return JointCommand(entity, tuple(names), position, velocity, effort, kp, kd)


def _decode(change: Change, world: WorldWriter) -> list[Action]:
    paths = world.paths
    if change.type == Change.GROUP_UPDATE:
        robot, names = world.joint_groups[change.entity_id]
        update, rows = read_group_payloads(change)
        if update != Change.JOINT_TARGET_SET:
            raise ValueError(f"a group can only be sent joint targets, not update {update}")
        return [_joint_command(robot, names, rows)]
    if change.type == Change.DELETE_ENTITY:
        return [Remove(paths[change.entity_id])]
    path, updates = read_change(change)
    if change.type == Change.ADD_ENTITY:
        values = dict(updates)
        spawn = values[Change.CBOR_KEY_SET][1]
        p = values[Change.WORLD_POSE_SET]
        return [
            Spawn(
                path,
                spawn["shape"],
                tuple(spawn["size"]),
                Pose(*p),
                spawn["mass"],
                tuple(spawn["rgba"]),
            )
        ]  # type: ignore[arg-type]
    target = paths[change.entity_id]
    actions: list[Action] = []
    pose: Any = None
    for update, value in updates:
        if update == Change.JOINT_TARGET_SET:
            entity, name = target.split("/", 1)
            actions.append(_joint_command(entity, (name,), value[None]))
        elif update == Change.JOINT_POSITION_SET:
            entity, name = target.split("/", 1)
            actions.append(SetJointPositions(entity, (name,), (float(value[0]),)))
        elif update == Change.WORLD_POSE_SET:
            pose = Pose(*value)
            actions.append(SetPose(target, pose))
        elif update == Change.WORLD_VELOCITY_SET and pose is not None:
            actions[-1] = SetPose(
                target, pose, Twist(linear=value[:3].tolist(), angular=value[3:].tolist())
            )
        elif update == Change.WRENCH:
            _, w = value
            force, torque, point = (tuple(map(float, w[i : i + 3])) for i in (0, 3, 6))
            actions.append(ApplyWrench(target, force, torque, point, float(w[9])))  # type: ignore[arg-type]
        elif update == Change.CBOR_KEY_SET and value[0] == BASE_VELOCITY:
            vx, vy, wz = value[1]
            actions.append(SetVelocity(target, Twist(linear=[vx, vy, 0.0], angular=[0.0, 0.0, wz])))
        elif update in (Change.ADDED_TAG, Change.REMOVED_TAG) and value == PAUSED:
            actions.append(Pause() if update == Change.ADDED_TAG else Resume())
        elif update == Change.EVENT:
            event, cbor = value
            if event == EVENTS["events/interact"]:
                actions.append(Interact(target, cbor["action"]))
            elif event == EVENTS["events/step"]:
                actions.append(Step(cbor["steps"]))
            elif event == EVENTS["events/reset"]:
                actions.append(Reset())
        elif update == Change.ENTITY_CONNECTION_ADDED:
            other, cbor = value
            actions.append(Attach(target, paths[other], (cbor or {}).get("carried_by")))
        elif update == Change.ENTITY_CONNECTION_REMOVED:
            actions.append(Detach(target))
        else:
            raise ValueError(f"{target}: no action for update {update}")
    return actions
