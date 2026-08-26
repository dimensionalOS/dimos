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

"""Provider-neutral public inputs for one exact simulation episode."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from itertools import chain
import math
from types import MappingProxyType
from typing import TypeVar

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _required_text(value: str, label: str) -> str:
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _vector3(value: Vector3 | None, label: str) -> Vector3 | None:
    if value is None:
        return None
    normalized = tuple(float(component) for component in value)
    if len(normalized) != 3 or not all(math.isfinite(component) for component in normalized):
        raise ValueError(f"{label} must contain three finite values")
    return normalized


def _quaternion(value: Quaternion | None, label: str) -> Quaternion | None:
    if value is None:
        return None
    normalized = tuple(float(component) for component in value)
    if len(normalized) != 4 or not all(math.isfinite(component) for component in normalized):
        raise ValueError(f"{label} must contain four finite values")
    norm = math.sqrt(sum(component * component for component in normalized))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{label} must be normalized")
    return normalized


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeRole:
    """One public semantic entity selected for a task role."""

    role_id: str
    entity_id: str
    name: str

    def __post_init__(self) -> None:
        for field_name in ("role_id", "entity_id", "name"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"public role {field_name}"),
            )


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeRegion:
    """One exact scene region selected by the task."""

    role_id: str
    entity_role_id: str
    entity_id: str
    region_id: str
    kind: str
    position: Vector3 | None = None
    quaternion_xyzw: Quaternion | None = None

    def __post_init__(self) -> None:
        for field_name in ("role_id", "entity_role_id", "entity_id", "region_id", "kind"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"public region {field_name}"),
            )
        object.__setattr__(
            self,
            "position",
            _vector3(self.position, "public region position"),
        )
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _quaternion(self.quaternion_xyzw, "public region quaternion_xyzw"),
        )


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeJoint:
    """One articulated joint selected as a public task input."""

    role_id: str
    entity_role_id: str
    entity_id: str
    joint_id: str

    def __post_init__(self) -> None:
        for field_name in ("role_id", "entity_role_id", "entity_id", "joint_id"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"public joint {field_name}"),
            )


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeDevice:
    """One controllable device selected as a public task input."""

    role_id: str
    entity_role_id: str
    entity_id: str
    device_id: str

    def __post_init__(self) -> None:
        for field_name in ("role_id", "entity_role_id", "entity_id", "device_id"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"public device {field_name}"),
            )


class PublicEpisodeTargetKind(StrEnum):
    NAVIGATION = "navigation"
    INTERACTION = "interaction"


class MultiObjectRelationKind(StrEnum):
    CONTAINED_IN = "contained-in"
    ON = "on"
    STACKED_ON = "stacked-on"


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeTarget:
    """A target deliberately exposed to a deterministic benchmark driver."""

    target_id: str
    kind: PublicEpisodeTargetKind
    entity_role_id: str
    entity_id: str
    region_role_id: str | None = None
    region_id: str | None = None
    joint_role_id: str | None = None
    joint_id: str | None = None
    device_role_id: str | None = None
    device_id: str | None = None
    position: Vector3 | None = None
    quaternion_xyzw: Quaternion | None = None

    def __post_init__(self) -> None:
        for field_name in ("target_id", "entity_role_id", "entity_id"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"public target {field_name}"),
            )
        object.__setattr__(self, "kind", PublicEpisodeTargetKind(self.kind))
        for field_name in (
            "region_role_id",
            "region_id",
            "joint_role_id",
            "joint_id",
            "device_role_id",
            "device_id",
        ):
            value = getattr(self, field_name)
            if value is not None:
                object.__setattr__(
                    self,
                    field_name,
                    _required_text(value, f"public target {field_name}"),
                )
        object.__setattr__(
            self,
            "position",
            _vector3(self.position, "public target position"),
        )
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _quaternion(self.quaternion_xyzw, "public target quaternion_xyzw"),
        )
        components = (
            (self.region_role_id, self.region_id),
            (self.joint_role_id, self.joint_id),
            (self.device_role_id, self.device_id),
        )
        if any((role_id is None) != (component_id is None) for role_id, component_id in components):
            raise ValueError("public target component role and ID must be declared together")
        component_count = sum(role_id is not None for role_id, _component_id in components)
        if component_count != 1:
            raise ValueError("public target must identify exactly one region, joint, or device")
        if self.kind is PublicEpisodeTargetKind.NAVIGATION and (
            self.region_id is None or self.position is None
        ):
            raise ValueError("public navigation target requires a positioned region")


@dataclass(frozen=True, kw_only=True)
class NavigationTask:
    goal: PublicEpisodeTarget


@dataclass(frozen=True, kw_only=True)
class LiftTask:
    object: PublicEpisodeRole
    source: PublicEpisodeRegion


@dataclass(frozen=True, kw_only=True)
class PlacementTask:
    object: PublicEpisodeRole
    source: PublicEpisodeRegion
    goal: PublicEpisodeRegion


@dataclass(frozen=True, kw_only=True)
class ContainmentTask:
    object: PublicEpisodeRole
    source: PublicEpisodeRegion
    interior: PublicEpisodeRegion


@dataclass(frozen=True, kw_only=True)
class FixtureTask:
    fixture: PublicEpisodeRole
    joint: PublicEpisodeJoint


@dataclass(frozen=True, kw_only=True)
class DeviceTask:
    fixture: PublicEpisodeRole
    device: PublicEpisodeDevice


@dataclass(frozen=True, kw_only=True)
class MultiObjectRelation:
    subject: PublicEpisodeRole
    kind: MultiObjectRelationKind
    target: PublicEpisodeRole
    region: PublicEpisodeRegion | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", MultiObjectRelationKind(self.kind))


@dataclass(frozen=True, kw_only=True)
class MultiObjectTask:
    objects: tuple[PublicEpisodeRole, ...]
    relations: tuple[MultiObjectRelation, ...]

    def __post_init__(self) -> None:
        if not self.objects:
            raise ValueError("multi-object task requires at least one object")
        if not self.relations:
            raise ValueError("multi-object task requires at least one goal relation")


_Reference = TypeVar(
    "_Reference",
    PublicEpisodeRole,
    PublicEpisodeRegion,
    PublicEpisodeJoint,
    PublicEpisodeDevice,
    PublicEpisodeTarget,
)


def _reference_mapping(
    values: Mapping[str, _Reference],
    *,
    expected_type: type[_Reference],
    key_attribute: str,
    label: str,
) -> Mapping[str, _Reference]:
    normalized = dict(values)
    for key, value in normalized.items():
        if not isinstance(value, expected_type):
            raise TypeError(f"{label} values must use the public episode reference type")
        normalized_key = _required_text(str(key), f"{label} key")
        if normalized_key != getattr(value, key_attribute):
            raise ValueError(f"{label} key {normalized_key!r} does not match its value")
    return MappingProxyType(normalized)


def _get_reference(
    values: Mapping[str, _Reference],
    key: str,
    label: str,
) -> _Reference:
    try:
        return values[key]
    except KeyError as error:
        available = ", ".join(sorted(values)) or "none"
        raise KeyError(f"unknown public {label} {key!r}; available: {available}") from error


@dataclass(frozen=True, kw_only=True)
class PublicEpisodeContext:
    """Public task inputs resolved by an exact episode provider.

    This intentionally excludes goal results, contacts, raw simulator state, and
    perception answers. Those remain private evaluator data.
    """

    case_id: str
    task_family_id: str
    instruction: str
    roles: Mapping[str, PublicEpisodeRole]
    regions: Mapping[str, PublicEpisodeRegion] = field(default_factory=dict)
    joints: Mapping[str, PublicEpisodeJoint] = field(default_factory=dict)
    devices: Mapping[str, PublicEpisodeDevice] = field(default_factory=dict)
    targets: Mapping[str, PublicEpisodeTarget] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "case_id", _required_text(self.case_id, "public case_id"))
        object.__setattr__(
            self,
            "task_family_id",
            _required_text(self.task_family_id, "public task_family_id"),
        )
        object.__setattr__(
            self,
            "instruction",
            _required_text(self.instruction, "public episode instruction"),
        )
        for field_name, expected_type, key_attribute in (
            ("roles", PublicEpisodeRole, "role_id"),
            ("regions", PublicEpisodeRegion, "role_id"),
            ("joints", PublicEpisodeJoint, "role_id"),
            ("devices", PublicEpisodeDevice, "role_id"),
            ("targets", PublicEpisodeTarget, "target_id"),
        ):
            object.__setattr__(
                self,
                field_name,
                _reference_mapping(
                    getattr(self, field_name),
                    expected_type=expected_type,
                    key_attribute=key_attribute,
                    label=f"public episode {field_name}",
                ),
            )
        entity_ids = {role.entity_id for role in self.roles.values()}
        if (
            any(region.entity_id not in entity_ids for region in self.regions.values())
            or any(joint.entity_id not in entity_ids for joint in self.joints.values())
            or any(device.entity_id not in entity_ids for device in self.devices.values())
        ):
            raise ValueError("public episode component references an entity without a public role")
        if any(target.entity_id not in entity_ids for target in self.targets.values()):
            raise ValueError("public episode target references an entity without a public role")
        for entity_role_id, entity_id in chain(
            ((region.entity_role_id, region.entity_id) for region in self.regions.values()),
            ((joint.entity_role_id, joint.entity_id) for joint in self.joints.values()),
            ((device.entity_role_id, device.entity_id) for device in self.devices.values()),
        ):
            role = self.roles.get(entity_role_id)
            if role is None or role.entity_id != entity_id:
                raise ValueError("public episode component has an inconsistent entity role")
        for target in self.targets.values():
            role = self.roles.get(target.entity_role_id)
            if role is None or role.entity_id != target.entity_id:
                raise ValueError("public episode target has an inconsistent entity role")

    def role(self, role_id: str) -> PublicEpisodeRole:
        return _get_reference(self.roles, role_id, "role")

    def region(self, role_id: str) -> PublicEpisodeRegion:
        return _get_reference(self.regions, role_id, "region")

    def joint(self, role_id: str) -> PublicEpisodeJoint:
        return _get_reference(self.joints, role_id, "joint")

    def device(self, role_id: str) -> PublicEpisodeDevice:
        return _get_reference(self.devices, role_id, "device")

    def target(self, target_id: str) -> PublicEpisodeTarget:
        return _get_reference(self.targets, target_id, "target")

    def navigation(self) -> NavigationTask:
        self._require_family("navigation", "navigate-to-region")
        return NavigationTask(goal=self.target("goal"))

    def lift(self) -> LiftTask:
        self._require_family("lift", "lift-object")
        return LiftTask(object=self.role("object"), source=self.region("source"))

    def placement(self) -> PlacementTask:
        region_roles = {
            "object-on-support": ("source", "destination"),
            "remove-object-from-fixture": ("source", "destination"),
            "room-transfer": ("source_surface", "destination_surface"),
        }
        try:
            source_role, goal_role = region_roles[self.task_family_id]
        except KeyError as error:
            raise ValueError(
                f"task family {self.task_family_id!r} does not expose a placement view"
            ) from error
        return PlacementTask(
            object=self.role("object"),
            source=self.region(source_role),
            goal=self.region(goal_role),
        )

    def containment(self) -> ContainmentTask:
        interior_roles = {
            "object-in-receptacle": "destination",
            "insert-object-in-fixture": "fixture_inside",
            "store-object-and-close-fixture": "fixture_inside",
        }
        try:
            interior_role = interior_roles[self.task_family_id]
        except KeyError as error:
            raise ValueError(
                f"task family {self.task_family_id!r} does not expose a containment view"
            ) from error
        return ContainmentTask(
            object=self.role("object"),
            source=self.region("source"),
            interior=self.region(interior_role),
        )

    def fixture(self) -> FixtureTask:
        self._require_family(
            "fixture",
            "open-fixture",
            "close-fixture",
            "insert-object-in-fixture",
            "remove-object-from-fixture",
            "store-object-and-close-fixture",
        )
        return FixtureTask(
            fixture=self.role("fixture"),
            joint=self.joint("fixture_joint"),
        )

    def device_interaction(self) -> DeviceTask:
        self._require_family("device", "set-device-state")
        return DeviceTask(
            fixture=self.role("fixture"),
            device=self.device("device"),
        )

    def multi_object(self) -> MultiObjectTask:
        if self.task_family_id == "collect-objects-in-receptacle":
            target = self.role("target")
            destination = self.region("destination")
            objects = (self.role("first_object"), self.role("second_object"))
            relations = tuple(
                MultiObjectRelation(
                    subject=object_role,
                    kind=MultiObjectRelationKind.CONTAINED_IN,
                    target=target,
                    region=destination,
                )
                for object_role in objects
            )
        elif self.task_family_id == "rearrange-objects":
            objects = (self.role("first_object"), self.role("second_object"))
            relations = (
                MultiObjectRelation(
                    subject=objects[0],
                    kind=MultiObjectRelationKind.CONTAINED_IN,
                    target=self.role("containment_target"),
                    region=self.region("containment_destination"),
                ),
                MultiObjectRelation(
                    subject=objects[1],
                    kind=MultiObjectRelationKind.ON,
                    target=self.role("support_target"),
                    region=self.region("support_destination"),
                ),
            )
        elif self.task_family_id == "stack-objects":
            objects = (self.role("object"), self.role("target"))
            relations = (
                MultiObjectRelation(
                    subject=objects[0],
                    kind=MultiObjectRelationKind.STACKED_ON,
                    target=objects[1],
                ),
            )
        else:
            raise ValueError(
                f"task family {self.task_family_id!r} does not expose a multi-object view"
            )
        return MultiObjectTask(objects=objects, relations=relations)

    def _require_family(self, view: str, *family_ids: str) -> None:
        if self.task_family_id not in family_ids:
            raise ValueError(f"task family {self.task_family_id!r} does not expose a {view} view")


__all__ = [
    "ContainmentTask",
    "DeviceTask",
    "FixtureTask",
    "LiftTask",
    "MultiObjectRelation",
    "MultiObjectRelationKind",
    "MultiObjectTask",
    "NavigationTask",
    "PlacementTask",
    "PublicEpisodeContext",
    "PublicEpisodeDevice",
    "PublicEpisodeJoint",
    "PublicEpisodeRegion",
    "PublicEpisodeRole",
    "PublicEpisodeTarget",
    "PublicEpisodeTargetKind",
]
