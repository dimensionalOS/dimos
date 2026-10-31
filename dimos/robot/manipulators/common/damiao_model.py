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

"""Lazy control/deployment geometry and numerical Damiao model checks."""

from __future__ import annotations

from dataclasses import dataclass, replace
from hashlib import sha256
import importlib
import os
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.manipulators.common.damiao_connection import DamiaoRobot

logger = setup_logger()


class CanonicalDescription(os.PathLike[str]):
    """Materialize canonical geometry before adding logical joints or namespacing."""

    def __init__(self, model: RobotModel, grippers: tuple[str, ...] = ()) -> None:
        self.model = model
        self.grippers = grippers

    def cbor_encode(self) -> tuple[RobotModel, tuple[str, ...]]:
        return self.model, self.grippers

    @classmethod
    def cbor_decode(cls, values: tuple[RobotModel, tuple[str, ...]]) -> CanonicalDescription:
        return cls(*values)

    def __fspath__(self) -> str:
        loaded = self.model.load()
        root = ET.fromstring(loaded.xml)
        names = {joint.name for joint in loaded.joints}
        for name in self.grippers:
            if name in names:
                raise ValueError(f"logical gripper joint already exists: {name}")
            link = f"{name}_control_link"
            ET.SubElement(root, "link", name=link)
            joint = ET.SubElement(root, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link=loaded.root_link)
            ET.SubElement(joint, "child", link=link)
            ET.SubElement(joint, "limit", lower="0", upper="1", velocity="0", effort="0")
        data = ET.tostring(root, encoding="unicode")
        folder = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "derived" / "damiao_control"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{sha256(data.encode()).hexdigest()}.urdf"
        if not path.exists():
            with tempfile.NamedTemporaryFile(mode="w", dir=folder, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            try:
                temporary.replace(path)
            finally:
                temporary.unlink(missing_ok=True)
        return str(path)


def local_control_model(robot: DamiaoRobot) -> RobotModel:
    return RobotModel.from_file(
        CanonicalDescription(
            robot.planning.model, tuple(gripper.name for gripper in robot.grippers)
        )
    )


@dataclass(frozen=True)
class Deployment:
    control: RobotModel
    planning: RobotModelConfig
    arm_joints: tuple[str, ...]
    gripper_joints: tuple[str, ...]

    @property
    def joints(self) -> tuple[str, ...]:
        return self.arm_joints + self.gripper_joints


def deployment(robot: DamiaoRobot, source: str) -> Deployment:
    names = {name: f"{source}/{name}" for name in robot.joint_names}
    planning = robot.planning.model_copy(
        update={
            "model": RobotModel.from_file(
                CanonicalDescription(robot.planning.model)
            ).with_renamed_joints({name: names[name] for name in robot.planning.joint_names}),
            "joint_names": [names[name] for name in robot.planning.joint_names],
            "planning_groups": [
                replace(group, joint_names=tuple(names[name] for name in group.joint_names))
                for group in robot.planning.planning_groups
            ],
            "gripper_hardware_id": source if len(robot.grippers) == 1 else None,
        }
    )
    return Deployment(
        control=local_control_model(robot).with_renamed_joints(names),
        planning=planning,
        arm_joints=tuple(names[joint.name] for arm in robot.arms for joint in arm.joints),
        gripper_joints=tuple(names[gripper.name] for gripper in robot.grippers),
    )


class PhysicalModel:
    """Numerical arm model: exact coordinates, bounded feedback and gravity."""

    def __init__(self, robot: DamiaoRobot) -> None:
        self._gravity_comp = robot.gravity_comp
        self._margin = robot.feedback_margin_rad
        self._names = tuple(joint.name for arm in robot.arms for joint in arm.joints)
        # Physical model support is only needed when a native session is opened.
        pinocchio = importlib.import_module("pinocchio")
        self._pinocchio = pinocchio
        model = pinocchio.buildModelFromXML(robot.physical_model.load().xml)
        locked = [
            index
            for index, name in enumerate(model.names)
            if index and str(name) not in self._names
        ]
        if locked:
            model = pinocchio.buildReducedModel(model, locked, pinocchio.neutral(model))
        count = len(self._names)
        if model.nq != count or model.nv != count:
            raise ValueError("physical model dimensions do not match angular joints")
        if tuple(str(name) for name in model.names[1:]) != self._names:
            raise ValueError("physical model joint order does not match angular joints")
        self._lower = np.asarray(model.lowerPositionLimit, dtype=np.float64)
        self._upper = np.asarray(model.upperPositionLimit, dtype=np.float64)
        if (
            self._lower.shape != (count,)
            or self._upper.shape != (count,)
            or not np.isfinite([self._lower, self._upper]).all()
            or np.any(self._lower >= self._upper)
        ):
            raise ValueError("physical model contains invalid arm position limits")
        self._model = model
        self._data = model.createData()
        self._warned: set[tuple[str, str]] = set()

    def gravity(self, positions: NDArray[np.float64]) -> NDArray[np.float64]:
        if positions.shape != (len(self._names),) or not np.isfinite(positions).all():
            raise ValueError("arm feedback contains non-finite or invalid positions")
        gravity = (
            np.asarray(
                self._pinocchio.computeGeneralizedGravity(self._model, self._data, positions),
                dtype=np.float64,
            )
            if self._gravity_comp
            else np.zeros(len(self._names))
        )
        if gravity.shape != positions.shape or not np.isfinite(gravity).all():
            raise ValueError("gravity compensation produced invalid torques")
        return gravity

    def normalize(self, positions: list[float]) -> list[float]:
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != self._lower.shape or not np.isfinite(q).all():
            raise RuntimeError("arm feedback contains non-finite or invalid positions")
        for index, name in enumerate(self._names):
            lower, upper, value = self._lower[index], self._upper[index], q[index]
            if lower <= value <= upper:
                continue
            boundary = "lower" if value < lower else "upper"
            limit = lower if value < lower else upper
            if abs(value - limit) > self._margin + 1e-12:
                raise RuntimeError(
                    f"{name} reported {value} outside [{lower}, {upper}] by more than {self._margin} rad"
                )
            if (name, boundary) not in self._warned:
                logger.warning(
                    "Clamping Damiao joint feedback at position limit",
                    joint=name,
                    value=value,
                    limit=limit,
                )
                self._warned.add((name, boundary))
            q[index] = limit
        return [float(value) for value in q]
