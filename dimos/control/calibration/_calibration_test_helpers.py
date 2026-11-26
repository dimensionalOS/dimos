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

"""Test helpers: robot models from disk only, never downloaded, arms heavier than modelled, and
the toy arm's calibration deployed as ``dimos run`` deploys it.

Robot models come from Git LFS archives and cached description repositories.
Inside ``offline_robot_assets`` a cached checkout is used as it is (no fetch),
and a model that is not on disk raises ``ModelNotOnDiskError`` instead of
reaching the network; ``require_model`` turns that into a skip with the reason.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from datetime import datetime, timezone
import math
import os
import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin
from reactivex.disposable import Disposable

from dimos.control.calibration._fake_arm import FakeArm
from dimos.control.calibration._offline_models import (
    require_model,
)
from dimos.control.calibration._toy_arm import TOY_URDF as TOY_ARM_URDF, toy_arm
from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.module import CalibrationModule
from dimos.control.calibration.schema import (
    ArmCalibration,
    GravityCalibration,
    JointGains,
    LinkGravity,
    RobotModelRef,
)
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    specs_from_damiao_robot,
    urdf_sha256,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.robot import DamiaoBus, DamiaoMotor, DamiaoRobot


def robot_specs_or_skip(robot: DamiaoRobot, what: str, **options: Any) -> list[ArmCalibrationSpec]:
    """``specs_from_damiao_robot(robot, **options)`` when its gravity model is on disk, else skip."""
    require_model(robot.gravity_model, what)
    return specs_from_damiao_robot(robot, **options)


# A camera and bracket on the gripper, a heavier forearm: what the rig likely has.
# Link: (d mass kg, d(m*c) kg m).
PERTURBATION: dict[str, tuple[float, NDArray[np.float64]]] = {
    "left_link6": (0.25, np.array([-0.02, 0.0, -0.01])),
    "left_link3": (0.15, np.array([0.0, 0.0, -0.02])),
}


def perturbed_model(
    arm: ArmModel, perturbation: Mapping[str, tuple[float, NDArray[np.float64]]]
) -> pin.Model:
    """``arm``'s model with masses and first moments added to the named links: the "true" arm."""
    model = arm.model.copy()
    for link, (d_mass, d_moment) in perturbation.items():
        joint_id = int(model.frames[model.getFrameId(link, pin.FrameType.BODY)].parentJoint)
        inertia = model.inertias[joint_id]
        mass = inertia.mass + d_mass
        com = (inertia.mass * inertia.lever + d_moment) / mass
        model.inertias[joint_id] = pin.Inertia(mass, com, inertia.inertia)
    return model


# A toy arm and calibration file for the schema and bake tests.
SHA = "a" * 64
CREATED = datetime(2026, 11, 18, 9, 30, tzinfo=timezone.utc)

# A base, one moving link and a tool fixed to it.
TOY_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <!-- kept -->
  <link name="base"/>
  <link name="arm">
    <inertial>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tool">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <link name="other">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="tool_mount" type="fixed">
    <parent link="arm"/>
    <child link="tool"/>
    <origin xyz="0.3 0 0" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="other_mount" type="fixed">
    <parent link="base"/>
    <child link="other"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>
</robot>
"""
TOY_MODEL = RobotModelRef(name="toy", sha256=urdf_sha256(TOY_URDF))


def calibration(**fields: object) -> ArmCalibration:
    values: dict[str, object] = {
        "arm_id": "toy/arm",
        "robot_model": RobotModelRef(name="toy", sha256=SHA),
        "created": CREATED,
        "gravity": GravityCalibration(
            links={"arm": LinkGravity(mass_kg=2.0, com_m=(0.2, 0.0, 0.01), merged_links=["tool"])},
            fit={"ridge_nm": 0.1, "holdout_poses": ["pose_05"]},
        ),
        "gains": {"shoulder": JointGains(kp=80.0, kd=5.0)},
        "notes": "bench run",
    }
    values.update(fields)
    return ArmCalibration.model_validate(values)


TOY_ARM_MODEL = RobotModel.from_urdf_string(TOY_ARM_URDF)
TOY_ARM_JOINTS = ["yaw", "shoulder", "elbow"]
TOY_ROBOT = DamiaoRobot(
    buses=[
        DamiaoBus(
            arm=[
                DamiaoMotor(name, "DM4310", i + 1, i + 17) for i, name in enumerate(TOY_ARM_JOINTS)
            ],
            kp=[20.0] * 3,
            kd=[1.0] * 3,
        )
    ],
    model=TOY_ARM_MODEL,
    gravity_model=TOY_ARM_MODEL,
    base_link="base",
    joint_names=TOY_ARM_JOINTS,
    name="toy",
)


class ToyConnection(Module):
    """The toy arm's connection: a ``FakeArm`` ticked every reading period, in real time."""

    joint_state: Out[JointState]
    position_command: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.RLock()
        self._stopped = threading.Event()
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        # No deadman: ticked in real time, a stall in the test process would fire it.
        self._arm = FakeArm(toy_arm(), deadman_s=math.inf)
        self._arm.listener = self.joint_state.publish
        self.register_disposable(Disposable(self.position_command.subscribe(self._command)))
        threading.Thread(target=self._tick, daemon=True, name="toy-arm").start()

    def _tick(self) -> None:
        while not self._stopped.wait(self._arm.period_s):
            with self._lock:
                self._arm.tick()

    def _command(self, msg: JointState) -> None:
        with self._lock:
            self._arm.command(msg)

    @rpc
    def stop(self) -> None:
        self._stopped.set()
        super().stop()

    @rpc
    def get_config(self) -> dict[str, Any]:
        with self._lock:
            return self._arm.get_config()

    @rpc
    def set_config(self, patch: dict[str, Any], save: bool = False) -> dict[str, Any]:
        with self._lock:
            return self._arm.set_config(patch, save)


@contextmanager
def toy_run(**settings: Any) -> Iterator[None]:
    """The toy arm's calibration deployed in this process, its coordinator on the bus as
    ``dimos run`` serves it, so ``Dimos.connect()`` reaches it as ``dimos shell`` does."""
    blueprint = autoconnect(
        ToyConnection.blueprint(),
        CalibrationModule.blueprint(robot=TOY_ROBOT, floor_z_m=-0.45, min_x_m=-1.0, **settings),
    ).solidify()
    parsed = BlueprintConfigParser(blueprint).parse(
        environ=os.environ, overrides={"g": {"viewer": "none"}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed, reserve_rpc_name=CoordinatorRPC.NAME)
    try:
        coordinator.start_rpc_service()
        yield
    finally:
        coordinator.stop()
