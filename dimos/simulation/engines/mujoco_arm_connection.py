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

"""An arm simulated by the blueprint's ``MujocoSimModule``, as a connection module."""

from __future__ import annotations

import threading
import time
from typing import Any

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path
from dimos.simulation.engines.spec import MujocoSimSpec

# How long connect waits for the sim's shared memory to appear, then to update, in seconds.
_ATTACH_TIMEOUT_S = 30.0
_READY_TIMEOUT_S = 60.0
_POLL_S = 0.1


class MujocoArmConnectionConfig(ConnectionConfig):
    """Which arm the sim drives, and which of its model joints is the gripper.

    Attributes:
        model: The arm's URDF. Its moving joints other than ``gripper_joint``
            are the sim's arm joints, in the scene's actuator order.
        gripper_joint: The model joint the scene's gripper actuator drives,
            e.g. "drive_joint", or None for an arm without one. It reads and
            takes the scene joint's own value (rad for the xArm Gripper).
        state_rate_hz: How many times a second the joints are read.
    """

    source: str = "arm"
    model: RobotModel
    gripper_joint: str | None = None
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _position_joints(self) -> Self:
        if "joint_commands" not in self.model_fields_set:
            self.joint_commands = {
                joint.name: (Interface.POSITION,) for joint in movable_joints(self.model)
            }
        return self


class MujocoArmConnection(ConnectionModule):
    """An arm in the blueprint's MuJoCo scene, with the ports a real arm's connection has.

    Joint targets come in on ``position_command`` and joint speeds on
    ``velocity_command`` (rad, rad/s); the gripper takes positions only.
    ``joint_state`` goes out ``state_rate_hz`` times a second. A halt holds
    the arm where it is measured.
    """

    config: MujocoArmConnectionConfig

    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]

    _mujoco_sim: MujocoSimSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        gripper = self.config.gripper_joint
        names = [joint.name for joint in movable_joints(self.config.model)]
        if gripper is not None and gripper not in names:
            raise ValueError(f"gripper joint {gripper!r} is not a moving joint of the model")
        self._arm = [self._key(name) for name in names if name != gripper]
        self._gripper = self._key(gripper) if gripper is not None else None
        self._shm: ManipShmReader | None = None
        # The scene's gripper range; the shared memory carries the gripper reflected in it.
        self._gripper_span = 0.0
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._listen(self.position_command, Interface.POSITION)
        self._listen(self.velocity_command, Interface.VELOCITY)
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="MujocoArmConnection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        shm, self._shm = self._shm, None
        if shm is not None:
            shm.cleanup()

    def connect(self) -> None:
        scene = self._mujoco_sim.scene_path()
        shm = ManipShmReader.wait_for_live(
            shm_key_from_path(scene),
            attach_timeout_s=_ATTACH_TIMEOUT_S,
            ready_timeout_s=_READY_TIMEOUT_S,
            poll_s=_POLL_S,
        )
        if shm is None:
            raise ConnectionError(f"the MuJoCo sim of {scene} never reported its joints")
        grippers = 0 if self._gripper is None else 1
        arm_joints, num_joints = shm.arm_joints(), shm.num_joints()
        if arm_joints != len(self._arm) or num_joints != len(self._arm) + grippers:
            shm.cleanup()
            raise ValueError(
                f"the sim of {scene} has {arm_joints} arm joints of {num_joints}; "
                f"the model has {len(self._arm)} arm joints and {grippers} gripper"
            )
        low, high = shm.read_gripper_range()
        self._gripper_span = low + high
        self._shm = shm

    def write(self, values: Values) -> None:
        shm = self._live()
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._arm]
        speeds = [values.get(f"{joint}/{Interface.VELOCITY}") for joint in self._arm]
        if any(speed is not None for speed in speeds):
            if any(target is not None for target in targets):
                raise ValueError("a command gives the arm positions or speeds, not both")
            shm.write_velocity_command([speed or 0.0 for speed in speeds])
        elif any(target is not None for target in targets):
            # A joint left out stays where it is.
            measured = shm.read_positions(len(self._arm))
            shm.write_position_command(
                [m if t is None else t for t, m in zip(targets, measured, strict=True)]
            )
        if self._gripper is not None:
            gripper = values.get(f"{self._gripper}/{Interface.POSITION}")
            if gripper is not None:
                shm.write_gripper_command(self._gripper_span - gripper)

    def halt(self) -> None:
        shm = self._shm
        if shm is not None and shm.is_live():
            shm.write_position_command(shm.read_positions(len(self._arm)))

    def _listen(self, port: In[JointState], field: Interface) -> None:
        """Pass each message on ``port`` to ``command``, reading only its ``field``."""
        self.register_disposable(
            Disposable(
                port.subscribe(lambda msg: self.command(joint_state_to_values(msg, (field,))))
            )
        )

    def _key(self, name: str) -> str:
        """``name`` under this robot, e.g. "arm/joint1"."""
        return name if SEPARATOR in name else f"{self.config.source}{SEPARATOR}{name}"

    def _live(self) -> ManipShmReader:
        shm = self._shm
        if shm is None or not shm.is_live():
            raise ConnectionError("the MuJoCo sim stopped updating its joints")
        return shm

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.wait(period):
            try:
                shm = self._live()
                dof = len(self._arm)
                names, positions = list(self._arm), shm.read_positions(dof)
                velocities, efforts = shm.read_velocities(dof), shm.read_efforts(dof)
                if self._gripper is not None:
                    names.append(self._gripper)
                    positions.append(self._gripper_span - shm.read_gripper_position())
                    velocities.append(0.0)
                    efforts.append(0.0)
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        name=names,
                        position=positions,
                        velocity=velocities,
                        effort=efforts,
                    )
                )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
