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

"""A retained connection controls real MuJoCo physics across worker processes."""

from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
import threading
from typing import ClassVar

import pytest
from reactivex.disposable import Disposable

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.connection.device import ConnectionDevice
from dimos.control.contract.convert import motor_command_to_values
from dimos.control.contract.limits import PD_INTERFACES, Limits
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.module import SimulationModule
from dimos.sim2.spec import SimulatedRobot


class _ArmConfig(ConnectionConfig):
    source: str = "left"
    robot: SimulatedRobot
    pd: bool = True
    deadman_timeout_s: float = 2.0


class _ArmConnection(ConnectionModule):
    """A test robot deployed in a worker, exercising the actual base lifecycle."""

    config: _ArmConfig
    default_sim_scene: ClassVar[str] = "workbench.xml"
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]

    @staticmethod
    def simulated_robot(config: _ArmConfig, scene: Path) -> SimulatedRobot:
        return config.robot

    @property
    def limits(self):
        return {
            f"{name}/{interface}": Limits()
            for name in self.config.robot.joint_names
            for interface in PD_INTERFACES
        }

    def create_hardware_device(self) -> ConnectionDevice:
        raise AssertionError("simulation must not construct a hardware device")

    @rpc
    def start(self):
        super().start()
        self.register_disposable(Disposable(self.motor_command.subscribe(self._command)))

    def _command(self, message):
        names = self.config.robot.joint_names
        self.command(motor_command_to_values(message, names))

    @rpc
    def halt_now(self):
        self.halt()


class _Controller(Module):
    """Exercise public robot streams from a separate worker, without a sim client."""

    motor_command: Out[MotorCommandArray]
    motor_states: In[JointState]
    imu: In[Imu]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._feedback = threading.Condition()
        self._joints = None
        self._imu = None

    @rpc
    def start(self):
        super().start()
        self.register_disposable(Disposable(self.motor_states.subscribe(self._on_joints)))
        self.register_disposable(Disposable(self.imu.subscribe(self._on_imu)))

    def _on_joints(self, message):
        with self._feedback:
            self._joints = message
            self._feedback.notify_all()

    def _on_imu(self, message):
        with self._feedback:
            self._imu = message
            self._feedback.notify_all()

    @rpc
    def wait_for_position(self, minimum):
        with self._feedback:
            ready = self._feedback.wait_for(
                lambda: self._joints is not None
                and self._imu is not None
                and self._joints.position[0] >= minimum,
                timeout=10.0,
            )
            if not ready:
                raise TimeoutError(f"public feedback never reached shoulder position {minimum}")
            return {
                "names": list(self._joints.name),
                "positions": list(self._joints.position),
                "timestamp": self._joints.ts,
                "joint_frame": self._joints.frame_id,
                "imu_frame": self._imu.frame_id,
            }

    @rpc
    def wait_for_stopped(self, after):
        with self._feedback:
            ready = self._feedback.wait_for(
                lambda: self._joints is not None
                and self._joints.ts > after
                and abs(self._joints.velocity[0]) < 0.01,
                timeout=10.0,
            )
            if not ready:
                raise TimeoutError("shoulder did not stop after halt")
            return list(self._joints.position)

    @rpc
    def move(self):
        self.motor_command.publish(
            MotorCommandArray(
                q=[0.5, 850.0],
                dq=[0.0, 0.0],
                kp=[2.0, 0.0],
                kd=[1.0, 0.0],
                tau=[0.0, 0.0],
            )
        )


def test_retained_connection_delivers_commands_and_feedback_through_the_world(
    world_config, scene_path, robot_path
):
    # Parallel axes isolate servo hold from inertial deflection during arm motion.
    robot_path.write_text(robot_path.read_text().replace('axis="1 0 0"', 'axis="0 1 0"'))
    robot = replace(
        world_config["robots"]["left"],
        joint_names=("left/shoulder", "left/gripper"),
        position_only_joints=("left/gripper",),
        command_timeout=2.0,
    )
    blueprint = autoconnect(
        _ArmConnection.blueprint(robot=robot, instance_name="arm"),
        _Controller.blueprint(),
    )
    with ExitStack() as cleanup:
        app = ModuleCoordinator.build(
            blueprint.global_config(
                simulation="mujoco",
                scene_package=str(scene_path),
                n_workers=1,
                viewer="none",
                transport="zenoh",
            )
        )
        cleanup.callback(app.stop)
        connection = app.get_instance(_ArmConnection)
        controller = app.get_instance(_Controller)
        world = app.get_instance(SimulationModule)

        observed = controller.wait_for_position(0.0)
        assert observed["names"] == ["left/shoulder", "left/gripper"]
        assert observed["positions"] == pytest.approx([0.1, 850.0], abs=1e-3)
        assert observed["joint_frame"] == observed["imu_frame"] == "left/base"
        assert connection.status().connected

        controller.move()
        moved = controller.wait_for_position(0.3)
        assert moved["positions"][0] >= 0.3
        status = connection.status()
        assert status.last_command_time is not None
        assert status.last_error is None
        assert status.last_rejection is None
        assert world.status()["rejected_commands"] == {}

        connection.halt_now()
        stopped = controller.wait_for_stopped(moved["timestamp"])
        assert 0.3 <= stopped[0] < 0.45
        assert stopped[1] == pytest.approx(850.0, abs=1e-3)
        assert world.status()["rejected_commands"] == {}
