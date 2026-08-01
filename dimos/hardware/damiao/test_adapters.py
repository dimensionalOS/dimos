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

from __future__ import annotations

import pytest

from dimos.hardware.damiao.arm_adapter import DamiaoArmAdapter
from dimos.hardware.damiao.runtime import DamiaoGroupState
from dimos.hardware.damiao.specs import (
    DamiaoArmSpec,
    DamiaoBusSpec,
    DamiaoJointGroupSpec,
    DamiaoMotorSpec,
    DamiaoRobotSpec,
)
from dimos.hardware.damiao.whole_body_adapter import DamiaoWholeBodyAdapter
from dimos.hardware.manipulators.spec import ControlMode


class _FakeRuntime:
    def __init__(self, *, fresh: bool = True, write_ok: bool = True) -> None:
        self.fresh = fresh
        self.write_ok = write_ok
        self.connected = False
        self.enabled = False
        self.disconnect_calls = 0
        self.batched_calls = 0
        self.writes: list[
            tuple[str, list[float], list[float], list[float], list[float], list[float]]
        ] = []
        self.loaded_gravity_models: list[tuple[str, str | None]] = []
        self.states = {
            "left": DamiaoGroupState(q=[0.1], dq=[0.2], tau=[0.3]),
            "right": DamiaoGroupState(q=[-0.1], dq=[-0.2], tau=[-0.3]),
            "arm": DamiaoGroupState(q=[0.4, -0.4], dq=[0.5, -0.5], tau=[0.6, -0.6]),
        }

    def connect(self) -> bool:
        self.connected = True
        return True

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.connected = False
        self.enabled = False

    def enable(self) -> bool:
        self.enabled = True
        return True

    def disable(self) -> bool:
        self.enabled = False
        return True

    def is_enabled(self) -> bool:
        return self.enabled

    def refresh_group_state(self, group_name: str, *, force: bool = False) -> DamiaoGroupState:
        del force
        return self.states[group_name]

    def has_group_states(self, group_names: tuple[str, ...]) -> bool:
        return self.fresh and all(group_name in self.states for group_name in group_names)

    def read_group_states(self, group_names: tuple[str, ...]) -> list[DamiaoGroupState]:
        if not self.has_group_states(group_names):
            raise RuntimeError("stale state")
        return [self.states[group_name] for group_name in group_names]

    def write_group_mit_commands(
        self,
        *,
        group_name: str,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> bool:
        if not self.write_ok:
            return False
        self.writes.append((group_name, list(q), list(dq), list(kp), list(kd), list(tau)))
        return True

    def write_groups_mit_commands(
        self,
        commands: dict[str, tuple[list[float], list[float], list[float], list[float], list[float]]],
    ) -> bool:
        self.batched_calls += 1
        if not self.write_ok:
            return False
        for group_name, values in commands.items():
            q, dq, kp, kd, tau = values
            self.writes.append((group_name, list(q), list(dq), list(kp), list(kd), list(tau)))
        return True

    def load_gravity_model(self, group_name: str, model_path: str | None = None) -> None:
        self.loaded_gravity_models.append((group_name, model_path))
        return None


def _arm_spec() -> DamiaoArmSpec:
    return DamiaoArmSpec(
        name="test_damiao",
        vendor="Damiao",
        model="TestArm",
        motors=(
            DamiaoMotorSpec("j1", "DM4310", 0x01, 0x11),
            DamiaoMotorSpec("j2", "DM4310", 0x02, 0x12),
        ),
        position_lower=(-1.0, -2.0),
        position_upper=(1.0, 2.0),
        velocity_max=(3.0, 4.0),
        kp=(5.0, 6.0),
        kd=(0.1, 0.2),
        gravity_torque_limits=(7.0, 8.0),
    )


def _whole_body_spec() -> DamiaoRobotSpec:
    return DamiaoRobotSpec(
        name="test_body",
        vendor="Damiao",
        model="TestBody",
        buses={
            "left_can": DamiaoBusSpec(address="can1", fd=True),
            "right_can": DamiaoBusSpec(address="can0", fd=True),
        },
        groups={
            "left": DamiaoJointGroupSpec(
                bus_name="left_can",
                motors=(DamiaoMotorSpec("left_joint", "DM4310", 0x01, 0x11),),
                position_lower=(-1.0,),
                position_upper=(1.0,),
                velocity_max=(3.0,),
                kp=(5.0,),
                kd=(0.1,),
            ),
            "right": DamiaoJointGroupSpec(
                bus_name="right_can",
                motors=(DamiaoMotorSpec("right_joint", "DM4310", 0x01, 0x11),),
                position_lower=(-2.0,),
                position_upper=(2.0,),
                velocity_max=(4.0,),
                kp=(6.0,),
                kd=(0.2,),
            ),
        },
    )


def test_robot_spec_rejects_unknown_group_bus() -> None:
    spec = DamiaoRobotSpec(
        name="bad",
        vendor="Damiao",
        model="Bad",
        buses={"can": DamiaoBusSpec()},
        groups={
            "arm": DamiaoJointGroupSpec(
                bus_name="missing",
                motors=(DamiaoMotorSpec("j1", "DM4310", 0x01, 0x11),),
                position_lower=(-1.0,),
                position_upper=(1.0,),
                velocity_max=(1.0,),
                kp=(1.0,),
                kd=(0.1,),
            )
        },
    )

    with pytest.raises(ValueError, match="unknown bus"):
        spec.validate()


def test_robot_spec_rejects_duplicate_send_ids_on_shared_bus() -> None:
    spec = DamiaoRobotSpec(
        name="bad_ids",
        vendor="Damiao",
        model="BadIds",
        buses={"can": DamiaoBusSpec()},
        groups={
            "left": DamiaoJointGroupSpec(
                bus_name="can",
                motors=(DamiaoMotorSpec("left_joint", "DM4310", 0x01, 0x11),),
                position_lower=(-1.0,),
                position_upper=(1.0,),
                velocity_max=(1.0,),
                kp=(1.0,),
                kd=(0.1,),
            ),
            "right": DamiaoJointGroupSpec(
                bus_name="can",
                motors=(DamiaoMotorSpec("right_joint", "DM4310", 0x01, 0x12),),
                position_lower=(-1.0,),
                position_upper=(1.0,),
                velocity_max=(1.0,),
                kp=(1.0,),
                kd=(0.1,),
            ),
        },
    )

    with pytest.raises(ValueError, match="duplicate send_id 1 on bus 'can'"):
        spec.validate()


def test_arm_adapter_reports_limits_and_modes() -> None:
    adapter = DamiaoArmAdapter.from_arm_spec(arm_spec=_arm_spec())

    assert adapter.get_dof() == 2
    assert adapter.get_limits().position_lower == [-1.0, -2.0]
    assert adapter.set_control_mode(ControlMode.TORQUE) is True
    assert adapter.set_control_mode(ControlMode.VELOCITY) is False


def test_arm_adapter_uses_fake_runtime_for_startup_hold(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter.from_arm_spec(arm_spec=_arm_spec())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is True

    assert runtime.writes[-1] == (
        "arm",
        [0.4, -0.4],
        [0.0, 0.0],
        [5.0, 6.0],
        [0.1, 0.2],
        [0.0, 0.0],
    )


def test_arm_adapter_passes_gravity_model_override_to_runtime(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter.from_arm_spec(
        arm_spec=_arm_spec(),
        gravity_model_path="override.urdf",
    )
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True

    assert runtime.loaded_gravity_models == [("arm", "override.urdf")]


def test_whole_body_requires_all_group_states(mocker) -> None:
    runtime = _FakeRuntime(fresh=False)
    adapter = DamiaoWholeBodyAdapter(robot_spec=_whole_body_spec(), group_names=("left", "right"))
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is False
    assert adapter.has_motor_states() is False
    assert adapter.write_joint_positions([0.0, 0.0]) is False
    assert runtime.writes == []


def test_whole_body_rejects_out_of_limit_frame_without_partial_send(
    mocker,
) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoWholeBodyAdapter(robot_spec=_whole_body_spec(), group_names=("left", "right"))
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    runtime.writes.clear()
    assert adapter.write_joint_positions([0.0, 3.0]) is False
    assert runtime.writes == []


def test_whole_body_position_write_splits_groups(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoWholeBodyAdapter(robot_spec=_whole_body_spec(), group_names=("left", "right"))
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    runtime.writes.clear()
    assert adapter.write_joint_positions([0.5, -0.5]) is True

    assert runtime.batched_calls == 2
    assert runtime.writes == [
        ("left", [0.5], [0.0], [5.0], [0.1], [0.0]),
        ("right", [-0.5], [0.0], [6.0], [0.2], [0.0]),
    ]


def test_whole_body_connect_sends_current_position_hold(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoWholeBodyAdapter(robot_spec=_whole_body_spec(), group_names=("left", "right"))
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True

    assert runtime.writes == [
        ("left", [0.1], [0.0], [5.0], [0.1], [0.0]),
        ("right", [-0.1], [0.0], [6.0], [0.2], [0.0]),
    ]
