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

from types import SimpleNamespace

import attrs
import pytest

from dimos.hardware.damiao.arm_adapter import DamiaoArmAdapter
from dimos.hardware.damiao.config import (
    DamiaoArmConfig,
    DamiaoMotorConfig,
    DamiaoRuntimeConfig,
)
from dimos.hardware.damiao.runtime import DamiaoArmRuntime, DamiaoGroupState
from dimos.hardware.manipulators.spec import ControlMode


class _FakeRuntime:
    def __init__(self, *, write_ok: bool = True) -> None:
        self.write_ok = write_ok
        self.connected = False
        self.enabled = False
        self.disconnect_calls = 0
        self.writes: list[
            tuple[list[float], list[float], list[float], list[float], list[float]]
        ] = []
        self.loaded_gravity_models: list[str] = []
        self.state = DamiaoGroupState(
            q=[0.4, -0.4],
            dq=[0.5, -0.5],
            tau=[0.6, -0.6],
        )

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

    def refresh_state(self, *, force: bool = False) -> DamiaoGroupState:
        del force
        return self.state

    def write_mit_commands(
        self,
        *,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> bool:
        if not self.write_ok:
            return False
        self.writes.append((list(q), list(dq), list(kp), list(kd), list(tau)))
        return True

    def load_gravity_model(self, model_path: str) -> tuple[object, object]:
        self.loaded_gravity_models.append(model_path)
        return SimpleNamespace(nq=2, nv=2, names=["universe", "j1", "j2"]), object()


def _arm_config(**changes: object) -> DamiaoArmConfig:
    config = DamiaoArmConfig(
        name="test_damiao",
        vendor="Damiao",
        model="TestArm",
        motors=(
            DamiaoMotorConfig("j1", "DM4310", 0x01, 0x11),
            DamiaoMotorConfig("j2", "DM4310", 0x02, 0x12),
        ),
        position_lower=(-1.0, -2.0),
        position_upper=(1.0, 2.0),
        velocity_max=(3.0, 4.0),
        kp=(5.0, 6.0),
        kd=(0.1, 0.2),
        gravity_torque_limits=(7.0, 8.0),
    )
    return attrs.evolve(config, **changes)


def test_arm_config_normalizes_sequences_and_is_frozen() -> None:
    config = _arm_config(position_lower=[-1, -2])

    assert config.position_lower == (-1.0, -2.0)
    assert config.joint_names == ("j1", "j2")
    with pytest.raises(attrs.exceptions.FrozenInstanceError):
        config.position_lower = (0.0, 0.0)


def test_arm_config_rejects_duplicate_motor_identity_at_construction() -> None:
    with pytest.raises(ValueError, match="duplicate send IDs"):
        _arm_config(
            motors=(
                DamiaoMotorConfig("j1", "DM4310", 0x01, 0x11),
                DamiaoMotorConfig("j2", "DM4310", 0x01, 0x12),
            )
        )


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"kp": (1.0,)}, "kp length"),
        ({"position_lower": (2.0, -2.0)}, "lower limits"),
        ({"velocity_max": (0.0, 1.0)}, "velocity limits"),
    ],
)
def test_arm_config_rejects_invalid_joint_vectors_at_construction(
    changes: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        _arm_config(**changes)


def test_runtime_config_rejects_invalid_typed_overrides() -> None:
    with pytest.raises(ValueError, match="kp_override"):
        DamiaoRuntimeConfig(kp_override=[1.0, float("nan")])


def test_runtime_builds_robot_with_binding_motor_types() -> None:
    runtime = DamiaoArmRuntime(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(use_mock_bus=True),
    )

    robot = runtime._build_robot()

    assert len(robot["arm"]) == 2


def test_arm_adapter_reports_limits_and_modes() -> None:
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )

    assert adapter.get_dof() == 2
    assert adapter.get_limits().position_lower == [-1.0, -2.0]
    assert adapter.set_control_mode(ControlMode.TORQUE) is True
    assert adapter.set_control_mode(ControlMode.VELOCITY) is False


def test_arm_adapter_resolves_runtime_gain_overrides() -> None:
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(
            gravity_comp=False,
            kp_override=[9.0, 8.0],
            kd_override=[0.9, 0.8],
        ),
    )

    assert adapter._kp == [9.0, 8.0]
    assert adapter._kd == [0.9, 0.8]


def test_arm_adapter_rejects_override_with_wrong_dof() -> None:
    with pytest.raises(ValueError, match="kp length"):
        DamiaoArmAdapter(
            arm_config=_arm_config(),
            runtime_config=DamiaoRuntimeConfig(gravity_comp=False, kp_override=[1.0]),
        )


def test_arm_adapter_uses_fake_runtime_for_startup_hold(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    assert runtime.writes[-1] == (
        [0.4, -0.4],
        [0.0, 0.0],
        [5.0, 6.0],
        [0.1, 0.2],
        [0.0, 0.0],
    )


def test_arm_adapter_passes_gravity_model_to_runtime(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(gravity_model_path="override.urdf"),
    )
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert runtime.loaded_gravity_models == ["override.urdf"]


def test_arm_adapter_rejects_nonfinite_positions_before_enable(mocker) -> None:
    runtime = _FakeRuntime()
    runtime.state = DamiaoGroupState(
        q=[float("nan"), 0.0],
        dq=[0.0, 0.0],
        tau=[0.0, 0.0],
    )
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is False
    assert runtime.enabled is False
    assert runtime.writes == []


def test_arm_adapter_gravity_compensation_rejects_missing_model_before_enable(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is False
    assert runtime.enabled is False
    assert runtime.writes == []


def test_arm_adapter_error_recovery_runs_gravity_preflight(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)
    preflight = mocker.patch.object(adapter, "_preflight_gravity")
    mocker.patch.object(adapter, "compute_gravity_torques", return_value=[0.0, 0.0])

    assert adapter.connect() is True
    assert adapter.write_clear_errors() is True
    preflight.assert_called_once_with()
    assert runtime.enabled is True


def test_arm_adapter_disables_without_zero_torque_on_gravity_state_failure(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)
    mocker.patch.object(adapter, "_load_gravity_model")
    mocker.patch.object(adapter, "_preflight_gravity")
    mocker.patch.object(adapter, "compute_gravity_torques", return_value=[0.0, 0.0])

    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    writes_before_failure = list(runtime.writes)
    mocker.patch.object(runtime, "refresh_state", side_effect=RuntimeError("state read failed"))

    assert adapter.write_joint_positions([0.2, -0.2]) is False
    assert runtime.enabled is False
    assert adapter.read_enabled() is False
    assert runtime.writes == writes_before_failure


def test_arm_adapter_rejects_incompatible_gravity_model_before_enable(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(gravity_model_path="arm.urdf"),
    )
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)
    mocker.patch.object(adapter, "compute_gravity_torques", return_value=[0.0, 0.0])

    assert adapter.connect() is True
    adapter._pin_model = SimpleNamespace(nq=2, nv=2, names=["universe", "j2", "j1"])
    adapter._pin_data = object()
    assert adapter.write_enable(True) is False
    assert runtime.enabled is False
    assert runtime.writes == []


def test_arm_adapter_rolls_back_when_hold_command_fails(mocker) -> None:
    runtime = _FakeRuntime(write_ok=False)
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is False
    assert runtime.enabled is False
    assert adapter.read_enabled() is False


def test_arm_adapter_preserves_enabled_state_when_rollback_disable_fails(mocker) -> None:
    runtime = _FakeRuntime(write_ok=False)
    mocker.patch.object(runtime, "disable", return_value=False)
    adapter = DamiaoArmAdapter(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)

    assert adapter.connect() is True
    assert adapter.write_enable(True) is False
    assert adapter.read_enabled() is True


def test_arm_adapter_preserves_enabled_state_when_safety_disable_fails(mocker) -> None:
    runtime = _FakeRuntime()
    adapter = DamiaoArmAdapter(arm_config=_arm_config())
    mocker.patch.object(adapter, "_create_runtime", return_value=runtime)
    mocker.patch.object(adapter, "_preflight_gravity")
    mocker.patch.object(adapter, "compute_gravity_torques", return_value=[0.0, 0.0])

    assert adapter.connect() is True
    assert adapter.write_enable(True) is True
    mocker.patch.object(runtime, "disable", return_value=False)
    mocker.patch.object(runtime, "refresh_state", side_effect=RuntimeError("state read failed"))

    assert adapter.write_joint_positions([0.2, -0.2]) is False
    assert adapter.read_enabled() is True


def test_runtime_retries_mit_command_when_can_queue_is_temporarily_full(mocker) -> None:
    runtime = DamiaoArmRuntime(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(),
    )
    robot = mocker.Mock()
    arm = mocker.Mock()
    arm.mit_control.side_effect = [
        RuntimeError("transport IO error: No buffer space available (os error 105)"),
        None,
    ]
    runtime._robot = robot
    runtime._arm = arm
    runtime._enabled = True
    sleep = mocker.patch("dimos.hardware.damiao.runtime.time.sleep")

    assert (
        runtime.write_mit_commands(
            q=[0.0] * 2, dq=[0.0] * 2, kp=[0.0] * 2, kd=[0.0] * 2, tau=[0.0] * 2
        )
        is True
    )
    assert arm.mit_control.call_count == 2
    robot.tick.assert_called_once_with(1_000)
    sleep.assert_called_once_with(0.001)


def test_runtime_selects_mit_mode_before_enable(mocker) -> None:
    runtime = DamiaoArmRuntime(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(),
    )
    robot = mocker.Mock()
    runtime._robot = robot

    assert runtime.enable() is True
    assert robot.method_calls[:4] == [
        mocker.call.set_mode("mit"),
        mocker.call.tick(1_000),
        mocker.call.enable(),
        mocker.call.tick(1_000),
    ]


def test_runtime_preserves_enabled_state_when_partial_enable_rollback_fails() -> None:
    class _FailingRobot:
        def set_mode(self, mode: str) -> None:
            assert mode == "mit"

        def tick(self, deadline_us: int) -> None:
            assert deadline_us == 1_000

        def enable(self) -> None:
            raise RuntimeError("partial enable")

        def disable(self) -> bool:
            return False

    runtime = DamiaoArmRuntime(
        arm_config=_arm_config(),
        runtime_config=DamiaoRuntimeConfig(),
    )
    runtime._robot = _FailingRobot()

    assert runtime.enable() is False
    assert runtime.is_enabled() is True
