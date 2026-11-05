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

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
import json
from pathlib import Path
from typing import Any

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.teleop.openarm_mini.calibration import OpenArmMiniCalibration
from dimos.teleop.yam_leader import teleop_module
from dimos.teleop.yam_leader.mapping import low_pass_alpha, yam_follower_joint_names
from dimos.teleop.yam_leader.teleop_module import (
    YAM_LEADER_STATE_DIR,
    YamLeaderTeleopModule,
    YamLeaderTeleopModuleConfig,
    default_calibration_path,
)

_LEFT = list(yam_follower_joint_names("left"))
_RIGHT = list(yam_follower_joint_names("right"))


class _FakeBus:
    def __init__(self, readings: dict[str, float]) -> None:
        self.readings = readings
        self.connected = False
        self.disconnected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def read_positions(self) -> dict[str, float]:
        return self.readings


class _FailingBus(_FakeBus):
    def read_positions(self) -> dict[str, float]:
        raise RuntimeError("bus read failed")


class _Recorder:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class _FakeControl:
    """Records streamed trajectories and answers with the queued statuses."""

    def __init__(self, *statuses: TrajectoryExecutionStatus) -> None:
        self.statuses = list(statuses)
        self.trajectories: list[Any] = []

    def execute_trajectory(self, trajectory: Any) -> TrajectoryExecutionResult:
        self.trajectories.append(trajectory)
        status = self.statuses.pop(0) if self.statuses else TrajectoryExecutionStatus.ACCEPTED
        return TrajectoryExecutionResult(status, status.name)


def _wire_outputs(module: YamLeaderTeleopModule) -> tuple[_Recorder, _Recorder, _Recorder]:
    joints, left, right = _Recorder(), _Recorder(), _Recorder()
    module.joint_command = joints  # type: ignore[assignment]
    module.left_gripper_command = left  # type: ignore[assignment]
    module.right_gripper_command = right  # type: ignore[assignment]
    return joints, left, right


def _write_calibration(path: Path, side: str) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    motors = {f"joint_{i}": {"id": i, "homing_offset": 2048, "flip": False} for i in range(1, 8)}
    (path / "calibration.json").write_text(
        json.dumps({"side": side, "motors": motors, "schema_version": 1})
    )
    return path


def _readings(**overrides: float) -> dict[str, float]:
    readings = {f"joint_{i}": 0.1 * i for i in range(1, 7)}
    readings["joint_7"] = 0.0
    readings.update(overrides)
    return readings


def _patch_buses(
    monkeypatch: pytest.MonkeyPatch, buses: Mapping[str, _FakeBus]
) -> list[tuple[str, str, str, int]]:
    created: list[tuple[str, str, str, int]] = []

    def factory(
        side: str, port: str, calibration: OpenArmMiniCalibration, baudrate: int
    ) -> _FakeBus:
        created.append((side, port, calibration.side, baudrate))
        return buses[side]

    monkeypatch.setattr(teleop_module, "YamLeaderReader", factory)
    return created


def _config(tmp_path: Path, **kwargs: Any) -> YamLeaderTeleopModuleConfig:
    return YamLeaderTeleopModuleConfig(
        port_left="left-port",
        port_right="right-port",
        left_calibration_path=_write_calibration(tmp_path / "left", "left"),
        right_calibration_path=_write_calibration(tmp_path / "right", "right"),
        baudrate=123,
        **kwargs,
    )


@contextmanager
def _connected(config: YamLeaderTeleopModuleConfig) -> Iterator[YamLeaderTeleopModule]:
    module = YamLeaderTeleopModule(**config.model_dump())
    try:
        module.connect_teleop()
        yield module
    finally:
        module.stop()


def test_ports_select_sides_and_default_paths_live_under_the_yam_state_dir() -> None:
    assert YamLeaderTeleopModuleConfig(port_left="/dev/ttyACM0").sides() == ("left",)
    assert YamLeaderTeleopModuleConfig(port_right="/dev/ttyACM1").sides() == ("right",)
    both = YamLeaderTeleopModuleConfig(port_left="/dev/ttyACM0", port_right="/dev/ttyACM1")
    assert both.sides() == ("left", "right")
    assert both.calibration_path("right") == default_calibration_path("right")
    assert default_calibration_path("left") == YAM_LEADER_STATE_DIR / "left"
    with pytest.raises(ValueError):
        YamLeaderTeleopModuleConfig().sides()
    with pytest.raises(ValueError):
        YamLeaderTeleopModuleConfig(port_left="p", enabled_sides=("left", "left"))


def test_module_connects_each_side_and_emits_both_arms_and_grippers(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    buses = {"left": _FakeBus(_readings()), "right": _FakeBus(_readings(joint_7=1.0))}
    created = _patch_buses(monkeypatch, buses)
    with _connected(_config(tmp_path)) as module:
        commands = module.get_current_commands()
        joint_state = module.get_current_command()

    assert created == [("left", "left-port", "left", 123), ("right", "right-port", "right", 123)]
    assert commands is not None and [c.side for c in commands] == ["left", "right"]
    assert commands[0].gripper_opening == 1.0
    assert commands[1].gripper_opening == 0.0
    assert joint_state is not None
    assert list(joint_state.name) == [*_LEFT, *_RIGHT]
    assert all(bus.connected and bus.disconnected for bus in buses.values())


def test_single_side_reads_only_its_bus(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    created = _patch_buses(monkeypatch, {"right": _FakeBus(_readings())})
    config = YamLeaderTeleopModuleConfig(
        port_right="right-port",
        right_calibration_path=_write_calibration(tmp_path / "right", "right"),
    )
    with _connected(config) as module:
        joint_state = module.get_current_command()

    assert created == [("right", "right-port", "right", module.config.baudrate)]
    assert joint_state is not None and list(joint_state.name) == _RIGHT


def test_failed_read_drops_the_command_and_a_jump_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_buses(monkeypatch, {"left": _FailingBus(_readings()), "right": _FakeBus(_readings())})
    with _connected(_config(tmp_path)) as module:
        assert module.get_current_command() is None

    jumping = _FakeBus(_readings())
    _patch_buses(monkeypatch, {"left": jumping, "right": _FakeBus(_readings())})
    with _connected(_config(tmp_path)) as module:
        assert module.get_current_command() is not None
        jumping.readings = _readings(joint_2=3.0)
        assert module.get_current_command() is None


def test_authority_off_publishes_nothing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())})
    with _connected(_config(tmp_path, authority_active=False)) as module:
        assert module.get_current_command() is None


def test_missing_calibration_fails_connect_and_closes_opened_buses(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    left = _FakeBus(_readings())
    _patch_buses(monkeypatch, {"left": left, "right": _FakeBus(_readings())})
    config = YamLeaderTeleopModuleConfig(
        port_left="left-port",
        port_right="right-port",
        left_calibration_path=_write_calibration(tmp_path / "left", "left"),
        right_calibration_path=tmp_path / "missing",
    )
    module = YamLeaderTeleopModule(**config.model_dump())
    try:
        with pytest.raises(Exception, match="calibration"):
            module.connect_teleop()
    finally:
        module.stop()
    assert left.disconnected


def test_trajectory_mode_anchors_each_target_to_the_last_one(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    right = _FakeBus(_readings())
    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": right})
    control = _FakeControl()
    with _connected(_config(tmp_path, command_filter_cutoff_hz=None)) as module:
        module._control = control
        joints, left, right_gripper = _wire_outputs(module)
        module.tick()
        right.readings = _readings(joint_1=0.3)
        module.tick()

    first, second = control.trajectories
    assert len(first.points) == 1 and first.points[0].time_from_start == 0.0
    assert [point.time_from_start for point in second.points] == [0.0, 0.04]
    assert second.points[0].positions == first.points[0].positions
    assert second.joint_names == [*_LEFT, *_RIGHT]
    right_index = second.joint_names.index(_RIGHT[0])
    assert second.points[1].positions[right_index] == pytest.approx(0.3)
    assert joints.messages == []
    assert len(left.messages) == 2 and len(right_gripper.messages) == 2


def test_rejected_anchor_falls_back_to_one_point_then_re_anchors(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())})
    control = _FakeControl(
        TrajectoryExecutionStatus.ACCEPTED,
        TrajectoryExecutionStatus.START_STATE_MISMATCH,
        TrajectoryExecutionStatus.ACCEPTED,
        TrajectoryExecutionStatus.ACCEPTED,
    )
    with _connected(_config(tmp_path, command_filter_cutoff_hz=None)) as module:
        module._control = control
        joints, _, _ = _wire_outputs(module)
        module.tick()
        module.tick()
        module.tick()

    assert [len(t.points) for t in control.trajectories] == [1, 2, 1, 2]
    assert joints.messages == []


def test_unserved_trajectory_rpc_falls_back_to_the_joint_command_stream(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())})
    control = _FakeControl(
        TrajectoryExecutionStatus.NO_TRAJECTORY_TASK,
        TrajectoryExecutionStatus.NO_TRAJECTORY_TASK,
    )
    with _connected(_config(tmp_path)) as module:
        module._control = control
        joints, _, _ = _wire_outputs(module)
        module.tick()
        module.tick()

    assert [len(t.points) for t in control.trajectories] == [1, 1]
    assert len(joints.messages) == 2
    assert module._last_sent_by_joint == {}


def test_joint_command_stream_without_a_control_link_or_horizon(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())})
    with _connected(_config(tmp_path)) as module:
        joints, _, _ = _wire_outputs(module)
        module.tick()
    assert len(joints.messages) == 1

    _patch_buses(monkeypatch, {"left": _FakeBus(_readings()), "right": _FakeBus(_readings())})
    control = _FakeControl()
    with _connected(_config(tmp_path, trajectory_horizon_s=None)) as module:
        module._control = control
        joints, _, _ = _wire_outputs(module)
        module.tick()
    assert len(joints.messages) == 1 and control.trajectories == []


def test_command_filter_blends_toward_new_readings(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    left = _FakeBus(_readings())
    _patch_buses(monkeypatch, {"left": left, "right": _FakeBus(_readings())})
    config = _config(tmp_path, command_filter_cutoff_hz=6.0, tick_period_s=0.02)
    alpha = low_pass_alpha(6.0, 0.02)
    with _connected(config) as module:
        first = module.get_current_command()
        left.readings = _readings(joint_1=0.6)
        second = module.get_current_command()
        third = module.get_current_command()

    assert first is not None and second is not None and third is not None
    index = list(first.name).index(_LEFT[0])
    assert first.position[index] == pytest.approx(0.1)
    assert second.position[index] == pytest.approx(0.1 + alpha * 0.5)
    assert third.position[index] == pytest.approx(
        second.position[index] + alpha * (0.6 - second.position[index])
    )
    with pytest.raises(ValueError):
        _config(tmp_path, command_filter_cutoff_hz=0.0)
    with pytest.raises(ValueError):
        _config(tmp_path, trajectory_horizon_s=-1.0)
