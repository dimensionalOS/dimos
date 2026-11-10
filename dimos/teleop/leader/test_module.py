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

from collections.abc import Iterator
import math
from pathlib import Path
import threading
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.leader import module as leader_module
from dimos.teleop.leader.calibration import (
    LeaderCalibration,
    LeaderCalibrationError,
    LeaderMotorCalibration,
    save_calibration,
)
from dimos.teleop.leader.mapping import GripperMapping, JointMapping, SideMapping
from dimos.teleop.leader.module import (
    GUARDED_JOINT_FRAME_ID,
    LeaderTeleopModule,
    LeaderTeleopModuleConfig,
)

MAPPINGS = {
    "left": SideMapping(
        joints=(
            JointMapping(motor="joint_1", joint="arm/left_joint1", lower=-3.0, upper=3.0),
            JointMapping(motor="joint_2", joint="arm/left_joint2", lower=-3.0, upper=3.0),
        ),
        gripper=GripperMapping(motor="joint_7", travel_radians=1.0),
    ),
    "right": SideMapping(
        joints=(
            JointMapping(motor="joint_1", joint="arm/right_joint1", lower=-3.0, upper=3.0),
            JointMapping(motor="joint_2", joint="arm/right_joint2", lower=-3.0, upper=0.5),
        )
    ),
}
MOCK_POSITIONS = {"joint_1": 0.1, "joint_2": 0.9, "joint_7": 0.5}


def _config(**kwargs: Any) -> LeaderTeleopModuleConfig:
    return LeaderTeleopModuleConfig(
        mappings=MAPPINGS, reader="mock", mock_positions=MOCK_POSITIONS, **kwargs
    )


@pytest.fixture
def module_factory() -> Iterator[Any]:
    modules: list[LeaderTeleopModule] = []

    def make(config: LeaderTeleopModuleConfig, *, connect: bool = True) -> LeaderTeleopModule:
        module = LeaderTeleopModule(**config.model_dump())
        modules.append(module)
        if connect:
            module.connect_teleop()
        return module

    yield make
    for module in modules:
        module.stop()


def _command(module: LeaderTeleopModule) -> JointState:
    command = module.get_current_command()
    assert command is not None
    return command


def test_mock_reader_drives_every_mapped_side_in_order(module_factory: Any) -> None:
    command = _command(module_factory(_config()))
    assert command.name == [
        "arm/left_joint1",
        "arm/left_joint2",
        "arm/right_joint1",
        "arm/right_joint2",
    ]
    assert command.position == pytest.approx([0.1, 0.9, 0.1, 0.5])
    assert command.frame_id == GUARDED_JOINT_FRAME_ID


def test_unguarded_commands_carry_no_frame(module_factory: Any) -> None:
    assert _command(module_factory(_config(guarded=False))).frame_id == ""


def test_enabled_sides_limit_the_follower_joints(module_factory: Any) -> None:
    command = _command(module_factory(_config(enabled_sides=("right",))))
    assert command.name == ["arm/right_joint1", "arm/right_joint2"]


def test_authority_off_yields_no_command(module_factory: Any) -> None:
    assert module_factory(_config(authority_active=False)).get_current_command() is None


def test_jump_guard_drops_the_command_and_logs_once(module_factory: Any, mocker: Any) -> None:
    module = module_factory(_config(max_joint_jump_radians=0.2))
    _command(module)
    mocker.patch.object(
        module._readers["left"],
        "read_positions",
        return_value={"joint_1": 1.0, "joint_2": 0.9, "joint_7": 0.5},
    )
    warning = mocker.patch.object(leader_module.logger, "warning")
    assert module.get_current_command() is None
    assert module.get_current_command() is None
    warning.assert_called_once()
    assert "arm/left_joint1 jump 0.900 rad exceeds 0.200" in warning.call_args.args[1]


def test_filter_moves_targets_a_fraction_toward_each_reading(
    module_factory: Any, mocker: Any
) -> None:
    module = module_factory(_config(command_filter_cutoff_hz=10.0, tick_period_s=0.02))
    first = _command(module)
    mocker.patch.object(
        module._readers["left"],
        "read_positions",
        return_value={"joint_1": 0.6, "joint_2": 0.9, "joint_7": 0.5},
    )
    second = _command(module)
    alpha = 0.02 / (1.0 / (2.0 * math.pi * 10.0) + 0.02)
    assert first.position[0] == pytest.approx(0.1)
    assert second.position[0] == pytest.approx(0.1 + alpha * 0.5)


def test_tick_publishes_joint_targets_and_only_mapped_grippers(
    module_factory: Any, mocker: Any
) -> None:
    module = module_factory(_config())
    joints = mocker.patch.object(module.leader_joint_command, "publish")
    left = mocker.patch.object(module.left_gripper_command, "publish")
    right = mocker.patch.object(module.right_gripper_command, "publish")

    module.tick()

    assert joints.call_args.args[0].name[0] == "arm/left_joint1"
    assert left.call_args.args[0].data == pytest.approx(0.5)
    right.assert_not_called()


def test_tick_suppresses_a_failed_read_and_recovers(module_factory: Any, mocker: Any) -> None:
    module = module_factory(_config(enabled_sides=("left",)))
    publish = mocker.patch.object(module.leader_joint_command, "publish")
    mocker.patch.object(
        module._readers["left"],
        "read_positions",
        side_effect=[RuntimeError("read"), MOCK_POSITIONS],
    )

    module.tick()
    module.tick()

    publish.assert_called_once()


def test_feetech_reader_is_built_from_the_calibration_the_mapping_names(
    tmp_path: Path, mocker: Any, module_factory: Any
) -> None:
    for side in ("left", "right"):
        save_calibration(
            tmp_path / side,
            LeaderCalibration(
                side=side,
                motors={
                    motor: LeaderMotorCalibration(id=index + 1, homing_offset=0)
                    for index, motor in enumerate(MAPPINGS[side].motor_names())
                },
            ),
        )
    created: list[tuple[str, str, int, tuple[str, ...]]] = []

    class _Reader:
        def __init__(
            self, side: str, port: str, calibration: LeaderCalibration, baudrate: int, motors: Any
        ) -> None:
            created.append((side, port, baudrate, tuple(motors)))

        def connect(self) -> None:
            pass

        def disconnect(self) -> None:
            pass

        def read_positions(self) -> dict[str, float]:
            return MOCK_POSITIONS

    mocker.patch.object(leader_module, "CalibratedLeaderReader", _Reader)
    module = module_factory(
        LeaderTeleopModuleConfig(
            mappings=MAPPINGS,
            port_left="/dev/left",
            port_right="/dev/right",
            calibration_root=tmp_path,
            baudrate=123,
        )
    )
    assert created == [
        ("left", "/dev/left", 123, ("joint_1", "joint_2", "joint_7")),
        ("right", "/dev/right", 123, ("joint_1", "joint_2")),
    ]
    assert _command(module).name[0] == "arm/left_joint1"


def test_a_missing_calibration_fails_connect_and_leaves_nothing_open(
    tmp_path: Path, module_factory: Any
) -> None:
    module = module_factory(
        LeaderTeleopModuleConfig(
            mappings=MAPPINGS, port_left="/dev/left", calibration_root=tmp_path
        ),
        connect=False,
    )
    with pytest.raises(LeaderCalibrationError, match="Missing left leader calibration"):
        module.connect_teleop()
    assert module._readers == {}


def test_config_validation() -> None:
    with pytest.raises(ValueError, match="duplicate sides"):
        _config(enabled_sides=("left", "left"))
    with pytest.raises(ValueError, match="no mapping configured for the right leader"):
        LeaderTeleopModuleConfig(mappings={"left": MAPPINGS["left"]}, port_right="/dev/r")
    with pytest.raises(ValueError, match="port_left and/or port_right"):
        LeaderTeleopModuleConfig(mappings=MAPPINGS).sides()
    with pytest.raises(ValueError, match="command_filter_cutoff_hz must be positive"):
        _config(command_filter_cutoff_hz=0.0)
    config = LeaderTeleopModuleConfig(mappings=MAPPINGS, port_right="/dev/r")
    assert config.sides() == ("right",)
    assert config.calibration_path("left") == leader_module.LEADER_STATE_DIR / "left"
    assert _config().sides() == ("left", "right")


def test_start_runs_a_worker_and_stop_disconnects(
    module_factory: Any, mocker: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(leader_module, "DEFAULT_THREAD_JOIN_TIMEOUT", 1.0)
    module = module_factory(_config(tick_period_s=0.005), connect=False)
    ticked = threading.Event()
    mocker.patch.object(module, "tick", side_effect=ticked.set)

    module.start()
    assert ticked.wait(2.0)
    worker = module._thread
    assert worker is not None and worker.is_alive()
    module.start()
    assert module._thread is worker

    module.stop()
    assert module._thread is None
    assert module._readers == {}
