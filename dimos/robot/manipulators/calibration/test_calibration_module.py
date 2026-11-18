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

"""The calibration module's RPCs and streams, on a fake arm in real time."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import json
from pathlib import Path
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.memory.store.sqlite import SqliteStore
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.manipulators.calibration._fake_tuning_arm import FakeTuningArm
from dimos.robot.manipulators.calibration.module import (
    NOT_RECORDING,
    CalibrationModule,
    CalibrationModuleConfig,
)
from dimos.robot.manipulators.calibration.recording import SqliteRecording
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
from dimos.utils.testing.waiting import wait_until

OUTPUTS = (
    "gravity_events",
    "step_events",
    "float_events",
    "calibration_events",
    "calibration_status",
)
RECORDED_STEPS = ("gravity_poses", "gravity_fit", "step_sweep", "pick_gains")


def _plain(value: Any) -> Any:
    """``value`` after crossing JSON and CBOR, as an RPC client would see it."""
    assert json.loads(json.dumps(value)) == value
    return decode(encode(value))


@pytest.fixture
def make_module(
    mocker: Any, monkeypatch: pytest.MonkeyPatch, left_spec: ArmCalibrationSpec, tmp_path: Path
) -> Iterator[Callable[..., tuple[CalibrationModule, dict[str, LocalTransport]]]]:
    monkeypatch.delenv(DIMOS_RUN_ID_ENV, raising=False)
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    made: list[CalibrationModule] = []

    def make(**config: Any) -> tuple[CalibrationModule, dict[str, LocalTransport]]:
        module = CalibrationModule(robot="dual-openyam", arm="left", **config)
        outputs = {name: LocalTransport() for name in OUTPUTS}
        for name, transport in outputs.items():
            module.set_transport(name, transport)
        module.set_module_ref("_connection", FakeTuningArm(left_spec, calibration_root=tmp_path))
        made.append(module)
        module.start()
        return module, outputs

    yield make
    for module in made:
        module.stop()


def test_the_plan_says_why_recorded_steps_cannot_run_outside_a_dimos_run(make_module):
    module, _ = make_module()

    plan = _plain(module.plan())

    assert [step["id"] for step in plan][:2] == ["check", "gravity_poses"]
    unsupported = {step["id"]: step["reason"] for step in plan if not step["supported"]}
    assert unsupported == dict.fromkeys(RECORDED_STEPS, NOT_RECORDING)


def test_a_run_answers_over_rpc_and_reports_plain_json(make_module):
    module, outputs = make_module()

    assert module.confirm() is False
    started = _plain(module.run(["check"]))
    assert started["state"] == "running" and started["run_id"]
    wait_until(lambda: module.status()["state"] == "waiting_confirm", timeout=10.0)
    waiting = _plain(module.status())
    assert waiting["prompt"].startswith("Arm will move to home at 10 deg/s")
    assert module.confirm(waiting["prompt_id"]) is True
    wait_until(lambda: module.status()["state"] == "done", timeout=10.0)

    status = _plain(module.status())
    assert {step["id"]: step["status"] for step in status["steps"]}["check"] == "done"
    results = _plain(module.results())
    assert results["check"]["arm_id"] == "dual_openyam/left"
    assert results["check"]["home"] == "reached"
    published = [json.loads(message.data) for message in outputs["calibration_status"].sent]
    assert published[-1] == status
    assert module.abort() is False
    with pytest.raises(ValueError, match="unknown steps"):
        module.run(["nope"])


def test_markers_go_out_on_the_streams_the_reports_read(make_module, mocker, tmp_path):
    with SqliteStore(path=str(tmp_path / "memory.db")):
        pass
    mocker.patch.object(SqliteRecording, "of_this_run", return_value=SqliteRecording(tmp_path))
    # The fake arm never moves in real time, so the first ramp stalls it.
    module, outputs = make_module(poses=3, speed_deg_s=30.0, confirm_each=False)
    assert all(step["supported"] for step in module.plan())

    module.run(["gravity_poses"])
    wait_until(lambda: module.status()["state"] == "waiting_confirm", timeout=30.0)
    module.confirm()
    wait_until(lambda: module.status()["state"] == "failed", timeout=30.0)

    assert "more than 10 deg from its target" in module.status()["message"]
    markers = [json.loads(message.data) for message in outputs["gravity_events"].sent]
    assert [marker["label"] for marker in markers] == ["pose_01", "end"]
    assert markers[0]["data"]["arm"] == "dual_openyam/left"
    events = [json.loads(message.data) for message in outputs["calibration_events"].sent]
    assert events == [{"stream": "gravity_events", "marker": marker} for marker in markers]
    assert outputs["step_events"].sent == [] and outputs["float_events"].sent == []


@pytest.mark.parametrize(
    "config,error",
    [
        ({"robot": "nope"}, "unknown robot 'nope'"),
        (
            {"robot": "dual-openyam"},
            r"dual-openyam has several arms; set arm to one of \['left', 'right'\]",
        ),
        ({"robot": "openyam", "floor_z_m": -0.03}, "give both floor_z_m and min_x_m"),
        ({"robot": "openyam", "speed_deg_s": 45.0}, "less than or equal to 30"),
        ({"robot": "openyam", "dither_deg": 6.0}, "less than or equal to 5"),
    ],
)
def test_the_config_refuses_what_cannot_run(config, error):
    with pytest.raises(ValidationError, match=error):
        CalibrationModuleConfig(**config)


def test_a_bench_guard_from_the_config_replaces_the_implicit_one():
    spec = CalibrationModuleConfig(robot="openyam", floor_z_m=-0.03, min_x_m=0.12).arm_spec()
    assert spec.require_workspace_guard().floor_z_m == -0.03
    assert spec.workspace.min_x_m == 0.12
