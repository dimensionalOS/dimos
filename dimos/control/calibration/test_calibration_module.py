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
import threading
from types import SimpleNamespace
from typing import Any

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.control.calibration._fake_tuning_arm import FakeTuningArm
from dimos.control.calibration.gravity_poses import ArmModel, generate_poses
from dimos.control.calibration.module import (
    CALL_TIMEOUTS_S,
    CONFIG_CALL_TIMEOUT_S,
    HOLD_CALL_TIMEOUT_S,
    NOT_RECORDING,
    PERSIST_CALL_TIMEOUT_S,
    CalibrationModule,
    CalibrationModuleConfig,
)
from dimos.control.calibration.recording import SqliteRecording
from dimos.control.calibration.spec import ArmCalibrationSpec
from dimos.control.connection.arm_config import ConfigPatch
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.memory.store.sqlite import SqliteStore
from dimos.protocol.encode.cbor import decode, encode
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
BENCH = {"floor_z_m": -0.03, "min_x_m": 0.15}


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

    def make(
        connection: Any = None, **config: Any
    ) -> tuple[CalibrationModule, dict[str, LocalTransport]]:
        module = CalibrationModule(robot="dual-openyam", arm="left", **{**BENCH, **config})
        outputs = {name: LocalTransport() for name in OUTPUTS}
        for name, transport in outputs.items():
            module.set_transport(name, transport)
        if connection is None:
            connection = FakeTuningArm(left_spec, calibration_root=tmp_path, realtime=True)
        # The arm's readings reach the module on its joint_state input, as a
        # connection's joint_state output does in a blueprint; its targets
        # reach the fake, which stands in for the coordinator's pass-through.
        readings = LocalTransport()
        module.set_transport("joint_state", readings)
        connection.joint_state = readings
        targets = LocalTransport()
        targets.subscribe(connection.send)
        module.set_transport("calibration_joint_command", targets)
        outputs["calibration_joint_command"] = targets
        module.set_module_ref("_connection", connection)
        module.set_module_ref("_coordinator", connection)
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

    assert module.confirm(0) is False
    with pytest.raises(TypeError, match="needs the prompt_id"):
        module.confirm(None)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="needs the prompt_id"):
        module.skip(None)  # type: ignore[arg-type]
    started = _plain(module.run(["check"]))
    assert started["state"] == "running" and started["run_id"]
    wait_until(lambda: module.status()["state"] == "waiting_confirm", timeout=10.0)
    waiting = _plain(module.status())
    assert waiting["prompt"].startswith("Arm will move to home at 10 deg/s")
    # The prompt says when it gives up: ten minutes unanswered aborts and parks.
    assert waiting["prompt_deadline"] == pytest.approx(waiting["updated"] + 600.0, abs=1.0)
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
    # The targets went out as the coordinator names the arm's joints, about 50 a second,
    # and the arm was held before the session ended.
    sent = outputs["calibration_joint_command"].sent
    assert {tuple(msg.name) for msg in sent} == {
        tuple(f"arm/{joint}" for joint in module.config.arm_spec().joint_names)
    }
    gaps = np.diff([msg.ts for msg in sent])
    assert len(sent) >= 5 and float(np.median(gaps)) == pytest.approx(0.02, abs=0.005)
    fake = module._connection
    assert fake.call_log[-2:] == ["hold", "config_reset"]


def test_go_home_is_an_rpc_that_confirms_first_and_float_pose_reaches_the_settings(make_module):
    module, _ = make_module(float_pose_deg=[0.0, 10.0, 20.0, 0.0, 0.0, 0.0])

    assert module.config.settings().float_pose_deg == (0.0, 10.0, 20.0, 0.0, 0.0, 0.0)
    started = _plain(module.go_home())
    assert started["state"] == "running" and started["step"] == "home"
    wait_until(lambda: module.status()["state"] == "waiting_confirm", timeout=10.0)
    waiting = _plain(module.status())
    assert waiting["prompt"].startswith("The arm will ramp to home at 10 deg/s")
    assert module.skip(waiting["prompt_id"]) is True
    wait_until(lambda: module.status()["state"] == "done", timeout=10.0)
    assert "not moved home" in module.status()["message"]


def test_the_module_names_the_dimos_run_it_runs_in(make_module, monkeypatch):
    module, _ = make_module()
    assert module.dimos_run_id() is None

    monkeypatch.setenv(DIMOS_RUN_ID_ENV, "20261118-101010-abcd-dual-openyam-calibrate")

    assert module.dimos_run_id() == "20261118-101010-abcd-dual-openyam-calibrate"
    assert "dimos_run_id" in CalibrationModule.rpcs


def test_markers_go_out_on_the_streams_the_reports_read(make_module, mocker, tmp_path):
    with SqliteStore(path=str(tmp_path / "memory.db")):
        pass
    mocker.patch.object(SqliteRecording, "of_this_run", return_value=SqliteRecording(tmp_path))
    module, outputs = make_module(poses=3, speed_deg_s=30.0, confirm_each=False)
    # A blocked elbow stalls the first ramp, which stops the sweep at once.
    module._connection.blocked = {"left_joint3"}
    assert all(step["supported"] for step in module.plan())

    module.run(["gravity_poses"])
    wait_until(lambda: module.status()["state"] == "waiting_confirm", timeout=30.0)
    module.confirm(module.status()["prompt_id"])
    wait_until(lambda: module.status()["state"] == "failed", timeout=30.0)

    assert "from its target (left_joint3" in module.status()["message"]
    markers = [json.loads(message.data) for message in outputs["gravity_events"].sent]
    assert [marker["label"] for marker in markers] == ["pose_01", "end"]
    assert markers[0]["data"]["arm"] == "dual_openyam/left"
    events = [json.loads(message.data) for message in outputs["calibration_events"].sent]
    assert events == [{"stream": "gravity_events", "marker": marker} for marker in markers]
    assert outputs["step_events"].sent == [] and outputs["float_events"].sent == []


def test_the_config_calls_time_out_in_seconds_and_a_save_in_ten(make_module, left_spec, tmp_path):
    fake = FakeTuningArm(left_spec, calibration_root=tmp_path, realtime=True)
    # The per-method timeouts of an RPC client, as a deployed module reference has.
    fake.rpc = SimpleNamespace(rpc_timeouts={"start": 1200.0})
    seen: list[tuple[str, float]] = []
    config_set = fake.config_set

    def timed(patch, scope="session", **kwargs):
        seen.append((scope, fake.rpc.rpc_timeouts["config_set"]))
        return config_set(patch, scope, **kwargs)

    fake.config_set = timed  # type: ignore[method-assign]

    module, _ = make_module(connection=fake)

    timeouts = fake.rpc.rpc_timeouts
    assert {name: timeouts[name] for name in CALL_TIMEOUTS_S} == {
        "config_set": 2.0,
        "config_get": 2.0,
        "config_reset": 2.0,
        "status": 2.0,
    }
    # The coordinator reference here is the same fake: its hold waits half a second.
    assert timeouts["task_invoke"] == HOLD_CALL_TIMEOUT_S == 0.5
    assert timeouts["start"] == 1200.0
    # A save waits longer, then the session's calls are short again.
    procedure = module._require()
    procedure.persist(ConfigPatch(arm_id=left_spec.arm_id, notes="x"))
    with procedure.open_session():
        pass
    assert seen == [("persist", PERSIST_CALL_TIMEOUT_S), ("session", CONFIG_CALL_TIMEOUT_S)]
    assert PERSIST_CALL_TIMEOUT_S == 10.0 and timeouts["config_set"] == 2.0


def test_stopping_parks_the_arm_before_the_connection_lets_it_go(make_module, left_spec, tmp_path):
    # The arm holds at a gravity pose after a failed run; then the run stops,
    # the module and the connection in parallel, as dimos stop does.
    arm = ArmModel(left_spec)
    nearest = min(
        generate_poses(arm, count=8).identification,
        key=lambda p: float(np.abs(np.asarray(p.joints_rad) - arm.home).max()),
    )
    # On the straight way there, 30 deg from home: a park of under 3 s.
    pose = arm.home + 0.4 * (np.asarray(nearest.joints_rad) - arm.home)
    fake = FakeTuningArm(left_spec, arm=arm, calibration_root=tmp_path, start=pose, realtime=True)
    module, _ = make_module(connection=fake)
    assert np.degrees(np.abs(fake.q - arm.home)).max() > 10.0

    connection = threading.Thread(target=fake.stop)
    connection.start()
    module.stop()
    connection.join(timeout=15.0)

    # The connection waited for the park's session, and let go only at home.
    assert fake.stop_log == ["stop", "waiting", "disabled"]
    assert fake.ended == ["config_reset"]
    assert fake.disabled_at is not None
    assert np.degrees(np.abs(fake.disabled_at - arm.home)).max() < 3.0


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
