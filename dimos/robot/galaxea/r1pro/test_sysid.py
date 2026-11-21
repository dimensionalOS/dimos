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

"""CPU-only SysID plan, guard, and real blueprint parsing tests."""

from __future__ import annotations

from itertools import pairwise
import json
import time
from types import SimpleNamespace

import pytest

from dimos.control.contract.limits import Limits
from dimos.control.coordinator import ControlCoordinatorConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import MetaBlueprint
from dimos.core.coordination.module_coordinator import _materialize_transports, stream_name_types
from dimos.core.global_config import global_config
from dimos.experimental.memory.rust_cli_recorder import make_plan
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_sysid import (
    RECORD_STREAMS,
    R1ProSysIdCoordinator,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.robot.galaxea.r1pro.sysid import BASE, CHASSIS, SEGMENTS, TRIALS, UPPER, R1ProSysIdTask
from dimos.robot.get_all_blueprints import load_blueprint


def test_waveforms_and_order() -> None:
    assert TRIALS["vx:-1.2"].velocity(1.49) == (-1.2, 0.0, 0.0)
    assert TRIALS["vy:0.8"].velocity(1.5) == (0.0, 0.0, 0.0)
    assert TRIALS["wz:-1.5"].velocity(0) == (0.0, 0.0, -1.5)
    assert TRIALS["deadband"].velocity(0)[0] == 0.01
    assert TRIALS["deadband"].velocity(1.98)[0] == pytest.approx(0.0991)
    assert TRIALS["coupled"].velocity(0) == (0.5, 0.0, 0.8)
    q = (0.0,) * 18
    step = TRIALS["elbow:step:-0.25"]
    assert step.position(0, q) == tuple(-0.25 if i == 7 else 0.0 for i in range(18))
    assert step.position(1.5, q) == q
    chirp = TRIALS["torso-j2:chirp"]
    samples = [chirp.position(i / 50, q) for i in range(301)]
    assert samples[0] == samples[-1] == q
    assert 0.10 < max(abs(x[1]) for x in samples) <= 0.15
    assert sum(x[1] * y[1] < 0 for x, y in pairwise(samples)) >= 15
    for case in ("zero", "arms-forward", "torso-bent"):
        trial = TRIALS[case]
        assert trial.position(0, q) == q
        assert trial.position(1.5, q) == trial.position(4.5, q) == trial.pose


@pytest.fixture
def task() -> R1ProSysIdTask:
    return R1ProSysIdTask("sysid", 10, dict.fromkeys(UPPER, Limits(-2, 2)))


def feedback(
    task: R1ProSysIdTask, now: float, *, omit: str = "", angle: float = 0.0, age: float = 0.0
) -> None:
    for topic, size in {**SEGMENTS, CHASSIS: 3}.items():
        if topic == omit:
            continue
        task.on_diagnostics(
            String(
                json.dumps(
                    {
                        "schema": 1,
                        "direction": "rx",
                        "topic": topic,
                        "monotonic_ns": round(now * 1e9),
                        "unix_ns": round(1000e9),
                        "header_ns": round((1000 - age) * 1e9),
                        "values": {
                            "position": [angle] * size,
                            "linear": [0.0] * 3,
                            "angular": [0.0] * 3,
                        },
                    }
                )
            )
        )


def tick(task: R1ProSysIdTask, now: float):
    return task.compute(CoordinatorState(JointStateSnapshot(), t_now=now))


def test_wire_gate_preview_abort_and_no_replay(task) -> None:
    wrote = []

    def publish() -> None:
        wrote.append(True)

    assert task.run_trial("vx:1.2")["chassis_within_limits"] is False
    with pytest.raises(ValueError, match="preview only"):
        task.run_trial("vx:1.2", live=True)
    task.admit_write(publish)
    assert wrote == []
    t = time.perf_counter()
    feedback(task, t)
    task.run_trial("vx:0.5", live=True)
    assert tick(task, t + 0.01).joint_names == list(UPPER)  # prime the measured hold
    motion = tick(task, t + 0.03)
    assert motion.joint_names == list(BASE)
    assert motion.velocities == [0.5, 0.0, 0.0]
    task.admit_write(publish)
    assert task.abort()
    task.admit_write(publish)
    assert wrote == [True]
    stop = tick(task, t + 0.05)
    assert stop.joint_names == list(UPPER) and stop.positions == [0.0] * 18
    task.admit_write(publish)
    assert wrote == [True, True]
    assert tick(task, t + 0.60) is None
    assert task.status()["phase"] == "aborted"
    task.set_estop(True)
    with pytest.raises(ValueError, match="E-stop"):
        task.run_trial("vx:0.5", live=True)
    task.set_estop(False)
    assert not task.is_active()


def test_estop_while_abort_hold_closes_write_gate(task) -> None:
    t = time.perf_counter()
    feedback(task, t)
    task.run_trial("elbow:step:0.1", live=True)
    assert tick(task, t + 0.01).positions[7] == 0.1
    assert task.abort()
    assert tick(task, t + 0.03).positions == [0.0] * 18
    assert not task.abort()  # duplicate abort does not extend the hold window
    task.set_estop(True)
    task.set_estop(False)
    task.admit_write(lambda: pytest.fail("E-stop clear replayed an aborted trial"))
    assert tick(task, t + 0.05) is None


@pytest.mark.parametrize(
    "invalid", ["stale", "old header", "position limit", "unsafe pose", "tick gap"]
)
def test_fails_closed(task, invalid) -> None:
    t = time.perf_counter()
    feedback(
        task,
        t,
        omit=CHASSIS if invalid == "stale" else "",
        angle=1.9 if invalid == "position limit" else 0.0,
        age=0.16 if invalid == "old header" else 0.0,
    )
    task.run_trial("arms-forward" if invalid == "unsafe pose" else "elbow:step:0.25", live=True)
    first = tick(task, t + 0.01)
    if invalid == "tick gap":
        assert first.positions[7] == 0.25
        feedback(task, t + 0.1)
        assert tick(task, t + 0.1).positions == [0.0] * 18
        assert task.status()["phase"] == "settling"
    else:
        assert first is None and task.status()["phase"] == "aborted"


def test_registered_blueprint_config_and_rust_plan(monkeypatch) -> None:
    blueprint = load_blueprint("r1pro-sysid")
    if isinstance(blueprint, MetaBlueprint):
        blueprint = blueprint.solidify()
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "record": "mcap",
            "record_engine": "rust",
            "record_topics": ",".join(RECORD_STREAMS),
            "zenoh_domain": "sysid_test",
            "zenoh_scout_addr": "224.0.0.224:19546",
        },
    )
    config = ControlCoordinatorConfig(**parsed.module_kwargs("R1ProSysIdCoordinator"))
    assert config.tick_rate == 50 and [t.type for t in config.tasks] == ["r1pro_sysid"]
    assert set(config.connections[0].joint_commands) == {*BASE, *UPPER}
    driver = next(atom for atom in blueprint.blueprints if atom.module is R1ProConnection)
    assert parsed.module_kwargs(driver.name)["diagnostic_tap"] is True
    assert parsed.module_kwargs(driver.name)["boot_vendor_stack"] is False
    assert set(RECORD_STREAMS.items()) <= stream_name_types(blueprint)
    for key, value in parsed.global_config_values().items():
        if key in {"transport", "record_topics", "zenoh_domain", "zenoh_scout_addr"}:
            monkeypatch.setattr(global_config, key, value)
    plan = make_plan(_materialize_transports(blueprint, {}))
    assert set(plan.payload_types) == set(RECORD_STREAMS)
    coordinator = R1ProSysIdCoordinator(**parsed.module_kwargs("R1ProSysIdCoordinator"))
    transports = {name: LocalTransport() for name in {**coordinator.inputs, **coordinator.outputs}}
    for name, transport in transports.items():
        coordinator.set_transport(name, transport)
    coordinator._connect_robots()
    coordinator._setup_from_config()
    # The real subscription lifecycle requires a running tick loop; keep
    # execution deterministic here and step the task explicitly.
    coordinator._tick_loop = SimpleNamespace(is_running=True, stop=lambda: None)
    try:
        coordinator._sync_stream_subscriptions()
        task = coordinator.get_task("sysid")
        assert isinstance(task, R1ProSysIdTask)
        assert len(transports["r1pro_diagnostics"].subscribers) == 1
        source = coordinator._connections["r1pro"]
        zeros = dict.fromkeys(
            [*(f"{j}/position" for j in UPPER), *(f"{j}/velocity" for j in BASE)], 0.0
        )
        coordinator._publish_command("r1pro", zeros)
        assert not transports["r1pro_velocity_command"].sent
        t = time.perf_counter()
        feedback(task, t)
        coordinator.task_invoke("sysid", "run_trial", {"case": "vx:0.2", "live": True})
        prime = tick(task, t + 0.01)
        assert prime and prime.positions
        coordinator._publish_command("r1pro", zeros)
        motion = tick(task, t + 0.035)
        assert motion and motion.velocities
        values = source.command(
            {
                joint: (value, motion.mode)
                for joint, value in zip(motion.joint_names, motion.velocities, strict=True)
            }
        )
        coordinator._publish_command("r1pro", values)
        assert list(transports["r1pro_velocity_command"].sent[-1].velocity) == [0.2, 0.0, 0.0]
        coordinator.task_invoke("sysid", "abort")
        coordinator._publish_command("r1pro", values)  # already computed before abort
        assert len(transports["r1pro_velocity_command"].sent) == 2
        hold = tick(task, t + 0.05)
        assert hold and hold.positions
        values = source.command(
            {
                joint: (value, hold.mode)
                for joint, value in zip(hold.joint_names, hold.positions, strict=True)
            }
        )
        coordinator._publish_command("r1pro", values)
        assert list(transports["r1pro_velocity_command"].sent[-1].velocity) == [0.0] * 3
        assert list(transports["r1pro_position_command"].sent[-1].position) == [0.0] * 18
    finally:
        coordinator.stop()
