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

"""Run a shipped Damiao blueprint in real workers against a fake native arm.

This is the worker-side half of
``test_damiao_blueprints.py::test_shipped_metal_blueprint_runs_the_direct_cycle_in_real_workers``.
It must be started in a fresh process with
``dimos/robot/manipulators/damiao/testing`` first on ``PYTHONPATH``, so
``sitecustomize`` installs the fake CAN boundary in this process and, through
the forkserver, in every worker before any robot code runs. It prints one
``EVIDENCE:{...}`` JSON line; any failed assertion exits non-zero.

It is a driver, not a test module: nothing here is collected by pytest.
"""

from __future__ import annotations

import json
import os
import sys
import time
from typing import Any

import fake_native
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.makermods_metal.blueprints.basic import metal_control
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.utils.testing.waiting import wait_until

EVIDENCE_PREFIX = "EVIDENCE:"
TARGET_STEP = 0.02


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _read_log(path: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _pairing(
    events: list[dict[str, Any]], start: float, end: float
) -> tuple[bool, list[dict[str, Any]], list[tuple[int, int, str]]]:
    """Whether every write in the window follows a read of its own cycle/thread."""
    window = [event for event in events if start <= event["time"] <= end and event["event"] in ("read", "write")]
    pairs: list[dict[str, Any]] = []
    writers: set[tuple[int, int, str]] = set()
    ok = True
    for index, event in enumerate(window):
        if event["event"] != "write":
            continue
        writers.add((event["pid"], event["tid"], event["thread"]))
        previous = window[index - 1] if index else None
        if previous is None:
            # The window may start in the middle of a tick; that write's read
            # is outside it, so there is nothing to pair it with.
            continue
        same = (
            previous["event"] == "read"
            and previous["pid"] == event["pid"]
            and previous["tid"] == event["tid"]
            and previous["epoch"] == event["epoch"]
        )
        ok = ok and same
        pairs.append(
            {
                "same_cycle_and_thread": same,
                "pid": event["pid"],
                "tid": event["tid"],
                "thread": event["thread"],
                "epoch": event["epoch"],
            }
        )
    return ok, pairs, sorted(writers)


def main() -> int:
    log_path = os.environ.get(fake_native.LOG_ENV)
    assert log_path, f"{fake_native.LOG_ENV} must name the fake boundary event log"
    assert fake_native.installed(), "the fake native boundary is not installed in the driver"
    assert fake_native.sealed_boundary_is_fail_closed(), (
        "a real vendor CAN transport constructor is still callable"
    )

    blueprint = metal_control()
    parsed = BlueprintConfigParser(blueprint).parse(["--can-port", "fake-test-can"], environ={})
    built = ModuleCoordinator.build(blueprint, parsed)
    manager: Any = built._managers["python"]
    pids = [worker.pid for worker in manager.workers if worker.pid is not None]
    assert pids, "the shipped blueprint must deploy into real worker processes"
    assert [worker.module_count for worker in manager.workers if worker.module_count] == [2], (
        "the shipped coordinator and its connection must share one worker"
    )

    evidence: dict[str, Any] = {
        "worker_pids": pids,
        "module_counts": [worker.module_count for worker in manager.workers if worker.module_count],
        "installed": True,
        "fail_closed": True,
    }
    try:
        coordinator = built.get_instance(ControlCoordinator)
        connection = built.get_instance(MetalConnection)
        joints = [f"arm/{name}" for name in MetalConnection.robot.arm_joint_names]
        wait_until(
            lambda: all(joint in coordinator.get_joint_positions() for joint in joints),
            timeout=15.0,
        )
        start_positions = coordinator.get_joint_positions()
        limits = {
            joint.name: (joint.lower, joint.upper)
            for joint in movable_joints(MetalConnection.robot.model)
        }
        target = []
        for joint in joints:
            lower, upper = limits[joint.removeprefix("arm/")]
            value = start_positions[joint]
            step = TARGET_STEP if upper is None or value + TARGET_STEP <= upper else -TARGET_STEP
            if lower is not None and value + step < lower:
                step = 0.005 if upper is None or value + 0.005 <= upper else -0.005
            target.append(value + step)
        trajectory = JointTrajectory(
            joint_names=joints,
            points=[
                TrajectoryPoint(
                    positions=[start_positions[joint] for joint in joints],
                    velocities=[0.0] * len(joints),
                ),
                TrajectoryPoint(
                    positions=target,
                    velocities=[0.0] * len(joints),
                    time_from_start=0.3,
                ),
            ],
        )
        started = time.monotonic()
        result = coordinator.execute_trajectory(trajectory)
        print(
            f"DEBUG result={result!r} status_type={type(result.status)} accepted={TrajectoryExecutionStatus.ACCEPTED!r}",
            file=sys.stderr,
        )
        assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

        def reached() -> bool:
            seen = coordinator.get_joint_positions()
            return all(
                joint in seen and abs(seen[joint] - goal) <= 1e-6
                for joint, goal in zip(joints, target, strict=True)
            )

        wait_until(reached, timeout=15.0)
        finished = time.monotonic()
        status = connection.status()
        assert status.connected, status
        assert status.last_command_time is not None, status
        assert status.last_error is None, status
        assert status.last_rejection is None, status
        evidence.update(
            {
                "joints": len(joints),
                "target": target,
                "start": [start_positions[joint] for joint in joints],
                "status_connected": status.connected,
                "last_command_time": status.last_command_time,
                "last_error": status.last_error,
            }
        )

        events = _read_log(log_path)
        assert {event["pid"] for event in events} <= set(pids), (
            f"fake boundary was used outside the workers: {sorted({event['pid'] for event in events})}"
        )
        ok, pairs, writers = _pairing(events, started, finished)
        assert len(pairs) >= 2, f"too few write cycles to prove pairing: {pairs}"
        assert ok, f"a write did not follow a read in the same worker, thread and cycle: {pairs}"
        assert len(writers) == 1, f"writes came from more than one thread: {writers}"
        wrote_target = any(
            event["event"] == "write"
            and len(event["q"]) == len(target)
            and all(abs(a - b) <= 1e-6 for a, b in zip(event["q"], target, strict=True))
            for event in events
        )
        assert wrote_target, "the fake arm never received the trajectory target"
        evidence.update(
            {
                "read_write_pairs": len(pairs),
                "same_cycle_and_thread": ok,
                "writer_identities": [list(identity) for identity in writers],
                "wrote_target": wrote_target,
            }
        )

        # Deadman: with the coordinator stopped, no further commands are
        # authorized, so the connection halts the arm and reports it.
        coordinator.set_estop(True)
        wait_until(lambda: connection.status().deadman_fired, timeout=15.0)
        halted = connection.status()
        assert halted.deadman_fired, halted
        coordinator.set_estop(False)
        wait_until(lambda: not connection.status().deadman_fired, timeout=15.0)
        evidence["deadman_fired_then_cleared"] = True

        # Fault: the fake boundary fails a reading; the connection latches it.
        trigger = os.environ.get(fake_native.FAIL_ENV)
        assert trigger, f"{fake_native.FAIL_ENV} must name the fault trigger file"
        with open(trigger, "w") as handle:
            handle.write("fail the next reading\n")
        wait_until(
            lambda: (connection.status().last_error or "").startswith("feedback failed:"),
            timeout=15.0,
        )
        os.unlink(trigger)
        fault = connection.status().last_error
        assert fault and fault.startswith("feedback failed:"), fault
        evidence["fault"] = fault
    finally:
        built.stop()

    wait_until(lambda: not any(_alive(pid) for pid in pids), timeout=15.0)
    assert not any(_alive(pid) for pid in pids), f"worker processes survived shutdown: {pids}"
    evidence["workers_exited"] = True
    print(EVIDENCE_PREFIX + json.dumps(evidence, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
