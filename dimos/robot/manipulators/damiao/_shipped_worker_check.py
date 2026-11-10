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
#: Environment carrying the private multicast group this run's RPC bus joins.
#: Without it the run can share a bus with an unrelated stack on the machine
#: that also serves a ``ControlCoordinator``, and calls race between the two.
SCOUT_ENV = "DIMOS_FAKE_DAMIAO_SCOUT"


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _read_log(path: str) -> list[dict[str, Any]]:
    with open(path) as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _pairing(
    events: list[dict[str, Any]], start: float, end: float
) -> tuple[bool, int, list[tuple[int, int, str]]]:
    """Whether every write in the window follows a read of its own cycle/thread."""
    window = [
        event
        for event in events
        if start <= event["time"] <= end and event["event"] in ("read", "write")
    ]
    pairs = 0
    writers: set[tuple[int, int, str]] = set()
    ok = True
    previous: dict[str, Any] | None = None
    for event in window:
        if event["event"] != "write":
            previous = event
            continue
        writers.add((event["pid"], event["tid"], event["thread"]))
        if previous is not None:
            pairs += 1
            ok = ok and (
                previous["event"] == "read"
                and previous["pid"] == event["pid"]
                and previous["tid"] == event["tid"]
                and previous["epoch"] == event["epoch"]
            )
        previous = event
    return ok, pairs, sorted(writers)


def main() -> int:
    log_path = os.environ.get(fake_native.LOG_ENV)
    assert log_path, f"{fake_native.LOG_ENV} must name the fake boundary event log"
    assert fake_native.installed(), "the fake native boundary is not installed in the driver"
    assert fake_native.sealed_boundary_is_fail_closed(), (
        "a real vendor CAN transport constructor is still callable"
    )

    scout_addr = os.environ.get(SCOUT_ENV)
    assert scout_addr, f"{SCOUT_ENV} must name a private multicast group for this run"
    blueprint = metal_control()
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--can-port", "fake-test-can"],
        environ={},
        global_overrides={"zenoh_scout_addr": scout_addr},
    )
    built = ModuleCoordinator.build(blueprint, parsed)
    manager: Any = built._managers["python"]
    pids = [worker.pid for worker in manager.workers if worker.pid is not None]
    assert pids, "the shipped blueprint must deploy into real worker processes"
    counts = [worker.module_count for worker in manager.workers if worker.module_count]
    assert counts == [2], "the shipped coordinator and its connection must share one worker"

    evidence: dict[str, Any] = {
        "worker_pids": pids,
        "module_counts": counts,
        "installed": True,
        "fail_closed": True,
    }
    try:
        coordinator = built.get_instance(ControlCoordinator)
        connection = built.get_instance(MetalConnection)
        joints = [f"arm/{name}" for name in MetalConnection.robot.arm_joint_names]
        limits = {
            joint.name: (joint.lower, joint.upper)
            for joint in movable_joints(MetalConnection.robot.model)
        }

        # Readiness is one atomic, complete snapshot. The tick replaces the
        # whole positions dict, so a snapshot must never be assembled from
        # several reads: that would manufacture a pose out of partial samples
        # and hide a real loss of feedback.
        wait_until(
            lambda: set(joints) <= set(coordinator.get_joint_positions()),
            timeout=15.0,
        )
        start_positions = coordinator.get_joint_positions()
        target = []
        for joint in joints:
            lower, upper = limits[joint.removeprefix("arm/")]
            value = start_positions[joint]
            step = TARGET_STEP if upper is None or value + TARGET_STEP <= upper else -TARGET_STEP
            goal = value + step
            assert (lower is None or goal >= lower) and (upper is None or goal <= upper), (
                f"{joint}: start {value} and target {goal} must be inside {lower}..{upper}"
            )
            target.append(goal)
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
        assert status.last_error is None and status.last_rejection is None, status

        events = _read_log(log_path)
        used = {event["pid"] for event in events}
        assert used <= set(pids), f"fake boundary was used outside the workers: {sorted(used)}"
        ok, pairs, writers = _pairing(events, started, finished)
        assert pairs >= 2, f"too few write cycles to prove pairing: {pairs}"
        assert ok, "a write did not follow a read in the same worker, thread and cycle"
        assert len(writers) == 1, f"writes came from more than one thread: {writers}"
        wrote_target = any(
            event["event"] == "write"
            and len(event["q"]) == len(target)
            and all(abs(a - b) <= 1e-6 for a, b in zip(event["q"], target, strict=True))
            for event in events
        )
        assert wrote_target, "the fake arm never received the trajectory target"

        # Deadman: with the coordinator stopped, no further commands are
        # authorized, so the connection halts the arm and reports it.
        coordinator.set_estop(True)
        wait_until(lambda: connection.status().deadman_fired, timeout=15.0)
        coordinator.set_estop(False)
        wait_until(lambda: not connection.status().deadman_fired, timeout=15.0)

        # Fault: the fake boundary fails a reading; the connection latches it.
        trigger = os.environ.get(fake_native.FAIL_ENV)
        assert trigger, f"{fake_native.FAIL_ENV} must name the fault trigger file"
        with open(trigger, "w") as handle:
            handle.write("fail the next reading\n")
        # The connection latches the feedback failure; the hold commands that
        # follow then fail against that latch, so the last recorded error is a
        # write failure that carries the feedback failure. Assert the fault
        # itself is recorded, not the shape of whichever report came last.
        expected_fault = f"feedback failed: RuntimeError: {fake_native.FAILURE_MESSAGE}"
        wait_until(
            lambda: expected_fault in (connection.status().last_error or ""),
            timeout=15.0,
        )
        os.unlink(trigger)
        fault = connection.status().last_error
        assert fault and expected_fault in fault, fault

        evidence.update(
            {
                "joints": len(joints),
                "read_write_pairs": pairs,
                "same_cycle_and_thread": ok,
                "writer_identities": [list(identity) for identity in writers],
                "wrote_target": wrote_target,
                "status_connected": status.connected,
                "last_command_time": status.last_command_time,
                "last_error": status.last_error,
                "deadman_fired_then_cleared": True,
                "fault": fault,
            }
        )
    finally:
        built.stop()

    wait_until(lambda: not any(_alive(pid) for pid in pids), timeout=15.0)
    assert not any(_alive(pid) for pid in pids), f"worker processes survived shutdown: {pids}"
    evidence["workers_exited"] = True
    print(EVIDENCE_PREFIX + json.dumps(evidence, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
