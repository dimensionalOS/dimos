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

"""Fixture-specific replay of public actuator commands, not an agent policy.

Source commands.jsonl SHA256:
1d6296aed3786407c28f3d7a7389046690aa3a4b77273e3b878b23d0e16c61b4.
Nine repeated target plateaus retain the original public trajectory phases.
The successful robot log also includes two five-second post-trajectory waits;
those public settling pauses are retained in DWELLS.
No scene, seed, object truth or grader input is loaded by this client.
"""

import threading
import time

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    TrajectoryExecutionStatus,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.manipulators.mujoco_robot_io import MujocoRobotIO

DURATIONS = (1.0, 4.0, 1.5, 3.0, 1.5, 1.5, 2.0, 3.0, 1.5)
DWELLS = (3.0, 3.0, 3.0, 3.0, 8.0, 8.0, 3.0, 3.0, 3.0)
TARGETS = (
    (
        -1.4348686105145873e-08,
        -0.23993342937276743,
        -6.364965486267097e-05,
        0.8967337616605995,
        -3.8136364841287283e-05,
        1.1574310514991597,
        1.4586784930951009e-08,
        0.85,
    ),
    (
        0.009497068644077828,
        0.48954656049401674,
        0.006975302450385032,
        1.2843212733292388,
        -0.004601544487603221,
        0.7947791988164598,
        0.01887798948891132,
        0.85,
    ),
    (
        0.00927295318225222,
        0.4958888238853346,
        0.007114474222213122,
        1.3450510936431326,
        -0.004514343803396107,
        0.8491669540153666,
        0.018514688890921813,
        0.85,
    ),
    (
        0.011040182339585442,
        0.8436417625059869,
        0.004743147616008311,
        1.3315381187284043,
        -0.007567672869448998,
        0.48789951856760827,
        0.020875835062025224,
        0.85,
    ),
    (
        0.010319966493514519,
        0.8415358519721186,
        0.005067200024129388,
        1.3711261768255878,
        -0.007487354796326161,
        0.5295844336818428,
        0.020156345446378104,
        0.85,
    ),
    (
        0.010320595767707328,
        0.838936375368708,
        0.005248812778865213,
        1.3852116865151074,
        -0.00752357896816485,
        0.5462856921568104,
        0.0202551270834629,
        0.85,
    ),
    (
        0.01032059577329054,
        0.8593483721599394,
        0.005562013662012627,
        1.3736499294635478,
        -0.007558774749604401,
        0.5481245305215572,
        0.02025518807088589,
        0.0,
    ),
    (
        0.008836465213204527,
        0.5655673051249357,
        0.007567300249961218,
        1.2727432720686758,
        -0.00624976012460456,
        0.7071488095552292,
        0.019977637661823183,
        0.0,
    ),
    (
        0.008532539983320236,
        0.5692231347105625,
        0.00778640710815809,
        1.3327032156979124,
        -0.006075324311405875,
        0.7634929858086125,
        0.01948129781235213,
        0.0,
    ),
)


def run_lift(coordinator: ControlCoordinator, camera: MujocoRobotIO) -> None:
    arm = [f"arm/joint{i}" for i in range(1, 8)]
    public_image = threading.Event()
    camera.color_image.subscribe(lambda _: public_image.set())
    assert public_image.wait(5), "Public camera unavailable"
    task = coordinator.get_task("joint_trajectory")
    assert isinstance(task, JointTrajectoryTask)

    def positions() -> list[float]:
        state = coordinator.get_joint_positions()
        return [state[name] for name in arm]

    for duration, dwell, target in zip(DURATIONS, DWELLS, TARGETS, strict=True):
        opening = 1.0 if target[7] > 0 else 0.0
        assert coordinator.task_invoke("arm_gripper", "set_normalized", {"values": [opening]})
        result = coordinator.execute_trajectory(
            JointTrajectory(
                joint_names=arm,
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=positions()),
                    TrajectoryPoint(time_from_start=duration, positions=list(target[:7])),
                    TrajectoryPoint(time_from_start=duration + dwell, positions=list(target[:7])),
                ],
            )
        )
        assert result.status == TrajectoryExecutionStatus.ACCEPTED, result
        deadline = time.monotonic() + duration + 15
        while time.monotonic() < deadline:
            status = task.get_status(time.monotonic())
            if status.state == TrajectoryState.COMPLETED:
                break
            assert status.state == TrajectoryState.EXECUTING, status.state
            threading.Event().wait(0.05)
        else:
            raise RuntimeError("Public trajectory did not complete")
        # Public feedback gates phase advancement after each recorded dwell.
        # Position-servo gravity bias must remain inside this bound.
        assert (
            max(abs(q - desired) for q, desired in zip(positions(), target[:7], strict=True)) < 0.04
        )

    # Keep the normal gripper task until trusted sensor publication ends.
    # Only the host evaluates private object evidence; script completion is no score.
    deadline = time.monotonic() + 150
    while time.monotonic() < deadline:
        try:
            positions()
        except RuntimeError:
            return
        threading.Event().wait(0.1)
    raise RuntimeError("Trusted episode did not end")
