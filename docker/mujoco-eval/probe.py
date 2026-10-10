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

"""Deterministic, model-free acceptance client, including arbitrary Zenoh probes."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import socket
import threading
import time

import zenoh

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.protocol.mujoco_eval import Command, key, session_config
from dimos.robot.manipulators.mujoco_robot_io import MujocoRobotIO


def main() -> None:
    endpoint = os.environ["DIMOS_MUJOCO_EVAL_ENDPOINT"]
    run = os.environ["DIMOS_MUJOCO_EVAL_RUN"]
    episode = os.environ["DIMOS_MUJOCO_EVAL_EPISODE"]
    mode = os.environ.get("EVAL_PROBE_MODE", "normal")
    with ExitStack() as cleanup:
        joints = [f"arm/joint{i}" for i in range(1, 8)] + ["arm/gripper"]
        coordinator = ControlCoordinator(
            hardware=[
                HardwareComponent(
                    hardware_id="arm",
                    hardware_type=HardwareType.MANIPULATOR,
                    joints=joints,
                    adapter_type="mujoco_eval",
                    address=endpoint,
                    adapter_kwargs={"run": run, "episode": episode},
                )
            ],
            tasks=[joint_trajectory_task(joints)],
            publish_joint_state=False,
        )
        cleanup.callback(coordinator.stop)
        coordinator.start()
        camera = MujocoRobotIO(endpoint=endpoint, run=run, episode=episode)
        cleanup.callback(camera.stop)
        calibrated = threading.Event()
        camera.camera_info.subscribe(lambda _: calibrated.set())
        camera.start()
        assert calibrated.wait(5), "Trusted camera calibration unavailable"
        # Deterministic hold baseline on the exported lift task: expected score 0.
        positions = list(coordinator.get_joint_positions().values())
        accepted = coordinator.execute_trajectory(
            JointTrajectory(
                joint_names=joints,
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=positions),
                    TrajectoryPoint(time_from_start=0.5, positions=positions),
                ],
            )
        )
        assert accepted.status == TrajectoryExecutionStatus.ACCEPTED
        if mode == "blocked":
            assert not Path("/private").exists()
            assert not Path("/results").exists()
            assert not Path("/var/run/docker.sock").exists()
            assert not list(Path("/dev/shm").glob("dimos*"))
            assert not Path("/app/dimos/evals").exists()
            assert not Path("/app/dimos/simulation").exists()
            with socket.socket() as outbound:
                outbound.settimeout(0.5)
                assert outbound.connect_ex(("1.1.1.1", 443)) != 0
        if mode in ("blocked", "forgery"):
            client = zenoh.open(session_config(endpoint, trusted=False, run=run, episode=episode))
            cleanup.callback(client.close)
            for target in (
                "tf",
                "score",
                "private/tf",
                "admin/reset",
                "@/router/config",
                key(run, episode, "sensor"),
                key(run, "other-episode", "command"),
            ):
                client.put(target, b'{"score":1,"reset":true}')
            command_key = key(run, episode, "command")
            for payload in (
                b"not-json",
                b"x" * 4097,
                b'{"kind":"reset"}',
                Command(
                    run=run, episode="other", sequence=5, sent_at=time.time(), kind="stop"
                ).model_dump_json(),
            ):
                client.put(command_key, payload)
            # This uses an already consumed sequence; even a custom client cannot replay it.
            client.put(
                command_key,
                Command(
                    run=run, episode=episode, sequence=1, sent_at=time.time(), kind="disable"
                ).model_dump_json(),
            )
        coordinator.cancel_trajectory()
        print(json.dumps({"mode": mode, "normal_io": True, "probes_sent": mode != "normal"}))


if __name__ == "__main__":
    main()
