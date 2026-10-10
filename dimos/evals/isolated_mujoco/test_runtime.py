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

"""Engine-produced evidence stays private even when a custom client forges TF/score."""

from contextlib import ExitStack
import json
import socket
import threading

import pytest
import zenoh

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.evals.isolated_mujoco.runtime import EpisodeConfig, TrustedRuntime
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.protocol.mujoco_eval import key, session_config
from dimos.robot.manipulators.mujoco_robot_io import MujocoRobotIO

pytestmark = pytest.mark.mujoco


def synthetic_scene(path):
    # Eight actuated joints and an unactuated object: no exported benchmark data.
    bodies = "".join(
        f'<body name="link{i}" pos="0 0 0.05"><joint name="joint{i}" '
        'type="hinge" range="-1 1"/><geom type="sphere" size="0.01" mass="1"/>'
        for i in range(1, 8)
    )
    bodies += '<camera name="wrist_camera" pos="0.3 0 0.3"/>' + "</body>" * 7
    actuators = "".join(
        f'<position name="act{i}" joint="joint{i}" kp="20" ctrllimited="true" ctrlrange="-1 1"/>'
        for i in range(1, 8)
    )
    path.write_text(
        '<mujoco><compiler angle="radian"/><option gravity="0 0 0" timestep="0.01" integrator="implicitfast"/><default><joint damping="5" armature="1"/><geom contype="0" conaffinity="0"/></default><worldbody>'
        + bodies
        + '<body name="gripper"><joint name="gripper_joint" type="slide" range="0 0.8"/>'
        '<geom type="sphere" size="0.01" mass="1"/></body>'
        '<body name="cube_main" pos="0.1 0 0.5"><freejoint/>'
        '<geom type="box" size="0.02 0.02 0.02" mass="0.1"/></body></worldbody><actuator>'
        + actuators
        + '<position name="gripper_act" joint="gripper_joint" kp="20" '
        'ctrllimited="true" ctrlrange="0 1"/></actuator></mujoco>'
    )


def test_trusted_engine_records_and_grades_without_agent_evidence(tmp_path):
    scene = tmp_path / "scene.xml"
    synthetic_scene(scene)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{sock.getsockname()[1]}"
    runtime = TrustedRuntime(
        EpisodeConfig(
            run="run",
            episode="episode",
            endpoint=endpoint,
            scene=scene,
            output=tmp_path / "private",
            duration_s=3,
            home=[0.0] * 7,
        )
    )
    result = []
    completed = threading.Event()

    def run():
        try:
            result.append(runtime.run())
        finally:
            completed.set()

    with ExitStack() as cleanup:
        thread = threading.Thread(target=run)
        thread.start()
        cleanup.callback(thread.join, 5)
        joints = [f"arm/joint{i}" for i in range(1, 8)] + ["arm/gripper"]
        coordinator = ControlCoordinator(
            hardware=[
                HardwareComponent(
                    hardware_id="arm",
                    hardware_type=HardwareType.MANIPULATOR,
                    joints=joints,
                    adapter_type="mujoco_eval",
                    address=endpoint,
                    adapter_kwargs={"run": "run", "episode": "episode"},
                )
            ],
            tasks=[joint_trajectory_task(joints)],
            publish_joint_state=False,
        )
        cleanup.callback(coordinator.stop)
        coordinator.start()
        positions = list(coordinator.get_joint_positions().values())
        acceptance = coordinator.execute_trajectory(
            JointTrajectory(
                joint_names=joints,
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=positions),
                    TrajectoryPoint(time_from_start=0.4, positions=[0.2] * 7 + [0.4]),
                ],
            )
        )
        assert acceptance.status == TrajectoryExecutionStatus.ACCEPTED
        camera = MujocoRobotIO(endpoint=endpoint, run="run", episode="episode")
        cleanup.callback(camera.stop)
        received = threading.Event()
        calibrated = threading.Event()
        camera.color_image.subscribe(lambda _: received.set())
        camera.camera_info.subscribe(lambda _: calibrated.set())
        camera.start()
        assert received.wait(2)
        assert calibrated.wait(2)
        assert camera.get_color_camera_info() is not None
        client = zenoh.open(session_config(endpoint, trusted=False, run="run", episode="episode"))
        cleanup.callback(client.close)
        for target in ("tf", "score", "admin/reset", key("run", "episode", "sensor")):
            client.put(target, b'{"cube_main":{"z":100},"score":1,"reset":true}')
        assert completed.wait(5)
    assert result[0]["error"] == ""
    assert result[0]["score"] == 0.0
    assert result[0]["attempts"] >= 2
    with SqliteStore(path=str(runtime.recording), must_exist=True) as store:
        assert "tf" in store.streams
        assert "color_image" in store.streams
        assert store.streams.coordinator_joint_state.last().data.position[0] > 0


def test_missing_scene_produces_unknown_infrastructure_result(tmp_path):
    config = EpisodeConfig(
        run="run",
        episode="episode",
        endpoint="tcp/127.0.0.1:7449",
        scene=tmp_path / "missing.xml",
        output=tmp_path / "private",
        duration_s=1,
        home=[0.0] * 7,
    )
    with pytest.raises(FileNotFoundError):
        TrustedRuntime(config)
    result = json.loads((config.output / "result.json").read_text())
    assert result["score"] is None
    assert result["error"].startswith("Infrastructure startup error:")
