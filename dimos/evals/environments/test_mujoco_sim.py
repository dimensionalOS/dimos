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

import json
import time

import numpy as np
import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def environment(**kwargs):
    return MujocoEnvironment(blueprint=["xarm-perception-sim", "mcp-server"], **kwargs)


def _tf(ts: float, child: str, z: float) -> TFMessage:
    return TFMessage(
        Transform(translation=Vector3(0.4, 0.08, z), frame_id="world", child_frame_id=child, ts=ts)
    )


def test_launch_flags():
    env = environment(tracked_bodies=("apple", "cup"))
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator == "mujoco"
    assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "true"
    assert json.loads(proc.extra_env["MUJOCOSIMMODULE__TRACKED_BODIES"]) == ["apple", "cup"]

    proc = DimosCliCall()
    environment(headless=False).configure_launch(proc)
    assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "false"
    assert "MUJOCOSIMMODULE__TRACKED_BODIES" not in proc.extra_env

    proc = DimosCliCall()
    environment(
        module_env={
            "OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND": "yoloe",
            "MUJOCOSIMMODULE__HEADLESS": "false",  # the explicit headless field wins
        }
    ).configure_launch(proc)
    assert proc.extra_env["OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND"] == "yoloe"
    assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "true"


def test_module_env_reaches_blueprint_parser():
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.evals.suites.mujoco_xarm import LOCAL_PERCEPTION
    from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim

    proc = DimosCliCall()
    environment(
        headless=False, tracked_bodies=("apple", "cup"), module_env=LOCAL_PERCEPTION
    ).configure_launch(proc)
    parsed = BlueprintConfigParser(xarm_perception_sim).parse(environ=proc.extra_env)
    perception = parsed.module_kwargs("objectsceneregistrationmodule")
    assert perception["detector_backend"] == "owlv2"
    assert perception["segmentation_backend"] == "yolo"
    sim = parsed.module_kwargs("mujocosimmodule")
    assert sim["headless"] is False  # the environment beats the blueprint's pinned value
    assert sim["tracked_bodies"] == ["apple", "cup"]


def test_latest_pose_needs_odom():
    env = environment()
    with MemoryStore() as store:
        with pytest.raises(LookupError):
            env.latest_pose(store)
        store.stream("odom", PoseStamped).append(PoseStamped(ts=5, frame_id="world"))
        assert env.latest_pose(store).ts == 5


def test_ready_needs_fresh_streams_and_tracked_body_poses():
    env = environment(tracked_bodies=("apple",))
    with MemoryStore() as store:
        with pytest.raises(TimeoutError, match="apple"):
            env.wait_ready(store, deadline=time.monotonic() + 0.3)
        now = time.time()
        store.stream("color_image", Image).append(
            Image(data=np.zeros((1, 1, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=now)
        )
        store.stream("coordinator_joint_state", JointState).append(
            JointState(ts=now, name=["j1"], position=[0.0], velocity=[0.0])
        )
        store.stream("tf", TFMessage).append(_tf(now, "apple", 0.17))
        env.wait_ready(store, deadline=time.monotonic() + 2.0)
        assert env.episode_metadata()["initial_body_positions"] == {"apple": [0.4, 0.08, 0.17]}


def test_settle_waits_for_joints_to_stop():
    env = environment(at_rest_s=0.0, settle_poll_s=0.01)
    with MemoryStore() as store:
        joints = store.stream("coordinator_joint_state", JointState)
        joints.append(JointState(ts=1.0, name=["j1"], position=[0.0], velocity=[0.5]))
        env._recording = store
        started = time.monotonic()
        env.settle(0.2)
        assert time.monotonic() - started >= 0.2  # still moving: waits out the budget

        joints.append(JointState(ts=2.0, name=["j1"], position=[0.0], velocity=[0.0]))
        started = time.monotonic()
        env.settle(5.0)
        assert time.monotonic() - started < 1.0  # at rest: returns early
    env._recording = None


def test_launch_and_cleanup(tmp_path, mocker):
    proc = mocker.patch("dimos.evals.environments.sim.DimosCliCall").return_value
    proc.extra_env = {}
    proc.global_args = []
    mocker.patch(
        "dimos.evals.environments.sim.McpAdapter"
    ).return_value.wait_for_ready.return_value = True
    store = mocker.patch("dimos.memory.store.sqlite.SqliteStore").return_value
    env = environment(tracked_bodies=("apple",), disable=("rerun-bridge-module",))
    mocker.patch.object(env, "_wait_recording", return_value=tmp_path / "memory.db")
    ready = mocker.patch.object(env, "wait_ready")
    try:
        result = env.start(("speak-skill",))
        assert proc.simulator == "mujoco"
        assert proc.global_args == ["--record"]
        assert proc.demo_args == [
            "run",
            "xarm-perception-sim",
            "mcp-server",
            "speak-skill",
            "--disable",
            "rerun-bridge-module",
        ]
        assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "true"
        ready.assert_called_once()
        episode = json.loads(result.artifacts["episode"].read_text())
        assert episode["backend"] == "mujoco"
        assert episode["tracked_bodies"] == ["apple"]
    finally:
        env.stop()
    proc.stop.assert_called_once()
    store.stop.assert_called_once()
