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
from pathlib import Path
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


def test_launch_flags(monkeypatch):
    monkeypatch.delenv("MUJOCOSIMMODULE__HEADLESS", raising=False)
    env = environment(tracked_bodies=("apple", "cup"))
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator == "mujoco"
    assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "true"
    assert json.loads(proc.extra_env["MUJOCOSIMMODULE__TRACKED_BODIES"]) == ["apple", "cup"]
    assert proc.global_args == [
        "--record-topics",
        "color_image,camera_info,coordinator_joint_state,tf,odom",
    ]

    proc = DimosCliCall()
    environment().configure_launch(proc)
    assert "MUJOCOSIMMODULE__TRACKED_BODIES" not in proc.extra_env

    proc = DimosCliCall()
    environment(scene=Path("scenes/table.xml")).configure_launch(proc)
    assert proc.global_args[-2:] == ["--mujoco-scene", str(Path("scenes/table.xml").resolve())]

    monkeypatch.setenv("MUJOCOSIMMODULE__HEADLESS", "false")
    proc = DimosCliCall()
    environment(
        module_env={"OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND": "yoloe"}
    ).configure_launch(proc)
    assert proc.extra_env["MUJOCOSIMMODULE__HEADLESS"] == "false"
    assert proc.extra_env["OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND"] == "yoloe"


def test_module_env_reaches_blueprint_parser(monkeypatch):
    monkeypatch.delenv("MUJOCOSIMMODULE__HEADLESS", raising=False)
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim

    proc = DimosCliCall()
    environment(
        tracked_bodies=("apple", "cup"),
        module_env={
            "OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND": "owlv2",
            "OBJECTSCENEREGISTRATIONMODULE__SEGMENTATION_BACKEND": "yolo",
        },
    ).configure_launch(proc)
    parsed = BlueprintConfigParser(xarm_perception_sim).parse(environ=proc.extra_env)
    perception = parsed.module_kwargs("objectsceneregistrationmodule")
    assert perception["detector_backend"] == "owlv2"
    assert perception["segmentation_backend"] == "yolo"
    sim = parsed.module_kwargs("mujocosimmodule")
    assert sim["headless"] is True
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


def test_settle_waits_for_joints_to_stop():
    env = environment(at_rest_s=0.0, settle_poll_s=0.01)
    env.settle(1.0)
    with MemoryStore() as store:
        env._recording = store
        started = time.monotonic()
        env.settle(1.0)
        joints = store.stream("coordinator_joint_state", JointState)
        env.settle(1.0)
        assert time.monotonic() - started < 0.5
        joints.append(JointState(ts=1.0, name=["j1"], position=[0.0], velocity=[0.5]))
        env._recording = store
        started = time.monotonic()
        env.settle(0.2)
        assert time.monotonic() - started >= 0.2

        joints.append(JointState(ts=2.0, name=["j1"], position=[0.0], velocity=[0.0]))
        started = time.monotonic()
        env.settle(5.0)
        assert time.monotonic() - started < 1.0
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
        assert proc.global_args[0] == "--record-topics" and proc.global_args[-1] == "--record"
        assert proc.demo_args == [
            "run",
            "xarm-perception-sim",
            "mcp-server",
            "speak-skill",
            "--disable",
            "rerun-bridge-module",
        ]
        assert "MUJOCOSIMMODULE__HEADLESS" in proc.extra_env
        ready.assert_called_once()
        assert set(result.artifacts) == {"recording"}
    finally:
        env.stop()
    proc.stop.assert_called_once()
    store.stop.assert_called_once()
