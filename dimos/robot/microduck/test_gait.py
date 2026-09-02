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

"""Observation contract (``MicroduckObserver``), ONNX session loading and the
single-policy ``MicroduckGaitPolicy`` wrapper the plain microduck-sim
blueprint runs. Tests needing the asset cache skip when it is absent."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.robot.microduck import assets_fetch
from dimos.robot.microduck.gait import (
    COMMAND_LEN,
    CONTROL_DT,
    OBS_LEN,
    VX_RANGE,
    VY_RANGE,
    WZ_RANGE,
    MicroduckGaitPolicy,
    MicroduckObserver,
    load_policy_session,
)

ROOM_SCENE = Path(__file__).with_name("assets") / "room_scene.xml"
EXPECTED_JOINTS = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)


def _cache_or_skip() -> None:
    if (
        not assets_fetch.robot_mjcf_path().exists()
        or not assets_fetch.walking_policy_path().exists()
    ):
        pytest.skip(f"Microduck asset cache not present ({assets_fetch.assets_root()})")


def _model(robot_xml: Path) -> Any:
    import mujoco

    scene = mujoco.MjSpec.from_file(str(ROOM_SCENE))
    robot = mujoco.MjSpec.from_file(str(robot_xml))
    scene.option.timestep = 0.005
    robot.option.timestep = 0.005
    scene.attach(robot, prefix="", frame=scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0]))
    return scene.compile()


def test_contract_constants() -> None:
    assert OBS_LEN == 61 and COMMAND_LEN == 13
    assert CONTROL_DT == pytest.approx(0.02)
    assert 3 + 3 + 14 + 14 + 14 + COMMAND_LEN == OBS_LEN
    assert VX_RANGE == (-0.25, 0.3) and VY_RANGE == (-0.2, 0.2) and WZ_RANGE == (-1.5, 1.5)


def test_load_policy_session_reads_metadata() -> None:
    pytest.importorskip("onnxruntime")
    _cache_or_skip()
    session = load_policy_session(assets_fetch.walking_policy_path())
    assert session.path == assets_fetch.walking_policy_path()
    assert session.joint_names == EXPECTED_JOINTS
    assert session.default_pose.shape == (14,) and session.default_pose.dtype == np.float32
    assert session.default_pose[2] == pytest.approx(-0.458, abs=1e-3)
    assert session.action_scale == 1.0
    assert session.input_name == "obs" and session.output_name == "actions"
    action = session.run(np.zeros(OBS_LEN, dtype=np.float32))
    assert action.shape == (14,) and action.dtype == np.float32 and np.isfinite(action).all()


def test_load_policy_session_validates_metadata(monkeypatch: pytest.MonkeyPatch) -> None:
    ort = pytest.importorskip("onnxruntime")

    class Meta:
        def __init__(self, custom: dict[str, str]) -> None:
            self.custom_metadata_map = custom

    class IO:
        def __init__(self, name: str, shape: list[Any]) -> None:
            self.name = name
            self.shape = shape

    class FakeSession:
        meta: dict[str, str] = {}
        obs_dim = OBS_LEN

        def __init__(self, path: str) -> None:
            self.path = path

        def get_modelmeta(self) -> Meta:
            return Meta(dict(self.meta))

        def get_inputs(self) -> list[IO]:
            return [IO("obs", [1, self.obs_dim])]

        def get_outputs(self) -> list[IO]:
            return [IO("actions", [1, 14])]

    monkeypatch.setattr(ort, "InferenceSession", FakeSession)
    names = ",".join(EXPECTED_JOINTS)
    pose = ",".join(["0.1"] * 14)

    FakeSession.meta = {"default_joint_pos": pose}
    with pytest.raises(RuntimeError, match="metadata"):
        load_policy_session("x.onnx")
    FakeSession.meta = {"joint_names": names, "default_joint_pos": ",".join(["0.1"] * 13)}
    with pytest.raises(RuntimeError, match="13-long default pose"):
        load_policy_session("x.onnx")
    FakeSession.meta = {"joint_names": names, "default_joint_pos": pose}
    FakeSession.obs_dim = 57
    with pytest.raises(RuntimeError, match="obs dim 57"):
        load_policy_session("x.onnx")
    FakeSession.obs_dim = OBS_LEN
    FakeSession.meta = {"joint_names": names, "default_joint_pos": pose, "action_scale": "0.5"}
    session = load_policy_session("x.onnx")
    assert session.action_scale == 0.5 and session.joint_names == EXPECTED_JOINTS
    assert session.default_pose.tolist() == pytest.approx([0.1] * 14)


@pytest.mark.mujoco
def test_observer_builds_the_61_float_observation() -> None:
    mujoco = pytest.importorskip("mujoco")
    _cache_or_skip()
    model = _model(assets_fetch.variant_mjcf_path("default"))
    home = np.linspace(-0.3, 0.3, 14, dtype=np.float32)
    observer = MicroduckObserver(model, EXPECTED_JOINTS, home)
    assert observer.num_joints == 14 and observer.joint_names == list(EXPECTED_JOINTS)
    data = mujoco.MjData(model)
    data.qpos[observer.root_qpos_adr : observer.root_qpos_adr + 2] = (0.4, -0.2)
    observer.initial_qpos(data)
    mujoco.mj_forward(model, data)
    assert data.qpos[observer.root_qpos_adr : observer.root_qpos_adr + 3].tolist() == pytest.approx(
        [0.4, -0.2, 0.125]
    )
    last_action = np.full(14, 0.25, dtype=np.float32)
    command = np.arange(COMMAND_LEN, dtype=np.float32)
    obs = observer.build(data, last_action, command)
    assert obs.shape == (OBS_LEN,) and obs.dtype == np.float32
    assert obs[0:3].tolist() == pytest.approx([0.0, 0.0, 0.0])  # gyro at rest
    assert obs[3:6].tolist() == pytest.approx([0.0, 0.0, -1.0])  # upright gravity
    assert obs[6:20].tolist() == pytest.approx([0.0] * 14, abs=1e-6)  # at home pose
    assert obs[20:34].tolist() == pytest.approx([0.0] * 14)
    assert obs[34:48].tolist() == pytest.approx([0.25] * 14)
    assert obs[48:61].tolist() == pytest.approx(list(range(COMMAND_LEN)))

    # Yaw and tilt come from the root quaternion.
    half = math.pi / 4
    data.qpos[observer.root_qpos_adr + 3 : observer.root_qpos_adr + 7] = (
        math.cos(half),
        0,
        0,
        math.sin(half),
    )
    assert observer.root_yaw(data) == pytest.approx(math.pi / 2)
    assert observer.projected_gravity(data).tolist() == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)
    data.qpos[observer.root_qpos_adr + 3 : observer.root_qpos_adr + 7] = (
        0.0,
        1.0,
        0.0,
        0.0,
    )  # upside down
    assert observer.projected_gravity(data)[2] == pytest.approx(1.0)

    with pytest.raises(ValueError):
        MicroduckObserver(model, EXPECTED_JOINTS, home[:3])
    with pytest.raises(RuntimeError, match="not found"):
        MicroduckObserver(model, ("left_hip_yaw", "no_such_joint"), home[:2])


@pytest.mark.mujoco
def test_gait_policy_wrapper_keeps_its_api() -> None:
    pytest.importorskip("onnxruntime")
    mujoco = pytest.importorskip("mujoco")
    _cache_or_skip()
    model = _model(assets_fetch.robot_mjcf_path())
    gait = MicroduckGaitPolicy(assets_fetch.walking_policy_path(), model)
    assert gait.joint_names == list(EXPECTED_JOINTS) and gait.num_joints == 14
    assert gait.default_pose.shape == (14,) and gait.action_scale == 1.0
    assert isinstance(gait.root_qpos_adr, int)

    data = mujoco.MjData(model)
    gait.initial_qpos(data)
    mujoco.mj_forward(model, data)
    assert gait.projected_gravity(data).tolist() == pytest.approx([0.0, 0.0, -1.0])
    gait.set_twist(1.0, -1.0, 9.0)
    obs = gait.build_observation(data)
    assert obs.shape == (OBS_LEN,)
    assert obs[48:51].tolist() == pytest.approx([VX_RANGE[1], VY_RANGE[0], WZ_RANGE[1]])
    targets = gait.step(data)
    assert targets.shape == (14,) and targets.dtype == np.float32 and np.isfinite(targets).all()
    assert gait.last_action.any()
    assert np.allclose(targets, gait.default_pose + gait.last_action)
    gait.reset()
    assert not gait.last_action.any()
    assert not gait.build_observation(data)[48:61].any()
