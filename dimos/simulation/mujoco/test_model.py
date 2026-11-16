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

from collections.abc import Iterator
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import pytest
from pytest_mock import MockerFixture
from structlog.testing import capture_logs

from dimos.simulation.mujoco import model, policy


@pytest.mark.parametrize(
    ("robot", "scene_name", "asset_robot"),
    [
        ("unitree_go2", "unitree_go2_scene (unitree_go1 stand-in)", "unitree_go1"),
        ("unitree_go1", "unitree_go1_scene", "unitree_go1"),
        ("unitree_g1", "unitree_g1_scene", "unitree_g1"),
    ],
)
def test_scene_identifies_requested_robot_and_keeps_asset(
    robot: str, scene_name: str, asset_robot: str
) -> None:
    root = ET.fromstring(model.get_model_xml(robot, "<mujoco/>"))

    assert root.get("model") == scene_name
    include = root.find("include")
    assert include is not None
    assert include.get("file") == f"{asset_robot}.xml"


@pytest.fixture
def model_loading(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> Iterator[None]:
    warning_callback = mujoco.get_mju_user_warning()
    control_callback = mujoco.get_mjcb_control()
    mujoco.set_mju_user_warning(None)
    monkeypatch.chdir(tmp_path)
    mocker.patch.object(
        model,
        "get_model_xml",
        return_value="""<mujoco><worldbody><body>
          <freejoint/><geom type="sphere" size="0.1" mass="1"/>
        </body></worldbody><keyframe><key name="home"/></keyframe></mujoco>""",
    )
    mocker.patch.object(model, "get_assets", return_value={})
    mocker.patch.object(model, "_get_data_dir", return_value=tmp_path)
    try:
        yield
    finally:
        mujoco.set_mjcb_control(control_callback)
        mujoco.set_mju_user_warning(warning_callback)


@pytest.mark.parametrize(
    ("robot", "robot_label", "asset_robot", "controller_type", "timestep"),
    [
        (
            "unitree_go2",
            "unitree_go2 (unitree_go1 stand-in)",
            "unitree_go1",
            policy.Go1OnnxController,
            0.005,
        ),
        ("unitree_go1", "unitree_go1", "unitree_go1", policy.Go1OnnxController, 0.005),
        ("unitree_g1", "unitree_g1", "unitree_g1", policy.G1OnnxController, 0.002),
    ],
)
def test_loaded_policy_identifies_requested_robot_and_keeps_controller(
    model_loading: None,
    tmp_path: Path,
    robot: str,
    robot_label: str,
    asset_robot: str,
    controller_type: type[policy.OnnxController],
    timestep: float,
    mocker: MockerFixture,
) -> None:
    session = mocker.patch.object(policy.ort, "InferenceSession")
    session.return_value.get_providers.return_value = ["CPUExecutionProvider"]
    with capture_logs() as records:
        mj_model, _ = model.load_model(mocker.Mock(), robot=robot, scene_xml="")

    policy_path = (tmp_path / f"{asset_robot}_policy.onnx").as_posix()
    session.assert_called_once_with(
        policy_path, sess_options=mocker.ANY, providers=["CPUExecutionProvider"]
    )
    assert isinstance(mujoco.get_mjcb_control().__self__, controller_type)
    assert mj_model.opt.timestep == timestep
    assert records == [
        {
            "event": "Loaded policy",
            "robot": robot_label,
            "policy_path": policy_path,
            "providers": ["CPUExecutionProvider"],
            "log_level": "info",
        }
    ]


@pytest.fixture
def g1_model(model_loading: None, mocker: MockerFixture) -> tuple[mujoco.MjModel, mujoco.MjData]:
    mocker.patch.object(model, "G1OnnxController")
    return model.load_model(mocker.Mock(), robot="unitree_g1", scene_xml="")


def test_g1_instability_warning_is_logged_without_cwd_file(
    g1_model: tuple[mujoco.MjModel, mujoco.MjData], tmp_path: Path
) -> None:
    mj_model, data = g1_model
    data.qfrc_applied[3] = float("inf")

    with capture_logs() as records:
        mujoco.mj_step(mj_model, data)

    assert not (tmp_path / "MUJOCO_LOG.TXT").exists()
    assert records == [
        {
            "event": "MuJoCo warning",
            "warning": "Nan, Inf or huge value in QACC at DOF 3. The simulation is unstable. Time = 0.0000.",
            "log_level": "warning",
        }
    ]
