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
import sys
import tarfile
import xml.etree.ElementTree as ET

from git import Actor, Repo
import mujoco
import pytest
from pytest_mock import MockerFixture
from structlog.testing import capture_logs

from dimos.simulation.mujoco import constants, model, policy
from dimos.utils import data


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


def test_meshes_after_offline_upgrade(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    data_dir = tmp_path / "data"
    (data_dir / "mujoco_sim").mkdir(parents=True)
    monkeypatch.setattr(data, "get_data_dir", lambda: data_dir)
    monkeypatch.setattr(data, "_get_lfs_dir", lambda: data_dir / ".lfs")

    package = tmp_path / "mujoco_playground"
    legacy = package / "external_deps/mujoco_menagerie"
    mesh = legacy / "unitree_go1/assets/hip.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"legacy mesh")
    pelvis = legacy / "unitree_g1/assets/pelvis.STL"
    pelvis.parent.mkdir(parents=True)
    pelvis.write_bytes(b"pelvis mesh")
    with Repo.init(legacy) as repo:
        repo.index.add([str(mesh), str(pelvis)])
        actor = Actor("Fixture", "fixture@example.com")
        pin = repo.index.commit("Robot meshes", author=actor, committer=actor).hexsha
        repo.git.checkout(pin)
    mocker.patch.dict(vars(constants), MENAGERIE_COMMIT=pin)
    # An exact sync removes the installed package but leaves the downloaded meshes.
    with monkeypatch.context() as isolated:
        isolated.setattr(sys, "path", [str(tmp_path)])
        assert model.get_assets()["hip.stl"] == b"legacy mesh"

    (package / "__init__.py").touch()
    monkeypatch.syspath_prepend(str(tmp_path))
    head = legacy / ".git/HEAD"

    assert model.get_assets()["hip.stl"] == b"legacy mesh"

    pointer = data_dir / ".lfs/mujoco_menagerie.tar.gz"
    pointer.parent.mkdir()
    pointer.write_text(
        "version https://git-lfs.github.com/spec/v1\n"
        "oid sha256:5d19265c8aefb6e80b92b803ae164f18d214b827f9f090436eccdf831c5db236\n"
        "size 18980818\n"
    )
    native = mocker.patch.object(data, "_pull_lfs_archive", side_effect=RuntimeError("offline"))
    assert model.get_assets()["hip.stl"] == b"legacy mesh"

    pelvis.unlink()
    with pytest.raises(RuntimeError, match="offline"):
        model.get_assets()
    pelvis.write_bytes(b"broken mesh")
    with pytest.raises(RuntimeError, match="offline"):
        model.get_assets()
    pelvis.write_bytes(b"pelvis mesh")

    head.write_text("0" * 40 + "\n")
    with pytest.raises(RuntimeError, match="offline"):
        model.get_assets()

    head.write_text(pin + "\n")
    original_pointer = pointer.read_text()
    pointer.write_text(original_pointer.replace("5d19265", "0" * 7))
    with pytest.raises(RuntimeError, match="offline"):
        model.get_assets()

    pointer.write_text(original_pointer)
    pelvis.unlink()
    bundled = tmp_path / "bundled-hip.stl"
    bundled.write_bytes(b"bundled mesh")
    archive = tmp_path / "mujoco_menagerie.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(bundled, arcname="mujoco_menagerie/unitree_go1/assets/hip.stl")
    native.side_effect = None
    native.return_value = archive
    assert model.get_assets()["hip.stl"] == b"bundled mesh"
    pelvis.write_bytes(b"pelvis mesh")
    assert model.get_assets()["hip.stl"] == b"bundled mesh"


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
