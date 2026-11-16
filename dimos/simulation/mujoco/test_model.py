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

import mujoco
import pytest
from pytest_mock import MockerFixture
from structlog.testing import capture_logs

from dimos.simulation.mujoco import model


@pytest.fixture
def g1_model(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> Iterator[tuple[mujoco.MjModel, mujoco.MjData]]:
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
    mocker.patch.object(model, "G1OnnxController")
    try:
        yield model.load_model(mocker.Mock(), robot="unitree_g1", scene_xml="")
    finally:
        mujoco.set_mjcb_control(control_callback)
        mujoco.set_mju_user_warning(warning_callback)


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
