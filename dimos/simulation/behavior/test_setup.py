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
import sys

import pytest

from dimos.simulation.behavior import setup


def test_setup_requires_both_license_acceptances(mocker):
    mocker.patch.object(sys, "argv", ["setup", "--accept-nvidia-eula"])
    run = mocker.patch.object(setup.subprocess, "run")
    with pytest.raises(SystemExit) as error:
        setup.main()
    assert error.value.code == 2
    run.assert_not_called()


def test_upstream_shadowing_fix_preserves_other_packages(tmp_path):
    root = tmp_path / ".venv/lib/python3.11/site-packages/isaacsim/extscache"
    archive = root / "omni.services/pip_prebundle"
    for name in ("websockets", "packaging", "other"):
        (archive / name).mkdir(parents=True)
        (archive / name / "__init__.py").write_text("# vendored")
    setup.repair_isaac_prebundles(tmp_path)
    assert sorted(p.name for p in archive.iterdir()) == ["other"]


def test_runtime_environment_uses_recorded_assets_and_own_toolchain(tmp_path, monkeypatch):
    (tmp_path / setup.MARKER).write_text(
        json.dumps({"data_path": "/assets/behavior", "complete": True})
    )
    monkeypatch.setenv("ISAAC_PATH", "/host/isaac")
    monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", "/host/venv")
    env = setup.runtime_environment(tmp_path)
    assert env["OMNIGIBSON_DATA_PATH"] == "/assets/behavior"
    assert env["CUDA_HOME"] == str(tmp_path / ".pixi/envs/default")
    assert "ISAAC_PATH" not in env
    assert "UV_PROJECT_ENVIRONMENT" not in env
