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

from __future__ import annotations

from pathlib import Path

import mujoco
import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.lib.benchmark_scenes import (
    cases_from_manifest,
    load_manifest,
    xarm_table_env,
)
from dimos.evals.environments.lib.export_benchmark_scenes import export_libero_pro, export_robocasa
from dimos.evals.environments.mujoco_sim import MujocoEnvironment

_LIBERO = Path("/tmp/bench_inv/LIBERO-PRO")
_ROBOCASA = Path("/tmp/bench_inv/robocasa")


@pytest.fixture(scope="module")
def exported(tmp_path_factory: pytest.TempPathFactory) -> Path:
    if not _LIBERO.is_dir() or not _ROBOCASA.is_dir():
        pytest.skip("need /tmp/bench_inv/{LIBERO-PRO,robocasa}")
    xarm = Path(__file__).resolve().parents[2] / "data" / "xarm7"
    if not xarm.is_dir():
        pytest.skip(f"missing {xarm}")
    root = tmp_path_factory.mktemp("benchmark_data")
    (root / "xarm7").symlink_to(xarm)
    export_libero_pro(_LIBERO, root / "libero_pro", data_dir=root)
    export_robocasa(_ROBOCASA, root / "robocasa", data_dir=root)
    return root


def test_export_loadable_and_autogen(exported: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "dimos.evals.environments.lib.benchmark_scenes.get_data_dir", lambda: exported
    )
    for root, body in (("libero_pro", "akita_black_bowl"), ("robocasa", "apple")):
        manifest = load_manifest(root)
        scene = exported / root / manifest.cases[0].scene
        model = mujoco.MjModel.from_xml_path(str(scene))
        names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)}
        assert body in names

        suite = cases_from_manifest(manifest)
        assert len(suite) == 1
        assert isinstance(suite[0].environment, MujocoEnvironment)
        assert suite[0].inputs  # language from manifest


def test_scene_flag(exported: Path) -> None:
    scene = exported / "libero_pro" / "smoke" / "scene.xml"
    proc = DimosCliCall()
    xarm_table_env(scene, ("akita_black_bowl",)).configure_launch(proc)
    assert proc.simulator == "mujoco"
    assert proc.global_args[-2:] == ["--mujoco-scene", str(scene.resolve())]
