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
import struct
import zlib

import mujoco
from pydantic import ValidationError
import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.lib.benchmark_scenes import (
    BenchmarkCase,
    GradeSpec,
    cases_from_manifest,
    load_manifest,
    tracked_bodies_for_case,
    write_manifest,
    xarm_table_env,
)
from dimos.evals.environments.lib.export_benchmark_scenes import export_libero_pro, export_robocasa
from dimos.evals.environments.mujoco_sim import MujocoEnvironment


def _write_png(path: Path) -> None:
    """Write a tiny valid 1x1 RGB PNG."""
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = b"\x00\xff\x00\x00"  # filter byte + RGB
    compressed = zlib.compress(raw)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", compressed)
        + chunk(b"IEND", b"")
    )


def _fake_xarm7(root: Path) -> Path:
    """Minimal xarm pack so export/load tests never pull LFS."""
    xarm = root / "xarm7"
    (xarm / "assets").mkdir(parents=True)
    (xarm / "xarm7.xml").write_text(
        '<mujoco model="xarm7">\n'
        "  <worldbody>\n"
        '    <body name="link_base" pos="0 0 0.12">\n'
        '      <geom type="cylinder" size="0.05 0.05" rgba="0.8 0.8 0.8 1"/>\n'
        "    </body>\n"
        "  </worldbody>\n"
        "</mujoco>\n"
    )
    return xarm


def _fake_libero(root: Path) -> Path:
    textures = root / "libero" / "libero" / "assets" / "textures"
    _write_png(textures / "tile_grigia_caldera_porcelain_floor.png")
    _write_png(textures / "table_light_wood.png")
    return root


def _fake_robocasa(root: Path) -> Path:
    _write_png(root / "robocasa" / "models" / "assets" / "textures" / "marble.png")
    return root


@pytest.fixture
def data_root(tmp_path: Path) -> Path:
    _fake_xarm7(tmp_path)
    return tmp_path


@pytest.fixture
def exported(data_root: Path) -> Path:
    export_libero_pro(
        _fake_libero(data_root / "upstream_libero"), data_root / "libero_pro", data_dir=data_root
    )
    export_robocasa(
        _fake_robocasa(data_root / "upstream_robocasa"), data_root / "robocasa", data_dir=data_root
    )
    return data_root


def test_export_loadable_and_autogen(exported: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "dimos.evals.environments.lib.benchmark_scenes.get_data",
        lambda name: exported / name,
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
        assert suite[0].inputs
        assert body in suite[0].environment.config.tracked_bodies


def test_scene_flag(exported: Path) -> None:
    scene = exported / "libero_pro" / "smoke" / "scene.xml"
    proc = DimosCliCall()
    xarm_table_env(scene, ("akita_black_bowl",)).configure_launch(proc)
    assert proc.simulator == "mujoco"
    assert proc.extra_env["MUJOCOSIMMODULE__ADDRESS"] == str(scene.resolve())


def test_grade_body_is_tracked_when_manifest_omits_it() -> None:
    case = BenchmarkCase(
        id="lift",
        language="lift",
        scene="smoke/scene.xml",
        tracked_bodies=(),
        grade=GradeSpec(type="lifted", body="cup"),
    )
    assert tracked_bodies_for_case(case) == ("cup",)


def test_by_m_must_be_positive() -> None:
    with pytest.raises(ValidationError):
        GradeSpec(type="lifted", body="cup", by_m=0.0)
    with pytest.raises(ValidationError):
        GradeSpec(type="lifted", body="cup", by_m=-0.1)


def test_local_manifest_fixture(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Wiring without upstream checkouts: hand-written pack under a temp data root."""
    pack = tmp_path / "fixture_bench"
    scene_dir = pack / "smoke"
    scene_dir.mkdir(parents=True)
    (scene_dir / "scene.xml").write_text(
        '<mujoco model="fixture"><worldbody><body name="cup" pos="0 0 0.1">'
        '<freejoint/><geom type="sphere" size="0.03"/></body></worldbody></mujoco>\n'
    )
    write_manifest(
        pack / "manifest.json",
        {
            "source": "FIXTURE",
            "root": "fixture_bench",
            "cases": [
                {
                    "id": "fixture_lift",
                    "language": "Lift the cup.",
                    "scene": "smoke/scene.xml",
                    "tracked_bodies": [],
                    "grade": {"type": "lifted", "body": "cup", "by_m": 0.05},
                }
            ],
        },
    )
    monkeypatch.setattr(
        "dimos.evals.environments.lib.benchmark_scenes.get_data",
        lambda name: tmp_path / name,
    )
    suite = cases_from_manifest(load_manifest("fixture_bench"))
    assert suite[0].id == "fixture_lift"
    assert suite[0].environment.config.tracked_bodies == ("cup",)
