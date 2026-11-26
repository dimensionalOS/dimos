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
from typing import Any

import numpy as np
import pytest

from dimos.sim2.dimsim import dimsim_scene

pytest.importorskip("pxr")
trimesh = pytest.importorskip("trimesh")


@pytest.fixture
def pxr() -> Any:
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    return load_pxr()


@pytest.fixture
def meshes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    cache = tmp_path / "meshes"
    monkeypatch.setattr(dimsim_scene, "_MESHES", cache)
    return cache


def _glb(path: Path, visual: Any = None) -> Path:
    box = trimesh.creation.box()
    if visual is not None:
        box.visual = visual
    path.write_bytes(trimesh.Scene(box).export(file_type="glb"))
    return path


def test_the_mesh_cache_is_made_for_this_user_alone(pxr: Any, meshes: Path, tmp_path: Path) -> None:
    dimsim_scene._mesh_parts(pxr, _glb(tmp_path / "box.glb"))
    assert meshes.stat().st_mode & 0o777 == 0o700


def test_a_mesh_cache_others_can_write_to_is_refused(
    pxr: Any, meshes: Path, tmp_path: Path
) -> None:
    meshes.mkdir()
    meshes.chmod(0o777)
    with pytest.raises(PermissionError, match="writable by you alone"):
        dimsim_scene._mesh_parts(pxr, _glb(tmp_path / "box.glb"))


def test_the_mesh_cache_is_not_in_the_shared_temporary_directory() -> None:
    import tempfile

    from dimos.constants import CACHE_DIR

    assert dimsim_scene._MESHES.is_relative_to(CACHE_DIR)
    assert not dimsim_scene._MESHES.is_relative_to(tempfile.gettempdir())


def _compiled(pxr: Any, glb: Path, tmp_path: Path) -> Any:
    """The GLB added to a scene elsewhere, as the sensors' MuJoCo compiles it."""
    import mujoco

    from dimos.sim2.mujoco.world import compile_world

    stage = pxr.Usd.Stage.CreateNew(str(tmp_path / "scene" / "scene.usda"))
    stage.SetDefaultPrim(pxr.UsdGeom.Xform.Define(stage, "/scene").GetPrim())
    dimsim_scene._add_geometry(pxr, stage, "/scene/box", glb)
    stage.Save()
    model = compile_world(tmp_path / "scene" / "scene.usda").compile()
    assert isinstance(model, mujoco.MjModel)
    return model


def test_a_glb_keeps_its_base_color(pxr: Any, meshes: Path, tmp_path: Path) -> None:
    red = trimesh.visual.material.PBRMaterial(baseColorFactor=[255, 0, 0, 255])
    glb = _glb(tmp_path / "red.glb", trimesh.visual.TextureVisuals(material=red))
    model = _compiled(pxr, glb, tmp_path)
    (geom,) = np.flatnonzero(model.geom_matid >= 0)
    assert model.mat_rgba[model.geom_matid[geom]] == pytest.approx([1.0, 0.0, 0.0, 1.0])


def test_a_glb_keeps_its_texture(pxr: Any, meshes: Path, tmp_path: Path) -> None:
    from PIL import Image

    blue = Image.fromarray(np.full((4, 4, 3), [0, 0, 255], np.uint8))
    box = trimesh.creation.box()
    textured = trimesh.visual.TextureVisuals(
        uv=np.zeros((len(box.vertices), 2)),
        material=trimesh.visual.material.PBRMaterial(baseColorTexture=blue),
    )
    glb = _glb(tmp_path / "blue.glb", textured)
    model = _compiled(pxr, glb, tmp_path)
    assert model.ntex == 1
    pixels = model.tex_data[: model.tex_width[0] * model.tex_height[0] * 3].reshape(-1, 3)
    assert (pixels == [0, 0, 255]).all()


def test_the_scenes_git_lfs_pointers_are_fetched_in_one_pull_before_any_is_read(
    meshes: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    scene = tmp_path / "scenes" / "room"
    (scene / "objects").mkdir(parents=True)
    (scene / "index.js").write_text("")
    pointer = "version https://git-lfs.github.com/spec/v1\noid sha256:0\n"
    for glb in (scene / "structure.glb", scene / "objects" / "door.glb"):
        glb.write_text(pointer)
    state = {"id": "shut", "name": "shut", "file": "door.glb"}
    door = {
        "title": "door",
        "transform": {
            key: dict.fromkeys("xyz", 1 if key == "scale" else 0)
            for key in ("position", "rotation", "scale")
        },
        "states": [state],
        "currentStateId": "shut",
    }
    (scene / "objects" / "manifest.json").write_text(json.dumps([door]))
    monkeypatch.setattr(dimsim_scene, "DIMSIM_SCENES", tmp_path / "scenes")
    pulled: list[list[Path]] = []

    def pull(paths: list[Path]) -> None:
        pulled.append(list(paths))
        for path in paths:
            _glb(path)

    monkeypatch.setattr(dimsim_scene, "pull_lfs_files", pull)

    layer = dimsim_scene.dimsim_scene_usd("room")

    assert pulled == [[scene / "structure.glb", scene / "objects" / "door.glb"]]
    assert layer.count(str(meshes)) == 2


def test_the_scenes_lights_and_dimsims_own_become_usd_lights(
    pxr: Any, meshes: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    scene = tmp_path / "scenes" / "room"
    scene.mkdir(parents=True)
    (scene / "index.js").write_text(
        "const lamp = new THREE.PointLight(new THREE.Color('#ffffff'), 2, 8);\n"
        "lamp.position.set(1, 2.5, -1);\n"
        "for (const z of [1, 2]) { const bulb = new THREE.PointLight(0xffffff, 3); bulb.position.set(0, 2, z); }\n"
    )
    monkeypatch.setattr(dimsim_scene, "DIMSIM_SCENES", tmp_path / "scenes")
    stage = pxr.Usd.Stage.Open(pxr.Sdf.Layer.CreateAnonymous(".usda"))
    stage.GetRootLayer().ImportFromString(dimsim_scene.dimsim_scene_usd("room"))
    lights = {
        prim.GetTypeName(): prim for prim in stage.Traverse() if "Light" in prim.GetTypeName()
    }
    kinds = sorted(prim.GetTypeName() for prim in stage.Traverse() if "Light" in prim.GetTypeName())

    # the lamp; DimSim's directional light and its hemisphere's light from above; one dome for the rest
    # (the bulbs, placed in a loop, are left out)
    assert kinds == ["DistantLight", "DistantLight", "DomeLight", "SphereLight"]
    lamp = lights["SphereLight"]
    assert lamp.GetAttribute("inputs:intensity").Get() == pytest.approx(2)
    assert lamp.GetAttribute("xformOp:translate").Get() == pxr.Gf.Vec3d(1, 2.5, -1)
    settings = next(prim for prim in stage.Traverse() if prim.GetTypeName() == "RenderSettings")
    assert settings.GetAttribute("dimos:look:exposure").Get() == pytest.approx(
        dimsim_scene.LOOK["exposure"]
    )
