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

import numpy as np
import pytest

from dimos.simulation.mujoco.mesh_scene import SceneMeshAlignment, load_scene_prims

pytest.importorskip("pxr.Usd")
from pxr import Gf, Usd, UsdGeom  # type: ignore[import-not-found, import-untyped]


def test_load_scene_prims_expands_usd_point_instancers(tmp_path):
    scene_path = tmp_path / "instanced.usda"
    stage = Usd.Stage.CreateNew(str(scene_path))

    root = UsdGeom.Xform.Define(stage, "/Root")
    root.AddTranslateOp().Set(Gf.Vec3d(10, 0, 0))

    instancer = UsdGeom.PointInstancer.Define(stage, "/Root/Instancer")
    instancer.CreatePositionsAttr([Gf.Vec3f(1, 2, 3), Gf.Vec3f(4, 5, 6)])
    instancer.CreateOrientationsAttr(
        [
            Gf.Quath(1, Gf.Vec3h(0, 0, 0)),
            Gf.Quath(1, Gf.Vec3h(0, 0, 0)),
        ]
    )
    instancer.CreateScalesAttr([Gf.Vec3f(1, 1, 1), Gf.Vec3f(2, 1, 1)])
    instancer.CreateProtoIndicesAttr([0, 0])

    proto = UsdGeom.Xform.Define(stage, "/Root/Instancer/Prototypes/Proto")
    mesh = UsdGeom.Mesh.Define(stage, "/Root/Instancer/Prototypes/Proto/Tri")
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(0, 0, 0),
            Gf.Vec3f(1, 0, 0),
            Gf.Vec3f(0, 1, 0),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    instancer.CreatePrototypesRel().SetTargets([proto.GetPath()])

    stage.Save()

    prims = load_scene_prims(scene_path, SceneMeshAlignment(y_up=False))

    assert len(prims) == 2
    assert np.allclose(prims[0].vertices.min(axis=0), [11, 2, 3])
    assert np.allclose(prims[0].vertices.max(axis=0), [12, 3, 3])
    assert np.allclose(prims[1].vertices.min(axis=0), [14, 5, 6])
    assert np.allclose(prims[1].vertices.max(axis=0), [16, 6, 6])
