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

"""Admission rejects ambiguous assemblies and retains authored mass properties."""

from copy import deepcopy
import json
from pathlib import Path
import struct
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.articulation import rigid_hinge, subtree, verify_collision
from dimos.experimental.gta_sim2.world_geometry import SourceGeometry


def door(tmp_path: Path, *, fixed_vertex: bool = False) -> tuple[dict[str, Any], Path]:
    joints = np.zeros((3, 4), dtype=np.uint8)
    joints[:, 0] = 1
    if fixed_vertex:
        joints[0, 0] = 0
    weights = np.zeros((3, 4), dtype="<f4")
    weights[:, 0] = 1
    binary = joints.tobytes() + weights.tobytes()
    doc = {
        "nodes": [{"name": "fixed"}, {"name": "panel"}, {"mesh": 0, "skin": 0}],
        "skins": [{"joints": [0, 1]}],
        "meshes": [
            {"primitives": [{"attributes": {"JOINTS_0": 0, "WEIGHTS_0": 1, "POSITION": 2}}]}
        ],
        "bufferViews": [{"byteOffset": 0, "byteLength": 12}, {"byteOffset": 12, "byteLength": 48}],
        "accessors": [
            {"bufferView": 0, "componentType": 5121, "type": "VEC4", "count": 3},
            {"bufferView": 1, "componentType": 5126, "type": "VEC4", "count": 3},
            {"count": 3},
        ],
    }
    data = json.dumps(doc).encode()
    data += b" " * (-len(data) % 4)
    glb = tmp_path / "model.glb"
    glb.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 28 + len(data) + len(binary))
        + struct.pack("<I4s", len(data), b"JSON")
        + data
        + struct.pack("<I4s", len(binary), b"BIN\0")
        + binary
    )
    bones = [
        {
            "index": i,
            "name": name,
            "parent": None if i == 0 else 0,
            "bind_matrix": np.eye(4).ravel().tolist(),
        }
        for i, name in enumerate(("fixed", "panel"))
    ]
    children = [
        {
            "index": i,
            "in_use": True,
            "group": 1,
            "bone": 1,
            "mass": mass,
            "inertia": [0.2, 0.4, 0.5],
            "position": position,
            "rotation": Rotation.from_euler("z", angle).as_quat().tolist(),
            "include_flags": "0000000000002000",
            "bound": {"type": 3, "cg": [0, 0, 0], "bbox": [[-0.1] * 3, [0.1] * 3]},
        }
        for i, (mass, position, angle) in enumerate(((2, [0, 0, 0], np.pi / 4), (3, [1, 1, 0], 0)))
    ]
    return {
        "source": {"id": "synthetic-door"},
        "drawable": {"skeleton": bones},
        "physics": {
            "version": 3,
            "groups": [
                {"index": 0, "parent": None, "bone": 0},
                {"index": 1, "parent": 0, "bone": 1},
            ],
            "rotation_limits": [
                {"group": 1, "bone_name": "panel", "min": [0, 0, -1], "max": [0, 0, 1]}
            ],
            "children": children,
        },
    }, glb


def test_hinge_preserves_off_axis_mass_properties(tmp_path: Path) -> None:
    meta, glb = door(tmp_path)
    result = rigid_hinge(meta, glb)
    assert result["mass"] == 5
    np.testing.assert_allclose(result["center_of_mass"], [0.6, 0.6, 0])
    frame = Rotation.from_quat(result["inertia_quaternion_wxyz"], scalar_first=True).as_matrix()
    tensor = frame @ np.diag(result["inertia"]) @ frame.T
    np.testing.assert_allclose(tensor, [[1.7, -1.3, 0], [-1.3, 1.9, 0], [0, 0, 3.4]], atol=1e-12)


@pytest.mark.parametrize(
    "reason", ["fixed_vertex", "tunable", "second_axis", "unknown_bound", "cycle", "bone_name"]
)
def test_reject_ambiguous_door(tmp_path: Path, reason: str) -> None:
    meta, glb = door(tmp_path, fixed_vertex=reason == "fixed_vertex")
    if reason == "tunable":
        meta["archetype"] = {"tunable": {"mass": 30}}
    elif reason == "second_axis":
        meta["physics"]["rotation_limits"][0]["max"][0] = 1
    elif reason == "unknown_bound":
        meta["physics"]["children"][0]["bound"]["type"] = 13
    elif reason == "cycle":
        meta["physics"]["groups"][0]["parent"] = 1
    elif reason == "bone_name":
        meta["drawable"]["skeleton"][1]["name"] = "other"
    with pytest.raises(ValueError):
        rigid_hinge(meta, glb)


def test_collision_requires_exact_child_correspondence(tmp_path: Path) -> None:
    meta, _ = door(tmp_path)
    primitives = []
    for child in meta["physics"]["children"]:
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(child["rotation"]).as_matrix()
        matrix[:3, 3] = child["position"]
        primitives.append(
            {"type": "box", "bbox": child["bound"]["bbox"], "matrix": matrix.T.ravel().tolist()}
        )
    geometry = SourceGeometry(np.empty((0, 3, 3)), primitives, {"stats": {}}, 0)
    verify_collision(meta, geometry)
    damaged = deepcopy(geometry)
    damaged.primitives[0]["matrix"][12] += 0.01
    with pytest.raises(ValueError, match="transform mismatch"):
        verify_collision(meta, damaged)
    geometry.primitives.pop()
    with pytest.raises(ValueError, match="transform mismatch"):
        verify_collision(meta, geometry)


def test_incomplete_hierarchy() -> None:
    with pytest.raises(ValueError, match="incomplete"):
        subtree({0: None, 1: 7}, 0)
