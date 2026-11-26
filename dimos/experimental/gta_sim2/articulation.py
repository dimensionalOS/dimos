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

"""Conservative admission of intact, rigid single-hinge fragment assemblies."""

import json
from pathlib import Path
import struct
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.world_geometry import SourceGeometry
from dimos.experimental.gta_sim2.world_index import file_digest


def subtree(parents: dict[int, int | None], root: int) -> set[int]:
    if root not in parents:
        raise ValueError("Missing hierarchy root")
    for index in parents:
        seen = set()
        current: int | None = index
        while current is not None:
            if current in seen or current not in parents:
                raise ValueError("Cyclic or incomplete fragment hierarchy")
            seen.add(current)
            current = parents[current]
    result = {root}
    for _ in range(len(parents)):
        more = {index for index, parent in parents.items() if parent in result}
        if more <= result:
            return result
        result |= more
    raise ValueError("Cyclic fragment hierarchy")


def skin_bones(path: Path, names: dict[int, str]) -> set[int]:
    """Require every rendered vertex to belong to one of the returned bones."""
    raw = path.read_bytes()
    if (
        len(raw) < 28
        or raw[:4] != b"glTF"
        or struct.unpack_from("<II", raw, 4) != (2, len(raw))
        or raw[16:20] != b"JSON"
    ):
        raise ValueError("Invalid fragment GLB")
    size = struct.unpack_from("<I", raw, 12)[0]
    if 28 + size > len(raw) or raw[24 + size : 28 + size] != b"BIN\0":
        raise ValueError("Invalid fragment GLB chunks")
    if struct.unpack_from("<I", raw, 20 + size)[0] != len(raw) - 28 - size:
        raise ValueError("Invalid fragment GLB binary length")
    doc = json.loads(raw[20 : 20 + size])
    binary = raw[28 + size :]

    def accessor(index: int) -> NDArray[Any]:
        item = doc["accessors"][index]
        view = doc["bufferViews"][item["bufferView"]]
        if item["type"] != "VEC4" or item.get("sparse") or view.get("buffer", 0) != 0:
            raise ValueError("Unsupported skin accessor")
        dtype = np.dtype({5121: "u1", 5123: "<u2", 5126: "<f4"}[item["componentType"]])
        stride = view.get("byteStride", 4 * dtype.itemsize)
        offset = item.get("byteOffset", 0)
        count = item["count"]
        if (
            count <= 0
            or stride < 4 * dtype.itemsize
            or offset + (count - 1) * stride + 4 * dtype.itemsize > view["byteLength"]
            or view.get("byteOffset", 0) + view["byteLength"] > len(binary)
        ):
            raise ValueError("Invalid skin accessor bounds")
        return np.ndarray(
            (count, 4), dtype, binary, view.get("byteOffset", 0) + offset, (stride, dtype.itemsize)
        )

    used: set[int] = set()
    for node in doc["nodes"]:
        if "mesh" not in node:
            continue
        if "skin" not in node:
            raise ValueError("Unskinned geometry cannot be moved as a whole hinge")
        joints = doc["skins"][node["skin"]]["joints"]
        for primitive in doc["meshes"][node["mesh"]]["primitives"]:
            attrs = primitive["attributes"]
            if "JOINTS_1" in attrs or doc["accessors"][attrs["JOINTS_0"]]["componentType"] not in (
                5121,
                5123,
            ):
                raise ValueError("Unsupported additional skin influences")
            indices, weights = accessor(attrs["JOINTS_0"]), accessor(attrs["WEIGHTS_0"])
            if (
                indices.shape != weights.shape
                or len(indices) != doc["accessors"][attrs["POSITION"]]["count"]
                or not np.isfinite(weights).all()
                or np.any(weights < 0)
                or np.any(weights.sum(axis=1) <= 0)
            ):
                raise ValueError("Invalid skin weights")
            for index in np.unique(indices[weights > 0]):
                if index >= len(joints):
                    raise ValueError("Invalid skin bone")
                joint = int(joints[int(index)])
                if joint not in names or doc["nodes"][joint].get("name") != names[joint]:
                    raise ValueError("GLB/source bone binding mismatch")
                used.add(joint)
    if not used:
        raise ValueError("No skinned geometry")
    return used


def rigid_hinge(meta: dict[str, Any], glb: Path) -> dict[str, Any]:
    """Admit a whole moving panel, with all descendants welded in their rest pose.

    This deliberately excludes multi-axis/sliding joints, fixed visible frames,
    tunable overrides and unsupported colliders. It does not simulate fracture
    or independent motion of descendants (glass, handles, decorative parts).
    The source latch/drive/damping semantics are not inferred.
    """
    physics = meta.get("physics") or {}
    if physics.get("version") != 3 or meta.get("archetype", {}).get("tunable"):
        raise ValueError("Unknown fragment physics or tunable overrides")
    limits = physics.get("rotation_limits", [])
    if len(limits) != 1 or physics.get("translation_limits"):
        raise ValueError("Not a single rotational joint")
    limit = limits[0]
    groups = {int(g["index"]): g for g in physics["groups"]}
    group = limit.get("group")
    if group not in groups:
        raise ValueError("Joint has no fragment group")
    moving = subtree({i: g["parent"] for i, g in groups.items()}, group)
    bones = {int(b["index"]): b for b in meta["drawable"]["skeleton"]}
    bone = bones[groups[group]["bone"]]
    if bone["name"] != limit["bone_name"]:
        raise ValueError("Joint/bone identity mismatch")
    descendants = subtree({i: b["parent"] for i, b in bones.items()}, int(bone["index"]))
    if not skin_bones(glb, {i: b["name"] for i, b in bones.items()}) <= descendants:
        raise ValueError("Visible fixed parts or a second moving assembly")
    minimum, maximum = np.array(limit["min"]), np.array(limit["max"])
    axes = np.flatnonzero(maximum - minimum > 1e-5)
    if (
        minimum.shape != (3,)
        or maximum.shape != (3,)
        or not np.isfinite([minimum, maximum]).all()
        or len(axes) != 1
        or np.any(minimum > 0)
        or np.any(maximum < 0)
    ):
        raise ValueError("Unsupported hinge range")
    bind = np.array(bone["bind_matrix"]).reshape(4, 4).T
    if (
        not np.allclose(bind[:3, :3].T @ bind[:3, :3], np.eye(3), atol=1e-5)
        or np.linalg.det(bind[:3, :3]) < 0
    ):
        raise ValueError("Non-rigid hinge bone")
    children = [c for c in physics["children"] if c["in_use"]]
    for child in children:
        collides = int(child["include_flags"], 16) & ((1 << 13) | (1 << 25))
        if collides and (child["group"] not in moving or child["bound"]["type"] != 3):
            raise ValueError("Fixed or undecoded collision within the source assembly")
    pieces = [c for c in children if c["group"] in moving]
    if not pieces:
        raise ValueError("Empty moving assembly")
    if any(c["bone"] not in descendants for c in pieces):
        raise ValueError("Fragment group/bone hierarchy mismatch")
    masses, centers, inertias = [], [], []
    for child in pieces:
        mass = float(child["mass"])
        inertia = np.array(child["inertia"], dtype=float)
        frame = np.array(bones[child["bone"]]["bind_matrix"]).reshape(4, 4).T[:3, :3]
        rotation = frame @ Rotation.from_quat(child["rotation"]).as_matrix()
        if child["position"] is None or child["bound"] is None:
            raise ValueError("Missing collider pose")
        center = np.array(child["position"]) + rotation @ child["bound"]["cg"]
        if mass <= 0 or np.any(inertia <= 0) or not np.isfinite(np.r_[mass, inertia, center]).all():
            raise ValueError("Invalid authored mass properties")
        masses.append(mass)
        centers.append(center)
        inertias.append(rotation @ np.diag(inertia) @ rotation.T)
    mass = sum(masses)
    center = np.average(centers, axis=0, weights=masses)
    tensor = np.zeros((3, 3))
    for m, c, inertia in zip(masses, centers, inertias, strict=True):
        delta = c - center
        tensor += inertia + m * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
    values, frame = np.linalg.eigh(tensor)
    if np.any(values <= 0) or max(values) > sum(values) - max(values) + 1e-5:
        raise ValueError("Nonphysical aggregate inertia")
    if np.linalg.det(frame) < 0:
        frame[:, 0] *= -1
    axis = int(axes[0])
    return {
        "kind": "hinge",
        "pivot": bind[:3, 3].tolist(),
        "axis": bind[:3, axis].tolist(),
        "range": [float(minimum[axis]), float(maximum[axis])],
        "home": 0,
        "damping": 0,
        "mass": mass,
        "center_of_mass": center.tolist(),
        "inertia": values.tolist(),
        "inertia_quaternion_wxyz": Rotation.from_matrix(frame).as_quat(scalar_first=True).tolist(),
        "provenance": {
            "method": "fragment-single-hinge-v1",
            "model": meta["source"]["id"],
            "glb_sha256": file_digest(glb),
            "bone": bone["name"],
            "groups": sorted(moving),
            "children": [c["index"] for c in pieces],
            "limitations": [
                "intact descendants welded",
                "latch, drive and joint damping not recovered",
            ],
        },
    }


def verify_collision(meta: dict[str, Any], geometry: SourceGeometry) -> None:
    """Require a one-to-one match of every admitted child to the collision export."""
    stats = geometry.metadata.get("stats", {})
    if (
        geometry.metadata.get("unresolved")
        or stats.get("other_bound_types")
        or stats.get("other_polygons")
        or len(geometry.triangles)
        or not geometry.primitives
    ):
        raise ValueError("Incomplete or unexpected door collision geometry")
    bones = {int(b["index"]): b for b in meta["drawable"]["skeleton"]}
    remaining = list(geometry.primitives)
    for child in meta["physics"]["children"]:
        if not child["in_use"] or not int(child["include_flags"], 16) & ((1 << 13) | (1 << 25)):
            continue
        expected = np.eye(4)
        bind = np.array(bones[child["bone"]]["bind_matrix"]).reshape(4, 4).T
        expected[:3, :3] = bind[:3, :3] @ Rotation.from_quat(child["rotation"]).as_matrix()
        expected[:3, 3] = child["position"]
        matches = [
            i
            for i, p in enumerate(remaining)
            if p["type"] == "box"
            and np.allclose(p["bbox"], child["bound"]["bbox"], rtol=0, atol=1e-4)
            and np.allclose(np.array(p["matrix"]).reshape(4, 4).T, expected, rtol=0, atol=1e-4)
        ]
        if len(matches) != 1:
            raise ValueError("Authored child/collider transform mismatch")
        remaining.pop(matches[0])
    if remaining:
        raise ValueError("Unaccounted collision within door assembly")
