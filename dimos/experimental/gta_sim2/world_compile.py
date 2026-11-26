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

"""Cook an exact, bounded set of source colliders into a MuJoCo window."""

import json
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.world_geometry import (
    WorldCatalog,
    primitive_bounds,
    primitive_vertices,
    transformed_triangles,
)
from dimos.experimental.gta_sim2.world_index import instance_number
from dimos.experimental.gta_sim2.world_objects import (
    OBJECT_PREFIX,
    ObjectStateStore,
    restore_object,
)

PRISM_FACES = [0, 1, 2, 5, 4, 3, 0, 3, 4, 0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 3, 2, 3, 0]


def add_mesh(
    spec: mujoco.MjSpec,
    name: str,
    vertices: NDArray[np.float64],
    faces: list[int] | None = None,
    body: Any = None,
) -> None:
    owner = spec.worldbody if body is None else body
    center = vertices.mean(axis=0)
    arguments: dict[str, Any] = {"name": name, "uservert": (vertices - center).ravel()}
    if faces is not None:
        arguments["userface"] = faces
    spec.add_mesh(**arguments)
    owner.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=name,
        pos=center,
        contype=1,
        conaffinity=1,
        group=3,
        friction=[0.8, 0.005, 0.0001],
    )


def add_triangle(
    spec: mujoco.MjSpec, name: str, triangle: NDArray[np.float64], body: Any = None
) -> bool:
    normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
    length = float(np.linalg.norm(normal))
    if length <= 1e-8:
        return False
    normal /= length
    if normal[2] < 0:
        triangle, normal = triangle[[0, 2, 1]], -normal
    add_mesh(spec, name, np.concatenate((triangle, triangle - normal * 0.04)), PRISM_FACES, body)
    return True


def add_primitive(
    spec: mujoco.MjSpec,
    name: str,
    primitive: dict[str, Any],
    transform: NDArray[np.float64],
    body: Any = None,
) -> bool:
    owner = spec.worldbody if body is None else body
    linear, translation = transform[:3, :3], transform[:3, 3]
    vertices = primitive_vertices(primitive)
    if vertices is not None:
        vertices = vertices @ linear.T + translation
        if np.linalg.matrix_rank(vertices - vertices.mean(axis=0), tol=1e-8) < 3:
            return False
        add_mesh(spec, name, vertices, body=body)
        return True
    scale = np.linalg.norm(linear, axis=0)
    if not np.allclose(linear.T @ linear, np.eye(3) * scale[0] ** 2, rtol=1e-5, atol=1e-8):
        raise ValueError(f"Analytic round collider has a nonuniform transform: {name}")
    radius = float(primitive["radius"] * scale[0])
    if radius <= 0 or not np.isfinite(radius):
        raise ValueError(f"Invalid analytic radius: {name}")
    kwargs: dict[str, Any] = {
        "name": name,
        "size": [radius, 0, 0],
        "group": 3,
        "contype": 1,
        "conaffinity": 1,
        "friction": [0.8, 0.005, 0.0001],
    }
    if primitive["type"] == "sphere":
        kwargs.update(
            type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=linear @ primitive["centre"] + translation
        )
    else:
        a, b = linear @ primitive["a"] + translation, linear @ primitive["b"] + translation
        if np.linalg.norm(b - a) < 1e-8:
            return False
        kwargs.update(
            type=mujoco.mjtGeom.mjGEOM_CAPSULE
            if primitive["type"] == "capsule"
            else mujoco.mjtGeom.mjGEOM_CYLINDER,
            fromto=np.concatenate((a, b)),
        )
    owner.add_geom(**kwargs)
    return True


def compile_world_window(
    base: mujoco.MjSpec,
    catalog: WorldCatalog,
    center: NDArray[np.float64],
    half_extent: float,
    state_path: Path | None = None,
    pinned: dict[str, dict[str, Any]] | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData, NDArray[np.int64], dict[str, Any]]:
    started = time.monotonic()
    spec = base.copy()
    ids = []
    pinned = pinned or {}
    regions = [(center, half_extent)]
    for pinned_state in pinned.values():
        # Quantized support patches make cache keys stable while awake bodies move.
        position = np.floor(np.asarray(pinned_state["position"]) / 4 + 0.5) * 4
        regions.append((position, max(6.0, float(pinned_state["radius"]) + 4)))
    records_by_id: dict[str, dict[str, Any]] = {}
    for point, extent in regions:
        records_by_id.update((row["identity"], row) for row in catalog.instances(point, extent))
        if len(records_by_id) > 10000:
            raise MemoryError(
                "Combined support regions exceed the 10,000-instance admission budget"
            )
    saved = {
        identity: state
        for point, extent in regions
        for identity, state in (
            ObjectStateStore.nearby(state_path, point, extent) if state_path else {}
        ).items()
    }
    for identity in saved.keys() | pinned.keys():
        records_by_id[identity] = catalog.instance(identity)
    records = list(records_by_id.values())
    object_records = [row for row in records if row.get("object")]
    if state_path:
        saved.update(ObjectStateStore.read(state_path, (row["identity"] for row in object_records)))
    saved.update(pinned)
    actors: dict[str, dict[str, Any]] = {}
    actor_shapes = 0
    for record in object_records:
        definition = json.loads(record["object"])
        state = saved.get(record["identity"])
        if state and record["identity"] not in pinned:
            position, radius = np.asarray(state["position"]), float(state["radius"])
            if not any(
                np.all(position + radius >= point - extent - 0.04)
                and np.all(position - radius <= point + extent + 0.04)
                for point, extent in regions
            ):
                continue
        if len(actors) >= 64:
            raise MemoryError("Active object admission budget exceeded")
        instance = catalog.geometry(record)
        transform = instance.transform
        scale = np.linalg.norm(transform[:3, :3], axis=0)
        if not np.allclose(scale, scale[0]) or np.linalg.det(transform[:3, :3]) <= 0:
            raise ValueError("Dynamic objects require a positive uniform source scale")
        rotation = transform[:3, :3] / scale[0]
        pivot = np.asarray(definition.get("pivot", [0, 0, 0]), dtype=np.float64)
        center_of_mass = np.asarray(definition.get("center_of_mass", [0, 0, 0]), dtype=np.float64)
        if (
            pivot.shape != (3,)
            or center_of_mass.shape != (3,)
            or not np.isfinite([pivot, center_of_mass]).all()
        ):
            raise ValueError("Invalid object pivot or center of mass")
        mass = float(definition["mass"]) * scale[0] ** 3
        inertia = np.asarray(definition["inertia"], dtype=np.float64) * scale[0] ** 5
        if (
            mass <= 0
            or not np.isfinite(mass)
            or inertia.shape != (3,)
            or np.any(inertia <= 0)
            or not np.isfinite(inertia).all()
        ):
            raise ValueError("Invalid object mass/inertia")
        name = f"{OBJECT_PREFIX}{instance.number:x}"
        body = spec.worldbody.add_body(
            name=name,
            pos=transform[:3, 3] + transform[:3, :3] @ pivot,
            quat=Rotation.from_matrix(rotation).as_quat(scalar_first=True),
        )
        body.explicitinertial = True
        body.mass, body.inertia = mass, inertia
        body.ipos = (center_of_mass - pivot) * scale[0]
        if definition["kind"] == "free":
            body.add_freejoint(name=name + "/joint")
        elif definition["kind"] == "hinge":
            limits = np.asarray(definition["range"], dtype=np.float64)
            if limits.shape != (2,) or limits[0] >= limits[1] or not np.isfinite(limits).all():
                raise ValueError("Invalid authored hinge limits")
            axis = np.asarray(definition["axis"], dtype=np.float64)
            home, damping = float(definition.get("home", 0)), float(definition.get("damping", 0.1))
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
                raise ValueError("Invalid hinge axis")
            if not np.isfinite([home, damping]).all() or damping < 0:
                raise ValueError("Invalid hinge home/damping")
            angle_scale = 180 / np.pi if spec.compiler.degree else 1
            body.add_joint(
                name=name + "/joint",
                type=mujoco.mjtJoint.mjJNT_HINGE,
                axis=axis,
                range=limits * angle_scale,
                limited=True,
                damping=damping,
                ref=home * angle_scale,
            )
        else:
            raise ValueError("Only explicit free/hinged environment bodies are supported")
        local = np.eye(4)
        local[:3, :3] *= scale[0]
        local[:3, 3] = -pivot * scale[0]
        shape_count = 0
        for index, triangle in enumerate(instance.source.triangles):
            shape_count += add_triangle(
                spec, f"{name}/t{index}", (triangle - pivot) * scale[0], body
            )
        for index, primitive in enumerate(instance.source.primitives):
            shape_count += add_primitive(spec, f"{name}/p{index}", primitive, local, body)
        if shape_count == 0 or instance.source.metadata.get("stats", {}).get("other_bound_types"):
            raise ValueError(f"Dynamic object's collider is incomplete: {instance.identity}")
        actor_shapes += shape_count
        if actor_shapes > 50000:
            raise MemoryError("Active objects exceed the 50,000-shape admission budget")
        box = np.asarray(json.loads(record["bbox"]), dtype=np.float64)
        body_origin = transform[:3, 3] + transform[:3, :3] @ pivot + catalog.origin
        radius = float(np.linalg.norm(np.max(np.abs(box - body_origin), axis=0))) + 0.1
        actors[instance.identity] = {
            "body": name,
            "radius": radius,
            "state": state,
            "definition": definition,
            "visual": {
                "source_model": record["source"],
                "source_map": json.loads(record["metadata"]).get("map", ""),
                "source_position": (transform[:3, 3] + catalog.origin).tolist(),
                "source_position_local": transform[:3, 3].tolist(),
                "scale": float(scale[0]),
                "pivot": pivot.tolist(),
            },
        }
    triangles_count, primitives_count = 0, 0
    missing: list[str] = []
    for record in records:
        if record.get("object"):
            continue
        instance = catalog.geometry(record)
        if instance.source.metadata.get("unresolved"):
            missing.append(instance.identity)
        triangles = transformed_triangles(instance)
        selected_mask = np.zeros(len(triangles), dtype=bool)
        if len(triangles):
            lower, upper = triangles.min(axis=1), triangles.max(axis=1)
            for point, extent in regions:
                selected_mask |= np.all(upper >= point - extent - 0.04, axis=1) & np.all(
                    lower <= point + extent + 0.04, axis=1
                )
        selected = np.flatnonzero(selected_mask)
        for index in selected:
            name = f"gta_stream/{instance.number:x}/t{index}"
            if add_triangle(spec, name, triangles[index]):
                ids.append(instance_number(name))
                triangles_count += 1
        for index, primitive in enumerate(instance.source.primitives):
            box = primitive_bounds(primitive, instance.transform)
            if not any(
                np.all(box[1] >= point - extent - 0.04) and np.all(box[0] <= point + extent + 0.04)
                for point, extent in regions
            ):
                continue
            name = f"gta_stream/{instance.number:x}/p{index}"
            if add_primitive(spec, name, primitive, instance.transform):
                ids.append(instance_number(name))
                primitives_count += 1
        if len(ids) + actor_shapes > 50000:
            raise MemoryError("Collision window exceeds the 50,000-shape admission budget")
    if len(set(ids)) != len(ids):
        raise ValueError("Collider identity collision")
    if not ids:
        raise ValueError("Requested window contains no decoded contact geometry")
    model = spec.compile()
    data = mujoco.MjData(model)
    for actor in actors.values():
        if actor["state"]:
            restore_object(model, data, actor["state"])
    return (
        model,
        data,
        np.asarray(ids, dtype=np.int64),
        {
            "source_instances": len(records),
            "triangle_prisms": triangles_count,
            "analytic_shapes": primitives_count,
            "unresolved_instances": missing,
            "cache": dict(catalog.stats),
            "build_s": time.monotonic() - started,
            "actors": actors,
        },
    )
