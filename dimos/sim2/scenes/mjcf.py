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

"""Offline authoring of scene components from a MuJoCo (MJCF) environment.

An MJCF scene becomes the URDF components that ``prepare_scene`` composes with a
robot: one fixed component for everything welded to the world, and one component
per movable tree (a free object, or furniture with hinge and slide joints).
Geometry, placement, mass and inertia are read from the compiled model, so they
are what MuJoCo itself simulates. This is preparation tooling, like ``urdf.py``.

Not carried over: textures (a textured material becomes its mean color), lights,
actuators, sensors, tendons, equality constraints, height fields, and joints
other than one free, hinge or slide joint per body (such a body is welded where
it stands and reported).
"""

import argparse
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path
import re
from typing import Any
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
import trimesh

from dimos.robot.assets.model import RobotModel
from dimos.sim2.scenes.urdf import Component, prepare_scene

# A plane has no thickness and no URDF form: it becomes a slab this thick, in
# metres, under its surface, and this wide where MuJoCo's plane is infinite.
_PLANE_THICKNESS_M = 0.02
_INFINITE_PLANE_M = 100.0
# MuJoCo hides geom groups from this one up; scenes keep collision-only shapes there.
_FIRST_HIDDEN_GROUP = 3
# Looking for a free place: the spacing of the spots tried, the height a floor is
# looked for from, and the gap kept above it; in metres.
_PLACE_STEP_M = 0.2
_PLACE_LOOK_FROM_M = 1.0
_PLACE_CLEARANCE_M = 0.05
# A floor is a surface at least this wide both ways and at most this thick, in metres.
_FLOOR_MIN_EXTENT_M = 2.0
_FLOOR_MAX_THICKNESS_M = 0.3
# Model arrays hold plain integers; MuJoCo's enum values do not compare equal to
# them inside a tuple, so every comparison here is between integers.
_BOX, _SPHERE, _CYLINDER, _PLANE, _MESH, _CAPSULE = (
    int(kind)
    for kind in (
        mujoco.mjtGeom.mjGEOM_BOX,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        mujoco.mjtGeom.mjGEOM_CYLINDER,
        mujoco.mjtGeom.mjGEOM_PLANE,
        mujoco.mjtGeom.mjGEOM_MESH,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
    )
)
_ELLIPSOID = int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)  # type: ignore[attr-defined]  # MuJoCo 4.0
_FREE, _HINGE, _SLIDE = (
    int(kind)
    for kind in (
        mujoco.mjtJoint.mjJNT_FREE,
        mujoco.mjtJoint.mjJNT_HINGE,
        mujoco.mjtJoint.mjJNT_SLIDE,
    )
)


# What a filtered pair names: a movable rigid body (its id, None), or the fixed
# geoms with one contact mask (0, (contype, conaffinity)).
_Unit = tuple[int, tuple[int, int] | None]


@dataclass(frozen=True)
class MjcfScene:
    """The components of one MJCF environment and what could not be carried over."""

    components: dict[str, Component]
    # Whether the environment has its own floor, so the caller adds no ground plane.
    has_floor: bool
    # Bodies whose joints have no URDF form; they are welded at their start pose.
    welded: tuple[str, ...] = ()
    # Geoms with no URDF form (height fields, signed distance fields), by name.
    skipped: tuple[str, ...] = field(default=())


def components_from_mjcf(scene_xml: Path, output: Path, *, name: str = "env") -> MjcfScene:
    """Write ``output/*.urdf`` and ``output/meshes`` for a scene and return its components.

    Args:
        scene_xml: The MJCF file; its assets resolve as MuJoCo resolves them.
        output: An existing or new directory for the URDF and mesh files.
        name: Starts every link name, so several environments and a robot can share a scene.
    """
    output = output.resolve()
    (output / "meshes").mkdir(parents=True, exist_ok=True)
    model = mujoco.MjSpec.from_file(str(scene_xml)).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    writer = _Writer(model, data, output, name)
    return writer.write()


class _Writer:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, output: Path, name: str) -> None:
        self.model, self.data, self.output, self.name = model, data, output, name
        self._mesh_files: dict[int | tuple[str, tuple[float, ...]], Path] = {}
        self._texture_colors: dict[int, NDArray[np.float64]] = {}
        self._skipped: list[str] = []
        self._welded: list[str] = []
        # The rigid body each body moves with; 0 is the world.
        self._rigid = model.body_weldid.copy()
        for body in range(1, model.nbody):
            if self._rigid[body] == body and not self._movable(body):
                self._welded.append(self._body_name(body))
                self._rigid[body] = self._rigid[model.body_parentid[body]]
        # A welded body's children were welded to it by MuJoCo; follow the chain.
        for body in range(1, model.nbody):
            while self._rigid[self._rigid[body]] != self._rigid[body]:
                self._rigid[body] = self._rigid[self._rigid[body]]

    def _movable(self, body: int) -> bool:
        """Whether a jointed body has exactly one joint that URDF can state."""
        if self.model.body_jntnum[body] != 1:
            return False
        kind = int(self.model.jnt_type[self.model.body_jntadr[body]])
        if kind == _FREE:
            return bool(self._rigid[self.model.body_parentid[body]] == 0)
        return kind in (_HINGE, _SLIDE)

    def write(self) -> MjcfScene:
        model = self.model
        components: dict[str, Component] = {}
        # A unit is what a filtered pair names: a movable rigid body, or the fixed
        # geoms that share one contact mask. Each is one link of one component.
        home: dict[_Unit, str] = {}
        link_of: dict[_Unit, str] = {}
        has_floor = False
        fixed_masks = sorted(mask for body, mask in self._masks() if body == 0 and mask is not None)
        for index, mask in enumerate(fixed_masks or [None]):
            stem = "static" if index == 0 or mask is None else f"static_{mask[0]}_{mask[1]}"
            static = ET.Element("robot", name=f"{self.name}_{stem}")
            link = ET.SubElement(static, "link", name=f"{self.name}/{stem}")
            # Nothing moves it, but without a stated mass the importer weighs its
            # visual meshes, and a flat one has no volume to weigh.
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass", value="1")
            ET.SubElement(inertial, "inertia", ixx="1", ixy="0", ixz="0", iyy="1", iyz="0", izz="1")
            # What is only drawn goes with the first.
            has_floor |= self._write_geoms(link, 0, np.eye(4), mask=mask, drawn_only=index == 0)
            if len(link) > 1:
                components[f"{self.name}_{stem}"] = Component(self._save(static, stem))
                home[0, mask] = f"{self.name}_{stem}"
                link_of[0, mask] = f"{self.name}/{stem}"
        parents = {
            int(body): int(self._rigid[model.body_parentid[body]])
            for body in range(1, model.nbody)
            if self._rigid[body] == body
        }
        for root in (body for body, parent in parents.items() if parent == 0):
            key = f"{self.name}_{_safe(self._body_name(root))}"
            components[key] = self._tree(root, parents, key)
            pending = [root]
            while pending:
                body = pending.pop()
                home[body, None] = key
                link_of[body, None] = self._link_name(body)
                pending += [child for child, parent in parents.items() if parent == body]
        # A pair is stated on the component that comes later, when both of its links exist.
        order = {key: index for index, key in enumerate(components)}
        apart: dict[str, list[tuple[str, str]]] = {}
        for first, second in sorted(self._never_touching(), key=str):
            if first in home and second in home:
                later = max(home[first], home[second], key=order.__getitem__)
                apart.setdefault(later, []).append((link_of[first], link_of[second]))
        for key, pairs in apart.items():
            components[key] = replace(components[key], collision_exclusions=tuple(pairs))
        return MjcfScene(components, has_floor, tuple(self._welded), tuple(self._skipped))

    def _masks(self) -> dict["_Unit", set[tuple[int, int]]]:
        """The contact masks of the colliding geoms of every unit."""
        model = self.model
        masks: dict[_Unit, set[tuple[int, int]]] = {}
        for geom in range(model.ngeom):
            mask = (int(model.geom_contype[geom]), int(model.geom_conaffinity[geom]))
            if mask == (0, 0):
                continue
            body = int(self._rigid[model.geom_bodyid[geom]])
            masks.setdefault((body, mask if body == 0 else None), set()).add(mask)
        return masks

    def _never_touching(self) -> set[tuple["_Unit", "_Unit"]]:
        """Pairs of units the source never lets collide.

        MuJoCo decides by two bit masks on each geom and by a list of excluded
        body pairs. A USD collider has no masks: every collider meets every other.
        So a pair of units with no geom pair that passes the masks is stated as
        a filtered pair, as is every excluded pair. Without this a cabinet door
        collides with its own frame. The fixed geoms are split by mask, so this is
        exact for them; a movable body whose geoms differ in mask is one unit, and
        keeps every contact any of its geoms would have.
        """
        model = self.model
        masks = self._masks()
        units = sorted(masks, key=str)
        apart: set[tuple[_Unit, _Unit]] = set()
        for index, first in enumerate(units):
            for second in units[index + 1 :]:
                if first[0] == 0 and second[0] == 0:
                    continue  # Two fixed parts cannot collide anyway.
                if not any(
                    (a_type & b_affinity) or (b_type & a_affinity)
                    for a_type, a_affinity in masks[first]
                    for b_type, b_affinity in masks[second]
                ):
                    apart.add((first, second))
        for signature in model.exclude_signature:
            first_body = int(self._rigid[int(signature) >> 16])
            second_body = int(self._rigid[int(signature) & 0xFFFF])
            if first_body == second_body:
                continue
            for first in (unit for unit in units if unit[0] == first_body):
                for second in (unit for unit in units if unit[0] == second_body):
                    apart.add((first, second))
        return apart

    def _tree(self, root: int, parents: dict[int, int], key: str) -> Component:
        """One movable tree as a URDF, placed where its root stands at the start."""
        model = self.model
        robot = ET.Element("robot", name=key)
        frames = {body: self._link_frame(body) for body in parents}
        floating = int(model.jnt_type[model.body_jntadr[root]]) == _FREE
        origin = frames[root]
        if not floating:
            ET.SubElement(robot, "link", name=f"{self.name}/{_safe(self._body_name(root))}/base")
        # A joint's own damping, as a drive with no stiffness: the scene format
        # states joint damping no other way, and a hinge without it never rests.
        drives: dict[str, tuple[float, float, float]] = {}
        friction = 0.0
        pending = [root]
        while pending:
            body = pending.pop()
            link = ET.SubElement(robot, "link", name=self._link_name(body))
            self._write_inertial(link, body, frames[body])
            self._write_geoms(link, body, frames[body])
            if body != root or not floating:
                parent = parents[body]
                parent_frame = origin if body == root else frames[parent]
                parent_name = (
                    f"{self.name}/{_safe(self._body_name(root))}/base"
                    if body == root
                    else self._link_name(parent)
                )
                self._write_joint(robot, body, parent_name, parent_frame, frames[body])
                joint = model.body_jntadr[body]
                dof = model.jnt_dofadr[joint]
                if model.dof_damping[dof] > 0:
                    drives[self._joint_name(joint)] = (0.0, float(model.dof_damping[dof]), 0.0)
                    friction = max(friction, float(model.dof_frictionloss[dof]))
            pending += [child for child, parent in parents.items() if parent == body]
        xyz = tuple(float(v) for v in origin[:3, 3])
        rpy = tuple(float(v) for v in Rotation.from_matrix(origin[:3, :3]).as_euler("xyz"))
        return Component(
            self._save(robot, _safe(self._body_name(root))),
            xyz=(xyz[0], xyz[1], xyz[2]),
            rpy=(rpy[0], rpy[1], rpy[2]),
            floating=bool(floating),
            drives=drives,
            friction=friction,
        )

    def _link_frame(self, body: int) -> NDArray[np.float64]:
        """The body's frame, moved onto its joint anchor: a URDF joint turns about its origin."""
        frame = np.eye(4)
        frame[:3, :3] = self.data.xmat[body].reshape(3, 3)
        frame[:3, 3] = self.data.xpos[body]
        joint = self.model.body_jntadr[body]
        if int(self.model.jnt_type[joint]) != _FREE:
            frame[:3, 3] += frame[:3, :3] @ self.model.jnt_pos[joint]
        return frame

    def _write_joint(
        self,
        robot: ET.Element,
        body: int,
        parent: str,
        parent_frame: NDArray[np.float64],
        frame: NDArray[np.float64],
    ) -> None:
        model = self.model
        joint = model.body_jntadr[body]
        hinge = int(model.jnt_type[joint]) == _HINGE
        limited = bool(model.jnt_limited[joint])
        kind = ("revolute" if limited else "continuous") if hinge else "prismatic"
        element = ET.SubElement(robot, "joint", name=self._joint_name(joint), type=kind)
        ET.SubElement(element, "parent", link=parent)
        ET.SubElement(element, "child", link=self._link_name(body))
        _origin(element, np.linalg.inv(parent_frame) @ frame)
        ET.SubElement(element, "axis", xyz=_text(model.jnt_axis[joint]))
        if limited or not hinge:
            # The range is about the reference pose, which is this URDF's zero.
            low, high = model.jnt_range[joint] - model.qpos0[model.jnt_qposadr[joint]]
            if not limited:
                low, high = -_INFINITE_PLANE_M, _INFINITE_PLANE_M
            ET.SubElement(
                element,
                "limit",
                lower=f"{low:.9g}",
                upper=f"{high:.9g}",
                effort="1000",
                velocity="100",
            )

    def _write_inertial(self, link: ET.Element, body: int, frame: NDArray[np.float64]) -> None:
        """The mass, center and inertia of every body that moves with this one."""
        model, data = self.model, self.data
        members = [b for b in range(1, model.nbody) if self._rigid[b] == body]
        mass = float(sum(model.body_mass[b] for b in members))
        if mass <= 0:
            return
        center = sum(model.body_mass[b] * data.xipos[b] for b in members) / mass
        tensor = np.zeros((3, 3))
        for b in members:
            rotation = data.ximat[b].reshape(3, 3)
            offset = data.xipos[b] - center
            tensor += rotation @ np.diag(model.body_inertia[b]) @ rotation.T
            tensor += model.body_mass[b] * (offset @ offset * np.eye(3) - np.outer(offset, offset))
        rotation = frame[:3, :3]
        local = rotation.T @ tensor @ rotation
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(
            inertial, "origin", xyz=_text(rotation.T @ (center - frame[:3, 3])), rpy="0 0 0"
        )
        ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
        ET.SubElement(
            inertial,
            "inertia",
            ixx=f"{local[0, 0]:.9g}",
            ixy=f"{local[0, 1]:.9g}",
            ixz=f"{local[0, 2]:.9g}",
            iyy=f"{local[1, 1]:.9g}",
            iyz=f"{local[1, 2]:.9g}",
            izz=f"{local[2, 2]:.9g}",
        )

    def _write_geoms(
        self,
        link: ET.Element,
        body: int,
        frame: NDArray[np.float64],
        *,
        mask: tuple[int, int] | None = None,
        drawn_only: bool = True,
    ) -> bool:
        """The geoms that move with ``body``, as visuals and collisions. True if one is a floor.

        ``mask`` keeps the colliding geoms with that contact mask only, and
        ``drawn_only`` says whether the geoms that collide with nothing are written.
        """
        model, data = self.model, self.data
        inverse = np.linalg.inv(frame)
        has_floor = False
        for geom in range(model.ngeom):
            if self._rigid[model.geom_bodyid[geom]] != body:
                continue
            own = (int(model.geom_contype[geom]), int(model.geom_conaffinity[geom]))
            if (own == (0, 0) and not drawn_only) or (
                mask is not None and own not in ((0, 0), mask)
            ):
                continue
            shape = self._shape(geom)
            if shape is None:
                continue
            element, offset = shape
            pose = np.eye(4)
            pose[:3, :3] = data.geom_xmat[geom].reshape(3, 3)
            pose[:3, 3] = data.geom_xpos[geom]
            pose = inverse @ pose @ offset
            rgba = self._rgba(geom)
            if model.geom_group[geom] < _FIRST_HIDDEN_GROUP and rgba[3] > 0:
                visual = ET.SubElement(link, "visual")
                _origin(visual, pose)
                ET.SubElement(visual, "geometry").append(element)
                material = ET.SubElement(visual, "material", name=f"color_{geom}")
                ET.SubElement(material, "color", rgba=_text(rgba))
            if model.geom_contype[geom] or model.geom_conaffinity[geom]:
                collision = ET.SubElement(link, "collision")
                _origin(collision, pose)
                ET.SubElement(collision, "geometry").append(ET.fromstring(ET.tostring(element)))
                has_floor |= self._is_floor(geom)
        return has_floor

    def _is_floor(self, geom: int) -> bool:
        return _is_floor(self.model, self.data, geom)

    def _shape(self, geom: int) -> tuple[ET.Element, NDArray[np.float64]] | None:
        """The URDF geometry of a geom, and where that geometry sits in the geom's frame."""
        model = self.model
        kind, size = int(model.geom_type[geom]), model.geom_size[geom]
        offset = np.eye(4)
        if kind == _BOX:
            return ET.Element("box", size=_text(2 * size)), offset
        if kind == _SPHERE:
            return ET.Element("sphere", radius=f"{size[0]:.9g}"), offset
        if kind == _CYLINDER:
            return ET.Element(
                "cylinder", radius=f"{size[0]:.9g}", length=f"{2 * size[1]:.9g}"
            ), offset
        if kind == _PLANE:
            extent = [2 * v if v > 0 else _INFINITE_PLANE_M for v in size[:2]]
            offset[2, 3] = -_PLANE_THICKNESS_M / 2
            return ET.Element("box", size=_text([*extent, _PLANE_THICKNESS_M])), offset
        if kind == _MESH:
            mesh = int(model.geom_dataid[geom])
            return ET.Element("mesh", filename=str(self._mesh_file(mesh))), offset
        if kind in (_CAPSULE, _ELLIPSOID):
            capsule = kind == _CAPSULE
            made = self._made_file(
                "capsule" if capsule else "ellipsoid",
                tuple(round(float(v), 6) for v in (size[:2] if capsule else size)),
            )
            return ET.Element("mesh", filename=str(made)), offset
        self._skipped.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or str(geom))
        return None

    def _mesh_file(self, mesh: int) -> Path:
        if mesh not in self._mesh_files:
            model = self.model
            vertices = model.mesh_vert[
                model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
            ]
            faces = model.mesh_face[
                model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]
            ]
            path = self.output / "meshes" / f"mesh_{mesh}.obj"
            trimesh.Trimesh(vertices, faces, process=False).export(path)
            self._mesh_files[mesh] = path
        return self._mesh_files[mesh]

    def _made_file(self, kind: str, size: tuple[float, ...]) -> Path:
        """A capsule (radius, half length) or ellipsoid (radii) as a mesh: URDF has neither."""
        key = (kind, size)
        if key not in self._mesh_files:
            if kind == "capsule":
                shape = trimesh.creation.capsule(radius=size[0], height=2 * size[1])
            else:
                shape = trimesh.creation.icosphere(subdivisions=3)
                shape.apply_scale(size)
            path = self.output / "meshes" / f"{kind}_{'_'.join(str(v) for v in size)}.obj"
            shape.export(path)
            self._mesh_files[key] = path
        return self._mesh_files[key]

    def _rgba(self, geom: int) -> NDArray[np.float64]:
        model = self.model
        material = int(model.geom_matid[geom])
        if material < 0:
            return np.asarray(model.geom_rgba[geom], dtype=float)
        rgba = np.asarray(model.mat_rgba[material], dtype=float).copy()
        texture = int(model.mat_texid[material][mujoco.mjtTextureRole.mjTEXROLE_RGB])
        if texture >= 0:
            rgba[:3] *= self._texture_color(texture)
        return rgba

    def _texture_color(self, texture: int) -> NDArray[np.float64]:
        """The mean color of a texture, 0 to 1: what a flat-shaded surface keeps of it."""
        if texture not in self._texture_colors:
            model = self.model
            channels = int(model.tex_nchannel[texture])
            count = int(model.tex_height[texture]) * int(model.tex_width[texture]) * channels
            start = int(model.tex_adr[texture])
            pixels = model.tex_data[start : start + count].reshape(-1, channels)
            mean = pixels[:, :3].mean(axis=0) / 255 if channels >= 3 else np.ones(3)
            self._texture_colors[texture] = mean
        return self._texture_colors[texture]

    def _body_name(self, body: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or f"body_{body}"

    def _link_name(self, body: int) -> str:
        return f"{self.name}/{_safe(self._body_name(body))}"

    def _joint_name(self, joint: int) -> str:
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint) or f"joint_{joint}"
        return f"{self.name}/{_safe(name)}"

    def _save(self, robot: ET.Element, stem: str) -> RobotModel:
        path = self.output / f"{stem}.urdf"
        ET.ElementTree(robot).write(path, encoding="unicode")
        return RobotModel.from_file(path)


def environment_from_mjcf(scene_xml: Path, output: Path, *, name: str = "env") -> MjcfScene:
    """Write ``output/scene.usda``: the MJCF scene as a USD environment with no robot in it."""
    converted = components_from_mjcf(scene_xml, output / "environment", name=name)
    scene = prepare_scene(
        converted.components,
        ground_plane=not converted.has_floor,
        mesh_dir=output / "usd_meshes",
    )
    (output / "scene.usda").write_text(scene.usd)
    return converted


def free_place(
    scene_xml: Path,
    near: tuple[float, float],
    *,
    radius: float,
    height: float,
    reach: float = 4.0,
) -> tuple[float, float, float] | None:
    """The free floor spot nearest ``near`` where an upright cylinder touches nothing.

    A place that suits one robot can be too tight for a wider one. This looks
    outward from ``near`` in rings for a spot with floor under it and no scene
    geometry, fixed or movable, inside the cylinder.

    Args:
        scene_xml: The MJCF environment.
        near: Where to start looking: x and y, in metres.
        radius: The cylinder's radius, in metres: the robot's footprint.
        height: The cylinder's height, in metres.
        reach: How far from ``near`` to look, in metres.

    Returns:
        x, y and the floor's height there, in metres; None if no spot is free.
    """
    spec = mujoco.MjSpec.from_file(str(scene_xml))
    # A free body, not a fixed one: MuJoCo reports no contact between two fixed bodies.
    probe = spec.worldbody.add_body(name="free_place_probe")
    probe.add_freejoint()
    probe.add_geom(
        name="free_place_probe",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[radius, height / 2, 0],
        contype=1,
        conaffinity=1,
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body, geom = model.body("free_place_probe").id, model.geom("free_place_probe").id
    start = model.jnt_qposadr[model.body_jntadr[body]]
    hit = np.array([-1], dtype=np.int32)
    down = np.array([0.0, 0.0, -1.0])
    for ring in range(int(reach / _PLACE_STEP_M) + 1):
        distance = ring * _PLACE_STEP_M
        count = max(1, int(math.tau * distance / _PLACE_STEP_M))
        for step in range(count):
            angle = math.tau * step / count
            x, y = near[0] + distance * math.cos(angle), near[1] + distance * math.sin(angle)
            # Look straight down for a floor; the ray passes through the probe itself.
            eye = np.array([x, y, _PLACE_LOOK_FROM_M])
            drop = mujoco.mj_ray(model, data, eye, down, None, 1, body, hit)
            if drop < 0 or not _is_floor(model, data, int(hit[0])):
                continue
            floor = _PLACE_LOOK_FROM_M - drop
            data.qpos[start : start + 3] = (x, y, floor + _PLACE_CLEARANCE_M + height / 2)
            mujoco.mj_forward(model, data)
            touching = any(
                geom in (contact.geom1, contact.geom2) and contact.dist < 0
                for contact in data.contact[: data.ncon]
            )
            if not touching:
                return (float(x), float(y), float(floor))
    return None


def _is_floor(model: mujoco.MjModel, data: mujoco.MjData, geom: int) -> bool:
    """Whether a geom is a wide, thin, level surface: a floor to stand on."""
    if int(model.geom_type[geom]) == _PLANE:
        return True
    center, half = model.geom_aabb[geom][:3], model.geom_aabb[geom][3:]
    corners = center + half * np.array(
        [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    )
    world = corners @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    extent = world.max(axis=0) - world.min(axis=0)
    return bool(
        min(extent[0], extent[1]) >= _FLOOR_MIN_EXTENT_M and extent[2] <= _FLOOR_MAX_THICKNESS_M
    )


def _safe(name: str) -> str:
    """A name that is one URDF link segment and one USD prim name."""
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


def _text(values: object) -> str:
    return " ".join(f"{float(v):.9g}" for v in np.asarray(values).ravel())


def _origin(parent: ET.Element, pose: NDArray[np.float64]) -> None:
    rpy = Rotation.from_matrix(pose[:3, :3]).as_euler("xyz")
    ET.SubElement(parent, "origin", xyz=_text(pose[:3, 3]), rpy=_text(rpy))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("source", type=Path, help="an MJCF file, or a directory with scene.xml")
    parser.add_argument("--output", required=True, type=Path, help="the directory to write")
    parser.add_argument(
        "--near",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        help="also look for the free floor spot nearest this point, in metres",
    )
    parser.add_argument("--radius", type=float, default=0.45, help="a robot's footprint, metres")
    parser.add_argument("--height", type=float, default=1.7, help="a robot's height, metres")
    args = parser.parse_args()
    source = args.source.expanduser().resolve()
    source = source / "scene.xml" if source.is_dir() else source
    destination = args.output.expanduser().resolve()
    result = environment_from_mjcf(source, destination)
    report: dict[str, Any] = {
        "scene": str(destination / "scene.usda"),
        "welded": list(result.welded),
        "skipped": list(result.skipped),
    }
    if args.near is not None:
        report["place"] = free_place(
            source, (args.near[0], args.near[1]), radius=args.radius, height=args.height
        )
    # One line of JSON, last, for a caller that reads it.
    print(json.dumps(report))
