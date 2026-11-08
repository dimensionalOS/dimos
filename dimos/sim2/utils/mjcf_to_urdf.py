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

"""The URDF of an MJCF robot, so a URDF consumer (UsdManager, renderers) stays in sync with what MuJoCo simulates.

Each body becomes a link of the same name (the root may be renamed), each hinge/slide joint a revolute/prismatic
joint of the same name, a body without joints a fixed joint, and each named site (a sensor's frame, see
mjcf_with_frames) a link fixed to its body. A free root joint is dropped: tf poses the root.
Meshes are written out as compiled (MuJoCo recenters them), so the URDF needs no MuJoCo to render.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import RobotModel

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-urdf"
# MuJoCo puts collision-only geoms in group 3 by convention; the rest are drawn.
_VISUAL_GROUPS = frozenset({0, 1, 2})


def mjcf_to_urdf(
    mjcf: Path, name: str | None = None, root_link: str | None = None, out_dir: Path | None = None
) -> RobotModel:
    """The URDF of `mjcf`, written (with its meshes) under out_dir and reused while the MJCF and options are unchanged.

    The robot is named `name`, else the MJCF's model name, else the file's stem.
    """
    mjcf = Path(mjcf).resolve()
    options = f"{mjcf}\0{mjcf.stat().st_mtime_ns}\0{name}\0{root_link}"
    key = hashlib.sha256(options.encode()).hexdigest()[:16]
    directory = (out_dir or _CACHE) / f"{mjcf.stem}-{key}"
    urdf = directory / f"{mjcf.stem}.urdf"
    if not urdf.exists():
        directory.mkdir(parents=True, exist_ok=True)
        model = mujoco.MjModel.from_xml_path(str(mjcf))
        model_name = bytes(model.names).split(b"\0")[0].decode()
        xml = _urdf(model, name or model_name or mjcf.stem, root_link, directory)
        partial = urdf.with_suffix(".partial")
        partial.write_text(xml)
        partial.replace(urdf)
    return RobotModel.from_file(urdf)


def _urdf(model: mujoco.MjModel, name: str, root_link: str | None, directory: Path) -> str:
    roots = [b for b in range(1, model.nbody) if model.body_parentid[b] == 0]
    if len(roots) != 1:
        raise ValueError(f"{name}: a robot MJCF has one root body, this one has {len(roots)}")
    links = {b: _body_name(model, b) for b in range(1, model.nbody)}
    if root_link is not None:
        links[roots[0]] = root_link
    robot = ET.Element("robot", name=name)
    meshes = {m: _write_mesh(model, m, directory) for m in range(model.nmesh)}
    for body in range(1, model.nbody):
        link = ET.SubElement(robot, "link", name=links[body])
        _add_inertial(link, model, body)
        for geom in np.flatnonzero(model.geom_bodyid == body):
            _add_geom(link, model, int(geom), meshes)
        if body == roots[0]:
            _check_root_joints(model, body)
            continue
        _add_joints(robot, model, body, links[int(model.body_parentid[body])], links[body])
    names = set(links.values())
    for site in range(model.nsite):
        site_name = model.site(site).name
        if site_name and site_name not in names:
            ET.SubElement(robot, "link", name=site_name)
            on_body = _transform(model.site_pos[site], model.site_quat[site])
            _joint(
                robot,
                f"{site_name}_fixed",
                "fixed",
                links[int(model.site_bodyid[site])],
                site_name,
                on_body,
            )
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode")


def _body_name(model: mujoco.MjModel, body: int) -> str:
    return model.body(body).name or f"body_{body}"


def _check_root_joints(model: mujoco.MjModel, body: int) -> None:
    for joint in _joints_of(model, body):
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"root body {_body_name(model, body)!r} has a non-free joint")


def _joints_of(model: mujoco.MjModel, body: int) -> range:
    start = int(model.body_jntadr[body])
    return range(start, start + int(model.body_jntnum[body])) if start >= 0 else range(0)


def _add_joints(
    robot: ET.Element, model: mujoco.MjModel, body: int, parent: str, child: str
) -> None:
    """The body's pose on its parent, then its joints in order; a joint off the body origin pivots about its pos."""
    joints = list(_joints_of(model, body))
    body_on_parent = _transform(model.body_pos[body], model.body_quat[body])
    if not joints:
        _joint(robot, f"{child}_fixed", "fixed", parent, child, body_on_parent)
        return
    for index, joint in enumerate(joints):
        kind = model.jnt_type[joint]
        if kind not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            raise ValueError(
                f"joint {model.joint(joint).name!r}: only hinge and slide joints have a URDF form"
            )
        name = model.joint(joint).name or f"joint_{joint}"
        last = index == len(joints) - 1
        target = child if last else f"{child}__{name}"
        if not last:
            ET.SubElement(robot, "link", name=target)
        pivot = np.asarray(model.jnt_pos[joint], dtype=np.float64)
        # The child sits at origin @ motion(q); pivoting about `pivot` is T(p) @ motion(q) @ T(-p).
        if np.any(pivot):
            pivot_link = f"{child}__{name}__pivot"
            ET.SubElement(robot, "link", name=pivot_link)
            _joint(
                robot,
                f"{name}__pivot",
                "fixed",
                parent,
                pivot_link,
                body_on_parent @ _translation(pivot),
            )
            parent, origin = pivot_link, _translation(-pivot)
        else:
            origin = body_on_parent
        urdf_type = (
            "prismatic"
            if kind == mujoco.mjtJoint.mjJNT_SLIDE
            else ("revolute" if model.jnt_limited[joint] else "continuous")
        )
        element = _joint(robot, name, urdf_type, parent, target, origin)
        ET.SubElement(element, "axis", xyz=_numbers(model.jnt_axis[joint]))
        if urdf_type != "continuous":
            lower, upper = model.jnt_range[joint] if model.jnt_limited[joint] else (-1e9, 1e9)
            ET.SubElement(
                element,
                "limit",
                lower=_number(lower),
                upper=_number(upper),
                effort=_number(_effort(model, joint)),
                velocity="100",
            )
        parent, body_on_parent = target, np.eye(4)


def _effort(model: mujoco.MjModel, joint: int) -> float:
    actuators = np.flatnonzero(
        (model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
        & (model.actuator_trnid[:, 0] == joint)
    )
    for actuator in actuators:
        if model.actuator_forcelimited[actuator]:
            return float(np.max(np.abs(model.actuator_forcerange[actuator])))
    return 1000.0


def _joint(
    robot: ET.Element, name: str, kind: str, parent: str, child: str, origin: NDArray[np.float64]
) -> ET.Element:
    element = ET.SubElement(robot, "joint", name=name, type=kind)
    _origin(element, origin)
    ET.SubElement(element, "parent", link=parent)
    ET.SubElement(element, "child", link=child)
    return element


def _add_inertial(link: ET.Element, model: mujoco.MjModel, body: int) -> None:
    if model.body_mass[body] <= 0:
        return
    inertial = ET.SubElement(link, "inertial")
    _origin(inertial, _transform(model.body_ipos[body], model.body_iquat[body]))
    ET.SubElement(inertial, "mass", value=_number(model.body_mass[body]))
    ixx, iyy, izz = model.body_inertia[body]
    ET.SubElement(
        inertial,
        "inertia",
        ixx=_number(ixx),
        iyy=_number(iyy),
        izz=_number(izz),
        ixy="0",
        ixz="0",
        iyz="0",
    )


def _add_geom(link: ET.Element, model: mujoco.MjModel, geom: int, meshes: dict[int, Path]) -> None:
    kind, size = model.geom_type[geom], model.geom_size[geom]
    pose = _transform(model.geom_pos[geom], model.geom_quat[geom])
    shapes: list[tuple[NDArray[np.float64], str, dict[str, str]]] = []
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        shapes.append((pose, "mesh", {"filename": str(meshes[int(model.geom_dataid[geom])])}))
    elif kind == mujoco.mjtGeom.mjGEOM_BOX:
        shapes.append((pose, "box", {"size": _numbers(2 * size)}))
    elif kind in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        shapes.append((pose, "sphere", {"radius": _number(size[0])}))
    elif kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        shapes.append(
            (pose, "cylinder", {"radius": _number(size[0]), "length": _number(2 * size[1])})
        )
        if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            # URDF has no capsule: a cylinder and its two end spheres.
            for end in (-size[1], size[1]):
                shapes.append(
                    (pose @ _translation((0.0, 0.0, end)), "sphere", {"radius": _number(size[0])})
                )
    else:
        return
    visual = model.geom_group[geom] in _VISUAL_GROUPS
    collides = bool(model.geom_contype[geom] or model.geom_conaffinity[geom])
    rgba = (
        model.mat_rgba[model.geom_matid[geom]]
        if model.geom_matid[geom] >= 0
        else model.geom_rgba[geom]
    )
    for transform, shape, attributes in shapes:
        for tag, wanted in (("visual", visual), ("collision", collides)):
            if not wanted:
                continue
            element = ET.SubElement(link, tag)
            _origin(element, transform)
            ET.SubElement(ET.SubElement(element, "geometry"), shape, attributes)
            if tag == "visual":
                material = ET.SubElement(element, "material", name=f"geom_{geom}")
                ET.SubElement(material, "color", rgba=_numbers(rgba))


def _write_mesh(model: mujoco.MjModel, mesh: int, directory: Path) -> Path:
    """The compiled mesh, in its geom's frame, as an OBJ."""
    vert_start, vert_count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    face_start, face_count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    vertices = model.mesh_vert[vert_start : vert_start + vert_count]
    faces = model.mesh_face[face_start : face_start + face_count] + 1
    name = model.mesh(mesh).name or f"mesh_{mesh}"
    path = directory / f"{name}.obj"
    lines = [f"v {x:.6g} {y:.6g} {z:.6g}" for x, y, z in vertices]
    lines += [f"f {a} {b} {c}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")
    return path


def _transform(pos: NDArray[np.float64], wxyz: NDArray[np.float64]) -> NDArray[np.float64]:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(wxyz, scalar_first=True).as_matrix()
    transform[:3, 3] = pos
    return transform


def _translation(xyz: tuple[float, float, float] | NDArray[np.float64]) -> NDArray[np.float64]:
    transform = np.eye(4)
    transform[:3, 3] = xyz
    return transform


def _origin(element: ET.Element, transform: NDArray[np.float64]) -> None:
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
    ET.SubElement(element, "origin", xyz=_numbers(transform[:3, 3]), rpy=_numbers(rpy))


def _number(value: float) -> str:
    return f"{float(value):.9g}"


def _numbers(values: NDArray[np.float64]) -> str:
    return " ".join(_number(v) for v in values)
