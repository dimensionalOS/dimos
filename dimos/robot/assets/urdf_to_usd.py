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

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.usd import Prim, Pxr, Quat, Stage, load_pxr, prim_identifier, usd_matrix
from dimos.robot.assets.usd_assets import write_asset_info
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.robot.assets.model import JointDescription, LoadedRobotModel


logger = setup_logger()


_MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})
_OUT_OF_PLANE = ("transZ", "rotX", "rotY")
PASSTHROUGH_KEY = "urdf:unconverted"


@dataclass(frozen=True)
class _Geometry:
    kind: str
    origin: NDArray[np.float64]
    size: tuple[float, ...]
    filename: str | None
    mesh_scale: tuple[float, float, float]
    color: tuple[float, float, float] | None
    collision: bool


def urdf_to_usd(
    model: LoadedRobotModel,
    *,
    name: str | None = None,
    include_collision: bool = True,
) -> str:
    pxr = load_pxr()
    root_element = ET.fromstring(model.xml)
    name = name or root_element.get("name") or "robot"
    links = _urdf_links_of(model.xml, include_collision=include_collision)
    children = _children_by_parent(model.joints)

    stage = pxr.Usd.Stage.CreateInMemory(f"{name}.usda")
    pxr.UsdGeom.SetStageUpAxis(stage, pxr.UsdGeom.Tokens.z)
    pxr.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = pxr.UsdGeom.Xform.Define(stage, f"/{prim_identifier(pxr, name)}")
    stage.SetDefaultPrim(root.GetPrim())

    link_extras, joint_extras = _urdf_extras(model.xml)

    paths: dict[str, str] = {}
    _write_link_tree(
        pxr,
        stage,
        parent_path=str(root.GetPath()),
        link=model.root_link,
        links=links,
        children=children,
        paths=paths,
        extras=link_extras,
        base_dir=Path(model.source_path).parent if model.source_path else None,
        inertials=_inertial_of(model.xml),
    )
    _write_joints(
        pxr,
        stage,
        root_path=str(root.GetPath()),
        joints=model.joints,
        paths=paths,
        extras=joint_extras,
    )
    return str(stage.GetRootLayer().ExportToString())


def _inertial_of(xml: str) -> dict[str, tuple[float, np.ndarray, np.ndarray]]:
    found: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
    for link in ET.fromstring(xml).iter("link"):
        name, inertial = link.get("name"), link.find("inertial")
        if name is None or inertial is None:
            continue
        mass_element, tensor = inertial.find("mass"), inertial.find("inertia")
        if mass_element is None or tensor is None:
            continue
        origin = inertial.find("origin")
        zero = (0.0, 0.0, 0.0)
        xyz = np.array(_floats(origin.get("xyz") if origin is not None else None, 3, zero))
        rpy = np.array(_floats(origin.get("rpy") if origin is not None else None, 3, zero))
        read = {
            key: float(tensor.get(key, 0.0)) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        }
        matrix = np.array(
            [
                [read["ixx"], read["ixy"], read["ixz"]],
                [read["ixy"], read["iyy"], read["iyz"]],
                [read["ixz"], read["iyz"], read["izz"]],
            ]
        )
        rotation = Rotation.from_euler("xyz", rpy).as_matrix()
        found[name] = (float(mass_element.get("value", 0.0)), xyz, rotation @ matrix @ rotation.T)
    return found


def _write_mass(pxr: Pxr, prim: Prim, mass: float, centre: np.ndarray, tensor: np.ndarray) -> None:
    moments, axes = np.linalg.eigh(tensor)
    if np.linalg.det(axes) < 0:
        axes[:, 0] = -axes[:, 0]
    api = pxr.UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    api.CreateCenterOfMassAttr(pxr.Gf.Vec3f(*(float(value) for value in centre)))
    api.CreateDiagonalInertiaAttr(pxr.Gf.Vec3f(*(float(max(value, 0.0)) for value in moments)))
    quaternion = Rotation.from_matrix(axes).as_quat()
    api.CreatePrincipalAxesAttr(
        pxr.Gf.Quatf(
            float(quaternion[3]), float(quaternion[0]), float(quaternion[1]), float(quaternion[2])
        )
    )


def _write_link_tree(
    pxr: Pxr,
    stage: Stage,
    *,
    parent_path: str,
    link: str,
    links: Mapping[str, Sequence[_Geometry]],
    children: Mapping[str, Sequence[JointDescription]],
    paths: dict[str, str],
    extras: Mapping[str, Sequence[str]],
    base_dir: Path | None,
    inertials: Mapping[str, tuple[float, np.ndarray, np.ndarray]] | None = None,
    joint: JointDescription | None = None,
) -> None:
    prim_path = f"{parent_path}/{prim_identifier(pxr, link)}"
    if link in paths:
        logger.warning(f"URDF link {link!r} is reached more than once; ignoring the later edge")
        return
    xform = pxr.UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    paths[link] = prim_path
    prim.SetDisplayName(link)
    _set_passthrough(pxr, prim, extras.get(link))
    pxr.UsdPhysics.RigidBodyAPI.Apply(prim)
    inertial = (inertials or {}).get(link)
    if inertial is not None:
        _write_mass(pxr, prim, *inertial)
    if joint is not None:
        xform.AddTransformOp().Set(usd_matrix(pxr, _zero_pose(joint)))

    for index, geometry in enumerate(links.get(link, ())):
        _write_geometry(
            pxr,
            stage,
            f"{prim_path}/{_geometry_name(geometry, index)}",
            geometry,
            base_dir,
        )

    for child_joint in children.get(link, ()):
        _write_link_tree(
            pxr,
            stage,
            parent_path=prim_path,
            link=child_joint.child_link,
            links=links,
            children=children,
            paths=paths,
            extras=extras,
            base_dir=base_dir,
            inertials=inertials,
            joint=child_joint,
        )


def _geometry_name(geometry: _Geometry, index: int) -> str:
    return f"{'collision' if geometry.collision else 'visual'}_{index}"


def _zero_pose(joint: JointDescription) -> NDArray[np.float64]:
    if joint.type in _MULTI_DOF_JOINT_TYPES:
        return joint.origin()
    return joint.parent_from_child(0.0)


def _write_geometry(
    pxr: Pxr, stage: Stage, path: str, geometry: _Geometry, base_dir: Path | None
) -> None:
    gprim = _define_gprim(pxr, stage, path, geometry, base_dir)
    if gprim is None:
        return
    gprim.AddTransformOp().Set(usd_matrix(pxr, geometry.origin))
    if geometry.color is not None:
        gprim.CreateDisplayColorAttr([pxr.Gf.Vec3f(*geometry.color)])
    if geometry.collision:
        pxr.UsdGeom.Imageable(gprim).CreateVisibilityAttr(pxr.UsdGeom.Tokens.invisible)
        pxr.UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())


def _define_gprim(
    pxr: Pxr, stage: Stage, path: str, geometry: _Geometry, base_dir: Path | None
) -> Prim:
    if geometry.kind == "box":
        cube = pxr.UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(pxr.Gf.Vec3f(*geometry.size))
        return cube
    if geometry.kind == "cylinder":
        cylinder = pxr.UsdGeom.Cylinder.Define(stage, path)
        cylinder.CreateAxisAttr(pxr.UsdGeom.Tokens.z)
        cylinder.CreateRadiusAttr(geometry.size[0])
        cylinder.CreateHeightAttr(geometry.size[1])
        return cylinder
    if geometry.kind == "sphere":
        sphere = pxr.UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(geometry.size[0])
        return sphere
    if geometry.kind == "mesh" and geometry.filename is not None:
        return _define_mesh(pxr, stage, path, geometry, base_dir)
    logger.warning(f"skipping URDF geometry of unsupported kind {geometry.kind!r}")
    return None


def _define_mesh(
    pxr: Pxr, stage: Stage, path: str, geometry: _Geometry, base_dir: Path | None
) -> Prim:
    assert geometry.filename is not None
    mesh = pxr.UsdGeom.Mesh.Define(stage, path)
    mesh.AddScaleOp().Set(pxr.Gf.Vec3f(*geometry.mesh_scale))
    write_asset_info(pxr, mesh.GetPrim(), geometry.filename, base_dir)
    return mesh


def _write_joints(
    pxr: Pxr,
    stage: Stage,
    *,
    root_path: str,
    joints: Sequence[JointDescription],
    paths: Mapping[str, str],
    extras: Mapping[str, Sequence[str]],
) -> None:
    scope = pxr.UsdGeom.Scope.Define(stage, f"{root_path}/joints")
    for joint in joints:
        parent = paths.get(joint.parent_link)
        child = paths.get(joint.child_link)
        if parent is None or child is None:
            logger.warning(f"skipping joint {joint.name!r}: it names a link outside the tree")
            continue
        prim = _define_joint(
            pxr, stage, f"{scope.GetPath()}/{prim_identifier(pxr, joint.name)}", joint
        )
        if prim is None:
            continue
        prim.CreateBody0Rel().SetTargets([pxr.Sdf.Path(parent)])
        prim.CreateBody1Rel().SetTargets([pxr.Sdf.Path(child)])
        prim.GetPrim().SetDisplayName(joint.name)
        _set_passthrough(pxr, prim.GetPrim(), extras.get(joint.name))
        _set_joint_frames(pxr, prim, joint)
        _set_joint_limits(prim, joint)


def _define_joint(pxr: Pxr, stage: Stage, path: str, joint: JointDescription) -> Prim:
    if joint.type in {"revolute", "continuous"}:
        return pxr.UsdPhysics.RevoluteJoint.Define(stage, path)
    if joint.type == "prismatic":
        return pxr.UsdPhysics.PrismaticJoint.Define(stage, path)
    if joint.type == "fixed":
        return pxr.UsdPhysics.FixedJoint.Define(stage, path)
    if joint.type in _MULTI_DOF_JOINT_TYPES:
        prim = pxr.UsdPhysics.Joint.Define(stage, path)
        if joint.type == "planar":
            for axis in _OUT_OF_PLANE:
                locked = pxr.UsdPhysics.LimitAPI.Apply(prim.GetPrim(), axis)
                locked.CreateLowAttr(0.0)
                locked.CreateHighAttr(0.0)
        return prim
    logger.warning(f"joint {joint.name!r} of type {joint.type!r} has no USD equivalent; left out")
    return None


def _set_joint_frames(pxr: Pxr, prim: Prim, joint: JointDescription) -> None:
    single_axis = joint.type not in {"fixed", *_MULTI_DOF_JOINT_TYPES}
    to_z = _rotation_to_z(joint.axis) if single_axis else Rotation.identity()
    origin = Rotation.from_euler("xyz", joint.origin_rpy)
    prim.CreateLocalPos0Attr(pxr.Gf.Vec3f(*joint.origin_xyz))
    prim.CreateLocalRot0Attr(_quat(pxr, origin * to_z))
    prim.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    prim.CreateLocalRot1Attr(_quat(pxr, to_z))
    if single_axis:
        prim.CreateAxisAttr(pxr.UsdPhysics.Tokens.z)


def _set_joint_limits(prim: Prim, joint: JointDescription) -> None:
    if joint.type == "continuous" or joint.lower is None or joint.upper is None:
        return
    scale = 180.0 / np.pi if joint.type in {"revolute", "continuous"} else 1.0
    prim.CreateLowerLimitAttr(float(joint.lower * scale))
    prim.CreateUpperLimitAttr(float(joint.upper * scale))


def _quat(pxr: Pxr, rotation: Rotation) -> Quat:
    x, y, z, w = rotation.as_quat()
    return pxr.Gf.Quatf(float(w), float(x), float(y), float(z))


def _rotation_to_z(axis: tuple[float, float, float]) -> Rotation:
    vector = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return Rotation.identity()
    vector = vector / norm
    z = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z, vector)
    sin = float(np.linalg.norm(cross))
    cos = float(np.dot(z, vector))
    if sin < 1e-9:
        return Rotation.identity() if cos > 0 else Rotation.from_rotvec([np.pi, 0.0, 0.0])
    return Rotation.from_rotvec(cross / sin * np.arctan2(sin, cos))


def _children_by_parent(
    joints: Sequence[JointDescription],
) -> dict[str, list[JointDescription]]:
    children: dict[str, list[JointDescription]] = {}
    for joint in joints:
        children.setdefault(joint.parent_link, []).append(joint)
    return children


def _urdf_links_of(xml: str, *, include_collision: bool) -> dict[str, list[_Geometry]]:
    links: dict[str, list[_Geometry]] = {}
    for link in ET.fromstring(xml).iter("link"):
        name = link.get("name")
        if name is None:
            continue
        geometries = [_geometry_of(element, collision=False) for element in link.iter("visual")]
        if include_collision:
            geometries += [
                _geometry_of(element, collision=True) for element in link.iter("collision")
            ]
        links[name] = [geometry for geometry in geometries if geometry is not None]
    return links


_UNDERSTOOD_LINK_TAGS = frozenset({"visual", "collision"})
_UNDERSTOOD_JOINT_TAGS = frozenset({"origin", "parent", "child", "axis", "limit", "mimic"})


def _passthrough_xml(element: ET.Element, understood: frozenset[str]) -> list[str]:
    return [
        ET.tostring(child, encoding="unicode").strip()
        for child in element
        if child.tag not in understood
    ]


def _urdf_extras(xml: str) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
    root = ET.fromstring(xml)
    links = {
        name: extras
        for link in root.iter("link")
        if (name := link.get("name")) is not None
        and (extras := _passthrough_xml(link, _UNDERSTOOD_LINK_TAGS))
    }
    joints = {
        name: extras
        for joint in root.iter("joint")
        if (name := joint.get("name")) is not None
        and (extras := _passthrough_xml(joint, _UNDERSTOOD_JOINT_TAGS))
    }
    return links, joints


def _set_passthrough(pxr: Pxr, prim: Prim, xml: Sequence[str] | None) -> None:
    if xml:
        prim.SetCustomDataByKey(PASSTHROUGH_KEY, pxr.Vt.StringArray(list(xml)))


def _geometry_of(element: ET.Element, *, collision: bool) -> _Geometry | None:
    shape = element.find("geometry")
    if shape is None:
        return None
    origin = _origin_of(element.find("origin"))
    color = _color_of(element.find("material"))
    if (box := shape.find("box")) is not None:
        size = _floats(box.get("size"), 3, (1.0, 1.0, 1.0))
        return _Geometry("box", origin, size, None, (1.0, 1.0, 1.0), color, collision)
    if (cylinder := shape.find("cylinder")) is not None:
        radius = float(cylinder.get("radius", 1.0))
        length = float(cylinder.get("length", 1.0))
        return _Geometry(
            "cylinder", origin, (radius, length), None, (1.0, 1.0, 1.0), color, collision
        )
    if (sphere := shape.find("sphere")) is not None:
        radius = float(sphere.get("radius", 1.0))
        return _Geometry("sphere", origin, (radius,), None, (1.0, 1.0, 1.0), color, collision)
    if (mesh := shape.find("mesh")) is not None:
        filename = mesh.get("filename")
        if filename is None:
            return None
        sx, sy, sz = _floats(mesh.get("scale"), 3, (1.0, 1.0, 1.0))
        return _Geometry("mesh", origin, (), filename, (sx, sy, sz), color, collision)
    return None


def _origin_of(element: ET.Element | None) -> NDArray[np.float64]:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = Rotation.from_euler(
        "xyz", _floats(element.get("rpy"), 3, (0.0,) * 3)
    ).as_matrix()
    transform[:3, 3] = _floats(element.get("xyz"), 3, (0.0,) * 3)
    return transform


def _color_of(element: ET.Element | None) -> tuple[float, float, float] | None:
    if element is None:
        return None
    color = element.find("color")
    if color is None:
        return None
    rgba = _floats(color.get("rgba"), 4, (0.8, 0.8, 0.8, 1.0))
    return (rgba[0], rgba[1], rgba[2])


def _floats(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) != count:
        logger.warning(f"expected {count} numbers in {text!r}, using {default}")
        return default
    return tuple(values)
