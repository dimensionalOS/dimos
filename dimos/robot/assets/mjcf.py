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


"""A MuJoCo MJCF robot as a URDF, so one MJCF is the source for both the simulator and RobotModel users."""

from __future__ import annotations

from hashlib import sha256
import os
from pathlib import Path
from typing import Any
import warnings
import xml.etree.ElementTree as ET

from filelock import FileLock
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.constants import CACHE_DIR
from dimos.robot.assets.model import RobotModel

_URDF_CACHE = CACHE_DIR / "mjcf_urdf"


def mjcf_to_urdf(mjcf: Path | str | os.PathLike[str]) -> RobotModel:
    """The robot in an MJCF file as a lazy RobotModel; the MJCF is read and converted when the model is loaded."""
    return RobotModel.from_file(UrdfFromMjcf(mjcf))


class UrdfFromMjcf(os.PathLike[str]):
    """A path to the URDF converted from an MJCF, written to the cache the first time the path is used."""

    def __init__(self, mjcf: Path | str | os.PathLike[str]) -> None:
        self.mjcf = mjcf

    def __fspath__(self) -> str:
        mjcf = Path(os.fspath(self.mjcf)).resolve()
        xml = mjcf_spec_to_urdf_xml(mujoco.MjSpec.from_file(str(mjcf)))
        urdf = _URDF_CACHE / f"{sha256(xml.encode()).hexdigest()[:16]}" / f"{mjcf.stem}.urdf"
        urdf.parent.mkdir(parents=True, exist_ok=True)
        with FileLock(str(urdf) + ".lock"):
            if not urdf.exists():
                urdf.write_text(xml)
        return str(urdf)

    def __str__(self) -> str:
        return self.__fspath__()

    def __repr__(self) -> str:
        return f"UrdfFromMjcf({self.mjcf!r})"

    def __eq__(self, other: object) -> bool:
        return isinstance(other, UrdfFromMjcf) and self.mjcf == other.mjcf

    def __hash__(self) -> int:
        return hash(("UrdfFromMjcf", self.mjcf))

    def cbor_encode(self) -> dict[str, Any]:
        return {"mjcf": self.mjcf}

    @classmethod
    def cbor_decode(cls, data: dict[str, Any]) -> UrdfFromMjcf:
        return cls(data["mjcf"])

    def __reduce__(self) -> tuple[Any, ...]:
        return (UrdfFromMjcf, (self.mjcf,))


def _xyz(values: object) -> str:
    return " ".join(f"{float(v):.9g}" for v in np.asarray(values, dtype=float))


def _rpy(wxyz: object) -> str:
    rotation = Rotation.from_quat(np.asarray(wxyz, dtype=float), scalar_first=True)
    # a 90 degree pitch (MuJoCo's x-axis cylinders) is gimbal-locked, but the angles given are still exact
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return _xyz(rotation.as_euler("xyz"))


def _origin(parent: ET.Element, pos: object, quat: object) -> None:
    ET.SubElement(parent, "origin", xyz=_xyz(pos), rpy=_rpy(quat))


def _geometry(parent: ET.Element, geom: Any, mesh_files: dict[str, str]) -> bool:
    size = np.asarray(geom.size, dtype=float)
    shape = ET.SubElement(parent, "geometry")
    if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
        ET.SubElement(shape, "mesh", filename=mesh_files[geom.meshname])
    elif geom.type == mujoco.mjtGeom.mjGEOM_BOX:
        ET.SubElement(shape, "box", size=_xyz(2 * size))
    elif geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        ET.SubElement(shape, "cylinder", radius=f"{size[0]:.9g}", length=f"{2 * size[1]:.9g}")
    elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
        ET.SubElement(shape, "sphere", radius=f"{size[0]:.9g}")
    else:
        return False
    return True


def mjcf_spec_to_urdf_xml(spec: mujoco.MjSpec) -> str:
    """The spec's bodies as URDF links, its hinge joints as revolute (continuous when unlimited), meshes by absolute path."""
    mesh_dir = Path(spec.modelfiledir) / spec.meshdir
    mesh_files = {mesh.name: str((mesh_dir / mesh.file).resolve()) for mesh in spec.meshes}
    robot = ET.Element("robot", name=spec.modelname or "robot")
    world = spec.worldbody
    for body in spec.bodies:
        if body.name == world.name:
            continue
        link = ET.SubElement(robot, "link", name=body.name)
        inertial = ET.SubElement(link, "inertial")
        _origin(inertial, body.ipos, body.iquat)
        ET.SubElement(inertial, "mass", value=f"{body.mass:.9g}")
        ixx, iyy, izz = (float(v) for v in body.inertia)
        ET.SubElement(
            inertial,
            "inertia",
            ixx=f"{ixx:.9g}",
            iyy=f"{iyy:.9g}",
            izz=f"{izz:.9g}",
            ixy="0",
            ixz="0",
            iyz="0",
        )
        for geom in body.geoms:
            kind = "collision" if geom.contype or geom.conaffinity else "visual"
            element = ET.SubElement(link, kind)
            _origin(element, geom.pos, geom.quat)
            if not _geometry(element, geom, mesh_files):
                link.remove(element)
        parent = body.parent
        if parent.name == world.name:
            continue
        hinges = [j for j in body.joints if j.type == mujoco.mjtJoint.mjJNT_HINGE]
        if hinges:
            hinge = hinges[0]
            limited = hinge.range[0] < hinge.range[1]
            joint = ET.SubElement(
                robot, "joint", name=hinge.name, type="revolute" if limited else "continuous"
            )
        else:
            hinge = None
            limited = False
            joint = ET.SubElement(robot, "joint", name=f"{body.name}_joint", type="fixed")
        _origin(joint, body.pos, body.quat)
        ET.SubElement(joint, "parent", link=parent.name)
        ET.SubElement(joint, "child", link=body.name)
        if hinge is not None:
            ET.SubElement(joint, "axis", xyz=_xyz(hinge.axis))
            limit = ET.SubElement(joint, "limit", effort=f"{abs(hinge.actfrcrange[1]):.9g}")
            if limited:
                limit.set("lower", f"{hinge.range[0]:.9g}")
                limit.set("upper", f"{hinge.range[1]:.9g}")
    return ET.tostring(robot, encoding="unicode")
