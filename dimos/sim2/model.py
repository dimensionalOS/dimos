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

"""Import authored robot MJCF through robosuite without changing its physics."""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from functools import partial
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from robosuite.models.robots.robot_model import RobotModel
from robosuite.utils.mjcf_utils import MUJOCO_NAMED_ATTRIBUTES

from dimos.sim2.spec import ModelAsset, RobotInstanceSpec
from dimos.sim2.world.scene import scene_spawn

Defaults = dict[str, dict[str, dict[str, str]]]


def default_classes(default: ET.Element) -> Defaults:
    """Resolve native anonymous and nested default classes."""
    result: Defaults = {}

    def visit(node: ET.Element, inherited: dict[str, dict[str, str]]) -> None:
        values = deepcopy(inherited)
        for child in node:
            if child.tag != "default":
                values.setdefault(child.tag, {}).update(child.attrib)
        result[node.get("class", "main")] = values
        for child in node.findall("default"):
            visit(child, values)

    visit(default, {})
    return result


def inline_defaults(root: ET.Element, defaults: Defaults) -> None:
    """Expand defaults and childclass while retaining explicit authored values."""

    def visit(node: ET.Element, inherited: str) -> None:
        if node.tag == "default":
            return
        active = node.attrib.pop("class", inherited)
        children = node.attrib.pop("childclass", inherited)
        node.attrib = {**defaults[active].get(node.tag, {}), **node.attrib}
        for child in node:
            visit(child, children)

    visit(root, "main")


class ImportedRobotModel(RobotModel):
    """An existing MJCF asset with instance names and an optional initial pose.

    Backend import fixes live here once. Robot packages supply asset references,
    motor mappings and controller settings; they do not subclass robosuite.
    """

    def __init__(self, model: ModelAsset, idn: str | int) -> None:
        source = model.path.resolve()
        root = ET.parse(source).getroot()
        if root.find("include") is not None:
            raise ValueError("robot MJCF includes must be expanded before importing the model")
        compiler = root.find("compiler")
        if compiler is None or compiler.get("angle", "degree") != "radian":
            raise ValueError("robot MJCF assets must declare compiler angle='radian'")
        # Merge repeated sections as MuJoCo does; robosuite otherwise reads only
        # the first sensor block. Keep world references unresolved until assembly.
        for tag in ("asset", "sensor", "actuator", "contact", "equality", "tendon"):
            sections = root.findall(tag)
            for section in sections[1:]:
                sections[0].extend(section)
                root.remove(section)
        for tag, directory in (("mesh", "meshdir"), ("texture", "texturedir"), ("hfield", "")):
            for asset in root.findall(f"./asset/{tag}"):
                file = asset.get("file")
                if file:
                    base = "" if compiler is None else compiler.get(directory, "")
                    asset.set(
                        "file",
                        str(
                            (model.meshdir / Path(file).name).resolve()
                            if tag == "mesh" and model.meshdir is not None
                            else (source.parent / base / file).resolve()
                        ),
                    )
        if compiler is not None:
            compiler.attrib.pop("meshdir", None)
            compiler.attrib.pop("texturedir", None)
        body = root.find("./worldbody/body")
        assert body is not None
        floating = (
            body.find("freejoint") is not None or body.find("joint[@type='free']") is not None
        )
        initial = np.asarray(model.initial_qpos, dtype=np.float64)
        if model.keyframe is not None:
            key = root.find(f"./keyframe/key[@name='{model.keyframe}']")
            if key is None:
                raise ValueError(f"{source}: missing initial keyframe {model.keyframe!r}")
            key_pose = np.fromstring(key.get("qpos", ""), sep=" ")
            offset = 7 if floating else 0
            if floating:
                body.set("pos", " ".join(map(str, key_pose[:3])))
                body.set("quat", " ".join(map(str, key_pose[3:7])))
            if not initial.size:
                initial = key_pose[offset:]
        for keyframe in root.findall("keyframe"):
            root.remove(keyframe)
        names = {element.attrib["name"] for element in root.iter() if "name" in element.attrib}
        for index, geom in enumerate(root.findall(".//worldbody//geom")):
            if "name" not in geom.attrib:
                name = f"dimos_geom_{index}"
                while name in names:
                    name += "_"
                geom.set("name", name)
                names.add(name)
        self._external_names = {
            value
            for element in root.iter()
            for attr, value in element.attrib.items()
            if attr in MUJOCO_NAMED_ATTRIBUTES and attr != "name" and value not in names
        }
        with TemporaryDirectory(prefix="dimos-robot-") as directory:
            path = Path(directory) / "robot.xml"
            path.write_text(ET.tostring(root, encoding="unicode"))
            super().__init__(str(path), idn=idn)
        self._initial_qpos = initial if initial.size else np.zeros(self.dof - int(floating))
        for geom, appearance in self._authored_appearance.items():
            for attribute, value in appearance.items():
                if value is None:
                    geom.attrib.pop(attribute, None)
                else:
                    geom.set(
                        attribute, self.correct_naming(value) if attribute == "material" else value
                    )
        # RobotModel injects nonzero joint dynamics when an attribute is absent.
        for joint, values in self._authored_dynamics.items():
            for attribute, value in values.items():
                if value is None:
                    joint.attrib.pop(attribute, None)
                else:
                    joint.set(attribute, value)
        if model.control_imu is not None:
            site = self.correct_naming(model.control_imu)
            if self.worldbody.find(f".//site[@name='{site}']") is None:
                raise ValueError(f"control IMU site {model.control_imu!r} is absent from {source}")
            for kind, suffix in (("gyro", "gyro"), ("accelerometer", "accel")):
                name = self.correct_naming(f"{model.control_imu}_{suffix}")
                if self.sensor.find(f"*[@name='{name}']") is None:
                    ET.SubElement(self.sensor, kind, name=name, site=site)
            name = self.correct_naming(f"{model.control_imu}_quat")
            if self.sensor.find(f"*[@name='{name}']") is None:
                ET.SubElement(self.sensor, "framequat", name=name, objtype="site", objname=site)

    def _get_default_classes(self, default: ET.Element) -> Defaults:
        return default_classes(default)

    def _replace_defaults_inline(
        self, default_dic: Defaults, root: ET.Element | None = None
    ) -> None:
        inline_defaults(self.root, default_dic)
        self._authored_appearance = {
            geom: {attribute: geom.get(attribute) for attribute in ("rgba", "material")}
            for geom in self.worldbody.iter("geom")
        }
        self._authored_dynamics = {
            joint: {
                attribute: joint.get(attribute)
                for attribute in ("damping", "armature", "frictionloss")
            }
            for joint in self.worldbody.iter("joint")
        }

    def exclude_from_prefixing(self, inp: Any) -> bool:
        return isinstance(inp, str) and inp in self._external_names

    @property
    def sensors(self) -> list[str]:
        return [sensor.attrib["name"] for sensor in self.sensor]

    @property
    def init_qpos(self) -> NDArray[np.float64]:
        return self._initial_qpos.copy()

    @property
    def bottom_offset(self) -> NDArray[np.float64]:
        return -np.asarray(self.base_offset, dtype=np.float64)


def create_model(instance: RobotInstanceSpec, scene: Path) -> ImportedRobotModel:
    if instance.model.format != "mjcf":
        raise NotImplementedError("the current asset compiler supports MJCF sources only")
    # Hex encoding is injective, unlike replacing slashes with underscores.
    identifier = "i" + instance.robot_id.encode().hex()
    model = ImportedRobotModel(instance.model, idn=identifier)
    support = scene_spawn(scene, instance.spawn)
    position = instance.position if instance.position is not None else support.position.to_tuple()
    rpy = instance.rpy if instance.rpy is not None else support.orientation.to_euler().to_tuple()
    model.set_base_xpos(np.asarray(position))
    model.set_base_ori(np.asarray(rpy))
    if len(model.actuators) != len(instance.joint_names):
        raise ValueError(
            f"{instance.robot_id}: public motors must match the authored actuator count"
        )
    return model


def model_factory(instance: RobotInstanceSpec, scene: Path) -> Callable[[], ImportedRobotModel]:
    return partial(create_model, instance, scene)
