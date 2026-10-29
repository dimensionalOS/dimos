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

"""dimos's xArm7 as a LIBERO robot, so LIBERO builds every scene with it in the Panda's place.

Runs inside LIBERO's Python (robosuite 1.4): no dimos imports. ``register(xarm_xml)``
defines ``MountedXArm7`` (tables) and ``OnTheGroundXArm7`` (floor scenes), which LIBERO
selects for ``robots=["XArm7"]``.

robosuite merges a robot's worldbody, actuators, tendons and equalities into the scene
but not its ``<default>`` classes, so the MJCF is flattened first: every element gets the
attributes its class chain gives it, written out explicitly. Joint damping, armature and
frictionloss are always explicit, because robosuite fills missing ones with its own values.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
HOME = (0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0)
# robosuite overwrites these on joints that leave them unset; MuJoCo's own defaults.
_JOINT_EXPLICIT = {"damping": "0", "armature": "0", "frictionloss": "0"}
# Where robosuite attaches a gripper and reads the end effector; the xArm's hand is built in.
_EEF = "right_hand"


Classes = dict[str, dict[str, dict[str, str]]]  # class -> tag -> attributes


def _class_defaults(
    default: ET.Element, inherited: dict[str, dict[str, str]], out: Classes
) -> None:
    """Flatten nested ``<default>`` into class name -> tag -> attributes."""
    here = {tag: dict(attrs) for tag, attrs in inherited.items()}
    for child in default:
        if child.tag != "default":
            here.setdefault(child.tag, {}).update(child.attrib)
    out[default.get("class", "main")] = here
    for child in default:
        if child.tag == "default":
            _class_defaults(child, here, out)


def _apply(element: ET.Element, classes: Classes, active: str) -> None:
    cls = element.get("class", active)
    defaults = classes[cls].get(element.tag, {})
    for key, value in defaults.items():
        element.attrib.setdefault(key, value)
    element.attrib.pop("class", None)
    child_active = element.attrib.pop("childclass", None) or active
    for child in element:
        _apply(child, classes, child_active if element.tag == "body" else cls)


def flatten(xarm_xml: Path) -> ET.Element:
    """The xArm7 MJCF with defaults resolved, absolute asset paths and no keyframe."""
    root = ET.parse(xarm_xml).getroot()
    compiler = root.find("compiler")
    meshdir = xarm_xml.parent / (compiler.get("meshdir", "") if compiler is not None else "")
    worldbody = root.find("worldbody")
    assert worldbody is not None
    classes: Classes = {}
    for default in root.findall("default"):
        _class_defaults(default, {}, classes)
        root.remove(default)
    classes.setdefault("main", {})
    # Body joints/geoms/sites and actuators take class defaults; the <joint> entries of
    # tendons and equalities are references, not joints, and take none.
    for section in ("worldbody", "actuator"):
        for element in root.findall(section):
            for child in element:
                _apply(child, classes, "main")
    asset = root.find("asset")
    if asset is not None:
        for item in asset:
            file = item.get("file")
            if file:
                path = (meshdir / file).resolve()
                item.set("file", str(path))
                if item.tag == "mesh":
                    item.attrib.setdefault("name", path.stem)
    for joint in worldbody.iter("joint"):
        for key, value in _JOINT_EXPLICIT.items():
            joint.attrib.setdefault(key, value)
    # The xArm relies on compiler autolimits, which LIBERO's world does not set.
    for element in root.iter():
        for range_attr, flag in (
            ("range", "limited"),
            ("ctrlrange", "ctrllimited"),
            ("forcerange", "forcelimited"),
        ):
            if element.get(range_attr) is not None and element.tag != "default":
                element.attrib.setdefault(flag, "true")
    # robosuite only auto-names geoms in groups 0/1; the xArm's are 2 (visual) and 3.
    for body in root.iter("body"):
        for i, geom in enumerate(body.findall("geom")):
            geom.attrib.setdefault("name", f"{body.get('name')}_geom{i}")
    for tag in ("keyframe", "compiler", "option"):
        for element in root.findall(tag):
            root.remove(element)
    # Angles are radians, as in robosuite's world, so the file also stands on its own.
    root.insert(0, ET.Element("compiler", angle="radian"))
    # robosuite reads the root body's pos as the robot's base offset; dimos's file stands
    # the arm on a 0.12 m pedestal, which would sink the base below where the Panda's was.
    base = worldbody.find("body")
    assert base is not None
    base.set("pos", "0 0 0")
    # robosuite reads the end effector from a body named right_hand; put it at the tool point.
    for hand in root.iter("body"):
        tcp = hand.find("site[@name='link_tcp']")
        if tcp is not None:
            ET.SubElement(hand, "body", name=_EEF, pos=tcp.get("pos", "0 0 0"))
            break
    return root


def register(xarm_xml: Path, out_dir: Path, base_forward_m: float = 0.0) -> None:
    """Define LIBERO's ``MountedXArm7`` / ``OnTheGroundXArm7`` from ``xarm_xml``.

    The arm stands where LIBERO puts the Panda, moved ``base_forward_m`` towards the
    workspace: the xArm7 reaches ~0.70 m to the Panda's ~0.85 m.
    """
    # LIBERO's runtime only; these exist in its environment, not dimos's.
    from libero.libero.envs.robots.mounted_panda import (  # type: ignore[import-not-found]
        MountedPanda,
    )
    from libero.libero.envs.robots.on_the_ground_panda import (  # type: ignore[import-not-found]
        OnTheGroundPanda,
    )
    from robosuite.models.robots.manipulators.manipulator_model import (  # type: ignore[import-not-found]
        ManipulatorModel,
    )
    from robosuite.robots import ROBOT_CLASS_MAPPING  # type: ignore[import-not-found]
    from robosuite.robots.single_arm import SingleArm  # type: ignore[import-not-found]

    out_dir.mkdir(parents=True, exist_ok=True)
    robot_xml = out_dir / "xarm7_robot.xml"
    ET.ElementTree(flatten(xarm_xml)).write(robot_xml, encoding="unicode")
    looks = {
        geom.get("name"): {k: geom.get(k) for k in ("rgba", "material") if geom.get(k) is not None}
        for geom in ET.parse(robot_xml).getroot().iter("geom")
    }
    n_joints = sum(1 for _ in ET.parse(robot_xml).getroot().iterfind("worldbody//joint"))

    def make(name: str, panda: type) -> type:
        # The Panda's own placement: same mount and base offsets in every LIBERO arena.
        reference = panda()
        mount = _visual_mount(reference.default_mount)
        offsets = {
            arena: _forward(offset, base_forward_m)
            for arena, offset in reference.base_xpos_offset.items()
        }

        class XArm7(ManipulatorModel):  # type: ignore[misc]
            def __init__(self, idn: int | str = 0) -> None:
                super().__init__(str(robot_xml), idn=idn)
                # robosuite paints group-0 geoms as contact geoms; keep the xArm's looks.
                for geom in self.worldbody.iter("geom"):
                    original = looks.get(geom.get("name", "").removeprefix(self.naming_prefix))
                    if original is not None:
                        geom.attrib.pop("rgba", None)
                        geom.attrib.pop("material", None)
                        geom.attrib.update(
                            {
                                k: self.naming_prefix + v if k == "material" else v
                                for k, v in original.items()
                            }
                        )

            @property
            def default_mount(self) -> str | None:
                return mount

            @property
            def default_gripper(self) -> None:
                return None

            @property
            def default_controller_config(self) -> str:
                return "default_panda"

            @property
            def init_qpos(self) -> np.ndarray:
                return np.array([*HOME, *([0.0] * (n_joints - len(HOME)))])

            @property
            def base_xpos_offset(self) -> dict[str, Any]:
                return offsets

            @property
            def top_offset(self) -> np.ndarray:
                return np.array((0, 0, 1.0))

            @property
            def _horizontal_radius(self) -> float:
                return 0.5

            @property
            def arm_type(self) -> str:
                return "single"

        # robosuite registers robot models by class name when the class is created.
        return type(name, (XArm7,), {})

    for name, panda in (("MountedXArm7", MountedPanda), ("OnTheGroundXArm7", OnTheGroundPanda)):
        make(name, panda)
        ROBOT_CLASS_MAPPING[name] = SingleArm


def _forward(offset: Any, metres: float) -> Any:
    """A LIBERO base offset (a tuple, or a function of the table length) moved along +x."""
    if callable(offset):
        return lambda length: _forward(offset(length), metres)
    x, y, z = offset
    return (x + metres, y, z)


def _visual_mount(name: str | None) -> str | None:
    """The Panda's mount without collisions: moved forward, its pedestal can clip fixtures
    near the table edge (e.g. LIBERO-Spatial's stove knob) and must not move them."""
    if name is None:
        return None
    from robosuite.models.mounts import MOUNT_MAPPING  # type: ignore[import-not-found]

    base = MOUNT_MAPPING[name]

    class VisualMount(base):  # type: ignore[misc, valid-type]
        def __init__(self, idn: int | str = 0) -> None:
            super().__init__(idn=idn)
            for geom in self.worldbody.iter("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")

    visual = f"{name}NoCollision"
    MOUNT_MAPPING[visual] = VisualMount
    return visual
