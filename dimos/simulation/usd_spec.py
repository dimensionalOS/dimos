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

"""Reading a robot into MuJoCo from the one USD that describes it."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
import sys
from typing import Any

import mujoco

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def load_usd_spec(path: Path) -> mujoco.MjSpec:
    """An editable spec of the robot in ``path``, named the way an MJCF would have named it.

    MuJoCo reads the layer through its own USD decoder, so actuators, joint settings and
    collision groups arrive as the mjcPhysics prims they were written as -- nothing is
    round-tripped through URDF.

    It also names every element after its **full prim path**, so the G1's pelvis arrives as
    `/g1_29dof_rev_1_0/Geometry/pelvis`. Every `mj_name2id(model, BODY, "pelvis")` in the
    simulators would then return -1, which is a body index of nothing rather than an error --
    silent, and the kind of thing that shows up as a robot that will not stand. So the names
    are put back to their last path component here, once, and the rest of the sim code reads
    exactly as it did when the robot was an MJCF.
    """
    try:
        # Importing it registers MuJoCo's USD decoder plugin with the resolver. Without it
        # MjSpec.from_file refuses the layer outright: "could not find decoder for resource".
        import mujoco._usd
    except ImportError as error:
        raise RuntimeError(
            f"reading {path.name} needs MuJoCo's USD decoder, which comes from mujoco-usd and "
            f"wants Python 3.11 or newer (this is {sys.version_info.major}."
            f"{sys.version_info.minor}). The robots that ship 3.10 do not run the simulator."
        ) from error

    spec = mujoco.MjSpec.from_file(str(path))
    shorten_names(spec)
    return spec


def _shortened(elements: Iterable[Any], kind: str) -> dict[str, str]:
    """Rename each element to the last component of its prim path; the old name -> the new one.

    A collision stops the rename for the whole kind rather than for one element. A half renamed
    kind is worse than a consistently long one: a lookup would work for some names and not
    others, with nothing to say which.
    """
    named = [element for element in elements if element.name]
    was = [element.name for element in named]
    short = [name.rsplit("/", 1)[-1] for name in was]
    if len(set(short)) != len(short):
        logger.warning(f"left {kind} names as prim paths: two would collide once shortened")
        return {}
    for element, name in zip(named, short, strict=True):
        element.name = name
    return dict(zip(was, short, strict=True))


def shorten_names(spec: mujoco.MjSpec) -> None:
    """Rename every element to the last component of its prim path, references included.

    MjSpec holds its cross-references as **strings** -- a geom names its mesh and its material,
    a material names its textures, an actuator names its joint, a contact exclusion names two
    bodies -- so renaming without rewriting them fails at compile with "mesh ... not found in
    geom 0" or "body ... not found in bodypair 0".
    """
    meshes = _shortened(spec.meshes, "mesh")
    targets = {**_shortened(spec.joints, "joint"), **_shortened(spec.sites, "site")}
    bodies = _shortened(spec.bodies, "body")
    textures = _shortened(spec.textures, "texture")
    materials = _shortened(spec.materials, "material")
    _shortened(spec.geoms, "geom")
    _shortened(spec.actuators, "actuator")

    for geom in spec.geoms:
        if geom.meshname in meshes:
            geom.meshname = meshes[geom.meshname]
    # A material is looked up by name as much as a body is: the M20's sim paints its collision
    # shapes by asking the spec for `collision_material`, and a prim path there answers None.
    for element in (*spec.geoms, *spec.sites):
        if element.material in materials:
            element.material = materials[element.material]
    for material in spec.materials:
        material.textures = [textures.get(name, name) for name in material.textures]
    for actuator in spec.actuators:
        if actuator.target in targets:
            actuator.target = targets[actuator.target]
    # The M20 excludes 16 body pairs from contact; the G1 excludes none.
    for exclude in spec.excludes:
        for attribute in ("bodyname1", "bodyname2"):
            name = getattr(exclude, attribute)
            if name in bodies:
                setattr(exclude, attribute, bodies[name])
