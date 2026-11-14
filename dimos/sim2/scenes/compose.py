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

"""Put the robot of a sim2 blueprint in another environment.

A sim2 blueprint serves one USD scene that holds its robot and the surroundings
it was authored with. This takes the robot out of that scene (the part that
holds the joints the simulation drives, with its drives and collision rules as
authored) and stands it in an environment that holds no robot. No robot states
anything for this: the blueprint already names its scene and its joints.

    python -m dimos.sim2.scenes.compose --blueprint r1pro-openarm-joylo-sim2 \\
        --environment ~/scenes/kitchen/scene.usda --place 1.2 0.4 90 --output ~/kitchen-r1.usda
    dimos run r1pro-openarm-joylo-sim2 --usdmanager.model ~/kitchen-r1.usda ...

Offline authoring, like ``urdf.py`` and ``mjcf.py``: nothing here runs in a simulation.
"""

import argparse
from collections.abc import Sequence
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.pxr import frame_name, layer_from, load_pxr, usd_matrix
from dimos.robot.get_all_blueprints import get_by_name
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule


def robot_in_environment(
    robot_scene: UsdModel,
    joint_ids: Sequence[str],
    environment: UsdModel,
    *,
    place: tuple[float, float, float] = (0.0, 0.0, 0.0),
    heading: float = 0.0,
) -> UsdModel:
    """The robot of ``robot_scene`` standing in ``environment``.

    Args:
        robot_scene: A scene that holds the robot, as its blueprint serves it.
        joint_ids: Joints the simulation drives; the part of the scene that holds
            them is the robot. Everything else in that scene is left behind.
        environment: A scene with no robot. Everything in it is kept.
        place: Where the robot stands: x, y and the floor's height there, in metres.
            The robot keeps the height above the floor it has in its own scene.
        heading: The robot's turn about the vertical axis, in radians. It replaces
            the orientation the robot has in its own scene.

    Raises:
        ValueError: The driven joints are in no single part of the robot scene, or
            a part of the environment has the robot's name.
    """
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(_layer(robot_scene, "robot scene"))
    world = stage.GetDefaultPrim()
    wanted = set(joint_ids)
    holders = [
        child
        for child in world.GetChildren()
        if wanted & {frame_name(prim) for prim in pxr.Usd.PrimRange(child)}
    ]
    if len(holders) != 1:
        raise ValueError(
            f"the driven joints are in {len(holders)} parts of the robot scene; "
            f"the robot must be the one part that holds them"
        )
    robot = holders[0]
    for child in world.GetChildren():
        if child != robot:
            stage.RemovePrim(child.GetPath())

    surroundings = _layer(environment, "environment")
    # Held in a name: the stage's prims expire with it.
    other = pxr.Usd.Stage.Open(surroundings)
    for child in other.GetDefaultPrim().GetChildren():
        if child.GetName() == robot.GetName():
            raise ValueError(f"the environment has a part named {child.GetName()!r}, as the robot")
        # Both scenes keep their parts directly under the default prim, so a part
        # keeps its path and every relationship inside it stays valid.
        target = world.GetPath().AppendChild(child.GetName())
        pxr.Sdf.CopySpec(surroundings, child.GetPath(), stage.GetRootLayer(), target)
        if child.GetPath() != target:
            for prim in pxr.Usd.PrimRange(stage.GetPrimAtPath(target)):
                for relationship in prim.GetRelationships():
                    targets = relationship.GetTargets()
                    if targets:
                        relationship.SetTargets(
                            [path.ReplacePrefix(child.GetPath(), target) for path in targets]
                        )

    xform = pxr.UsdGeom.Xformable(robot)
    before = np.array(xform.GetLocalTransformation()).T
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("z", heading).as_matrix()
    pose[:3, 3] = (place[0], place[1], place[2] + before[2, 3])
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == pxr.UsdGeom.XformOp.TypeTransform:
            op.Set(usd_matrix(pxr, pose))
            break
    else:
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(usd_matrix(pxr, pose))
    # A fixed-base robot is held by a joint to the world, which states the same pose.
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    for prim in robot.GetChildren():
        if (
            prim.IsA(pxr.UsdPhysics.FixedJoint)
            and not pxr.UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()
        ):
            joint = pxr.UsdPhysics.Joint(prim)
            joint.GetLocalPos0Attr().Set(pxr.Gf.Vec3f(*(float(v) for v in pose[:3, 3])))
            joint.GetLocalRot0Attr().Set(
                pxr.Gf.Quatf(float(quaternion[3]), *(float(v) for v in quaternion[:3]))
            )
    return UsdModel.from_layer(str(stage.GetRootLayer().ExportToString()))


def blueprint_robot(blueprint: Blueprint) -> tuple[UsdModel, tuple[str, ...]]:
    """The scene a sim2 blueprint serves and the joints its simulation drives."""
    scene, joints = None, None
    for atom in blueprint.active_blueprints:
        if issubclass(atom.module, UsdManager) and isinstance(atom.kwargs.get("model"), UsdModel):
            scene = atom.kwargs["model"]
        if issubclass(atom.module, SimModule):
            joints = tuple(atom.kwargs.get("joint_ids", ()))
    if scene is None or not joints:
        raise ValueError(
            "not a sim2 blueprint with a robot: it needs a UsdManager that serves a "
            "USD scene and a SimModule that drives joints"
        )
    return scene, joints


def _layer(scene: UsdModel, what: str) -> Any:
    source = scene.source_path
    return layer_from(load_pxr(), scene.usd, what, None if source is None else source.parent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--blueprint", required=True, help="a sim2 blueprint, by its run name")
    parser.add_argument("--environment", required=True, type=Path, help="a USD scene with no robot")
    parser.add_argument("--output", required=True, type=Path, help="the scene file to write")
    parser.add_argument(
        "--place",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "HEADING"),
        help="where the robot stands: metres, metres, degrees",
    )
    parser.add_argument("--floor", type=float, default=0.0, help="the floor's height there, metres")
    args = parser.parse_args()
    own_scene, driven = blueprint_robot(get_by_name(args.blueprint))
    composed = robot_in_environment(
        own_scene,
        driven,
        UsdModel.from_file(args.environment.expanduser()),
        place=(args.place[0], args.place[1], args.floor),
        heading=math.radians(args.place[2]),
    )
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(composed.usd)
    print(output)
