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


from collections.abc import Iterable
from pathlib import Path
from typing import Any

import mujoco

from dimos.sim2.mujoco.names import resolved, shorten
from dimos.sim2.sim import SimulatedRobot, validate

USD_SUFFIXES = frozenset({".usd", ".usda", ".usdc", ".usdz"})


def _register_usd_reader(scene: Path) -> None:
    if scene.suffix.lower() not in USD_SUFFIXES:
        return
    try:
        import mujoco._usd  # noqa: F401
    except ImportError as error:
        raise ImportError(
            f"{scene} is a USD layer and this MuJoCo cannot read one: {error}"
        ) from error


def _sliding_friction(layer: Path) -> dict[str, float]:
    """Sliding friction per prim path from each collider's (or its nearest ancestor's) bound physics material, which MuJoCo's USD reader drops."""
    if layer.suffix.lower() not in USD_SUFFIXES:
        return {}
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    opened = pxr.Sdf.Layer.FindOrOpen(str(layer))
    if opened is None:
        return {}
    materials: dict[str, float] = {}
    bound: dict[str, str] = {}
    stack: list[tuple[Any, str | None]] = [(opened.pseudoRoot, None)]
    while stack:
        spec, inherited = stack.pop()
        attribute = spec.properties.get("physics:dynamicFriction") if spec.properties else None
        if attribute is not None and attribute.default is not None:
            materials[str(spec.path)] = float(attribute.default)
        binding = spec.relationships.get("material:binding:physics") if spec.relationships else None
        targets = binding.targetPathList.explicitItems if binding is not None else []
        here = str(targets[0]) if targets else inherited
        if here is not None:
            bound[str(spec.path)] = here
        stack.extend((child, here) for child in spec.nameChildren)
    return {path: materials[material] for path, material in bound.items() if material in materials}


def _apply_sliding_friction(spec: mujoco.MjSpec, layer: Path) -> int:
    """Give every geom the friction its layer declares. A geom MuJoCo named by prim path."""
    friction = _sliding_friction(layer)
    if not friction:
        return 0
    applied = 0
    for geom in spec.geoms:
        wanted = friction.get(geom.name)
        if wanted is None or wanted == geom.friction[0]:
            continue
        geom.friction = (wanted, geom.friction[1], geom.friction[2])
        applied += 1
    return applied


def _drop_spawn_volumes(spec: mujoco.MjSpec, layer: Path) -> None:
    """Leave out the spawn points' volumes: guides for what fits a spot, which neither collide nor render."""
    if layer.suffix.lower() not in USD_SUFFIXES:
        return
    from dimos.sim2.scene import spawn_volume_paths

    volumes = spawn_volume_paths(layer)
    for geom in [geom for geom in spec.geoms if geom.name in volumes]:
        spec.delete(geom)


def compile_world(
    scene: Path, simulated: Iterable[SimulatedRobot] = (), ids: Iterable[str] | None = None
) -> mujoco.MjSpec:
    """The world's MjSpec: what physics needs, so no lights or looks (a view adds those)."""
    _register_usd_reader(Path(scene))
    robots = list(simulated)
    validate(robots)
    owners = [robot.robot_id for robot in robots] if ids is None else list(ids)

    world = mujoco.MjSpec.from_file(str(scene))
    _apply_sliding_friction(world, Path(scene))
    _drop_spawn_volumes(world, Path(scene))
    shorten(world, ids=owners)
    for robot in robots:
        standing = resolved(
            [body.name for body in world.bodies], f"{robot.robot_id}/{robot.root_body}"
        )
        if standing is None:
            raise ValueError(
                f"{robot.robot_id}: no body {robot.root_body!r} in the world; a robot is in the "
                f"world by being mounted, so mount it onto a frame of the scene"
            )
    for key in list(world.keys) if robots else ():
        world.delete(key)
    fixed = {robot.robot_id for robot in robots if not robot.free}
    for robot_id in owners:
        if robot_id not in fixed:
            _free_if_unjointed(world, robot_id)
    return world


def _free_if_unjointed(spec: mujoco.MjSpec, robot_id: str) -> None:
    prefix = f"{robot_id}/"
    for body in list(spec.bodies):
        name = body.name or ""
        parent = body.parent
        if not name.startswith(prefix) or (parent.name or "").startswith(prefix):
            continue
        holder = body
        while holder.parent is not None and (holder.parent.name or "") != "world":
            holder = holder.parent
        if not any(part.joints for part in _chain(body, holder)):
            holder.add_freejoint()


def _chain(body: Any, holder: Any) -> list[Any]:
    parts = [body]
    while parts[-1] is not holder and parts[-1].parent is not None:
        parts.append(parts[-1].parent)
    return parts
