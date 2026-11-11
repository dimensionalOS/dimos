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
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.mujoco.names import resolved, shorten
from dimos.sim2.spec import Simulated, SimulatedRobot, Vector, validate
from dimos.sim2.usd_layer import usd_layer

_SITE_SIZE = (0.001, 0.001, 0.001)


@dataclass(frozen=True)
class SensorMount:
    kind: Literal["camera", "lidar", "imu"]
    frame_id: str
    name: str
    fovy: float = 60.0


_USD_SUFFIXES = frozenset({".usd", ".usda", ".usdc", ".usdz"})


def _register_usd_reader(scene: Path) -> None:
    if scene.suffix.lower() not in _USD_SUFFIXES:
        return
    try:
        import mujoco._usd  # noqa: F401
    except ImportError as error:
        raise ImportError(
            f"{scene} is a USD layer and this MuJoCo cannot read one: {error}"
        ) from error


def _sliding_friction(layer: Path) -> dict[str, float]:
    """Sliding friction per prim path, from the physics material each collider is bound to.

    MuJoCo's USD reader brings a geom's priority, condim, group and solref across, and a joint's
    armature and frictionloss, but it does not read sliding friction from USD at all: every geom
    arrives at MuJoCo's own default of 1, whatever the layer says. The G1's feet are declared at
    0.6 -- four spheres per foot, and the only priority=1 geoms it has, so theirs is the friction
    that decides whether a foot slides -- and a foot at 1.0 is a different robot to walk on.

    The number is in the layer, on a `PhysicsMaterialAPI` material the collider is bound to, so
    this reads it off the layer and puts it back. Bindings inherit, as USD binds them: a prim
    with no binding of its own takes its nearest ancestor's.
    """
    if layer.suffix.lower() not in _USD_SUFFIXES:
        return {}
    from dimos.robot.assets.usd import load_pxr

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


def compile_world(
    scene: Path, simulated: Iterable[Simulated] = (), ids: Iterable[str] | None = None
) -> mujoco.MjSpec:
    _register_usd_reader(Path(scene))
    robots = [d for d in simulated if isinstance(d, SimulatedRobot)]
    validate(robots)
    owners = [robot.robot_id for robot in robots] if ids is None else list(ids)

    world = mujoco.MjSpec.from_file(str(scene))
    _apply_sliding_friction(world, Path(scene))
    shorten(world, ids=owners)
    for robot in robots:
        prefix = f"{robot.robot_id}/"
        standing = resolved([body.name for body in world.bodies], prefix + robot.root_body)
        if standing is not None:
            _configure(world, robot, root=world.body(standing), prefix=prefix)
            continue
        layer = usd_layer(robot.model)
        _register_usd_reader(layer)
        spec = mujoco.MjSpec.from_file(str(layer))
        _apply_sliding_friction(spec, layer)
        shorten(spec)
        robot_geoms = {geom.name for geom in spec.geoms}
        root_name = resolved([body.name for body in spec.bodies], robot.root_body)
        if root_name is None:
            raise ValueError(f"{robot.robot_id}: no body {robot.root_body!r} in {layer}")
        root = spec.body(root_name)
        root.pos = (0.0, 0.0, 0.0)
        root.quat = (1.0, 0.0, 0.0, 0.0)
        _configure(spec, robot, root=root, prefix="", layer=layer)
        frame = world.worldbody.add_frame(pos=robot.xyz, quat=_quaternion(robot.rpy))
        world.attach(spec, prefix=prefix, frame=frame)
        for pair in world.pairs:
            for side in ("geomname1", "geomname2"):
                name = getattr(pair, side).removeprefix(prefix)
                if getattr(pair, side) != name and name not in robot_geoms:
                    setattr(pair, side, name)
    fixed = {robot.robot_id for robot in robots if not robot.floating}
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


def _configure(
    spec: mujoco.MjSpec,
    robot: SimulatedRobot,
    *,
    root: Any,
    prefix: str,
    layer: Path | None = None,
) -> None:
    if not robot.floating:
        root.mocap = True
    for key in list(spec.keys):
        spec.delete(key)


def mount_sensor(spec: mujoco.MjSpec, sensor: SensorMount) -> None:
    frame = sensor.frame_id
    site, body, pos, quat = None, spec.worldbody, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    if frame != "world":
        site_name = resolved([s.name for s in spec.sites], frame)
        body_name = resolved([b.name for b in spec.bodies], frame)
        site = spec.site(site_name) if site_name else None
        body = spec.body(body_name) if body_name else None
        if site is not None:
            compiled = spec.copy().compile()
            index = compiled.site(site_name).id
            body, pos, quat = site.parent, compiled.site_pos[index], compiled.site_quat[index]
        elif body is None:
            raise ValueError(
                f"no site or body {frame!r} to mount a {sensor.kind} on; this world has "
                f"{sorted(b.name for b in spec.bodies)[:8]}..."
            )
    name = sensor.name
    if sensor.kind == "camera":
        optical = Rotation.from_quat(quat, scalar_first=True) * Rotation.from_euler("x", np.pi)
        body.add_camera(
            name=name, fovy=sensor.fovy, pos=pos, quat=optical.as_quat(scalar_first=True)
        )
    elif resolved([s.name for s in spec.sites], name) != name:
        body.add_site(name=name, size=_SITE_SIZE, rgba=(0, 0, 0, 0), pos=pos, quat=quat)


def _quaternion(rpy: Vector) -> NDArray[np.float64]:
    return np.asarray(Rotation.from_euler("xyz", rpy).as_quat(scalar_first=True), dtype=np.float64)
