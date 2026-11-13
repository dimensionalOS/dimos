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


"""Load the composed USD directly with MuJoCo's native USD importer."""

from importlib import import_module
from pathlib import Path

import mujoco

from dimos.robot.assets.client import materialize_scene
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import layer_from, load_pxr
from dimos.robot.assets.usd_assets import resolve_usd_assets


def build_scene(
    scene: UsdModel,
    dt_ns: int = 2_000_000,
    *,
    kinematic: bool = False,
    gravity_world: tuple[float, float, float] = (0, 0, -9.81),
) -> mujoco.MjSpec:
    if not 0 < dt_ns <= 20_000_000:
        raise ValueError("dt must be in (0, .02] seconds")
    try:
        import_module("mujoco._usd")
    except ImportError as exc:
        raise ImportError(
            "Sim2 requires MuJoCo's USD importer: install dimos[sim] on Python 3.11+"
        ) from exc
    root = str(scene.source_path.parent) if scene.source_path else ""
    usd = resolve_usd_assets(scene.usd, root)
    if kinematic:
        pxr = load_pxr()
        stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "sensor geometry", root or None))
        # Removing joints makes the importer produce independent bodies. Keep
        # the exact geometry and body IDs; observations provide their poses.
        remove = []
        for prim in list(stage.Traverse()):
            if not prim:
                continue
            if prim.IsA(pxr.UsdPhysics.Joint) or str(prim.GetTypeName()).startswith(
                ("MjcActuator", "MjcSensor", "MjcTendon", "MjcKeyframe")
            ):
                remove.append(prim.GetPath())
            elif prim.IsA(pxr.UsdGeom.Gprim):
                group = prim.GetAttribute("mjc:group").Get()
                if pxr.UsdGeom.Imageable(
                    prim
                ).ComputeVisibility() == pxr.UsdGeom.Tokens.invisible or (
                    group is not None and group >= 3
                ):
                    # Query scenes contain optical surfaces. Hidden collision
                    # hulls remain in physics, but need no render/raycast resources.
                    remove.append(prim.GetPath())
        # Deactivation also hides geometry contributed by referenced layers;
        # deleting only the local spec would expose that weaker geometry again.
        with pxr.Sdf.ChangeBlock():
            for path in remove:
                pxr.Sdf.CreatePrimInLayer(stage.GetRootLayer(), path).active = False
        usd = str(stage.GetRootLayer().ExportToString())
    local = materialize_scene(usd, lambda name: Path(name).read_bytes())
    spec = mujoco.MjSpec.from_file(str(local.source_path))
    _bind_names(spec, local)
    spec.option.timestep = dt_ns / 1e9
    spec.option.gravity = gravity_world
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.compiler.fusestatic = False
    if not kinematic:
        return spec
    for kind in (
        "actuators",
        "sensors",
        "tendons",
        "equalities",
        "excludes",
        "pairs",
        "keys",
        "joints",
    ):
        for element in list(getattr(spec, kind)):
            spec.delete(element)
    for body in list(spec.bodies)[1:]:
        body.mocap = True
    return spec


def _bind_names(spec: mujoco.MjSpec, scene: UsdModel) -> None:
    """Bind native prim paths to the exact frame IDs published by UsdManager."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(str(scene.source_path))
    names = {}
    for kind in ("bodies", "joints", "sites", "cameras"):
        used: set[str] = set()
        for element in getattr(spec, kind):
            old = element.name
            if not old:
                continue
            prim = stage.GetPrimAtPath(old) if old.startswith("/") else None
            name = str(prim.GetDisplayName() or prim.GetName()) if prim else old
            if name in used:
                raise ValueError(f"duplicate USD {kind} ID: {name}")
            used.add(name)
            names[old] = name
            element.name = name
    # Follow references when replacing importer-generated paths, as in Jeff's loader.
    for kind, fields in {
        "actuators": ("target", "refsite", "slidersite"),
        "sensors": ("objname", "refname"),
        "equalities": ("name1", "name2"),
        "excludes": ("bodyname1", "bodyname2"),
    }.items():
        for element in getattr(spec, kind):
            for field in fields:
                value = getattr(element, field)
                if value in names:
                    setattr(element, field, names[value])
    for actuator in spec.actuators:
        actuator.name = actuator.target


def compile_scene(
    scene: UsdModel,
    dt_ns: int = 2_000_000,
    *,
    gravity_world: tuple[float, float, float] = (0, 0, -9.81),
) -> mujoco.MjModel:
    return build_scene(scene, dt_ns, gravity_world=gravity_world).compile()
