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

"""The scene loader: a scene USD, its spawn points (DimosSpawnAPI prims), and overlaying every HardwareModule's USD onto it."""

from collections.abc import AsyncIterator, Callable, Iterable, Iterator, Sequence
from dataclasses import dataclass
from functools import cached_property
import hashlib
from pathlib import Path
import tempfile
from typing import Any

import numpy as np
from pydantic import Field, model_validator

from dimos.constants import CACHE_DIR
from dimos.core.hardware_module import HardwareModuleConfig, Mount, Mounts
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.lfs import pull_usd_assets
from dimos.robot.assets.usd_utils.pxr import (
    Layer,
    Prim,
    Pxr,
    Stage,
    frame_name,
    layer_from,
    load_pxr,
    prim_identifier,
)
from dimos.sim2.sim import SimSensor
from dimos.utils.logging_config import setup_logger

# Layer files a stage or engine can open.

logger = setup_logger()

# Under this user's cache, not the shared temp dir: these files are loaded as scene content.
_CACHE = CACHE_DIR / "sim2-usd"


def usd_layer(model: UsdModel, transform: Callable[[str], str] | None = None) -> Path:
    """Write the model out as a layer file, optionally rewritten for whoever is about to read it."""
    from dimos.robot.assets.usd_utils.assets import resolve_usd_assets

    root = model.asset_root
    layer = resolve_usd_assets(model.usd, str(root) if root is not None else "")
    # Before the transform opens a stage of it, and as MuJoCo reads a Git LFS pointer as an empty mesh.
    pull_usd_assets([layer])
    if transform is not None:
        layer = transform(layer)
    return layer_file(layer, called=_name_of(model))


def layer_file(layer: str, called: str = "layer") -> Path:
    out = _CACHE / f"{called}-{hashlib.sha256(layer.encode()).hexdigest()[:16]}.usda"
    if not out.exists():
        _write_whole(out, layer.encode())
    return out


def _write_whole(out: Path, data: bytes) -> None:
    """Write ``out`` so a reader sees all of it or none; each writer, thread or process, has its own partial."""
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=out.parent, prefix=f"{out.name}.", suffix=".partial", delete=False
    ) as partial:
        partial.write(data)
    Path(partial.name).replace(out)


def _name_of(model: UsdModel) -> str:
    source = model.source_path
    return source.stem if source is not None else "layer"


# Overlaying namespaced layers into one world: each already sits at its tf frames.


def overlay_usds(root_usd: str, mounted: Sequence[tuple[str, str]]) -> str:
    """The root layer with each namespaced (usd, mount frame) layer flattened in at its frames: the root's opinions and metadata win, only its PhysicsScene survives, and each mounted rigid part is welded to its mount."""
    from pxr import UsdUtils

    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "the root", None)
    # Kept in a name: a sublayer is found by its identifier, and an anonymous layer lives only as long as its handle.
    mounted_layers = [(_without_physics_scenes(pxr, usd), frame) for usd, frame in mounted]
    root_layer.subLayerPaths = [layer.identifier for layer, _ in mounted_layers]
    flattened = UsdUtils.FlattenLayerStack(pxr.Usd.Stage.Open(root_layer))
    world = pxr.Usd.Stage.Open(flattened)
    for layer, frame in mounted_layers:
        _weld_to_mount(pxr, world, pxr.Usd.Stage.Open(layer), frame)
    return str(flattened.ExportToString())


def _without_physics_scenes(pxr: Pxr, usd: str) -> Layer:
    # A mounted part's PhysicsScene would be a second one for the world, so it goes.
    layer = layer_from(pxr, usd, "a mounted part", None)
    stage = pxr.Usd.Stage.Open(layer)
    for path in [prim.GetPath() for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Scene)]:
        stage.RemovePrim(path)
    return layer


def _weld_to_mount(pxr: Pxr, world: Stage, part: Stage, mount_frame: str) -> None:
    mount_path = pxr.Sdf.Path(f"/{mount_frame}")
    root_link = _first_link(pxr, part, mount_path)
    if root_link is None:
        return
    # The part's own top prim, the child of its mount on the way to its first link.
    top = next(
        path for path in root_link.GetPath().GetPrefixes() if path.GetParentPath() == mount_path
    )
    name = prim_identifier(pxr, f"{mount_frame}_mount")
    joint = pxr.UsdPhysics.Joint.Define(world, top.AppendChild(name))
    joint.CreateBody0Rel().SetTargets([mount_path])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: Any) -> Prim:
    root = stage.GetPrimAtPath(path)
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None


# Spawn points: a scene's ``DimosSpawnAPI`` prims (schema/generatedSchema.usda), each a pose plus the guide-purpose volume it accepts.

SPAWN_API = "DimosSpawnAPI"
_SCHEMA = Path(__file__).parent / "schema"
# Volumes are float scales, so a part exactly as big as its spot may overshoot it by float32 rounding.
_FIT_SLACK = float(np.finfo(np.float32).eps)


@dataclass(frozen=True)
class SpawnPoint:
    """A spawn point: its frame (its prim path without the leading "/") and the box it accepts, in its own frame."""

    frame: str
    low: tuple[float, float, float]
    high: tuple[float, float, float]

    @property
    def name(self) -> str:
        return self.frame.rpartition("/")[2]


def register_spawn_schema() -> None:
    """Register DimosSpawnAPI with USD, so ``prim.HasAPI("DimosSpawnAPI")`` works; it has to run before the process opens its first stage."""
    from pxr import Plug

    Plug.Registry().RegisterPlugins(str(_SCHEMA))


def is_spawn_point(prim: Prim) -> bool:
    """Whether ``prim`` applies DimosSpawnAPI, read off its apiSchemas, so it holds whether or not the schema was registered in time."""
    if SPAWN_API in prim.GetAppliedSchemas():
        return True
    listed = prim.GetMetadata("apiSchemas")
    return listed is not None and SPAWN_API in listed.ApplyOperations([])


def spawn_points(stage: Stage) -> list[SpawnPoint]:
    """Every spawn point of ``stage``, each with the volume its guide children bound."""
    pxr = load_pxr()
    cache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), [pxr.UsdGeom.Tokens.guide])
    points = []
    for prim in stage.Traverse():
        if not is_spawn_point(prim):
            continue
        volumes = [
            child
            for child in prim.GetChildren()
            if child.IsA(pxr.UsdGeom.Gprim)
            and pxr.UsdGeom.Imageable(child).ComputePurpose() == pxr.UsdGeom.Tokens.guide
        ]
        if not volumes:
            raise ValueError(
                f"the spawn point {prim.GetPath()} has no volume: give it a guide-purpose "
                f"Cube, Sphere, Cylinder or Capsule child bounding what it accepts"
            )
        box = pxr.Gf.Range3d()
        for volume in volumes:
            box.UnionWith(cache.ComputeRelativeBound(volume, prim).ComputeAlignedRange())
        points.append(
            SpawnPoint(str(prim.GetPath())[1:], _triple(box.GetMin()), _triple(box.GetMax()))
        )
    return points


def scene_spawn_points(usd: str, root: Path | str | None = None) -> list[SpawnPoint]:
    """A scene's spawn points; a scene with none is refused."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a scene", root))
    points = spawn_points(stage)
    if not points:
        raise ValueError(
            f"a scene needs spawn points (prims with {SPAWN_API}) for its robots and objects "
            f"to start on, and {stage.GetDefaultPrim().GetPath() or 'this one'} has none"
        )
    return points


def spawn_frames_in_file(path: Path) -> list[str]:
    """The spawn points' frames in a scene file's own layer, read without composing it (so without opening a mesh)."""
    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files

    pull_lfs_files([path])
    pxr = load_pxr()
    layer = pxr.Sdf.Layer.FindOrOpen(str(path))
    if layer is None:
        raise ValueError(f"{path} is not a USD layer")
    return sorted(str(spec.path)[1:] for spec in _prim_specs(layer) if _spec_is_spawn(spec))


def spawn_volume_paths(layer: Path) -> set[str]:
    """The prim paths of every spawn point's volume in a layer file, read without composing it; a renderer or physics engine leaves these out."""
    pxr = load_pxr()
    opened = pxr.Sdf.Layer.FindOrOpen(str(layer))
    if opened is None:
        return set()
    return {
        str(child.path)
        for spec in _prim_specs(opened)
        if _spec_is_spawn(spec)
        for child in spec.nameChildren
        if child.attributes.get("purpose") is not None
        and child.attributes["purpose"].default == pxr.UsdGeom.Tokens.guide
    }


def with_spawn_frames(usd: str, root: Path | str | None = None) -> str:
    """``usd`` with each spawn point made a tf frame: a rigid body welded where it stands to the rigid body it is under."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    cache = pxr.UsdGeom.XformCache()
    for prim in stage.Traverse():
        if not is_spawn_point(prim) or prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            continue
        _weld_where_it_stands(pxr, stage, cache, prim)
    return str(layer.ExportToString())


def check_spawn_fits(root_usd: str, mounted: Sequence[tuple[str, str]]) -> None:
    """Refuse a mounted (usd, mount frame) whose bounding box does not fit in the spawn point it is mounted on."""
    pxr = load_pxr()
    spots = {
        point.frame: point
        for point in spawn_points(pxr.Usd.Stage.Open(layer_from(pxr, root_usd, "a scene", None)))
    }
    too_big = []
    for usd, frame in mounted:
        spot = spots.get(frame)
        if spot is None:
            continue
        size = _bound(pxr, usd)
        if size is None:
            continue
        low, high = size
        if any(
            low[i] < spot.low[i] - _FIT_SLACK or high[i] > spot.high[i] + _FIT_SLACK
            for i in range(3)
        ):
            too_big.append(
                f"{frame}: takes {_box(spot.low, spot.high)}, what is mounted there spans {_box(low, high)}"
            )
    if too_big:
        raise ValueError("too big for the spawn points they are on:\n  " + "\n  ".join(too_big))


def _bound(pxr: Pxr, usd: str) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
    """A namespaced USD's bounding box, which is in its mount's frame since its namespace prims carry no transform."""
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a mounted part", None))
    cache = pxr.UsdGeom.BBoxCache(
        pxr.Usd.TimeCode.Default(),
        [pxr.UsdGeom.Tokens.default_, pxr.UsdGeom.Tokens.render, pxr.UsdGeom.Tokens.proxy],
    )
    box = pxr.Gf.Range3d()
    for prim in stage.GetPseudoRoot().GetChildren():
        box.UnionWith(cache.ComputeWorldBound(prim).ComputeAlignedRange())
    if box.IsEmpty():
        return None
    return tuple(box.GetMin()), tuple(box.GetMax())


def _triple(vector: Any) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])


def _box(low: Iterable[float], high: Iterable[float]) -> str:
    return "x".join(f"{b - a:.2f}" for a, b in zip(low, high, strict=True)) + " m"


def _weld_where_it_stands(pxr: Pxr, stage: Stage, cache: Any, prim: Prim) -> None:
    body = prim.GetParent()
    while body and not body.IsPseudoRoot() and not body.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        body = body.GetParent()
    pxr.UsdPhysics.RigidBodyAPI.Apply(prim)
    joint = pxr.UsdPhysics.FixedJoint.Define(stage, prim.GetPath().AppendChild("spawn_joint"))
    if body and not body.IsPseudoRoot():
        joint.CreateBody0Rel().SetTargets([body.GetPath()])
        relative, _ = cache.ComputeRelativeTransform(prim, body)
    else:
        relative = cache.GetLocalToWorldTransform(prim)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    transform = pxr.Gf.Transform(relative)
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(transform.GetTranslation()))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(transform.GetRotation().GetQuat()))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _prim_specs(layer: Layer) -> Iterable[Any]:
    stack = list(layer.rootPrims)
    while stack:
        spec = stack.pop()
        yield spec
        stack.extend(spec.nameChildren)


def _spec_is_spawn(spec: Any) -> bool:
    listed = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
    return listed is not None and SPAWN_API in listed.ApplyOperations([])


register_spawn_schema()


# The scene a simulated world is built on: the one unmounted HardwareModule, a USD whose spawn points are its mounts.


def scene_usd(path: Path) -> UsdModel:
    """A scene file as a scene's USD: its Git LFS files fetched and each spawn point made a tf frame."""
    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files, pull_usd_assets

    pull_lfs_files([path])
    text = path.read_text()
    pull_usd_assets([text], root=path.parent)
    return UsdModel.from_layer(with_spawn_frames(text, path.parent), path)


class SpawnMounts(Mounts):
    """A scene file's spawn points as its mounts, read from the file the first time one is asked for."""

    def __init__(self, path: Path) -> None:
        self._path = path

    def _read(self) -> None:
        if "_frames" not in vars(self):
            Mounts.__init__(self, *spawn_frames_in_file(self._path))

    def __getattr__(self, name: str) -> Mount:
        if name.startswith("_"):
            raise AttributeError(name)
        self._read()
        return super().__getattr__(name)

    def __iter__(self) -> Iterator[str]:
        self._read()
        return super().__iter__()

    def roles(self) -> dict[str, str]:
        self._read()
        return super().roles()


class SimSceneConfig(HardwareModuleConfig):
    """A scene's mounts have no reading to ride on: they go out on tf at static_tf_rate_hz."""

    @model_validator(mode="after")
    def _check_spawn_points(self) -> "SimSceneConfig":
        frames = self.usd.tf_frames
        unframed = [
            point.frame
            for point in scene_spawn_points(self.usd.usd, self.usd.asset_root)
            if point.frame not in frames
        ]
        if unframed:
            raise ValueError(
                f"the {SPAWN_API} prims {unframed} are not tf frames; load a scene with "
                f"scene_usd(), which makes each one a frame"
            )
        return self


class SimScene(SimSensor):
    """Static hardware: a room whose spawn points are where robots, sensors and objects start. Its tf goes out stamped with wall time."""

    config: SimSceneConfig

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        yield

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The edges from the room's root to each mount: a real room publishes no furniture, and the world USD places it."""
        transforms = super()._static_edges
        parent_of = {t.child_frame_id: t.frame_id for t in transforms}
        needed: set[str] = set()
        for mount in self.mounts:
            frame = self.tf_frames[mount]
            while frame in parent_of and frame not in needed:
                needed.add(frame)
                frame = parent_of[frame]
        return [t for t in transforms if t.child_frame_id in needed]


# A small logistics room, for tests and quick runs.

LOGISTICS = Path(__file__).parent / "assets" / "logistics.usda"


class LogisticsConfig(SimSceneConfig):
    usd: UsdModel = Field(default_factory=lambda: scene_usd(LOGISTICS))


class Logistics(SimScene):
    """A logistics room with one spawn point, `go2`, its root body's height above the floor."""

    config: LogisticsConfig
    mounts = SpawnMounts(LOGISTICS)


# A toy robot, for tests.

TOY = Path(__file__).parent / "assets" / "toy.usda"


def toy_robot() -> UsdModel:
    return UsdModel.from_file(TOY)
