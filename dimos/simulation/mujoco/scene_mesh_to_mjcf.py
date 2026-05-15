# Copyright 2025-2026 Dimensional Inc.
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

"""Bake a scene mesh into an MJCF wrapper around a robot MJCF.

The bake walks every prim returned by :func:`load_scene_prims` and asks
:func:`dimos.simulation.mujoco.collision_spec.decide_for_prim` what to
emit for it.  The dispatcher returns one of three modes:

- ``"primitive"`` -- a single MuJoCo primitive ``<geom>`` (box / sphere /
  cylinder / capsule / plane).  Used when the prim is approximately
  prismatic (auto-fit) or when a sidecar override forces it.
- ``"hulls"`` -- one or more mesh ``<geom>``s.  Either a single convex
  hull (small / near-convex prims) or a CoACD decomposition (genuine
  concave shells: stairs, planters).
- ``"skip"`` -- no collision geom at all.  Used for sidecar-tagged
  decoration (lamps, signs) and prims smaller than a threshold.

Hulls produced by either path are validated with :func:`_valid_hull`
(matrix-rank coplanarity check + scipy ``Qt`` qhull pre-flight); when a
hull is invalid we fall back to a thin OBB box via
:func:`_fallback_box_geom` rather than dropping the geometry, so the
robot doesn't sink through holes.

When ``include_visual_mesh=True`` the bake additionally writes the
prim's original triangles as a non-colliding visual geom (group 2,
``contype=0 conaffinity=0``).  UE's USD exporter culls hidden faces on
static meshes (a floor slab ships with only top + bottom face pairs,
no sides) -- we route visual writes through :func:`_write_visual_obj`,
which substitutes the prim's convex hull when it isn't watertight, so
the viewer renders solid geometry instead of see-through slabs.

Per-prim work is fanned across worker processes since each prim's
decision is independent and CoACD calls dominate wall time.  Standalone
CLI bakes use forked processes; in an already-threaded DimOS runtime we
use ``forkserver`` so workers do not inherit the parent process's active
threads.

Output is cached at ``~/.cache/dimos/scene_meshes/<hash>/`` keyed on
the SHA256 of (source mesh, robot MJCF, alignment, meshdir, sidecar
spec, visual flag, schema version).  :func:`load_or_bake` is the
recommended entry point -- it handles a three-tier cache:

  1. ``compiled.mjb`` exists -> load directly (~1 s)
  2. ``wrapper.xml`` + OBJs exist -> compile XML, save ``.mjb``
  3. Nothing exists -> full bake, then compile + save ``.mjb``
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace
import hashlib
import multiprocessing
import os
from pathlib import Path
import time
from typing import Any, cast

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.simulation.mujoco.collision_spec import (
    CollisionSpec,
    decide_for_prim,
)
from dimos.simulation.mujoco.mesh_scene import (
    SceneMeshAlignment,
    ScenePrimMesh,
    load_scene_prims,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


CACHE_DIR = Path.home() / ".cache" / "dimos" / "scene_meshes"
_MUJOCO_FROM_BINARY_PATH = "from_binary_path"
_MUJOCO_SAVE_MODEL = "mj_saveModel"


# ``<include>`` comes BEFORE ``<compiler>`` so the wrapper's absolute
# meshdir is the *last* compiler block in the merged spec -- MuJoCo's
# "last compiler wins" rule then routes the robot MJCF's
# ``<mesh file="X.STL"/>`` entries to ``<wrapper meshdir>/X.STL``.  If
# the order is reversed the robot's own ``<compiler meshdir="assets"/>``
# overrides the absolute path with a relative one that resolves against
# the wrapper's cache directory and the include's meshes can't be found.
#
# The dummy ``<inertial>`` on dimos_scene bypasses MuJoCo's auto-
# computation of body inertia from geom volumes -- the body is static
# (no joint) so the values don't affect physics, but without this any
# zero-volume visual mesh (road tiles, ceiling panels, flat slabs)
# triggers ``Error: mesh volume is too small`` at compile time.
_WRAPPER_TEMPLATE = """\
<mujoco model="{model_name}">
  <include file="{robot_mjcf_abs}"/>
  <compiler angle="radian" meshdir="{meshdir}" texturedir="{meshdir}"/>
  <statistic center="{statistic_center}" extent="{statistic_extent}"/>
  <visual>
    <map znear="0.01" zfar="{visual_zfar}"/>
  </visual>
  <asset>
{asset_meshes}
  </asset>
  <worldbody>
    <body name="dimos_scene" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
{scene_geoms}
    </body>
  </worldbody>
</mujoco>
"""

# ``inertia="shell"`` makes MuJoCo compute mesh inertia from surface
# area instead of enclosed volume -- robust to non-watertight visual
# meshes from art tools.  Safe for closed CoACD hulls too, so we apply
# it universally for one fewer code path.
_ASSET_LINE = '    <mesh name="{name}" file="{file}" inertia="shell"/>'

# Collision (group 3) -- actually collides.  rgba alpha < 1 lets the
# user toggle visibility independently of the visual mesh.
_COL_MESH_LINE = (
    '      <geom name="{name}" type="mesh" mesh="{mesh}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)
_COL_BOX_LINE = (
    '      <geom name="{name}" type="box" pos="{pos}" quat="{quat}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)
_COL_SPHERE_LINE = (
    '      <geom name="{name}" type="sphere" pos="{pos}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)
_COL_CYL_LINE = (
    '      <geom name="{name}" type="cylinder" pos="{pos}" quat="{quat}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)
_COL_CAP_LINE = (
    '      <geom name="{name}" type="capsule" pos="{pos}" quat="{quat}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)
_COL_PLANE_LINE = (
    '      <geom name="{name}" type="plane" pos="{pos}" quat="{quat}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.5 0.6 1.0 0.35"{friction}/>'
)

# Visual (group 2) -- drawn, doesn't collide.
_VISUAL_GEOM_LINE = (
    '      <geom name="{name}" type="mesh" mesh="{mesh}" '
    'contype="0" conaffinity="0" group="2" rgba="0.65 0.65 0.65 1"/>'
)


# Constants kept from the prior implementation -- conservative
# fallback thresholds for hull validity / box-fallback geometry.
_DEGENERATE_EPS = 1e-3
_MIN_HULL_EXTENT_M = 5e-3
_FALLBACK_BOX_THICKNESS_M = 0.03
_MIN_FALLBACK_BOX_EXTENT_M = 0.25
_MIN_FALLBACK_BOX_AREA_M2 = 0.05

_CACHE_KEY_LEN = 12
# Bump when the bake pipeline's output format changes so old caches
# invalidate on the next call.  Increment for any change that could
# affect MJCF emission (new geom kinds, rewritten visual policy, etc.).
# This is only a local cache salt; it is not a persisted file format
# contract and old cache directories can safely stay on disk.
_CACHE_SCHEMA_VERSION = "dispatcher-v8-point-instancer"


@dataclass
class _BakeArtifacts:
    """Aggregated stats + emission lines from one bake."""

    asset_lines: list[str]
    geom_lines: list[str]
    n_primitive: int
    n_hulls_total: int
    n_box_fallbacks: int
    n_skipped: int
    n_visuals: int
    n_degenerate_dropped: int
    decision_reasons: dict[str, int]


def _resolve_existing_file(path: str | Path, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"{label} not found: {resolved}")
    if not resolved.is_file():
        raise ValueError(f"{label} must be a file, got: {resolved}")
    return resolved


def bake_scene_mjcf(
    scene_mesh_path: str | Path,
    robot_mjcf_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
    meshdir: str | Path | None = None,
    cache_root: Path | None = None,
    collision_spec: CollisionSpec | None = None,
    include_visual_mesh: bool = False,
    rebake: bool = False,
) -> Path:
    """Convert ``scene_mesh_path`` to OBJs + MJCF wrapper around the robot.

    Args:
        scene_mesh_path: ``.usdz`` / ``.usda`` / ``.glb`` / ``.obj`` /
            etc.  Anything ``mesh_scene.load_scene_prims`` accepts.
        robot_mjcf_path: the base robot MJCF the wrapper will
            ``<include>``.
        alignment: scale / translation / rotation / y-up swap to bake
            into world frame before any geom is emitted.
        meshdir: directory MuJoCo resolves the *robot's* unqualified
            mesh filenames against.  Defaults to ``robot_mjcf_path.parent``.
            Override when the robot ships its assets in a sibling
            ``assets/`` folder (typical for menagerie robots).
        cache_root: override the cache root (defaults to
            ``~/.cache/dimos/scene_meshes``).
        collision_spec: per-prim policy.  ``None`` auto-discovers a
            sidecar ``<scene>.collision.json`` next to the source, or
            falls back to ``CollisionSpec()`` defaults (auto-fit
            primitives, CoACD on large concave shells).
        include_visual_mesh: also emit a non-colliding visual geom for
            every prim showing its original triangles.  The viewer
            renders these instead of the collision hulls -- much nicer
            for visual debugging, but doubles disk usage.  When ``True``
            non-watertight prim meshes are substituted with their convex
            hull so they don't appear see-through.
        rebake: ignore an existing ``wrapper.xml`` in the computed cache
            directory and regenerate the scene collision geometry.

    Returns:
        Path to the wrapper MJCF.  Pass to ``MujocoSimModule`` instead of
        the raw robot MJCF, or use :func:`load_or_bake` to also get an
        ``.mjb`` cache.
    """
    scene_mesh_path = _resolve_existing_file(scene_mesh_path, "scene mesh")
    robot_mjcf_path = _resolve_existing_file(robot_mjcf_path, "robot MJCF")
    align = alignment or SceneMeshAlignment()
    spec = collision_spec or CollisionSpec.auto_discover(scene_mesh_path)

    meshdir = Path(meshdir).expanduser().resolve() if meshdir else robot_mjcf_path.parent

    cache_key = _cache_key(
        scene_mesh_path,
        robot_mjcf_path,
        align,
        meshdir,
        spec=spec,
        include_visual_mesh=include_visual_mesh,
    )
    root = (cache_root or CACHE_DIR).expanduser()
    cache_dir = root / cache_key
    wrapper_path = cache_dir / "wrapper.xml"

    if not rebake and _cache_hit(wrapper_path):
        logger.info(f"bake_scene_mjcf: cache hit at {cache_dir}")
        return wrapper_path

    cache_dir.mkdir(parents=True, exist_ok=True)

    logger.info(f"bake_scene_mjcf: loading + aligning {scene_mesh_path}")
    prims = load_scene_prims(scene_mesh_path, alignment=align)
    logger.info(f"bake_scene_mjcf: {len(prims)} prims to process")
    if spec.enable_sheet_prisms and len(prims) > spec.sheet_prism_max_scene_prims:
        logger.info(
            "bake_scene_mjcf: disabling thin-sheet triangle prisms for "
            f"{len(prims)}-prim scene; use a collision sidecar to opt in"
        )
        spec = replace(spec, enable_sheet_prisms=False)
    scene_center, scene_extent = _scene_bounds(prims)

    artifacts = _bake_prims(
        prims,
        cache_dir,
        spec=spec,
        include_visual_mesh=include_visual_mesh,
    )
    if not artifacts.geom_lines:
        raise RuntimeError(
            "bake_scene_mjcf: every prim got skipped or produced only "
            "degenerate hulls; nothing left to collide against.  Check "
            "the source mesh and alignment."
        )

    logger.info(
        f"bake_scene_mjcf: {artifacts.n_primitive} primitive geoms, "
        f"{artifacts.n_hulls_total} hull geoms, "
        f"{artifacts.n_box_fallbacks} box fallbacks, "
        f"{artifacts.n_visuals} visual passthrough meshes, "
        f"{artifacts.n_skipped} skipped, "
        f"{artifacts.n_degenerate_dropped} degenerate hulls dropped"
    )
    # Top-10 decision reasons -- useful when tuning a sidecar.
    for reason, n in sorted(artifacts.decision_reasons.items(), key=lambda kv: -kv[1])[:10]:
        logger.info(f"  reason {reason:32s} {n}")

    _write_wrapper(
        wrapper_path=wrapper_path,
        cache_key=cache_key,
        meshdir=meshdir,
        robot_mjcf_path=robot_mjcf_path,
        asset_lines=artifacts.asset_lines,
        geom_lines=artifacts.geom_lines,
        statistic_center=scene_center,
        statistic_extent=scene_extent,
    )
    return wrapper_path


def load_or_bake(
    scene_mesh_path: str | Path,
    robot_mjcf_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
    meshdir: str | Path | None = None,
    cache_root: Path | None = None,
    collision_spec: CollisionSpec | None = None,
    include_visual_mesh: bool = False,
    rebake: bool = False,
) -> tuple[Any, Path]:
    """Three-tier cache: ``.mjb`` -> ``wrapper.xml`` -> full bake.

    Returns ``(MjModel, wrapper_path)``.

    1. If ``compiled.mjb`` exists in the bake's cache dir -> load it
       directly (~1 s for a 5k-prim mall).
    2. Else if ``wrapper.xml`` exists -> compile XML, save the ``.mjb``
       beside it, return the model.
    3. Else -> run the full bake, compile the resulting wrapper, save
       ``.mjb``, return the model.

    Set ``rebake=True`` to force step 3 even when caches exist.

    The cache key incorporates source-mesh signature, robot MJCF,
    alignment, sidecar spec and the schema version -- change any of
    those and a fresh cache directory is created automatically (the
    old one stays on disk until you clean it).
    """
    import mujoco  # type: ignore[import-untyped]

    scene_mesh_path = _resolve_existing_file(scene_mesh_path, "scene mesh")
    robot_mjcf_path = _resolve_existing_file(robot_mjcf_path, "robot MJCF")
    load_binary_model = cast(
        "Callable[[str], Any]",
        getattr(mujoco.MjModel, _MUJOCO_FROM_BINARY_PATH),
    )
    save_model = cast(
        "Callable[[Any, str], None]",
        getattr(mujoco, _MUJOCO_SAVE_MODEL),
    )

    if not rebake:
        # Compute the cache key without baking, so we can probe for an
        # existing .mjb / wrapper.xml without doing any work.
        sp = scene_mesh_path
        rp = robot_mjcf_path
        al = alignment or SceneMeshAlignment()
        sd = collision_spec or CollisionSpec.auto_discover(sp)
        md = Path(meshdir).expanduser().resolve() if meshdir else rp.parent
        key = _cache_key(sp, rp, al, md, spec=sd, include_visual_mesh=include_visual_mesh)
        root = (cache_root or CACHE_DIR).expanduser()
        cache_dir = root / key
        mjb = cache_dir / "compiled.mjb"
        wrapper = cache_dir / "wrapper.xml"

        wrapper_is_current = _cache_hit(wrapper)

        if (
            mjb.exists()
            and wrapper_is_current
            and mjb.stat().st_mtime_ns >= wrapper.stat().st_mtime_ns
        ):
            logger.info(f"load_or_bake: loading compiled binary {mjb}")
            t0 = time.time()
            model = load_binary_model(str(mjb))
            logger.info(
                f"  loaded in {time.time() - t0:.1f}s "
                f"(nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh})"
            )
            return model, wrapper

        if wrapper_is_current and any(cache_dir.glob("*.obj")):
            logger.info(f"load_or_bake: wrapper cached, compiling XML -> .mjb at {wrapper}")
            t0 = time.time()
            model = mujoco.MjModel.from_xml_path(str(wrapper))
            logger.info(
                f"  compiled in {time.time() - t0:.1f}s "
                f"(nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh})"
            )
            save_model(model, str(mjb))
            logger.info(f"  saved compiled binary: {mjb} ({mjb.stat().st_size / 1e6:.1f} MB)")
            return model, wrapper

    # Full bake path.
    wrapper = bake_scene_mjcf(
        scene_mesh_path=scene_mesh_path,
        robot_mjcf_path=robot_mjcf_path,
        alignment=alignment,
        meshdir=meshdir,
        cache_root=cache_root,
        collision_spec=collision_spec,
        include_visual_mesh=include_visual_mesh,
        rebake=rebake,
    )
    logger.info(f"load_or_bake: compiling baked wrapper {wrapper}")
    t0 = time.time()
    model = mujoco.MjModel.from_xml_path(str(wrapper))
    logger.info(
        f"  compiled in {time.time() - t0:.1f}s "
        f"(nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh})"
    )
    mjb = wrapper.with_name("compiled.mjb")
    save_model(model, str(mjb))
    logger.info(f"  saved compiled binary: {mjb} ({mjb.stat().st_size / 1e6:.1f} MB)")
    return model, wrapper


# --------------------------------------------------------------------------- #
# Cache key                                                                   #
# --------------------------------------------------------------------------- #


def _cache_key(
    scene_mesh_path: Path,
    robot_mjcf_path: Path,
    alignment: SceneMeshAlignment,
    meshdir: Path,
    *,
    spec: CollisionSpec,
    include_visual_mesh: bool,
) -> str:
    """SHA256-12 over every input that affects bake output.

    Source-mesh signature is ``(path, size, mtime_ns)`` -- much faster
    than reading the whole file (a mall scene's USDA + Assets is a few
    hundred MB) and reliably invalidates when the artist re-exports.
    Sidecar spec is hashed by its JSON encoding so any field change
    (new override pattern, tuned threshold) invalidates correctly.
    """
    import json

    def _file_signature(path: Path) -> str:
        st = path.stat()
        return f"{path}:{st.st_size}:{st.st_mtime_ns}"

    h = hashlib.sha256()
    h.update(_CACHE_SCHEMA_VERSION.encode())
    h.update(_file_signature(scene_mesh_path).encode())
    h.update(_file_signature(robot_mjcf_path).encode())
    h.update(repr(sorted(asdict(alignment).items())).encode())
    h.update(str(meshdir).encode())
    h.update(json.dumps(asdict(spec), sort_keys=True).encode())
    h.update(b"visual=" + (b"1" if include_visual_mesh else b"0"))
    return h.hexdigest()[:_CACHE_KEY_LEN]


def _cache_hit(wrapper_path: Path) -> bool:
    if not wrapper_path.exists():
        return False
    try:
        text = wrapper_path.read_text()
    except OSError:
        return False
    return "<statistic center=" in text and '<map znear="0.01" zfar=' in text


# --------------------------------------------------------------------------- #
# Per-prim worker (parallel-safe)                                             #
# --------------------------------------------------------------------------- #


def _process_one_prim(
    args: tuple[ScenePrimMesh, Path, CollisionSpec, bool],
) -> tuple[list[str], list[str], str, str, dict[str, int]]:
    """Worker entry-point -- must be picklable.

    Takes ``(prim, cache_dir, spec, include_visual_mesh)`` and returns
    ``(asset_lines, geom_lines, decision_mode, decision_reason, counters)``
    where ``counters`` carries small int deltas the parent aggregates
    (``hulls``, ``box_fallbacks``, ``visuals``, ``degenerate``).

    No shared state -- the only side effects are OBJ writes into
    ``cache_dir`` (each prim has a unique name, no contention).
    """
    prim, cache_dir, spec, include_visual_mesh = args
    v = np.asarray(prim.vertices, dtype=np.float64)
    t = np.asarray(prim.triangles)
    if len(v) < 3 or len(t) < 1:
        return (
            [],
            [],
            "skip",
            "empty-prim",
            {"hulls": 0, "box_fallbacks": 0, "visuals": 0, "degenerate": 0},
        )

    decision = decide_for_prim(vertices=v, triangles=t, prim_path=prim.name, spec=spec)

    asset_lines: list[str] = []
    geom_lines: list[str] = []
    counters = {"hulls": 0, "box_fallbacks": 0, "visuals": 0, "degenerate": 0}

    # Visual passthrough (always before the collision branch -- even
    # ``skip`` prims can have a visual).
    if include_visual_mesh:
        vis_name = f"{prim.name}_visual"
        vis_path = cache_dir / f"{vis_name}.obj"
        try:
            _write_visual_obj(vis_path, v, t.astype(np.int32))
            asset_lines.append(_ASSET_LINE.format(name=vis_name, file=str(vis_path)))
            geom_lines.append(_VISUAL_GEOM_LINE.format(name=f"{vis_name}_geom", mesh=vis_name))
            counters["visuals"] = 1
        except Exception:
            pass

    friction_attr = ""
    if decision.friction is not None:
        friction_attr = f' friction="{_fmt_vec(np.asarray(decision.friction))}"'

    if decision.mode == "skip":
        return asset_lines, geom_lines, "skip", decision.reason, counters

    if decision.mode == "primitive":
        prim_geom = _emit_primitive_geom(prim.name, decision.primitive, friction_attr)
        if prim_geom is not None:
            geom_lines.append(prim_geom)
        return asset_lines, geom_lines, "primitive", decision.reason, counters

    # mode == "hulls".  Each hull goes through _valid_hull; invalid
    # ones get a fallback OBB box (avoids dropping collision entirely
    # at locations where a hull happened to be degenerate).
    for j, (hv, ht) in enumerate(decision.hulls):
        v_arr = np.asarray(hv, dtype=np.float32)
        f_arr = np.asarray(ht, dtype=np.int32)
        if not _valid_hull(v_arr, f_arr):
            box_line = _fallback_box_geom(f"{prim.name}_h{j:03d}_box", v_arr, friction_attr)
            if box_line is None:
                counters["degenerate"] += 1
            else:
                geom_lines.append(box_line)
                counters["box_fallbacks"] += 1
            continue

        asset_name = f"{prim.name}_h{j:03d}"
        obj_file = cache_dir / f"{asset_name}.obj"
        _write_hull_obj(obj_file, v_arr, f_arr)
        asset_lines.append(_ASSET_LINE.format(name=asset_name, file=str(obj_file)))
        geom_lines.append(
            _COL_MESH_LINE.format(
                name=f"{asset_name}_geom", mesh=asset_name, friction=friction_attr
            )
        )
        counters["hulls"] += 1

    return asset_lines, geom_lines, "hulls", decision.reason, counters


def _bake_prims(
    prims: list[ScenePrimMesh],
    cache_dir: Path,
    *,
    spec: CollisionSpec,
    include_visual_mesh: bool,
) -> _BakeArtifacts:
    """Fan per-prim work across cores; aggregate the resulting MJCF lines.

    Standalone bakes use ``fork`` so workers inherit the parent's
    already-imported modules.  Runtime bakes inside DimOS may happen
    after other modules have started threads; in that case use
    ``forkserver`` so workers do not inherit locks from the parent
    process's C extension state.
    """
    asset_lines: list[str] = []
    geom_lines: list[str] = []
    n_primitive = 0
    n_hulls_total = 0
    n_box_fallbacks = 0
    n_skipped = 0
    n_visuals = 0
    n_degenerate = 0
    reasons: dict[str, int] = {}

    work_items = [(prim, cache_dir, spec, include_visual_mesh) for prim in prims]
    n_workers = max(1, (os.cpu_count() or 4) - 1)
    if _native_thread_count() > 1:
        n_workers = min(n_workers, 8)
        start_method = (
            "forkserver" if "forkserver" in multiprocessing.get_all_start_methods() else "spawn"
        )
    else:
        start_method = "fork"
    logger.info(
        f"_bake_prims: fanning {len(prims)} prims across {n_workers} workers ({start_method})"
    )

    t0 = time.time()
    mp_ctx = multiprocessing.get_context(start_method)
    executor = ProcessPoolExecutor(max_workers=n_workers, mp_context=mp_ctx)

    progress_every = 25 if len(prims) <= 500 else 250
    with executor as ex:
        futures = [ex.submit(_process_one_prim, item) for item in work_items]
        done = 0
        for fut in as_completed(futures):
            a_lines, g_lines, mode, reason, counters = fut.result()
            asset_lines.extend(a_lines)
            geom_lines.extend(g_lines)
            reasons[reason] = reasons.get(reason, 0) + 1
            if mode == "primitive":
                n_primitive += 1
            elif mode == "skip":
                n_skipped += 1
            n_hulls_total += counters["hulls"]
            n_box_fallbacks += counters["box_fallbacks"]
            n_visuals += counters["visuals"]
            n_degenerate += counters["degenerate"]
            done += 1
            if done % progress_every == 0 or done == len(prims):
                elapsed = time.time() - t0
                eta = elapsed * (len(prims) - done) / max(done, 1)
                logger.info(
                    f"  prim {done}/{len(prims)} "
                    f"({100 * done / len(prims):.0f}%) "
                    f"elapsed={elapsed:.0f}s eta={eta:.0f}s "
                    f"hulls_so_far={n_hulls_total}"
                )

    return _BakeArtifacts(
        asset_lines=asset_lines,
        geom_lines=geom_lines,
        n_primitive=n_primitive,
        n_hulls_total=n_hulls_total,
        n_box_fallbacks=n_box_fallbacks,
        n_skipped=n_skipped,
        n_visuals=n_visuals,
        n_degenerate_dropped=n_degenerate,
        decision_reasons=reasons,
    )


def _native_thread_count() -> int:
    try:
        return len(os.listdir("/proc/self/task"))
    except OSError:
        return 1


# --------------------------------------------------------------------------- #
# Geom emission helpers                                                       #
# --------------------------------------------------------------------------- #


def _emit_primitive_geom(
    prim_name: str,
    fit: dict[str, Any] | None,
    friction_attr: str,
) -> str | None:
    """Render one ``PrimDecision.primitive`` dict to MJCF text.

    Returns ``None`` if ``fit`` is missing required fields (defensive --
    ``decide_for_prim`` should always populate them, but a malformed
    sidecar override could slip through).
    """
    if fit is None:
        return None
    kind = fit.get("type")
    pos = _fmt_vec(np.asarray(fit["pos"]))
    size = _fmt_vec(np.asarray(fit["size"]))
    quat = (
        _fmt_vec(np.asarray(fit["quat"]))
        if "quat" in fit and fit["quat"] is not None
        else "1 0 0 0"
    )
    name = f"{prim_name}_col"
    if kind == "box":
        return _COL_BOX_LINE.format(
            name=name, pos=pos, quat=quat, size=size, friction=friction_attr
        )
    if kind == "sphere":
        return _COL_SPHERE_LINE.format(name=name, pos=pos, size=size, friction=friction_attr)
    if kind == "cylinder":
        return _COL_CYL_LINE.format(
            name=name, pos=pos, quat=quat, size=size, friction=friction_attr
        )
    if kind == "capsule":
        return _COL_CAP_LINE.format(
            name=name, pos=pos, quat=quat, size=size, friction=friction_attr
        )
    if kind == "plane":
        return _COL_PLANE_LINE.format(
            name=name, pos=pos, quat=quat, size=size, friction=friction_attr
        )
    return None


# --------------------------------------------------------------------------- #
# Hull validity & box fallback (preserved from prior implementation)          #
# --------------------------------------------------------------------------- #


def _valid_hull(v: np.ndarray, f: np.ndarray) -> bool:
    """Reject hulls that MuJoCo's qhull would choke on at compile time.

    Four layers:
      1. trivial -- < 4 vertices or < 4 faces.
      2. extent -- all-axis ``> 5 mm`` (matches MuJoCo's mj_loadXML
         coplanarity tolerance for ~100mm-wide hulls).
      3. rank -- centred vertex matrix must have rank 3 (catches
         coplanar hulls the extent check misses, e.g. a T-shaped
         hull whose XY extent is large but Z is zero).
      4. scipy ConvexHull pre-flight with ``Qt`` -- same options
         MuJoCo uses; if scipy can't build it, mj_loadXML can't either.
    """
    if len(v) < 4 or len(f) < 4:
        return False
    extent = v.max(axis=0) - v.min(axis=0)
    if (extent < _DEGENERATE_EPS).any():
        return False
    if float(extent.min()) < _MIN_HULL_EXTENT_M:
        return False
    centered = v.astype(np.float64) - v.astype(np.float64).mean(axis=0)
    if np.linalg.matrix_rank(centered, tol=_DEGENERATE_EPS) < 3:
        return False
    try:
        from scipy.spatial import ConvexHull, QhullError  # type: ignore[import-untyped]

        ConvexHull(v, qhull_options="Qt")
    except (QhullError, ValueError):
        return False
    return True


def _fallback_box_geom(name: str, vertices: np.ndarray, friction_attr: str = "") -> str | None:
    """Emit a thin OBB box geom for vertices that can't form a valid hull.

    The thickness floor (``_FALLBACK_BOX_THICKNESS_M = 3 cm``) keeps the
    box thick enough that the robot can stand on it without falling
    through.  Returns ``None`` for prims too small to bother (< 25 cm
    largest extent or < 0.05 m^2 face area) -- those fall through to
    the degenerate counter.
    """
    finite = vertices[np.isfinite(vertices).all(axis=1)].astype(np.float64)
    if len(finite) < 3:
        return None
    aabb_extent = finite.max(axis=0) - finite.min(axis=0)
    sorted_extents = np.sort(aabb_extent)
    if sorted_extents[-1] < _MIN_FALLBACK_BOX_EXTENT_M:
        return None
    if sorted_extents[-1] * sorted_extents[-2] < _MIN_FALLBACK_BOX_AREA_M2:
        return None

    center, rotation, extent = _oriented_box(finite)
    extent = np.maximum(extent, _FALLBACK_BOX_THICKNESS_M)
    half_size = 0.5 * extent
    quat = _rotation_matrix_to_wxyz(rotation)
    return _COL_BOX_LINE.format(
        name=name,
        pos=_fmt_vec(center),
        quat=_fmt_vec(quat),
        size=_fmt_vec(half_size),
        friction=friction_attr,
    )


def _oriented_box(
    vertices: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """OBB via trimesh's ``bounding_box_oriented``.

    Falls back to AABB if trimesh's OBB fitter produces non-finite
    output or the prim has < 3 vertices.
    """
    try:
        import trimesh  # type: ignore[import-untyped]

        tm = trimesh.Trimesh(vertices=vertices, faces=np.empty((0, 3), dtype=np.int32))
        obb = tm.bounding_box_oriented
        transform = np.asarray(obb.primitive.transform, dtype=np.float64)
        extent = np.asarray(obb.primitive.extents, dtype=np.float64)
        rotation = transform[:3, :3]
        center = transform[:3, 3]
        if np.linalg.det(rotation) < 0:
            rotation[:, 0] *= -1.0
        if np.isfinite(center).all() and np.isfinite(rotation).all() and np.isfinite(extent).all():
            return center, rotation, np.abs(extent)
    except Exception:
        pass

    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    return (lo + hi) * 0.5, np.eye(3), hi - lo


def _rotation_matrix_to_wxyz(rotation: np.ndarray) -> np.ndarray:
    """3x3 rotation -> ``(w, x, y, z)`` quaternion."""
    from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]

    xyzw = Rotation.from_matrix(rotation).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float64)


def _fmt_vec(values: np.ndarray) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def _scene_bounds(prims: list[ScenePrimMesh]) -> tuple[np.ndarray, float]:
    """Return a viewer-friendly center and extent for the aligned scene.

    MuJoCo's viewer uses ``statistic.center`` / ``statistic.extent`` for
    camera framing and clipping.  The included robot MJCF's defaults are
    much too small for baked building-scale scenes, so wrappers need to
    advertise the scene bounds explicitly.
    """
    mins: list[np.ndarray] = []
    maxs: list[np.ndarray] = []
    for prim in prims:
        vertices = np.asarray(prim.vertices, dtype=np.float64)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
            continue
        finite = vertices[np.isfinite(vertices).all(axis=1)]
        if len(finite) == 0:
            continue
        mins.append(finite.min(axis=0))
        maxs.append(finite.max(axis=0))

    if not mins:
        return np.zeros(3, dtype=np.float64), 1.0

    scene_min = np.min(np.vstack(mins), axis=0)
    scene_max = np.max(np.vstack(maxs), axis=0)
    center = (scene_min + scene_max) * 0.5
    diagonal = scene_max - scene_min
    extent = max(float(np.linalg.norm(diagonal) * 0.5 * 1.1), 1.0)
    return center, extent


# --------------------------------------------------------------------------- #
# OBJ I/O                                                                     #
# --------------------------------------------------------------------------- #


def _write_hull_obj(obj_file: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    """Write a CoACD/single-hull mesh.  No watertight check -- hulls are
    closed by construction."""
    _write_mesh_obj(obj_file, vertices, faces)


def _write_visual_obj(obj_file: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    """Write a *renderable* OBJ -- closed under all viewing angles.

    UE's static-mesh exporter culls hidden faces (a floor slab ships
    with only top + bottom face pairs, no sides), so writing the
    artist's geometry verbatim produces meshes that appear see-through
    in MuJoCo's viewer from any oblique angle.  We check
    ``trimesh.is_watertight`` and, if not, substitute the prim's
    convex hull (which is always closed).

    For non-prismatic prims (chairs, plants) the hull is a coarse
    visual approximation; for the most common offenders (floor / roof
    / wall / ceiling slabs that are box-shaped to begin with) the hull
    matches the original exactly.  Watertight prims (full furniture
    meshes from UE) keep their original geometry.
    """
    import trimesh  # type: ignore[import-untyped]

    tm = trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=np.float64),
        faces=np.asarray(faces, dtype=np.int32),
        process=False,
    )
    if not tm.is_watertight:
        try:
            hull = tm.convex_hull
            if len(hull.vertices) >= 4 and len(hull.faces) >= 4:
                vertices = np.asarray(hull.vertices, dtype=np.float64)
                faces = np.asarray(hull.faces, dtype=np.int32)
        except Exception:
            pass  # fall back to original; visual may look hollow
    _write_mesh_obj(obj_file, vertices, faces)


def _write_mesh_obj(obj_file: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    o3d_mesh = o3d.geometry.TriangleMesh()
    o3d_mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    o3d_mesh.triangles = o3d.utility.Vector3iVector(faces)
    o3d_mesh.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(
        str(obj_file),
        o3d_mesh,
        write_vertex_normals=True,
        write_vertex_colors=False,
    ):
        raise RuntimeError(f"open3d failed to write OBJ: {obj_file}")


# --------------------------------------------------------------------------- #
# Wrapper writer + CLI                                                        #
# --------------------------------------------------------------------------- #


def _write_wrapper(
    *,
    wrapper_path: Path,
    cache_key: str,
    meshdir: Path,
    robot_mjcf_path: Path,
    asset_lines: list[str],
    geom_lines: list[str],
    statistic_center: np.ndarray,
    statistic_extent: float,
) -> None:
    visual_zfar = max(float(statistic_extent) * 20.0, 10000.0)
    wrapper_xml = _WRAPPER_TEMPLATE.format(
        model_name=f"robot_with_scene_{cache_key}",
        meshdir=str(meshdir),
        robot_mjcf_abs=str(robot_mjcf_path),
        statistic_center=_fmt_vec(statistic_center),
        statistic_extent=f"{float(statistic_extent):.9g}",
        visual_zfar=f"{visual_zfar:.9g}",
        asset_meshes="\n".join(asset_lines),
        scene_geoms="\n".join(geom_lines),
    )
    wrapper_path.write_text(wrapper_xml)
    logger.info(f"_write_wrapper: wrote {wrapper_path}")


def cli_main() -> None:
    """``python -m dimos.simulation.mujoco.scene_mesh_to_mjcf <scene> <robot> [opts]``.

    Bake (or load from cache), optionally launch the MuJoCo viewer.
    """
    p = argparse.ArgumentParser(
        prog="python -m dimos.simulation.mujoco.scene_mesh_to_mjcf",
        description="Bake a USD/GLB/OBJ scene into an MJCF wrapping a robot MJCF.",
    )
    p.add_argument("scene", type=Path, help="scene mesh path (.usda, .usdz, .glb, ...)")
    p.add_argument("robot", type=Path, help="robot MJCF path")
    p.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="multiplicative scale (use 0.01 for UE / centimeter sources). Default 1.0.",
    )
    p.add_argument(
        "--no-y-up",
        action="store_true",
        help="source is already Z-up (UE exports with metersPerUnit=0.01 and "
        "upAxis=Z).  Default assumes Y-up source (Blender, glTF, Apple USDZ).",
    )
    p.add_argument(
        "--collision-spec",
        type=Path,
        default=None,
        help="path to a collision-spec sidecar JSON.  Default auto-discovers "
        "``<scene>.collision.json`` next to the source.",
    )
    p.add_argument(
        "--meshdir",
        type=Path,
        default=None,
        help="override meshdir for the robot's relative <mesh file=...> "
        "lookups.  Default: robot MJCF's parent directory.",
    )
    p.add_argument(
        "--visual",
        action="store_true",
        help="emit visual passthrough meshes (group 2).  Off by default -- "
        "saves disk and render cost, but the MuJoCo viewer only shows "
        "collision shapes without it.",
    )
    p.add_argument(
        "--rebake",
        action="store_true",
        help="ignore cached .mjb and wrapper.xml; do a full re-bake.",
    )
    p.add_argument(
        "--view",
        action="store_true",
        help="launch the MuJoCo native viewer after baking (blocks).",
    )
    args = p.parse_args()

    try:
        scene_path = _resolve_existing_file(args.scene, "scene mesh")
        robot_path = _resolve_existing_file(args.robot, "robot MJCF")
    except (FileNotFoundError, ValueError) as exc:
        p.error(str(exc))

    align = SceneMeshAlignment(scale=args.scale, y_up=not args.no_y_up)
    spec = (
        CollisionSpec.from_json(args.collision_spec)
        if args.collision_spec is not None
        else CollisionSpec.auto_discover(scene_path)
    )

    model, wrapper = load_or_bake(
        scene_mesh_path=scene_path,
        robot_mjcf_path=robot_path,
        alignment=align,
        meshdir=args.meshdir,
        collision_spec=spec,
        include_visual_mesh=args.visual,
        rebake=args.rebake,
    )
    print(f"wrapper: {wrapper}")
    print(f"loaded:  {model.nbody} bodies, {model.ngeom} geoms, {model.nmesh} meshes")
    print(f"joints:  {model.njnt}, dof:  {model.nv}")

    if args.view:
        import mujoco.viewer  # type: ignore[import-untyped]

        viewer: Any = mujoco.viewer
        # ``launch`` runs MuJoCo's interactive viewer with its own
        # internal physics loop.  Blocks until the user closes it.
        # Press F1 in the viewer for the keyboard cheatsheet; ``Tab``
        # toggles the rendering panel where you can switch geom groups
        # (group 3 = scene collision, group 2 = visual passthrough,
        # group 1 = robot visual, group 0 = robot collision).
        print("\nlaunching MuJoCo viewer (press Esc / close window to exit)")
        viewer.launch(model)


if __name__ == "__main__":
    cli_main()


__all__ = ["bake_scene_mjcf", "load_or_bake"]
