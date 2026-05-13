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

"""Bake a scene mesh into an MJCF wrapper around a robot MJCF."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path
import sys
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.simulation.mujoco.mesh_scene import SceneMeshAlignment, load_scene_prims
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


CACHE_DIR = Path.home() / ".cache" / "dimos" / "scene_meshes"


_WRAPPER_TEMPLATE = """\
<mujoco model="{model_name}">
  <compiler angle="radian" meshdir="{meshdir}" texturedir="{meshdir}"/>
  <include file="{robot_mjcf_abs}"/>
  <asset>
{asset_meshes}
  </asset>
  <worldbody>
    <body name="dimos_scene" pos="0 0 0">
{scene_geoms}
    </body>
  </worldbody>
</mujoco>
"""

_ASSET_LINE = '    <mesh name="{name}" file="{file}"/>'
_GEOM_LINE = (
    '      <geom name="{name}" type="mesh" mesh="{mesh}" '
    'contype="1" conaffinity="1" group="3" rgba="0.6 0.6 0.6 1"/>'
)
_VISUAL_GEOM_LINE = (
    '      <geom name="{name}" type="mesh" mesh="{mesh}" '
    'contype="0" conaffinity="0" group="2" rgba="0.6 0.6 0.6 1"/>'
)
_BOX_GEOM_LINE = (
    '      <geom name="{name}" type="box" pos="{pos}" quat="{quat}" size="{size}" '
    'contype="1" conaffinity="1" group="3" rgba="0.6 0.6 0.6 1"/>'
)
_DEGENERATE_EPS = 1e-3
_SHELL_VOLUME_M3 = 2.0
_CACHE_KEY_LEN = 12
_CACHE_SCHEMA_VERSION = "visual-proxy-v1"
_VHACD_MAX_HULLS = 64
_VHACD_RESOLUTION = 200_000
_VHACD_MIN_COMPONENTS = 512
_COMPONENT_VHACD_MAX_SCENE_PRIMS = 256
_MIN_HULL_EXTENT_M = 5e-3
_FALLBACK_BOX_THICKNESS_M = 0.03
_MIN_FALLBACK_BOX_EXTENT_M = 0.25
_MIN_FALLBACK_BOX_AREA_M2 = 0.05


@dataclass
class _BakeArtifacts:
    asset_lines: list[str]
    geom_lines: list[str]
    total_tris: int
    skipped_degenerate: int
    n_hulls: int
    n_decomposed: int
    n_box_fallbacks: int
    n_visuals: int


def bake_scene_mjcf(
    scene_mesh_path: str | Path,
    robot_mjcf_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
    meshdir: str | Path | None = None,
    cache_root: Path | None = None,
    include_visual_mesh: bool = False,
) -> Path:
    """Convert ``scene_mesh_path`` to OBJ and emit a wrapped MJCF.

    Args:
        scene_mesh_path: ``.usdz`` / ``.glb`` / ``.obj`` etc.; anything
            ``mesh_scene.load_scene_mesh`` accepts.
        robot_mjcf_path: the base robot MJCF the wrapper will
            ``<include>``.
        alignment: scale / translation / rotation / y-up swap to bake
            into the OBJ before MuJoCo sees it.
        meshdir: directory MuJoCo should resolve unqualified mesh
            filenames against.  ``None`` uses the robot MJCF's parent
            directory.  Blueprints for robot assets stored elsewhere
            should pass this explicitly.
        cache_root: override for the cache directory (defaults to
            ``~/.cache/dimos/scene_meshes``).
        include_visual_mesh: also add the original scene prim meshes as
            non-colliding visual geoms.  Useful for material-batched scenes
            where convex collision proxies are too coarse to inspect.

    Returns:
        Path to the wrapper MJCF.  Pass this to ``MujocoSimModule``
        instead of the raw robot MJCF.
    """
    scene_mesh_path = Path(scene_mesh_path).expanduser().resolve()
    robot_mjcf_path = Path(robot_mjcf_path).expanduser().resolve()
    align = alignment or SceneMeshAlignment()

    if not scene_mesh_path.exists():
        raise FileNotFoundError(f"scene mesh not found: {scene_mesh_path}")
    if not robot_mjcf_path.exists():
        raise FileNotFoundError(f"robot MJCF not found: {robot_mjcf_path}")

    meshdir = Path(meshdir).expanduser().resolve() if meshdir else robot_mjcf_path.parent

    cache_key = _cache_key(
        scene_mesh_path,
        robot_mjcf_path,
        align,
        meshdir,
        include_visual_mesh=include_visual_mesh,
    )
    root = (cache_root or CACHE_DIR).expanduser()
    cache_dir = root / cache_key
    wrapper_path = cache_dir / "wrapper.xml"

    if _cache_hit(wrapper_path, cache_dir):
        logger.info(f"bake_scene_mjcf: cache hit at {cache_dir}")
        return wrapper_path

    cache_dir.mkdir(parents=True, exist_ok=True)

    logger.info(f"bake_scene_mjcf: loading + aligning {scene_mesh_path} (per-prim)")
    prims = load_scene_prims(scene_mesh_path, alignment=align)
    logger.info(f"bake_scene_mjcf: {len(prims)} prims to bake")

    artifacts = _bake_collision_hulls(
        prims,
        cache_dir,
        include_visual_mesh=include_visual_mesh,
    )
    if not artifacts.asset_lines and not artifacts.geom_lines:
        raise RuntimeError(
            "bake_scene_mjcf: every hull came out degenerate; nothing left to collide against"
        )
    logger.info(
        f"bake_scene_mjcf: baked {artifacts.n_hulls} convex hulls from {len(prims)} prims "
        f"({artifacts.total_tris} tris total), VHACD-decomposed {artifacts.n_decomposed} "
        f"shell prims, added {artifacts.n_box_fallbacks} thin box fallbacks, "
        f"added {artifacts.n_visuals} visual passthrough meshes, "
        f"skipped {artifacts.skipped_degenerate} degenerate hulls"
    )

    _write_wrapper(
        wrapper_path=wrapper_path,
        cache_key=cache_key,
        meshdir=meshdir,
        robot_mjcf_path=robot_mjcf_path,
        asset_lines=artifacts.asset_lines,
        geom_lines=artifacts.geom_lines,
    )
    return wrapper_path


def _cache_key(
    scene_mesh_path: Path,
    robot_mjcf_path: Path,
    alignment: SceneMeshAlignment,
    meshdir: Path,
    *,
    include_visual_mesh: bool,
) -> str:
    def _file_signature(path: Path) -> str:
        stat = path.stat()
        return f"{path}:{stat.st_size}:{stat.st_mtime_ns}"

    h = hashlib.sha256()
    h.update(_CACHE_SCHEMA_VERSION.encode())
    h.update(_file_signature(scene_mesh_path).encode())
    h.update(_file_signature(robot_mjcf_path).encode())
    h.update(repr(sorted(asdict(alignment).items())).encode())
    h.update(str(meshdir).encode())
    h.update(str(include_visual_mesh).encode())
    return h.hexdigest()[:_CACHE_KEY_LEN]


def _cache_hit(wrapper_path: Path, cache_dir: Path) -> bool:
    return wrapper_path.exists() and any(cache_dir.glob("*.obj"))


def _bake_collision_hulls(
    prims: list[Any],
    cache_dir: Path,
    *,
    include_visual_mesh: bool = False,
) -> _BakeArtifacts:
    import trimesh

    asset_lines: list[str] = []
    geom_lines: list[str] = []
    total_tris = 0
    skipped_degenerate = 0
    n_hulls = 0
    n_decomposed = 0
    n_box_fallbacks = 0
    n_visuals = 0
    component_vhacd = len(prims) <= _COMPONENT_VHACD_MAX_SCENE_PRIMS
    logger.info(f"bake_scene_mjcf: per-prim convex-hulling {len(prims)} prims (one-time)")
    for prim in prims:
        if include_visual_mesh:
            asset_name = f"{prim.name}_visual"
            obj_file = cache_dir / f"{asset_name}.obj"
            _write_mesh_obj(obj_file, prim.vertices, prim.triangles)
            asset_lines.append(_ASSET_LINE.format(name=asset_name, file=str(obj_file)))
            geom_lines.append(_VISUAL_GEOM_LINE.format(name=f"{asset_name}_geom", mesh=asset_name))
            n_visuals += 1

        tm = trimesh.Trimesh(
            vertices=prim.vertices.astype(np.float64),
            faces=prim.triangles,
            process=False,
        )
        try:
            single_hull = tm.convex_hull
        except Exception as e:
            box_line = _fallback_box_geom(f"{prim.name}_box", prim.vertices)
            if box_line is None:
                logger.warning(f"  convex_hull failed for {prim.name}: {e}; skipping")
                skipped_degenerate += 1
            else:
                logger.warning(
                    f"  convex_hull failed for {prim.name}: {e}; using thin box fallback"
                )
                geom_lines.append(box_line)
                n_box_fallbacks += 1
            continue

        hulls, decomposed = _collision_hulls(
            tm,
            single_hull,
            prim.name,
            component_vhacd=component_vhacd,
        )
        if decomposed:
            n_decomposed += 1

        for j, hull in enumerate(hulls):
            v = np.asarray(hull.vertices, dtype=np.float32)
            f = np.asarray(hull.faces, dtype=np.int32)
            if not _valid_hull(v, f):
                box_line = _fallback_box_geom(f"{prim.name}_h{j:03d}_box", v)
                if box_line is None:
                    skipped_degenerate += 1
                else:
                    geom_lines.append(box_line)
                    n_box_fallbacks += 1
                continue

            asset_name = f"{prim.name}_h{j:03d}"
            obj_file = cache_dir / f"{asset_name}.obj"
            _write_hull_obj(obj_file, v, f)

            total_tris += len(f)
            n_hulls += 1
            asset_lines.append(_ASSET_LINE.format(name=asset_name, file=str(obj_file)))
            geom_lines.append(_GEOM_LINE.format(name=f"{asset_name}_geom", mesh=asset_name))

    return _BakeArtifacts(
        asset_lines=asset_lines,
        geom_lines=geom_lines,
        total_tris=total_tris,
        skipped_degenerate=skipped_degenerate,
        n_hulls=n_hulls,
        n_decomposed=n_decomposed,
        n_box_fallbacks=n_box_fallbacks,
        n_visuals=n_visuals,
    )


def _collision_hulls(
    tm: Any,
    single_hull: Any,
    prim_name: str,
    *,
    component_vhacd: bool,
) -> tuple[list[Any], bool]:
    hull_volume = float(single_hull.volume)
    component_count = _body_count(tm)
    should_decompose = hull_volume > _SHELL_VOLUME_M3 or (
        component_vhacd and component_count >= _VHACD_MIN_COMPONENTS
    )
    if not should_decompose:
        return [single_hull], False
    try:
        parts = tm.convex_decomposition(
            maxConvexHulls=_VHACD_MAX_HULLS,
            resolution=_VHACD_RESOLUTION,
        )
        hulls = parts if isinstance(parts, list) else [parts]
        logger.info(
            f"  {prim_name}: VHACD decomposed "
            f"({hull_volume:.3g} m³ shell, {component_count} components -> "
            f"{len(hulls)} sub-hulls)"
        )
        return hulls, True
    except Exception as e:
        logger.warning(
            f"  VHACD failed for {prim_name}: {e}; using single hull "
            "(large rooms may collide as a solid shell)"
        )
        return [single_hull], False


def _body_count(tm: Any) -> int:
    try:
        return int(tm.body_count)
    except Exception:
        return 1


def _valid_hull(v: np.ndarray, f: np.ndarray) -> bool:
    if len(v) < 4 or len(f) < 4:
        return False
    extent = v.max(axis=0) - v.min(axis=0)
    if (extent < _DEGENERATE_EPS).any():
        return False
    min_ext = float(extent.min())
    if min_ext < _MIN_HULL_EXTENT_M:
        return False
    centered = v.astype(np.float64) - v.astype(np.float64).mean(axis=0)
    if np.linalg.matrix_rank(centered, tol=_DEGENERATE_EPS) < 3:
        return False
    try:
        from scipy.spatial import ConvexHull, QhullError

        ConvexHull(v, qhull_options="Qt")
    except (QhullError, ValueError):
        return False
    return True


def _fallback_box_geom(name: str, vertices: np.ndarray) -> str | None:
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
    return _BOX_GEOM_LINE.format(
        name=name,
        pos=_fmt_vec(center),
        quat=_fmt_vec(quat),
        size=_fmt_vec(half_size),
    )


def _oriented_box(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    try:
        import trimesh

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
    from scipy.spatial.transform import Rotation

    xyzw = Rotation.from_matrix(rotation).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float64)


def _fmt_vec(values: np.ndarray) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def _write_hull_obj(obj_file: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
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


def _write_wrapper(
    *,
    wrapper_path: Path,
    cache_key: str,
    meshdir: Path,
    robot_mjcf_path: Path,
    asset_lines: list[str],
    geom_lines: list[str],
) -> None:
    wrapper_xml = _WRAPPER_TEMPLATE.format(
        model_name=f"robot_with_scene_{cache_key}",
        meshdir=str(meshdir),
        robot_mjcf_abs=str(robot_mjcf_path),
        asset_meshes="\n".join(asset_lines),
        scene_geoms="\n".join(geom_lines),
    )
    wrapper_path.write_text(wrapper_xml)
    logger.info(f"bake_scene_mjcf: wrote wrapper {wrapper_path}")


def cli_main() -> None:
    """Bake a wrapper, verify it loads, optionally open MuJoCo's native viewer."""
    args = list(sys.argv[1:])
    view = False
    if "--view" in args:
        view = True
        args.remove("--view")
    if len(args) < 2:
        print(
            "usage: python -m dimos.simulation.mujoco.scene_mesh_to_mjcf <scene_path> <robot_mjcf> [scale] [--view]"
        )
        sys.exit(2)
    scene = Path(args[0])
    robot = Path(args[1])
    scale = float(args[2]) if len(args) >= 3 else 0.05
    align = SceneMeshAlignment(scale=scale)
    wrapper = bake_scene_mjcf(scene, robot, alignment=align)
    print(f"wrapper: {wrapper}")

    import mujoco  # type: ignore[import-untyped]

    model = mujoco.MjModel.from_xml_path(str(wrapper))
    print(f"loaded:  {model.nbody} bodies, {model.ngeom} geoms, {model.nmesh} meshes")
    print(f"joints:  {model.njnt}, dof:  {model.nv}")

    if view:
        import mujoco.viewer  # type: ignore[import-untyped]

        viewer: Any = mujoco.viewer

        # ``launch`` runs MuJoCo's interactive viewer with its own
        # internal physics loop.  Blocks until the user closes it.
        # Press F1 in the viewer for the keyboard cheatsheet; ``Tab``
        # toggles the rendering panel where you can switch geom groups
        # (group 3 = our scene collision hulls, group 1 = robot
        # visual mesh, group 0 = robot collision mesh).
        print("\nlaunching MuJoCo viewer (press Esc / close window to exit)")
        viewer.launch(model)


if __name__ == "__main__":
    cli_main()


__all__ = ["bake_scene_mjcf"]
