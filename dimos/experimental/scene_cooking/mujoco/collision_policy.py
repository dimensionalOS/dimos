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

"""Per-prim collision-shape decision-making for ``bake_scene_mjcf``.

The bake's job is to turn each USD/glTF/OBJ prim into one or more MuJoCo
``<geom>``s.  This module separates the *decision* (what shape to emit)
from the *emission* (the OBJ/MJCF writing).  Three layers cooperate:

1. **Generic geometric heuristics** — applied to every prim regardless of
   source.  Tiny-prim skip, aspect-ratio box override, near-convex check.
   Safe defaults; no scene-specific knowledge.

2. **Primitive auto-fit** — try OBB box / Ritter sphere / PCA cylinder /
   PCA capsule.  Accept the best fit if
   ``hull_volume / primitive_volume >= fill_threshold``.  Geometric only.

3. **Sidecar overrides** — a JSON file (``<scene>.collision.json`` next
   to the source mesh, or explicit path) with ``fnmatch`` patterns over
   USD prim paths.  Lets users skip lamps, force cylinders on pillars,
   tune CoACD per pattern.  Whoever produces this file (a human, a
   future UE-side extractor, an LLM…) doesn't matter to the bake — the
   sidecar is the contract.

The dispatcher ``decide_for_prim()`` walks: sidecar override → generic
heuristics → primitive auto-fit → CoACD fallback, and returns a
``PrimDecision`` describing the ``<geom>``(s) the wrapper should emit
for the prim.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import fnmatch
import json
from pathlib import Path
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import ConvexHull, QhullError  # type: ignore[import-untyped]

from dimos.experimental.scene_cooking.coacd_util import silence_coacd_logging
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PrimitiveFit = dict[str, Any]
OverrideConfig = dict[str, Any]


# --------------------------------------------------------------------------- #
# Sidecar spec dataclasses                                                    #
# --------------------------------------------------------------------------- #


@dataclass
class CollisionSpec:
    """User-facing collision configuration loaded from ``<scene>.collision.json``.

    Patterns in ``prim_overrides`` are matched with ``fnmatch`` (Unix-shell
    globs) against the USD prim path of each prim — e.g. ``/Root/SM_Pillar_*``.
    First-match wins; iteration order of the dict is preserved (Python 3.7+).

    Each override value is a dict with at minimum ``"type"``:

    - ``"box"`` / ``"sphere"`` / ``"cylinder"`` / ``"capsule"`` / ``"plane"``:
      force the corresponding primitive.  Auto-fit picks the parameters
      unless explicit ``"size"`` / ``"pos"`` / ``"quat"`` is provided.
    - ``"hull"``: force single convex hull, no CoACD.
    - ``"mesh"`` / ``"decimate"``: emit this prim as a mesh geom. Optional
      ``"target_faces"`` simplifies the source mesh before MJCF emission.
    - ``"decompose"``: force CoACD even if auto-fit would have accepted a
      primitive.  Optional ``"max_hulls"`` overrides ``coacd_max_hulls``.
    - ``"skip"``: emit no collision geom.  Visual mesh still drawn.
    - ``"auto"``: same as the global default (useful to scope a pattern
      back to default behaviour inside a wider override).

    Optional override keys:

    - ``"visual"``: ``false`` suppresses non-colliding visual passthrough
      when MuJoCo visual meshes are enabled. This is intended for prims
      extracted into runtime entities so they are not drawn twice.
    - ``"friction"``: list ``[slide, spin, roll]``.
    - ``"min_thickness"``: for ``"box"`` overrides, minimum full
      thickness in metres along world Z.  Useful for authored walkable
      floor slabs exported as zero-thickness sheets.
    - ``"preserve"``: with ``"min_thickness"``, which world-Z face to
      keep fixed while expanding: ``"top"``, ``"bottom"``, or
      ``"center"``.  Defaults to ``"center"``.
    - ``"max_hulls"``: per-pattern CoACD cap.
    - ``"target_faces"``: per-pattern triangle target for ``mesh`` /
      ``decimate`` outputs, or a post-process cap for hull outputs.
    - ``"split_components"``: ``true`` forces this source prim to be split
      into disconnected components before collision fitting.  Use this for
      scene-graph nodes that group unrelated architectural pieces under one
      mesh and otherwise produce oversized convex hulls.
    """

    #: Fallback policy when no pattern matches.  ``"auto"`` runs the full
    #: heuristics→primitive→CoACD pipeline.  ``"hull"`` always emits one
    #: convex hull.  ``"skip"`` emits nothing (visual only).
    default: Literal["auto", "hull", "skip"] = "auto"

    #: A primitive is accepted in auto-fit if
    #: ``hull_volume / primitive_volume >= fill_threshold``.  Higher =
    #: stricter (more prims fall through to CoACD).
    fill_threshold: float = 0.85

    #: Prims whose largest extent is below this (metres) emit no geom.
    #: Catches trim/fasteners that the robot can't meaningfully contact.
    tiny_prim_extent_m: float = 0.03

    #: If one axis is ``>= aspect_ratio_box`` times the smaller two, the
    #: prim is forced to a box even if auto-fit fill ratio is borderline.
    #: Catches wall panels, floor slabs, doors.
    aspect_ratio_box: float = 20.0

    #: If mesh's hull is this close to its actual mesh volume, use one
    #: hull and skip CoACD entirely (mesh is already near-convex).
    near_convex_threshold: float = 0.9

    #: CoACD concavity threshold (URLab default).  Lower = finer
    #: decomposition (more sub-hulls).
    coacd_threshold: float = 0.05

    #: Hard cap on hulls per CoACD invocation.  -1 = unlimited.
    coacd_max_hulls: int = 64

    #: Only run decomposition when the prim's single-hull volume exceeds
    #: this (m³).  Smaller furniture-scale prims use one hull regardless.
    shell_volume_m3: float = 2.0

    #: Preserve large non-rectangular sheet footprints with thin triangle
    #: prisms.  This helps moderate indoor scenes with angular floors, but
    #: is disabled by ``bake_scene_mjcf`` for very large scenes unless
    #: explicitly overridden.
    enable_sheet_prisms: bool = True

    #: Scene-level guard used by ``bake_scene_mjcf``.  Above this many
    #: source prims, sheet prisms can explode the geom count; use sidecar
    #: overrides for specific floors instead.
    sheet_prism_max_scene_prims: int = 2500

    #: ``USD-path-glob -> override-dict``.  See class docstring.
    prim_overrides: dict[str, OverrideConfig] = field(default_factory=dict)

    #: Split suspicious scene-graph nodes that are really many disconnected
    #: tiny meshes spread over a large area before running primitive fitting.
    split_disconnected_components: bool = True

    #: Minimum component count before a prim is considered grouped clutter.
    split_min_components: int = 8

    #: Combined prim extent must be this much larger than the median component
    #: extent before splitting.  This avoids splitting normal multi-part props.
    split_extent_ratio: float = 4.0

    #: A prim must span at least this far before the cooker spends time
    #: checking disconnected components.  This keeps normal props cheap.
    split_prim_min_extent_m: float = 5.0

    #: The prim must also be slab-like by axis ratio before splitting.
    #: This targets the path that can otherwise emit one giant box.
    split_axis_ratio: float = 20.0

    #: Components smaller than this largest extent are dropped after splitting.
    #: This catches leaves, cups, bottles, and other decorative clutter that can
    #: destabilize navigation collision while being too small to matter.
    split_component_min_extent_m: float = 0.15

    #: Very small triangle islands are dropped after splitting.
    split_component_min_faces: int = 16

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> CollisionSpec:
        """Build a collision spec from decoded JSON.

        Unknown keys are ignored to keep authored sidecars forwards-compatible.
        """
        known = {
            "default",
            "fill_threshold",
            "tiny_prim_extent_m",
            "aspect_ratio_box",
            "near_convex_threshold",
            "coacd_threshold",
            "coacd_max_hulls",
            "shell_volume_m3",
            "enable_sheet_prisms",
            "sheet_prism_max_scene_prims",
            "prim_overrides",
            "split_disconnected_components",
            "split_min_components",
            "split_extent_ratio",
            "split_prim_min_extent_m",
            "split_axis_ratio",
            "split_component_min_extent_m",
            "split_component_min_faces",
        }
        kwargs = {k: v for k, v in raw.items() if k in known}
        # Ignore "$schema" and any future top-level keys silently.
        return cls(**kwargs)

    @classmethod
    def from_json(cls, path: Path | str) -> CollisionSpec:
        """Load a sidecar.  Unknown keys are ignored to keep the format forwards-compatible."""
        path = Path(path)
        raw = json.loads(path.read_text())
        return cls.from_dict(raw)

    @classmethod
    def auto_discover(cls, scene_path: Path | str) -> CollisionSpec:
        """Return the sidecar next to ``scene_path`` if it exists, else defaults."""
        scene_path = Path(scene_path)
        sidecar = scene_path.with_suffix(".collision.json")
        if sidecar.exists():
            logger.info(f"loading collision sidecar: {sidecar}")
            return cls.from_json(sidecar)
        return cls()

    def resolve(self, prim_path: str) -> OverrideConfig:
        """Find the matching override for ``prim_path`` (USD path).

        Returns a dict with at least ``"type"``.  Falls back to
        ``{"type": self.default}`` when no pattern matches.
        """
        stripped = prim_path.lstrip("/")
        sanitized = "".join(c if c.isalnum() else "_" for c in stripped)
        candidates = (prim_path, stripped, sanitized)
        for pattern, override in self.prim_overrides.items():
            if any(fnmatch.fnmatchcase(candidate, pattern) for candidate in candidates):
                # Pattern's "auto" defers to global default.
                if override.get("type") == "auto":
                    return {**override, "type": self.default}
                return override
        return {"type": self.default}


# --------------------------------------------------------------------------- #
# Math helpers                                                                #
# --------------------------------------------------------------------------- #


def _matrix_to_quat_wxyz(R: NDArray[np.float64]) -> tuple[float, float, float, float]:
    """3x3 right-handed rotation → quaternion ``(w, x, y, z)``.

    Standard Shepperd's method; avoids the singularity at ``trace == -1``.
    """
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


def _quat_z_to(axis: NDArray[np.float64]) -> tuple[float, float, float, float]:
    """Quaternion that rotates ``+Z`` onto ``axis`` (unit vector).

    Used for cylinder/capsule placement — MuJoCo's primitive long-axis
    is local +Z; we orient by aligning Z to the prim's PCA principal
    direction.
    """
    z = np.array([0.0, 0.0, 1.0])
    a = axis / (np.linalg.norm(axis) + 1e-12)
    d = float(np.dot(z, a))
    if d > 0.99999:
        return (1.0, 0.0, 0.0, 0.0)
    if d < -0.99999:
        # 180° about any axis perpendicular to Z; use X.
        return (0.0, 1.0, 0.0, 0.0)
    cross = np.cross(z, a)
    s = float(np.sqrt(2.0 * (1.0 + d)))
    w = s * 0.5
    xyz = cross / s
    return (w, float(xyz[0]), float(xyz[1]), float(xyz[2]))


# --------------------------------------------------------------------------- #
# Primitive fits                                                              #
# --------------------------------------------------------------------------- #


#: Lower bound on any primitive's half-extent / radius.  MuJoCo rejects
#: a ``<geom>`` with any size component <= 0, but truly flat prims (road
#: tiles, ceiling panels) can come out of the OBB / cylinder fit with one
#: axis at exactly 0.  Clamping to 1 mm yields a valid geom that's still
#: physically reasonable as a thin slab.
_MIN_SIZE_M = 1e-3
_SHEET_PRISM_THICKNESS_M = 0.03
_SHEET_BOX_FILL_MIN = 0.85
_SHEET_BOX_FILL_MAX = 1.15
_HORIZONTAL_BOX_MAX_THICKNESS_M = 0.05
_SHEET_PRISM_MIN_FOOTPRINT_AREA_M2 = 2.0
_SHEET_PRISM_MAX_TRIANGLES = 1024


def _fit_aabb_box(vertices: NDArray[np.float64]) -> PrimitiveFit:
    """Axis-aligned bounding box.  Identity quat."""
    mn, mx = vertices.min(0), vertices.max(0)
    half_ext = np.maximum((mx - mn) / 2.0, _MIN_SIZE_M)
    center = (mx + mn) / 2.0
    return {
        "type": "box",
        "size": tuple(map(float, half_ext)),
        "pos": tuple(map(float, center)),
        "quat": (1.0, 0.0, 0.0, 0.0),
        "volume": float(np.prod(2.0 * half_ext)),
    }


def _fit_obb_box(vertices: NDArray[np.float64]) -> PrimitiveFit:
    """Oriented bounding box via PCA.  Tighter than AABB when the prim
    is rotated relative to world axes (most UE props are world-aligned,
    so OBB ≈ AABB, but rotated assets benefit)."""
    centroid = vertices.mean(0)
    centered = vertices - centroid
    cov = np.cov(centered.T)
    _, evecs = np.linalg.eigh(cov)
    # Ensure right-handed.
    if np.linalg.det(evecs) < 0:
        evecs[:, 0] = -evecs[:, 0]
    local = centered @ evecs
    mn, mx = local.min(0), local.max(0)
    half_ext = np.maximum((mx - mn) / 2.0, _MIN_SIZE_M)
    local_center = (mx + mn) / 2.0
    world_center = centroid + evecs @ local_center
    return {
        "type": "box",
        "size": tuple(map(float, half_ext)),
        "pos": tuple(map(float, world_center)),
        "quat": _matrix_to_quat_wxyz(evecs),
        "volume": float(np.prod(2.0 * half_ext)),
    }


def _fit_sphere(vertices: NDArray[np.float64]) -> PrimitiveFit:
    """Centroid + farthest-vertex.  Looser than Welzl/Ritter but fine for
    fill-ratio comparison."""
    centroid = vertices.mean(0)
    r = max(float(np.linalg.norm(vertices - centroid, axis=1).max()), _MIN_SIZE_M)
    return {
        "type": "sphere",
        "size": (r,),
        "pos": tuple(map(float, centroid)),
        "quat": (1.0, 0.0, 0.0, 0.0),
        "volume": float((4.0 / 3.0) * np.pi * r**3),
    }


def _fit_cylinder(vertices: NDArray[np.float64]) -> PrimitiveFit:
    """Cylinder along PCA principal axis."""
    centroid = vertices.mean(0)
    centered = vertices - centroid
    cov = np.cov(centered.T)
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, -1]  # largest variance
    proj = centered @ axis
    half_h = max(float((proj.max() - proj.min()) / 2.0), _MIN_SIZE_M)
    centre_along = float((proj.max() + proj.min()) / 2.0)
    pos = centroid + axis * centre_along
    # radius = max perp distance from axis line
    perp = centered - np.outer(centered @ axis, axis)
    r = max(float(np.linalg.norm(perp, axis=1).max()), _MIN_SIZE_M)
    return {
        "type": "cylinder",
        "size": (r, half_h),
        "pos": tuple(map(float, pos)),
        "quat": _quat_z_to(axis),
        "volume": float(np.pi * r * r * 2.0 * half_h),
    }


def _fit_capsule(vertices: NDArray[np.float64]) -> PrimitiveFit:
    """Capsule along PCA principal axis.  MuJoCo capsule half-height is
    the *cylindrical* portion only; total length = 2*(half_h + r)."""
    cyl = _fit_cylinder(vertices)
    r, h = cyl["size"]
    new_h = max(float(h - r), _MIN_SIZE_M)
    vol = float(np.pi * r * r * 2.0 * new_h) + float((4.0 / 3.0) * np.pi * r**3)
    return {
        "type": "capsule",
        "size": (r, new_h),
        "pos": cyl["pos"],
        "quat": cyl["quat"],
        "volume": vol,
    }


def _hull_volume(vertices: NDArray[np.float64]) -> float | None:
    """Convex-hull volume in m³, or ``None`` if qhull rejects the points."""
    try:
        return float(ConvexHull(vertices).volume)
    except (QhullError, ValueError):
        return None


def _mesh_volume(vertices: NDArray[np.float64], triangles: NDArray[np.int32]) -> float:
    """Signed mesh volume (Gauss / divergence theorem on triangle fans).

    Closed meshes return a positive number; for non-closed inputs the
    absolute value is a coarse estimate."""
    v0 = vertices[triangles[:, 0]]
    v1 = vertices[triangles[:, 1]]
    v2 = vertices[triangles[:, 2]]
    return float(abs(np.sum(np.einsum("ij,ij->i", v0, np.cross(v1, v2))) / 6.0))


def _best_primitive_fit(
    vertices: NDArray[np.float64],
    hull_vol: float,
    candidates: tuple[str, ...] = ("box", "cylinder", "sphere", "capsule"),
) -> PrimitiveFit | None:
    """Try every primitive in ``candidates``; return the one with the
    highest fill ratio.  Returns ``None`` if no fit succeeds (e.g. < 4
    points)."""
    fitters = {
        "box": _fit_obb_box,
        "sphere": _fit_sphere,
        "cylinder": _fit_cylinder,
        "capsule": _fit_capsule,
    }
    fits: list[PrimitiveFit] = []
    for kind in candidates:
        try:
            f = fitters[kind](vertices)
            if f["volume"] <= 0:
                continue
            f["fill_ratio"] = hull_vol / f["volume"]
            fits.append(f)
        except (np.linalg.LinAlgError, ValueError, ZeroDivisionError) as e:
            logger.warning(f"  primitive fit {kind} failed: {e}")
    if not fits:
        return None
    return max(fits, key=lambda f: f["fill_ratio"])


# --------------------------------------------------------------------------- #
# Generic geometric heuristics                                                #
# --------------------------------------------------------------------------- #


def _is_tiny(extent: NDArray[np.float64], threshold_m: float) -> bool:
    return bool(extent.max() < threshold_m)


def _is_slab(extent: NDArray[np.float64], aspect_ratio: float) -> bool:
    """Wall / floor / door / panel — one axis is much smaller than the
    other two (or one much larger than the other two — covers beams)."""
    sorted_ext = np.sort(extent)
    if sorted_ext[0] < 1e-6:
        return True
    return bool((sorted_ext[2] / sorted_ext[0]) >= aspect_ratio)


def _sheet_footprint_stats(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    thin_axis: int,
) -> tuple[float, float] | None:
    """Return ``(projected_aabb_area, projected_triangle_fill)`` for a sheet."""
    axes = [i for i in range(3) if i != thin_axis]
    projected = vertices[:, axes]
    span = projected.max(axis=0) - projected.min(axis=0)
    box_area = float(span[0] * span[1])
    if box_area < 1e-9:
        return None

    tri = projected[triangles]
    edge_a = tri[:, 1] - tri[:, 0]
    edge_b = tri[:, 2] - tri[:, 0]
    area = 0.5 * np.abs(edge_a[:, 0] * edge_b[:, 1] - edge_a[:, 1] * edge_b[:, 0]).sum()
    fill = float(area / box_area)
    return box_area, fill


def _is_boxlike_sheet(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    thin_axis: int,
) -> bool:
    """Whether a thin mesh roughly fills its projected bounding rectangle.

    A single primitive box is only acceptable when the source sheet's
    projected triangle area is close to the projected AABB area.  Low
    ratios mean an L-shape / beam strip / holes; high ratios usually mean
    overlapping, folded, or angled sheets inside one prim.
    """
    stats = _sheet_footprint_stats(vertices, triangles, thin_axis)
    if stats is None:
        return False
    _, fill = stats
    return _SHEET_BOX_FILL_MIN <= fill <= _SHEET_BOX_FILL_MAX


def _should_emit_triangle_prisms(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    thin_axis: int,
) -> bool:
    """Use exact-ish triangle prisms only for large horizontal sheets.

    This avoids placing huge slabs over angular floors and roof strips,
    without exploding tiny decorative meshes into thousands of geoms.
    """
    if thin_axis != 2:
        return False
    if len(triangles) > _SHEET_PRISM_MAX_TRIANGLES:
        return False
    stats = _sheet_footprint_stats(vertices, triangles, thin_axis)
    if stats is None:
        return False
    footprint_area, _ = stats
    return footprint_area >= _SHEET_PRISM_MIN_FOOTPRINT_AREA_M2


def _thin_sheet_hulls(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    thickness: float = _SHEET_PRISM_THICKNESS_M,
) -> list[tuple[NDArray[np.float32], NDArray[np.int32]]]:
    """Represent a thin non-rectangular sheet as convex triangle prisms."""
    hulls: list[tuple[NDArray[np.float32], NDArray[np.int32]]] = []
    faces = np.asarray(
        [
            [0, 1, 2],
            [5, 4, 3],
            [0, 3, 4],
            [0, 4, 1],
            [1, 4, 5],
            [1, 5, 2],
            [2, 5, 3],
            [2, 3, 0],
        ],
        dtype=np.int32,
    )

    for tri_idx in triangles:
        tri = vertices[tri_idx].astype(np.float64)
        if not np.isfinite(tri).all():
            continue
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        offset = normal / norm * (thickness * 0.5)
        prism = np.vstack((tri + offset, tri - offset)).astype(np.float32)
        hulls.append((prism, faces))

    return hulls


def _is_flat_horizontal_box(extent: NDArray[np.float64], thin_axis: int) -> bool:
    """Thin in world Z, broad in world X/Y, and flat enough to box safely.

    PCA boxes are unstable for nearly flat floors/ceilings: any small
    triangulation asymmetry can rotate the OBB basis and turn a walkable
    surface into a shallow ramp.  For world-horizontal slabs, the AABB is
    the physically safer collision approximation.
    """
    if thin_axis != 2:
        return False
    xy_min = float(min(extent[0], extent[1]))
    z_extent = float(extent[2])
    if xy_min < 1e-6:
        return False
    return z_extent <= _HORIZONTAL_BOX_MAX_THICKNESS_M


# --------------------------------------------------------------------------- #
# Dispatcher: per-prim decision                                               #
# --------------------------------------------------------------------------- #


@dataclass
class PrimDecision:
    """What the dispatcher decided for one prim.  Consumed by the bake
    which materialises MJCF ``<geom>`` lines and writes OBJs."""

    #: ``"skip"`` (no collision), ``"primitive"`` (one ``<geom>`` with
    #: kind ∈ {box, sphere, cylinder, capsule, plane}), or ``"hulls"``
    #: (one or more mesh ``<geom>``s from convex-hull decomposition).
    mode: Literal["skip", "primitive", "hulls"]

    #: For ``"primitive"``: the fit dict (``type``, ``size``, ``pos``,
    #: ``quat``, ``volume``, ``fill_ratio``).
    primitive: PrimitiveFit | None = None

    #: For ``"hulls"``: list of ``(vertices, triangles)`` ready to write.
    #: Vertex precision varies by source (CoACD/sheet-prisms emit float32,
    #: single-hull pass-through keeps the input prim's float64).
    hulls: list[tuple[NDArray[np.floating[Any]], NDArray[np.int32]]] = field(default_factory=list)

    #: For diagnostics: which rule fired.
    reason: str = ""

    #: Optional friction override from sidecar.
    friction: tuple[float, float, float] | None = None

    #: Optional per-mesh triangle cap from sidecar.  The bake applies this
    #: before writing OBJ assets for mesh geoms.
    target_faces: int | None = None


def decide_for_prim(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    prim_path: str,
    spec: CollisionSpec,
) -> PrimDecision:
    """Resolve sidecar + heuristics + auto-fit for a single prim.

    Pure function — does no I/O.  The caller (bake) materialises the
    decision: writes hull OBJs to disk, emits MJCF lines.

    Args:
        vertices: ``(N, 3)`` float, world-frame after ``SceneMeshAlignment``.
        triangles: ``(M, 3)`` int vertex indices.
        prim_path: USD-style prim path used as sidecar key.
        spec: parsed sidecar.
    """
    extent = vertices.max(0) - vertices.min(0)
    override = spec.resolve(prim_path)
    kind = override.get("type", spec.default)
    friction = override.get("friction")
    if friction is not None:
        friction = tuple(float(x) for x in friction)
    target_faces = _target_faces(override)

    # 0. Explicit "skip" — short-circuit.
    if kind == "skip":
        return PrimDecision(mode="skip", reason="sidecar:skip", friction=friction)

    # 1. Tiny-prim guard (applies to "auto" path; explicit overrides win).
    if kind in ("auto",) and _is_tiny(extent, spec.tiny_prim_extent_m):
        return PrimDecision(mode="skip", reason="tiny-prim", friction=friction)

    # 2. Explicit primitive in sidecar — fit if size/pos not provided.
    if kind in ("box", "sphere", "cylinder", "capsule", "plane"):
        fit = _resolve_explicit_primitive(vertices, kind, override)
        fit["fill_ratio"] = float("nan")  # unknown — user asserted this shape
        return PrimDecision(
            mode="primitive", primitive=fit, reason=f"sidecar:{kind}", friction=friction
        )

    # 3. Explicit hull / decompose paths.
    if kind == "hull":
        return PrimDecision(
            mode="hulls",
            hulls=[(vertices, triangles)],  # signal: single-hull, no decomp
            reason="sidecar:hull",
            friction=friction,
            target_faces=target_faces,
        )
    if kind == "decompose":
        max_h = int(override.get("max_hulls", spec.coacd_max_hulls))
        hulls = _coacd_decompose(vertices, triangles, spec.coacd_threshold, max_h)
        return PrimDecision(
            mode="hulls",
            hulls=hulls,
            reason="sidecar:decompose",
            friction=friction,
            target_faces=target_faces,
        )
    if kind in {"mesh", "decimate"}:
        return PrimDecision(
            mode="hulls",
            hulls=[(vertices, triangles)],
            reason=f"sidecar:{kind}",
            friction=friction,
            target_faces=target_faces,
        )

    # 4. From here on: kind == "auto".  Generic heuristics first.

    # 4a. Aspect-ratio: slab/beam → force box (fill ratio may be
    # marginal because of moulding/profile, but a box collision is
    # the right physical answer for walls and slabs).  Non-rectangular
    # sheets are emitted as triangle prisms so we don't fill holes or
    # angular roof/floor outlines with one huge invisible slab.
    if _is_slab(extent, spec.aspect_ratio_box):
        thin_axis = int(np.argmin(extent))
        if (
            spec.enable_sheet_prisms
            and not _is_boxlike_sheet(vertices, triangles, thin_axis)
            and _should_emit_triangle_prisms(vertices, triangles, thin_axis)
        ):
            hulls = _thin_sheet_hulls(vertices, triangles)
            if hulls:
                return PrimDecision(
                    mode="hulls",
                    hulls=hulls,
                    reason=f"thin-sheet:triangle-prisms({len(hulls)})",
                    friction=friction,
                )

        if _is_flat_horizontal_box(extent, thin_axis):
            fit = _fit_aabb_box(vertices)
            reason = "aspect-ratio:horizontal-slab"
        else:
            fit = _fit_obb_box(vertices)
            reason = "aspect-ratio:slab"
        fit["fill_ratio"] = float("nan")
        return PrimDecision(mode="primitive", primitive=fit, reason=reason, friction=friction)

    # 4b. Need hull volume for the rest.
    hull_vol = _hull_volume(vertices)
    if hull_vol is None:
        return PrimDecision(mode="skip", reason="degenerate (qhull rejected)", friction=friction)

    # 4c. Try primitive auto-fit.
    auto_fit = _best_primitive_fit(vertices, hull_vol)
    if auto_fit is not None and 0.0 < auto_fit["fill_ratio"] <= 1.5:
        # fill_ratio > 1 happens for non-closed hulls; cap to keep this
        # finite when reporting.  Accept if within tolerance.
        if auto_fit["fill_ratio"] >= spec.fill_threshold:
            return PrimDecision(
                mode="primitive",
                primitive=auto_fit,
                reason=f"auto:{auto_fit['type']}({auto_fit['fill_ratio']:.2f})",
                friction=friction,
            )

    # 4d. Near-convex shortcut: skip CoACD, single hull.
    mesh_vol = _mesh_volume(vertices, triangles)
    if hull_vol > 0 and mesh_vol / hull_vol > spec.near_convex_threshold:
        return PrimDecision(
            mode="hulls",
            hulls=[(vertices, triangles)],
            reason=f"near-convex({mesh_vol / hull_vol:.2f})",
            friction=friction,
        )

    # 4e. Small concave prim → single hull (matches today's behaviour
    # for furniture-scale things; faster than CoACD).
    if hull_vol < spec.shell_volume_m3:
        return PrimDecision(
            mode="hulls",
            hulls=[(vertices, triangles)],
            reason="small-shell:single-hull",
            friction=friction,
        )

    # 4f. Large concave shell → CoACD.
    hulls = _coacd_decompose(vertices, triangles, spec.coacd_threshold, spec.coacd_max_hulls)
    return PrimDecision(
        mode="hulls",
        hulls=hulls,
        reason=f"coacd:{len(hulls)}",
        friction=friction,
    )


# --------------------------------------------------------------------------- #
# Helpers used by the dispatcher                                              #
# --------------------------------------------------------------------------- #


def _resolve_explicit_primitive(
    vertices: NDArray[np.float64],
    kind: str,
    override: OverrideConfig,
) -> PrimitiveFit:
    """Build a primitive fit dict from a sidecar override.

    If the override supplies ``size`` (and optionally ``pos`` / ``quat``),
    those win.  Otherwise we auto-fit the requested primitive and use
    those params.  ``plane`` is special-cased — we always derive from
    the prim's xy footprint at its min z.
    """
    if kind == "plane":
        mn = vertices.min(0)
        mx = vertices.max(0)
        return {
            "type": "plane",
            "size": (float((mx[0] - mn[0]) / 2.0), float((mx[1] - mn[1]) / 2.0), 0.5),
            "pos": (
                float((mx[0] + mn[0]) / 2.0),
                float((mx[1] + mn[1]) / 2.0),
                float(mn[2]),
            ),
            "quat": (1.0, 0.0, 0.0, 0.0),
            "volume": 0.0,
        }

    box_fitter = _fit_aabb_box if "min_thickness" in override else _fit_obb_box
    fitters = {
        "box": box_fitter,
        "sphere": _fit_sphere,
        "cylinder": _fit_cylinder,
        "capsule": _fit_capsule,
    }
    fit = fitters[kind](vertices)
    # Apply explicit overrides if provided.
    if "size" in override:
        fit["size"] = tuple(float(x) for x in override["size"])
    if "pos" in override:
        fit["pos"] = tuple(float(x) for x in override["pos"])
    if "quat" in override:
        fit["quat"] = tuple(float(x) for x in override["quat"])
    if kind == "box":
        _apply_box_min_thickness(fit, vertices, override)
    return fit


def _apply_box_min_thickness(
    fit: PrimitiveFit,
    vertices: NDArray[np.float64],
    override: OverrideConfig,
) -> None:
    raw_min_thickness = override.get("min_thickness")
    if raw_min_thickness is None:
        return

    min_half_z = max(float(raw_min_thickness) * 0.5, _MIN_SIZE_M)
    size = np.asarray(fit["size"], dtype=np.float64)
    pos = np.asarray(fit["pos"], dtype=np.float64)
    if size.shape[0] < 3 or pos.shape[0] < 3 or size[2] >= min_half_z:
        return

    old_half_z = float(size[2])
    size[2] = min_half_z

    preserve = str(override.get("preserve", "center")).lower()
    if preserve == "top":
        top_z = (
            pos[2] + old_half_z
            if "pos" in override or "size" in override
            else float(np.max(vertices[:, 2]))
        )
        pos[2] = top_z - min_half_z
    elif preserve == "bottom":
        bottom_z = (
            pos[2] - old_half_z
            if "pos" in override or "size" in override
            else float(np.min(vertices[:, 2]))
        )
        pos[2] = bottom_z + min_half_z
    elif preserve in {"center", "centre"}:
        pass
    else:
        raise ValueError("box min_thickness preserve must be one of: top, bottom, center")

    fit["size"] = tuple(map(float, size))
    fit["pos"] = tuple(map(float, pos))


def _target_faces(override: OverrideConfig) -> int | None:
    raw = override.get("target_faces", override.get("max_faces"))
    if raw is None:
        return None
    target_faces = int(raw)
    if target_faces <= 0:
        return None
    return max(4, target_faces)


def _coacd_decompose(
    vertices: NDArray[np.float64],
    triangles: NDArray[np.int32],
    threshold: float,
    max_hulls: int,
) -> list[tuple[NDArray[np.float32], NDArray[np.int32]]]:
    """Run CoACD on a single prim, return list of ``(verts, tris)`` hulls.

    CoACD is imported lazily — it ships its own C library and we don't
    want every import of ``collision_spec`` to pay that cost.
    """
    # open3d must load before coacd — importing coacd into a process that
    # later loads open3d segfaults open3d's extension module (clashing
    # vendored native libs). This function only needs numpy, so force the
    # safe order for whatever loads open3d afterwards.
    # isort: off
    import open3d  # noqa: F401  # type: ignore[import-untyped]
    import coacd  # type: ignore[import-not-found, import-untyped]

    # isort: on

    silence_coacd_logging()

    mesh = coacd.Mesh(vertices.astype(np.float64), triangles.astype(np.int32))
    # CoACD's MCTS defaults (mcts_iterations=150, resolution=2000) are tuned
    # for offline asset prep — minutes per shell on a multi-thousand-prim
    # scene.  We dial both down ~5x; the resulting hulls are slightly
    # noisier but the bake finishes in minutes, not hours.  For a one-off
    # final bake users can override via the sidecar (future work) or call
    # ``bake_scene_mjcf`` directly with a custom ``CollisionSpec``.
    parts = coacd.run_coacd(
        mesh,
        threshold=threshold,
        max_convex_hull=max_hulls,
        resolution=500,
        mcts_iterations=30,
        mcts_nodes=10,
    )
    out: list[tuple[NDArray[np.float32], NDArray[np.int32]]] = []
    for v, t in parts:
        v = np.asarray(v, dtype=np.float32)
        t = np.asarray(t, dtype=np.int32)
        if len(v) >= 4 and len(t) >= 1:
            out.append((v, t))
    return out
