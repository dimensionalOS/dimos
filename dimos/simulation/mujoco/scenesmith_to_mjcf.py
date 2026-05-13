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

"""Convert a SceneSmith House scene (Drake YAML + per-room SDF) to a MuJoCo MJCF.

The HuggingFace dataset ``nepfaff/scenesmith-example-scenes`` ships each
House/scene_NNN as:

  * ``combined_house/house.dmd.yaml`` — Drake directives describing the
    frame tree (``add_frame``) + which SDFs to load (``add_model``) +
    how to weld them in (``add_weld``).  Furniture also carries a
    ``default_free_body_pose`` with an ``!AngleAxis`` rotation.
  * ``room_geometry/room_geometry_<room>.sdf`` — per-room walls + floor
    box collisions and wall/window/floor visual mesh references (GLTFs).
  * ``room_<room>/generated_assets/furniture/sdf/<sku>/<name>.sdf`` —
    one SDF per piece of furniture, with inertials, mesh visuals, and
    convex-decomposed collision meshes.
  * ``mujoco/meshes/`` — every GLTF visual + every collision OBJ
    pre-converted to ``.obj`` and namespaced (``<room>_<sku>_<name>.obj``).

Only ``scene_186`` ships a pre-baked ``mujoco/scene.xml``; every other
scene number ships only the YAML+SDF graph.  This module reproduces what
SceneSmith's MJCF exporter does: walk the directives, parse each SDF,
compose one MuJoCo XML that references the meshes already on disk.

Naming conventions matched to scene_186's exporter output:

  * Room geometry body  → ``room_geometry_<room>``
  * Room body link      → ``room_geometry_<room>_room_geometry_body_link``
  * Furniture body      → ``<model_name>``                  (e.g. ``bedroom_nightstand_0``)
  * Furniture base link → ``<model_name>_base_link``
  * Geom name pattern   → ``<body>_<sdf-element-name>_<visual|collision>``
  * Mesh name pattern   → ``<geom_name>_mesh``
  * Mesh file pattern   → for room visuals,
        ``../floor_plans/<a>/<b>/<c>/<file>.gltf`` → ``<a>_<b>_<file_stem>.obj``
        (URI dropped components: floor_plans, the per-element subdir).
        For furniture, ``<uri>`` resolves to ``<room>_<sku>_<uri_stem>.obj``.

Coordinate-system conventions:

  * SDF `<pose>x y z r p y</pose>` is XYZ position + RPY (XYZ-Euler) in
    radians; ``X_PF.translation`` in YAML is XYZ in metres; furniture's
    ``rotation: !AngleAxis {angle_deg, axis}`` is degrees + unit axis.
  * MuJoCo body ``pos="x y z" quat="w x y z"`` (Hamiltonian, w-first).

CLI::

    python -m dimos.simulation.mujoco.scenesmith_to_mjcf <scene_dir> [-o <output>]

where ``<scene_dir>`` is the unpacked SceneSmith scene root (the dir
containing ``combined_house/`` and ``room_geometry/``).  The output path
defaults to ``<scene_dir>/mujoco/scene.xml`` (created next to the
existing ``mujoco/meshes/``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
import sys
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import yaml


# ── tiny math helpers ──────────────────────────────────────────────────────
def _axis_angle_deg_to_quat(
    axis: list[float], angle_deg: float
) -> tuple[float, float, float, float]:
    """Drake !AngleAxis → MuJoCo (w, x, y, z) quaternion."""
    axis_arr = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(axis_arr))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    ax = axis_arr / n
    half = np.deg2rad(angle_deg) * 0.5
    s = float(np.sin(half))
    return (float(np.cos(half)), float(ax[0] * s), float(ax[1] * s), float(ax[2] * s))


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """SDF XYZ-Euler (radians) → MuJoCo (w, x, y, z) quaternion."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (float(w), float(x), float(y), float(z))


def _fmt_xyz(v: tuple[float, float, float] | list[float]) -> str:
    return " ".join(f"{c:.6g}" for c in v)


def _fmt_quat(q: tuple[float, float, float, float]) -> str:
    return " ".join(f"{c:.6g}" for c in q)


def _quat_is_identity(q: tuple[float, float, float, float]) -> bool:
    return abs(q[0] - 1.0) < 1e-9 and all(abs(c) < 1e-9 for c in q[1:])


# ── YAML loading with Drake's !AngleAxis tag ───────────────────────────────
class _DrakeLoader(yaml.SafeLoader):
    pass


def _angle_axis_constructor(loader: yaml.Loader, node: yaml.Node) -> dict[str, Any]:
    return loader.construct_mapping(node, deep=True)  # type: ignore[arg-type]


_DrakeLoader.add_constructor("!AngleAxis", _angle_axis_constructor)


# ── parsed types ───────────────────────────────────────────────────────────
@dataclass
class FramePose:
    base_frame: str
    translation: tuple[float, float, float]
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass
class ModelDirective:
    name: str
    file: str  # package://-prefixed
    pose: FramePose | None = None  # furniture only; rooms get welded by frame


@dataclass
class GeomElem:
    name: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    is_collision: bool
    box_size: tuple[float, float, float] | None = None  # half-sizes already (size/2)
    mesh_uri: str | None = None  # raw, as written in the SDF
    mesh_scale: tuple[float, float, float] | None = None  # SDF <scale> if any


@dataclass
class LinkInfo:
    name: str
    geoms: list[GeomElem] = field(default_factory=list)
    mass: float | None = None
    inertial_pos: tuple[float, float, float] | None = None
    inertial_inertia: tuple[float, float, float] | None = None  # diagonal


# ── parsing ────────────────────────────────────────────────────────────────
def _load_directives(yaml_path: Path) -> tuple[dict[str, FramePose], list[ModelDirective]]:
    """Walk a Drake .dmd.yaml file → frames-by-name + ordered model list."""
    raw = yaml.load(yaml_path.read_text(), Loader=_DrakeLoader)
    if not isinstance(raw, dict) or "directives" not in raw:
        raise ValueError(f"{yaml_path}: not a Drake directives file")

    frames: dict[str, FramePose] = {}
    models: list[ModelDirective] = []
    pending_welds: list[dict[str, Any]] = []

    for entry in raw["directives"]:
        if not isinstance(entry, dict) or len(entry) != 1:
            continue
        ((kind, payload),) = entry.items()

        if kind == "add_frame":
            name = payload["name"]
            xpf = payload["X_PF"]
            t = xpf.get("translation", [0.0, 0.0, 0.0])
            rot = xpf.get("rotation")
            quat = (
                _axis_angle_deg_to_quat(rot["axis"], rot["angle_deg"])
                if isinstance(rot, dict) and "angle_deg" in rot
                else (1.0, 0.0, 0.0, 0.0)
            )
            frames[name] = FramePose(
                base_frame=xpf.get("base_frame", "world"),
                translation=(float(t[0]), float(t[1]), float(t[2])),
                quat=quat,
            )

        elif kind == "add_model":
            pose: FramePose | None = None
            dfbp = payload.get("default_free_body_pose")
            if isinstance(dfbp, dict):
                # Single base_link entry, in scene_186's format.
                ((_link_name, link_pose),) = dfbp.items()
                t = link_pose["translation"]
                rot = link_pose.get("rotation")
                quat = (
                    _axis_angle_deg_to_quat(rot["axis"], rot["angle_deg"])
                    if isinstance(rot, dict) and "angle_deg" in rot
                    else (1.0, 0.0, 0.0, 0.0)
                )
                pose = FramePose(
                    base_frame=link_pose.get("base_frame", "world"),
                    translation=(float(t[0]), float(t[1]), float(t[2])),
                    quat=quat,
                )
            models.append(ModelDirective(name=payload["name"], file=payload["file"], pose=pose))

        elif kind == "add_weld":
            # In house_furniture_welded.dmd.yaml the X_PC carries the pose;
            # in house.dmd.yaml furniture pose lives on the model's
            # default_free_body_pose so the weld is purely structural.
            # We collect welds and resolve them after all models are seen.
            pending_welds.append(payload)

    # Apply welds that carry an X_PC for furniture instances (the welded variant).
    by_name = {m.name: m for m in models}
    for w in pending_welds:
        x_pc = w.get("X_PC")
        if not x_pc:
            continue
        child = w.get("child", "")
        if "::" not in child:
            continue
        model_name = child.split("::", 1)[0]
        m = by_name.get(model_name)
        if m is None or m.pose is not None:
            continue
        t = x_pc["translation"]
        rot = x_pc.get("rotation")
        quat = (
            _axis_angle_deg_to_quat(rot["axis"], rot["angle_deg"])
            if isinstance(rot, dict) and "angle_deg" in rot
            else (1.0, 0.0, 0.0, 0.0)
        )
        m.pose = FramePose(
            base_frame=w.get("parent", "world"),
            translation=(float(t[0]), float(t[1]), float(t[2])),
            quat=quat,
        )

    return frames, models


def _parse_pose(
    text: str | None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """SDF ``<pose>x y z r p y</pose>`` → (xyz, quat); missing → identity."""
    if not text:
        return ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    parts = [float(p) for p in text.split()]
    if len(parts) < 6:
        parts += [0.0] * (6 - len(parts))
    xyz = (parts[0], parts[1], parts[2])
    return (xyz, _rpy_to_quat(parts[3], parts[4], parts[5]))


def _parse_link(elem: ET.Element) -> LinkInfo:
    info = LinkInfo(name=elem.get("name", "link"))

    inertial = elem.find("inertial")
    if inertial is not None:
        mass_node = inertial.find("mass")
        if mass_node is not None and mass_node.text:
            info.mass = float(mass_node.text)
        ipose_node = inertial.find("pose")
        if ipose_node is not None:
            info.inertial_pos, _ = _parse_pose(ipose_node.text)
        inertia_node = inertial.find("inertia")
        if inertia_node is not None:
            ixx = inertia_node.findtext("ixx", default="0")
            iyy = inertia_node.findtext("iyy", default="0")
            izz = inertia_node.findtext("izz", default="0")
            info.inertial_inertia = (float(ixx), float(iyy), float(izz))

    for kind, is_collision in (("collision", True), ("visual", False)):
        for node in elem.findall(kind):
            geom = node.find("geometry")
            if geom is None:
                continue
            pos, quat = _parse_pose(node.findtext("pose"))
            box = geom.find("box")
            mesh = geom.find("mesh")
            elem_name = node.get("name", kind)
            if box is not None:
                size_text = box.findtext("size", default="0 0 0")
                sx, sy, sz = (float(p) for p in size_text.split())
                info.geoms.append(
                    GeomElem(
                        name=elem_name,
                        pos=pos,
                        quat=quat,
                        is_collision=is_collision,
                        box_size=(sx * 0.5, sy * 0.5, sz * 0.5),
                    )
                )
            elif mesh is not None:
                uri = mesh.findtext("uri", default="").strip()
                scale_text = mesh.findtext("scale")
                scale: tuple[float, float, float] | None = None
                if scale_text:
                    sx, sy, sz = (float(p) for p in scale_text.split())
                    if not (abs(sx - 1) < 1e-9 and abs(sy - 1) < 1e-9 and abs(sz - 1) < 1e-9):
                        scale = (sx, sy, sz)
                info.geoms.append(
                    GeomElem(
                        name=elem_name,
                        pos=pos,
                        quat=quat,
                        is_collision=is_collision,
                        mesh_uri=uri,
                        mesh_scale=scale,
                    )
                )
    return info


def _parse_sdf(sdf_path: Path) -> tuple[str, list[LinkInfo]]:
    """Return (model_name, [LinkInfo, ...]) for every <link> in the model.

    Some SceneSmith furniture SDFs nest the model under ``<world>`` (older
    scenes use that layout); others put it directly under ``<sdf>``.
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model") or (
        root.find("world/model") if root.find("world") is not None else None
    )
    if model is None:
        raise ValueError(f"{sdf_path}: no <model>")
    return model.get("name", sdf_path.stem), [_parse_link(link) for link in model.findall("link")]


# ── mesh-name resolution ───────────────────────────────────────────────────
_PACKAGE_RE = re.compile(r"^package://[^/]+/(.+)$")


def _resolve_package_uri(uri: str, scene_dir: Path) -> Path:
    """``package://scene/foo/bar.sdf`` → ``<scene_dir>/foo/bar.sdf``."""
    m = _PACKAGE_RE.match(uri)
    if not m:
        # already a relative path
        return scene_dir / uri
    return scene_dir / m.group(1)


def _room_visual_mesh_filename(uri: str) -> str:
    """SceneSmith room-visual URI → ``mujoco/meshes/`` filename.

    Examples (URI on left, expected filename on right):

      ``../floor_plans/bedroom/floors/floor.gltf``           → ``bedroom_floors_floor.obj``
      ``../floor_plans/bedroom/walls/north_wall/wall.gltf``  → ``bedroom_north_wall_wall.obj``
      ``../floor_plans/bedroom/windows/window_1/window.gltf``→ ``bedroom_window_1_window.obj``
      ``../floor_plans/bedroom/walls/north_wall_exterior/wall.gltf`` →
        ``bedroom_north_wall_exterior_wall.obj``

    Rule: drop ``..``/``floor_plans`` and, when there's an explicit
    per-element subdir (i.e. exactly 4 path components remain — room /
    category / element / file), drop the category.  When the file sits
    directly under the category (3 components: room / category / file)
    keep both.
    """
    cleaned = [p for p in Path(uri).parts if p not in ("..", "floor_plans")]
    if not cleaned:
        return Path(uri).stem + ".obj"
    if len(cleaned) >= 4:
        # room / category / element / file.gltf  →  room / element / file_stem
        room, _category, *rest = cleaned
        cleaned = [room, *rest]
    *prefix, last = cleaned
    return "_".join([*prefix, Path(last).stem]) + ".obj"


def _furniture_mesh_filename(
    uri: str, model_file_url: str, scale: tuple[float, float, float] | None = None
) -> str:
    """SDF mesh URI + the URL of the SDF that referenced it → MJCF mesh filename.

    Example: model_file_url
        ``package://scene/room_bedroom/generated_assets/furniture/sdf/nightstand_1767928412/nightstand.sdf``
    URI ``nightstand_collision_0.obj`` → ``bedroom_nightstand_1767928412_nightstand_collision_0.obj``.

    If the SDF mesh node carries a non-identity ``<scale>``, SceneSmith's
    exporter emits a separately scaled .obj and tags the filename with
    ``_sX.XX_Y.YY_Z.ZZ`` (two-decimal rounding).  Reproduce that suffix
    so the lookup hits the existing pre-converted file.
    """
    m = _PACKAGE_RE.match(model_file_url)
    if not m:
        return Path(uri).stem + ".obj"
    parts = Path(
        m.group(1)
    ).parts  # ('room_bedroom', 'generated_assets', 'furniture', 'sdf', '<sku>', '<file>.sdf')
    room = parts[0].removeprefix("room_") if parts and parts[0].startswith("room_") else parts[0]
    sku = parts[-2] if len(parts) >= 2 else "model"
    # Some URIs include a subdir (e.g. ``E_body_1_combined_coacd/convex_piece_000.obj``).
    # SceneSmith flattens the path with ``_`` separators when emitting the
    # mesh into ``mujoco/meshes/``.
    uri_path = Path(uri)
    flattened = "_".join([*uri_path.parent.parts, uri_path.stem]).strip("_")
    base = f"{room}_{sku}_{flattened}"
    if scale is not None:
        # SceneSmith rounds to 2 decimals then strips trailing zeros — so
        # 1.216 → "1.22" but 0.8 → "0.8" (NOT "0.80").  ``:g`` does the
        # trailing-zero strip after we've rounded.
        sx, sy, sz = (round(s, 2) for s in scale)
        base += f"_s{sx:g}_{sy:g}_{sz:g}"
    return base + ".obj"


# ── MJCF emission ──────────────────────────────────────────────────────────
def _set_pos_quat(
    elem: ET.Element, pos: tuple[float, float, float], quat: tuple[float, float, float, float]
) -> None:
    if any(abs(p) > 1e-9 for p in pos):
        elem.set("pos", _fmt_xyz(pos))
    if not _quat_is_identity(quat):
        elem.set("quat", _fmt_quat(quat))


def _emit_mesh_geom(
    parent: ET.Element, name: str, mesh_name: str, is_collision: bool
) -> ET.Element:
    geom = ET.SubElement(parent, "geom")
    geom.set("name", name)
    geom.set("type", "mesh")
    geom.set("mesh", mesh_name)
    if is_collision:
        geom.set("group", "3")
        geom.set("friction", "0.4")
    else:
        # Match scene_186's exporter — visuals are non-colliding.
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    return geom


def _emit_box_geom(
    parent: ET.Element, name: str, half_sizes: tuple[float, float, float]
) -> ET.Element:
    geom = ET.SubElement(parent, "geom")
    geom.set("name", name)
    geom.set("size", _fmt_xyz(half_sizes))
    geom.set("type", "box")
    geom.set("group", "3")  # collision
    return geom


def _emit_room_body(
    worldbody: ET.Element,
    asset: ET.Element,
    model: ModelDirective,
    frames: dict[str, FramePose],
    scene_dir: Path,
    seen_meshes: set[str],
) -> None:
    """Emit the welded-room body and register every visual mesh in <asset>."""
    sdf_path = _resolve_package_uri(model.file, scene_dir)
    if not sdf_path.exists():
        print(f"warn: room SDF not found: {sdf_path}", file=sys.stderr)
        return
    _, links = _parse_sdf(sdf_path)
    if not links:
        return

    room_frame_name = f"room_{model.name.removeprefix('room_geometry_')}_frame"
    frame = frames.get(room_frame_name)
    pos = frame.translation if frame else (0.0, 0.0, 0.0)
    quat = frame.quat if frame else (1.0, 0.0, 0.0, 0.0)

    body = ET.SubElement(worldbody, "body", attrib={"name": model.name})
    _set_pos_quat(body, pos, quat)

    inner = ET.SubElement(body, "body", attrib={"name": f"{model.name}_{links[0].name}"})

    for link in links:
        for g in link.geoms:
            geom_name = (
                f"{model.name}_{link.name}_{g.name}_{'collision' if g.is_collision else 'visual'}"
            )
            if g.box_size is not None:
                box = _emit_box_geom(inner, geom_name, g.box_size)
                _set_pos_quat(box, g.pos, g.quat)
            elif g.mesh_uri:
                mesh_file = _room_visual_mesh_filename(g.mesh_uri)
                mesh_name = f"{geom_name}_mesh"
                if mesh_name not in seen_meshes:
                    ET.SubElement(asset, "mesh", attrib={"name": mesh_name, "file": mesh_file})
                    seen_meshes.add(mesh_name)
                geom = _emit_mesh_geom(inner, geom_name, mesh_name, g.is_collision)
                _set_pos_quat(geom, g.pos, g.quat)


def _resolve_furniture_mesh_file(
    uri: str,
    model_file_url: str,
    scale: tuple[float, float, float] | None,
    meshes_dir: Path,
) -> str | None:
    """Pick the mesh filename actually present under ``meshes_dir``.

    SceneSmith's exporter doesn't normalise naming across scenes — for
    scene_186 every furniture mesh is prefixed with ``<room>_<sku>_`` but
    for scene_209 the visuals are prefixed while the collisions are
    stored unprefixed (one shared copy per SKU).  Try the prefixed name
    first, fall back to the unprefixed URI stem if it isn't on disk.

    Returns ``None`` if no candidate file exists — the caller should skip
    emitting the geom rather than reference a missing mesh.  Some
    SceneSmith scenes ship incomplete mesh exports (visuals shared across
    rooms with different SKUs, only one copy actually written), so
    skipping is the only practical recovery.
    """
    prefixed = _furniture_mesh_filename(uri, model_file_url, scale)
    if (meshes_dir / prefixed).exists():
        return prefixed

    def _scaled(stem: str) -> str:
        if scale is None:
            return stem + ".obj"
        sx, sy, sz = (round(s, 2) for s in scale)
        return f"{stem}_s{sx:g}_{sy:g}_{sz:g}.obj"

    uri_path = Path(uri)

    # Fallback 1 — flattened path, no room/sku prefix (some scene_209 cases).
    flattened = "_".join([*uri_path.parent.parts, uri_path.stem]).strip("_")
    cand = _scaled(flattened)
    if (meshes_dir / cand).exists():
        return cand

    # Fallback 2 — basename only (scene_209 stores convex pieces directly
    # as ``convex_piece_000_s….obj`` without the parent ``E_body_1_combined_coacd``).
    cand = _scaled(uri_path.stem)
    if (meshes_dir / cand).exists():
        return cand

    return None


def _emit_furniture_body(
    worldbody: ET.Element,
    asset: ET.Element,
    model: ModelDirective,
    frames: dict[str, FramePose],
    scene_dir: Path,
    seen_meshes: set[str],
) -> None:
    """Emit a free-body furniture instance with collisions + visual meshes."""
    if model.pose is None:
        return  # not a placed-instance model
    sdf_path = _resolve_package_uri(model.file, scene_dir)
    if not sdf_path.exists():
        print(f"warn: furniture SDF not found: {sdf_path}", file=sys.stderr)
        return
    _, links = _parse_sdf(sdf_path)
    if not links:
        return

    # World-frame pose: parent frame's translation + the furniture's local pose.
    parent = frames.get(model.pose.base_frame)
    parent_t = parent.translation if parent else (0.0, 0.0, 0.0)
    # We don't compose rotations — SceneSmith's room frames ship with
    # identity orientation, so the furniture quat IS its world quat.
    world_pos = (
        parent_t[0] + model.pose.translation[0],
        parent_t[1] + model.pose.translation[1],
        parent_t[2] + model.pose.translation[2],
    )
    world_quat = model.pose.quat

    body = ET.SubElement(worldbody, "body", attrib={"name": model.name})
    _set_pos_quat(body, world_pos, world_quat)
    ET.SubElement(body, "joint", attrib={"name": f"{model.name}_freejoint", "type": "free"})

    for link in links:
        link_body = ET.SubElement(body, "body", attrib={"name": f"{model.name}_{link.name}"})
        if link.mass is not None and link.inertial_inertia is not None:
            attrs: dict[str, str] = {
                "mass": f"{link.mass:.6g}",
                "diaginertia": _fmt_xyz(link.inertial_inertia),
            }
            if link.inertial_pos is not None:
                attrs["pos"] = _fmt_xyz(link.inertial_pos)
            ET.SubElement(link_body, "inertial", attrib=attrs)

        for g in link.geoms:
            geom_name = (
                f"{model.name}_{link.name}_{g.name}_{'collision' if g.is_collision else 'visual'}"
            )
            if g.box_size is not None:
                box = _emit_box_geom(link_body, geom_name, g.box_size)
                _set_pos_quat(box, g.pos, g.quat)
            elif g.mesh_uri:
                mesh_file = _resolve_furniture_mesh_file(
                    g.mesh_uri, model.file, g.mesh_scale, scene_dir / "mujoco" / "meshes"
                )
                if mesh_file is None:
                    print(
                        f"warn: skip {model.name} {g.name}: mesh "
                        f"'{g.mesh_uri}' not in mujoco/meshes/",
                        file=sys.stderr,
                    )
                    continue
                mesh_name = f"{geom_name}_mesh"
                if mesh_name not in seen_meshes:
                    ET.SubElement(asset, "mesh", attrib={"name": mesh_name, "file": mesh_file})
                    seen_meshes.add(mesh_name)
                geom = _emit_mesh_geom(link_body, geom_name, mesh_name, g.is_collision)
                _set_pos_quat(geom, g.pos, g.quat)


# ── public entry point ─────────────────────────────────────────────────────
def convert(scene_dir: Path, output_path: Path | None = None) -> Path:
    """Convert a SceneSmith House scene_NNN dir to a single MJCF file.

    Reads ``<scene_dir>/combined_house/house.dmd.yaml`` plus every SDF it
    references and writes the resulting ``scene.xml`` (default location:
    ``<scene_dir>/mujoco/scene.xml``).  Mesh files are NOT created — they
    must already exist under ``<scene_dir>/mujoco/meshes/`` (SceneSmith
    pre-converts every GLTF / collision OBJ on dataset publish).
    """
    yaml_path = scene_dir / "combined_house" / "house.dmd.yaml"
    if not yaml_path.exists():
        raise FileNotFoundError(f"missing: {yaml_path}")

    if output_path is None:
        output_path = scene_dir / "mujoco" / "scene.xml"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    frames, models = _load_directives(yaml_path)

    root = ET.Element("mujoco", attrib={"model": f"scene_{scene_dir.name}"})
    ET.SubElement(
        root,
        "compiler",
        attrib={
            "angle": "radian",
            "boundmass": "0.001",
            "boundinertia": "0.001",
            "meshdir": "meshes",
            "texturedir": "meshes",
        },
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        attrib={"ambient": "0.4 0.4 0.4", "diffuse": "0.8 0.8 0.8", "specular": "0.1 0.1 0.1"},
    )
    # Match scene_186's bit-mask collision scheme so a robot with
    # contype=4 conaffinity=8 collides cleanly with the scene.
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", attrib={"contype": "8", "conaffinity": "4"})

    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        attrib={
            "type": "skybox",
            "name": "skybox",
            "builtin": "gradient",
            "rgb1": "0.3 0.5 0.7",
            "rgb2": "0 0 0",
            "width": "512",
            "height": "3072",
        },
    )
    ET.SubElement(
        asset,
        "texture",
        attrib={
            "type": "2d",
            "name": "grid",
            "builtin": "checker",
            "mark": "edge",
            "rgb1": "0.2 0.3 0.4",
            "rgb2": "0.1 0.2 0.3",
            "markrgb": "0.8 0.8 0.8",
            "width": "512",
            "height": "512",
        },
    )
    ET.SubElement(
        asset,
        "material",
        attrib={"name": "grid", "texture": "grid", "texrepeat": "10 10"},
    )

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        attrib={
            "name": "floor",
            "size": "15 15 0.1",
            "pos": "0 0 -0.001",
            "type": "plane",
            "contype": "8",
            "conaffinity": "4",  # match scene default — robot foot can stand on it
            "priority": "1",
            "friction": "1.0",
            "material": "grid",
        },
    )

    seen_meshes: set[str] = set()
    for model in models:
        if model.name.startswith("room_geometry_"):
            _emit_room_body(worldbody, asset, model, frames, scene_dir, seen_meshes)
        elif model.pose is not None:
            _emit_furniture_body(worldbody, asset, model, frames, scene_dir, seen_meshes)

    ET.indent(root, "  ")
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding="unicode", xml_declaration=False)
    return output_path


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0] if __doc__ else "")
    parser.add_argument("scene_dir", type=Path, help="Unpacked SceneSmith House/scene_NNN root")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output MJCF path (default: <scene_dir>/mujoco/scene.xml)",
    )
    args = parser.parse_args(argv)

    out = convert(args.scene_dir, args.output)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
