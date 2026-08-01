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

"""URDF visual logging helpers for Rerun."""

from __future__ import annotations

from collections.abc import Iterable
from math import cos, sin
from pathlib import Path
from types import ModuleType
import xml.etree.ElementTree as ET


def urdf_visuals_to_rerun(
    rr: ModuleType,
    urdf_path: Path | str,
    *,
    entity_prefix: str = "world/robot",
    axes_scale: float = 0.04,
    highlight_links: Iterable[str] = (),
) -> list[tuple[str, object]]:
    """Convert URDF link visuals into static Rerun asset logs.

    Dynamic link poses are expected to be logged separately as Rerun TF/entity
    transforms. This helper attaches each ``{entity_prefix}/{link}`` entity to
    the dynamic ``tf#/{link}`` frame, then logs visual meshes and axes below it
    so they follow the corresponding link transform in Rerun.
    """

    root = ET.parse(urdf_path).getroot()
    entries: list[tuple[str, object]] = []
    highlighted = set(highlight_links)
    for link in root.findall("link"):
        link_name = link.attrib.get("name")
        if not link_name:
            continue

        link_path = f"{entity_prefix}/{link_name}"
        entries.append((link_path, rr.Transform3D(parent_frame=f"tf#/{link_name}")))
        scale = axes_scale * 2.5 if link_name in highlighted else axes_scale
        entries.append((f"{link_path}/axes", _link_axes(rr, scale, link_name=link_name)))
        if link_name in highlighted:
            entries.append((f"{link_path}/origin", _link_origin(rr, link_name)))
        for index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.attrib.get("filename")
            if not filename:
                continue
            mesh_path = Path(filename)
            if not mesh_path.exists():
                continue

            origin = visual.find("origin")
            translation = _parse_triplet(origin.attrib.get("xyz") if origin is not None else None)
            rpy = _parse_triplet(origin.attrib.get("rpy") if origin is not None else None)
            scale = _parse_triplet(mesh.attrib.get("scale"), default=(1.0, 1.0, 1.0))
            visual_path = f"{link_path}/visual_{index}"
            entries.append(
                (
                    visual_path,
                    rr.Transform3D(
                        translation=translation,
                        rotation=rr.Quaternion(xyzw=_quat_xyzw_from_rpy(*rpy)),
                        scale=scale,
                    ),
                )
            )
            entries.append((visual_path, rr.Asset3D(path=mesh_path)))

    entries.extend(_scene_axes(rr))
    return entries


def _link_axes(rr: ModuleType, scale: float, *, link_name: str) -> object:
    return rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]] * 3,
        vectors=[[scale, 0.0, 0.0], [0.0, scale, 0.0], [0.0, 0.0, scale]],
        colors=[[255, 60, 60], [60, 255, 60], [80, 140, 255]],
        labels=[f"{link_name} +X", f"{link_name} +Y", f"{link_name} +Z"],
        show_labels=True,
    )


def _link_origin(rr: ModuleType, link_name: str) -> object:
    return rr.Points3D(
        positions=[[0.0, 0.0, 0.0]],
        colors=[[255, 0, 255]],
        radii=[0.018],
        labels=[link_name],
        show_labels=True,
    )


def _scene_axes(rr: ModuleType) -> list[tuple[str, object]]:
    return [
        (
            "world",
            # MuJoCo/xArm scenes use a robotics world convention: +X points out
            # from the robot toward the table, +Y is lateral, and +Z is up.
            # Without this, Rerun's default 3D basis can make the whole scene
            # look rolled even when all relative transforms are correct.
            rr.ViewCoordinates.FLU,
        ),
        (
            "world/debug/scene_axes",
            rr.Arrows3D(
                origins=[[0.0, 0.0, 0.0]] * 3,
                vectors=[[0.20, 0.0, 0.0], [0.0, 0.20, 0.0], [0.0, 0.0, 0.20]],
                colors=[[255, 60, 60], [60, 255, 60], [80, 140, 255]],
                labels=["world +X table", "world +Y lateral", "world +Z up"],
                show_labels=True,
            ),
        ),
    ]


def _parse_triplet(
    value: str | None, default: tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> list[float]:
    if value is None:
        return [*default]
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        return [*default]
    return parts


def _quat_xyzw_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def unique_link_names(names: Iterable[str]) -> list[str]:
    """Return link names in first-seen order without duplicates."""

    seen: set[str] = set()
    result: list[str] = []
    for name in names:
        if name in seen:
            continue
        seen.add(name)
        result.append(name)
    return result
