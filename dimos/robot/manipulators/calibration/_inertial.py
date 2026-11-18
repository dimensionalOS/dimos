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

"""Per-link inertial overrides and their application to a pinocchio model.

``LinkInertial`` is one fitted body (mass and centre of mass in the link frame);
``schema.to_inertial_overrides`` turns an arm's calibration file into these, and
``apply_inertial_overrides`` puts them into a model, as the gravity compensation
will on connect. ``load_inertial_overrides`` still reads the flat
``{"links": ...}`` files gravity fits wrote before the calibration schema.
"""

from __future__ import annotations

from collections.abc import Mapping
import json
from pathlib import Path
from typing import Annotated, Any

import numpy as np
import pinocchio
from pydantic import ConfigDict, Field
from pydantic.dataclasses import dataclass as pydantic_dataclass


@pydantic_dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class LinkInertial:
    """Mass and centre of mass of one link, in the link (= joint) frame."""

    mass_kg: Annotated[float, Field(gt=0.0)]
    com_m: tuple[float, float, float]


def load_inertial_overrides(
    source: str | Path | Mapping[str, Any] | None,
) -> dict[str, LinkInertial]:
    """Per-link inertial overrides from a ``gravity_fit`` JSON path or an equivalent mapping.

    Accepts the fit file (``{"links": {name: {mass_kg, com_m}}}``), a flat
    ``{name: {mass_kg, com_m}}`` mapping, or already-built ``LinkInertial``
    values. ``None`` means no overrides.
    """
    if source is None:
        return {}
    if isinstance(source, str | Path):
        path = Path(source)
        if not path.is_file():
            raise FileNotFoundError(f"inertial overrides file not found: {path}")
        with path.open() as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            raise ValueError(f"{path}: inertial overrides must be a JSON object")
        source = data
    links = source.get("links", source) if "links" in source else source
    if not isinstance(links, Mapping):
        raise ValueError("inertial overrides 'links' must be a mapping of link name to values")
    result: dict[str, LinkInertial] = {}
    for name, value in links.items():
        if not isinstance(name, str) or not name:
            raise ValueError(
                f"inertial override link name must be a non-empty string, got {name!r}"
            )
        if isinstance(value, LinkInertial):
            result[name] = value
        elif isinstance(value, Mapping):
            result[name] = LinkInertial(
                mass_kg=value["mass_kg"],
                com_m=tuple(value["com_m"]),
            )
        else:
            raise ValueError(f"inertial override for {name!r} must be a mapping")
    return result


def apply_inertial_overrides(model: pinocchio.Model, overrides: Mapping[str, LinkInertial]) -> None:
    """Replace the mass and COM of the named links in ``model`` (rotational inertia kept).

    A link must be a body frame coincident with its joint, which holds for every
    URDF link attached by a movable joint; pinocchio stores that body's inertia
    on the joint. Raises ``ValueError`` for unknown or merged links.
    """
    for name, inertial in overrides.items():
        if not model.existFrame(name):
            raise ValueError(f"inertial override names unknown link {name!r}")
        frame = model.frames[model.getFrameId(name)]
        if frame.type != pinocchio.FrameType.BODY or not frame.placement.isIdentity(1e-9):
            raise ValueError(
                f"inertial override for {name!r}: not a body frame coincident with its joint"
            )
        joint_id = int(frame.parentJoint)
        current = model.inertias[joint_id]
        model.inertias[joint_id] = pinocchio.Inertia(
            float(inertial.mass_kg),
            np.asarray(inertial.com_m, dtype=np.float64),
            current.inertia,
        )
