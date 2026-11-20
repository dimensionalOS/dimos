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

"""A fitted link's mass and centre of mass, and putting them into a pinocchio model."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Annotated

import numpy as np
import pinocchio
from pydantic import ConfigDict, Field
from pydantic.dataclasses import dataclass as pydantic_dataclass


@pydantic_dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class LinkInertial:
    """Mass and centre of mass of one link, in the link (= joint) frame."""

    mass_kg: Annotated[float, Field(gt=0.0)]
    com_m: tuple[float, float, float]


def body_joint_id(model: pinocchio.Model, link: str) -> int:
    """The joint whose inertia carries ``link``; the link frame must coincide with that joint."""
    if not model.existFrame(link):
        raise ValueError(f"model has no link {link!r}")
    frame = model.frames[model.getFrameId(link)]
    if frame.type != pinocchio.FrameType.BODY or not frame.placement.isIdentity():
        raise ValueError(f"{link!r} is not a body frame coincident with its joint")
    return int(frame.parentJoint)


def apply_inertial_overrides(model: pinocchio.Model, overrides: Mapping[str, LinkInertial]) -> None:
    """Replace the mass and COM of the named links in ``model``, keeping their rotational inertia."""
    for name, inertial in overrides.items():
        joint_id = body_joint_id(model, name)
        model.inertias[joint_id] = pinocchio.Inertia(
            float(inertial.mass_kg),
            np.asarray(inertial.com_m, dtype=np.float64),
            model.inertias[joint_id].inertia,
        )
