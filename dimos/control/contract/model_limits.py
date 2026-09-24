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

"""Joint limits read out of a URDF.

Every preset wants a limits table and nobody should be typing one in by hand:
the numbers already exist in the model the planner and the sim both load, and a
table that disagrees with the model is a table that stops a real arm halfway
through a motion.

Parsing is ``xml.etree`` and nothing else. Pulling in pinocchio or mujoco to
read six numbers would put a heavyweight import behind every description, and
the contract package is meant to stay importable from anywhere. MJCF comes with
the sim work, not here.

Nothing is inferred and nothing is skipped. A joint in the table that the model
does not have, or that the model gives no limits for, raises rather than
quietly returning a smaller table than the caller asked for.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.control.contract.description import LimitPolicy, Limits
from dimos.control.contract.keys import EFFORT, POSITION, VELOCITY, is_valid_segment, make_key


def _parse(urdf: str | Path) -> ET.Element:
    """The root element of a URDF given as a path or as the XML itself."""
    if isinstance(urdf, Path):
        return ET.fromstring(urdf.read_text())
    if urdf.lstrip().startswith("<"):
        return ET.fromstring(urdf)
    return ET.fromstring(Path(urdf).read_text())


def _split_joint(joint: str) -> tuple[str, str]:
    """``"arm/joint1"`` into its source and resource segments."""
    parts = joint.split("/")
    if len(parts) != 2 or not all(is_valid_segment(part) for part in parts):
        raise ValueError(f"joint name {joint!r} is not '<source>/<resource>'")
    return parts[0], parts[1]


def _attr(element: ET.Element, name: str) -> float | None:
    """One float attribute of ``<limit>``, or ``None`` when it is absent."""
    raw = element.get(name)
    if raw is None:
        return None
    try:
        return float(raw)
    except ValueError:
        raise ValueError(f"limit attribute {name}={raw!r} is not a number") from None


def limits_from_urdf(
    urdf: str | Path,
    joints: Mapping[str, str],
    *,
    policy: LimitPolicy = LimitPolicy.REJECT,
    position: bool = True,
    velocity: bool = True,
    effort: bool = True,
) -> dict[str, Limits]:
    """Limits for the mapped joints, keyed by canonical control key.

    ``joints`` maps the canonical joint name onto the name the URDF uses, which
    is normally where the two naming schemes are reconciled::

        limits_from_urdf(G1_URDF, {f"g1/{j}": f"{j}_joint" for j in G1_JOINTS})

    A URDF ``<limit>`` gives one position range and a symmetric bound on each of
    velocity and effort, so that is what comes back::

        "<joint>/position"  ->  Limits(lower, upper, policy)
        "<joint>/velocity"  ->  Limits(-velocity, velocity, policy)
        "<joint>/effort"    ->  Limits(-effort, effort, policy)

    A continuous joint turns without end and has no position range. Under
    REJECT it gets ``Limits(None, None)``, which is an honest "unbounded"; under
    CLAMP there is nothing to clamp to, so it raises rather than inventing a
    bound the validator would have to reject anyway.

    Args:
        urdf: A path to a URDF, or the XML text itself.
        joints: Canonical ``"<source>/<resource>"`` name -> URDF joint name.
        policy: Limit policy for every key produced.
        position: Whether to emit position limits.
        velocity: Whether to emit velocity limits.
        effort: Whether to emit effort limits.

    Returns:
        Limits keyed by full ``"<source>/<resource>/<interface>"`` key.

    Raises:
        ValueError: On a mapped joint the URDF does not have, a mapped joint
            with no ``<limit>``, a missing bound the caller asked for, or a
            continuous joint under CLAMP.
    """
    root = _parse(urdf)
    by_name: dict[str, ET.Element] = {}
    for element in root.findall("joint"):
        name = element.get("name")
        if name is not None:
            by_name[name] = element

    wanted = position or velocity or effort
    out: dict[str, Limits] = {}
    for canonical, urdf_name in joints.items():
        source, resource = _split_joint(canonical)
        joint_element = by_name.get(urdf_name)
        if joint_element is None:
            raise ValueError(
                f"joint {urdf_name!r} (for {canonical!r}) is not in the URDF; "
                f"it has {sorted(by_name)}"
            )
        if not wanted:
            continue

        limit = joint_element.find("limit")
        if limit is None:
            raise ValueError(
                f"joint {urdf_name!r} (for {canonical!r}) has no <limit>, so the URDF "
                f"cannot say what it is allowed to do"
            )

        if position:
            lower, upper = _attr(limit, "lower"), _attr(limit, "upper")
            if lower is None or upper is None:
                # A continuous joint is the legitimate case; anything else with
                # a half-written <limit> lands here too, and both are unbounded.
                if policy is LimitPolicy.CLAMP:
                    raise ValueError(
                        f"joint {urdf_name!r} (for {canonical!r}) has no position bounds "
                        f"to clamp to; it is continuous, so use LimitPolicy.REJECT"
                    )
                out[make_key(source, resource, POSITION)] = Limits(None, None, policy)
            else:
                out[make_key(source, resource, POSITION)] = Limits(lower, upper, policy)

        for wanted_it, name, interface in (
            (velocity, "velocity", VELOCITY),
            (effort, "effort", EFFORT),
        ):
            if not wanted_it:
                continue
            bound = _attr(limit, name)
            if bound is None:
                raise ValueError(
                    f"joint {urdf_name!r} (for {canonical!r}) declares no {name} limit"
                )
            out[make_key(source, resource, interface)] = Limits(-bound, bound, policy)

    return out
