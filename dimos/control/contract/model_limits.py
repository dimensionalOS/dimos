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

"""Read joint limits out of a URDF robot model.

A URDF describes a robot: its parts, and for each joint how far it may turn,
how fast, and how hard it may push. This pulls those numbers out so nobody has
to copy them by hand, where they could drift out of step with the model and
let a joint be driven further than the real robot can go.

Anything the model does not answer clearly is an error here, never a guess. A
limits table that is quietly wrong is more dangerous than one that refuses to
load.
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
    """Read the limits for a set of joints out of a URDF.

    A URDF names its joints its own way, so ``joints`` maps the name you want
    to use onto the name the model uses::

        limits_from_urdf(G1_URDF, {"g1/left_knee": "left_knee_joint"})

    You get up to three limits back per joint::

        "g1/left_knee/position"  ->  how far it may turn, in radians
        "g1/left_knee/velocity"  ->  how fast it may turn, in rad/s
        "g1/left_knee/effort"    ->  how hard it may push, in Nm

    Position has a separate lower and upper bound, which are often not
    symmetric. Velocity and effort have one figure each, applied the same in
    both directions.

    A joint that spins freely, with no end stops, has no position range and
    comes back with both bounds unset. It is the only joint allowed to do so.
    Any other joint missing a bound means the model is broken and raises,
    because treating it as unlimited would let it be driven anywhere.

    Args:
        urdf: Path to a URDF file, or the XML text itself.
        joints: The name you want to use -> the name used in the URDF. Your
            names look like ``"<source>/<resource>"``, e.g. ``"g1/left_knee"``.
        policy: What should happen to a command that falls outside these
            limits. REJECT turns it away; CLAMP pulls it back to the nearest
            bound. CLAMP needs both bounds, so it cannot be used on a joint
            that spins freely.
        position: Include the position limits.
        velocity: Include the velocity limits.
        effort: Include the effort limits.

    Returns:
        The limits, keyed by ``"<source>/<resource>/<interface>"``.

    Raises:
        ValueError: If a joint is not in the URDF, has no limits at all, is
            missing one you asked for, is missing a position bound without
            being a freely spinning joint, or spins freely while you asked
            for CLAMP.
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
            if lower is not None and upper is not None:
                out[make_key(source, resource, POSITION)] = Limits(lower, upper, policy)
            elif lower is None and upper is None:
                # Only a continuous joint legitimately has no position range.
                # A revolute one that lost its bounds is a broken model, not a
                # free spinner, and reading it as unbounded would take a real
                # arm's limits away.
                joint_type = joint_element.get("type", "")
                if joint_type != "continuous":
                    raise ValueError(
                        f"joint {urdf_name!r} (for {canonical!r}) is {joint_type or 'untyped'} "
                        f"but declares no position bounds; only a continuous joint may "
                        f"leave them out"
                    )
                if policy is LimitPolicy.CLAMP:
                    raise ValueError(
                        f"joint {urdf_name!r} (for {canonical!r}) has no position bounds "
                        f"to clamp to; it is continuous, so use LimitPolicy.REJECT"
                    )
                out[make_key(source, resource, POSITION)] = Limits(None, None, policy)
            else:
                # Half a range is worse than none: dropping the side the model
                # does declare would leave the joint unlimited under REJECT,
                # and nothing downstream would ever say so.
                missing, given = ("upper", "lower") if upper is None else ("lower", "upper")
                raise ValueError(
                    f"joint {urdf_name!r} (for {canonical!r}) declares a {given} position "
                    f"bound but no {missing} one; a half-written <limit> would silently "
                    f"leave the joint unlimited"
                )

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
