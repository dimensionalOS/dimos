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

"""Resolve physical joint limits for control tasks."""

from collections.abc import Mapping, Sequence
import math
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def resolve_velocity_limits(
    joint_names: Sequence[str],
    hardware: Mapping[str, Any],
    *,
    speed_scale: float,
) -> dict[str, float]:
    """Resolve physical limits from adapters plus component overrides."""
    if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
        raise ValueError("speed_scale must be finite and within (0, 1]")

    requested = set(joint_names)
    resolved: dict[str, float] = {}
    for connected in hardware.values():
        component = connected.component
        arm_names = list(component.joints)
        get_limits = getattr(connected.adapter, "get_limits", None)
        if callable(get_limits) and requested.intersection(arm_names):
            limits = get_limits().velocity_max
            if len(limits) != len(arm_names):
                raise ValueError(
                    f"hardware '{component.hardware_id}' returned {len(limits)} velocity "
                    f"limits for {len(arm_names)} joints"
                )
            resolved.update(
                (name, float(limit))
                for name, limit in zip(arm_names, limits, strict=True)
                if name in requested
            )
        overrides = component.joint_velocity_limits
        unknown = set(overrides) - set(component.all_joints)
        if unknown:
            raise ValueError(
                f"hardware '{component.hardware_id}' has velocity limits for unknown joints: "
                f"{sorted(unknown)}"
            )
        resolved.update(
            (name, float(limit)) for name, limit in overrides.items() if name in requested
        )

    if any(not math.isfinite(limit) or limit <= 0.0 for limit in resolved.values()):
        raise ValueError("hardware velocity limits must be finite and positive")
    missing = sorted(requested - resolved.keys())
    if missing:
        logger.warning(
            "Position control has no velocity limit for joints; commands are unbounded",
            joints=missing,
        )
    return {name: limit * speed_scale for name, limit in resolved.items()}
