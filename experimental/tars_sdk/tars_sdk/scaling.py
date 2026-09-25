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

"""Froude (dynamic-similarity) scaling.

Every tunable is written for the 1.52 m reference robot. Scaling lengths by s and keeping
gravity and density fixed gives a dynamically similar robot if each quantity scales by
s**k: length 1, time 0.5, speed 0.5, mass/force 3, torque 4, N/m 2, N*s/m 2.5,
Nm/rad 4, Nm*s/rad 4.5, rad/s -0.5. The same gaits then work unchanged at any size.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, TypeVar

T = TypeVar("T")

LENGTH, TIME, SPEED, MASS, FORCE, TORQUE = 1.0, 0.5, 0.5, 3.0, 3.0, 4.0
LIN_STIFF, LIN_DAMP, ROT_STIFF, ROT_DAMP, ANG_VEL = 2.0, 2.5, 4.0, 4.5, -0.5
INERTIA = 5.0


def froude(obj: T, s: float, dims: dict[str, float]) -> T:
    """Copy of dataclass `obj` with each field in `dims` multiplied by s**exponent."""
    if s == 1.0:
        return obj
    changes: dict[str, Any] = {}
    for name, k in dims.items():
        v = getattr(obj, name)
        f = s**k
        changes[name] = tuple(x * f for x in v) if isinstance(v, tuple) else v * f
    return replace(obj, **changes)  # type: ignore[type-var]
