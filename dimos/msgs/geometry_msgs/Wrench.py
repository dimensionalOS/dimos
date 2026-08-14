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

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, overload

from dimos_lcm.geometry_msgs import Wrench as LCMWrench
import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike


class Wrench(LCMWrench):  # type: ignore[misc]
    """Force (N) and torque (Nm) in 3D space."""

    force: Vector3
    torque: Vector3
    msg_name = "geometry_msgs.Wrench"

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(
        self,
        force: VectorLike | None = ...,
        torque: VectorLike | None = ...,
    ) -> None: ...

    @overload
    def __init__(self, wrench: Wrench | LCMWrench, /) -> None: ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a wrench.

        Supported forms:
            Wrench()                                # zero wrench
            Wrench(force, torque)
            Wrench(force=..., torque=...)           # keyword args, either optional
            Wrench(other_wrench)                    # copy constructor
            Wrench(lcm_wrench)                      # from LCM message
        """
        force: Any = None
        torque: Any = None

        if len(args) == 2:
            force, torque = args
        elif len(args) == 1:
            value = args[0]
            # Wrench before LCMWrench (it is a subclass); anything else is a
            # force, same as the `force=` keyword.
            if isinstance(value, Wrench | LCMWrench):
                force, torque = value.force, value.torque
            else:
                force = value
        elif args:
            raise TypeError(f"Wrench takes 1 or 2 positional arguments ({len(args)} given)")

        if kwargs:
            force = kwargs.pop("force", force)
            torque = kwargs.pop("torque", torque)
            if kwargs:
                raise TypeError(f"Wrench got unexpected keyword arguments {sorted(kwargs)}")

        self.force = Vector3() if force is None else Vector3(force)
        self.torque = Vector3() if torque is None else Vector3(torque)

    @classmethod
    def from_array(cls, ft: Sequence[float] | np.ndarray) -> Wrench:
        """Build from a 6-element ``[fx, fy, fz, tx, ty, tz]`` sensor reading."""
        if len(ft) != 6:
            raise ValueError(f"Expected 6 elements [fx, fy, fz, tx, ty, tz], got {len(ft)}")
        return cls(force=ft[:3], torque=ft[3:])

    def to_array(self) -> np.ndarray:
        """``[fx, fy, fz, tx, ty, tz]``."""
        return np.concatenate([self.force.to_numpy(), self.torque.to_numpy()])

    @classmethod
    def zero(cls) -> Wrench:
        return cls()

    def is_zero(self) -> bool:
        return self.force.is_zero() and self.torque.is_zero()

    def __add__(self, other: Wrench) -> Wrench:
        if not isinstance(other, Wrench):
            return NotImplemented
        return Wrench(force=self.force + other.force, torque=self.torque + other.torque)

    def __sub__(self, other: Wrench) -> Wrench:
        if not isinstance(other, Wrench):
            return NotImplemented
        return Wrench(force=self.force - other.force, torque=self.torque - other.torque)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Wrench):
            return False
        return self.force == other.force and self.torque == other.torque

    def __bool__(self) -> bool:
        return not self.is_zero()

    def __repr__(self) -> str:
        return f"Wrench(force={self.force!r}, torque={self.torque!r})"

    def __str__(self) -> str:
        return f"Wrench:\n  Force: {self.force}\n  Torque: {self.torque}"
