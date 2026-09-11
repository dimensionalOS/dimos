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

from typing import Any, overload

from dimos_lcm.geometry_msgs import Twist as LCMTwist

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike


class Twist(LCMTwist):  # type: ignore[misc]
    linear: Vector3
    angular: Vector3
    msg_name = "geometry_msgs.Twist"

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(
        self,
        linear: VectorLike | None = ...,
        angular: VectorLike | Quaternion | None = ...,
    ) -> None: ...

    @overload
    def __init__(self, twist: Twist | LCMTwist, /) -> None: ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a twist.

        Supported forms:
            Twist()                                 # zero twist
            Twist(linear, angular)
            Twist(linear=..., angular=...)          # keyword args, either optional
            Twist(linear, quaternion)               # angular converted to euler
            Twist(other_twist)                      # copy constructor
            Twist(lcm_twist)                        # from LCM message
        """
        linear: Any = None
        angular: Any = None

        if len(args) == 2:
            linear, angular = args
        elif len(args) == 1:
            value = args[0]
            # Twist before LCMTwist (it is a subclass); anything else is a
            # linear velocity, same as the `linear=` keyword.
            if isinstance(value, Twist | LCMTwist):
                linear, angular = value.linear, value.angular
            else:
                linear = value
        elif args:
            raise TypeError(f"Twist takes 1 or 2 positional arguments ({len(args)} given)")

        if kwargs:
            linear = kwargs.pop("linear", linear)
            angular = kwargs.pop("angular", angular)
            if kwargs:
                raise TypeError(f"Twist got unexpected keyword arguments {sorted(kwargs)}")

        self.linear = Vector3() if linear is None else Vector3(linear)
        if angular is None:
            self.angular = Vector3()
        elif isinstance(angular, Quaternion):
            self.angular = angular.to_euler()
        else:
            self.angular = Vector3(angular)

    def __repr__(self) -> str:
        return f"Twist(linear={self.linear!r}, angular={self.angular!r})"

    def __str__(self) -> str:
        return f"Twist:\n  Linear: {self.linear}\n  Angular: {self.angular}"

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two twists are equal."""
        if not isinstance(other, Twist):
            return False
        return self.linear == other.linear and self.angular == other.angular

    @classmethod
    def zero(cls) -> Twist:
        """Create a zero twist (no motion)."""
        return cls()

    def is_zero(self) -> bool:
        """Check if this is a zero twist (no linear or angular velocity)."""
        return self.linear.is_zero() and self.angular.is_zero()

    def __sub__(self, other: Twist) -> Twist:
        """Component-wise subtraction: self - other."""
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(
            linear=self.linear - other.linear,
            angular=self.angular - other.angular,
        )

    def __add__(self, other: Twist) -> Twist:
        """Component-wise addition: self + other."""
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(
            linear=self.linear + other.linear,
            angular=self.angular + other.angular,
        )

    def __bool__(self) -> bool:
        """Boolean conversion for Twist.

        A Twist is considered False if it's a zero twist (no motion),
        and True otherwise.

        Returns:
            False if twist is zero, True otherwise
        """
        return not self.is_zero()
