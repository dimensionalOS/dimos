# Copyright 2025 Dimensional Inc.
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

"""Standard components for manipulator drivers."""

from collections.abc import Callable
from typing import Any, TypeVar

F = TypeVar("F", bound=Callable[..., Any])


def component_api(fn: F) -> F:
    """Decorator to mark component methods that should be exposed as driver RPCs.

    Methods decorated with @component_api will be automatically discovered by the
    driver and exposed as @rpc methods on the driver instance. This allows external
    code to call these methods via the standard Module RPC system.

    Example:
        class MyComponent:
            @component_api
            def enable_servo(self):
                '''Enable servo motors.'''
                return self.sdk.enable_servos()

        # The driver will auto-generate:
        # @rpc
        # def enable_servo(self):
        #     return component.enable_servo()

        # External code can then call:
        # driver.enable_servo()
    """
    fn.__component_api__ = True  # type: ignore[attr-defined]
    return fn


# Import components AFTER defining component_api to avoid circular imports
from .motion import StandardMotionComponent
from .servo import StandardServoComponent
from .status import StandardStatusComponent

__all__ = [
    "StandardMotionComponent",
    "StandardServoComponent",
    "StandardStatusComponent",
    "component_api",
]
