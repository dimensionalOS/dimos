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

"""Base twist adapter for mobile robots.

Wraps BaseTwistAdapter to integrate with the manipulator adapter registry.
"""

from typing import Callable

from dimos.hardware.base.twist_adapter import BaseTwistAdapter
from dimos.hardware.manipulators.registry import AdapterRegistry
from dimos.msgs.geometry_msgs import Twist
from dimos.core.transport import LCMTransport


class BaseTwistAdapterWrapper(BaseTwistAdapter):
    """Wrapper for BaseTwistAdapter that works with adapter registry.

    This adapter accepts a twist_callback function that will be called
    with Twist commands. The callback should be set up to call
    GO2Connection.move() or similar. If no callback is provided, it will
    publish directly to the `/cmd_vel` LCM topic.

    Args:
        twist_callback: Callable that receives Twist commands
        dof: Degrees of freedom (must be 3 for base)
        cmd_vel_topic: LCM topic to publish Twist messages to when no callback
                       is provided (default: "/cmd_vel").
    """

    def __init__(
        self,
        twist_callback: Callable[[Twist], None] | None = None,
        dof: int = 3,
        cmd_vel_topic: str = "/cmd_vel",
        **kwargs,
    ):
        """Initialize base twist adapter wrapper.

        Args:
            twist_callback: Callback function for Twist commands. If None, the
                adapter publishes directly to `cmd_vel_topic` using LCM.
            dof: Degrees of freedom (must be 3)
            cmd_vel_topic: LCM topic to publish Twist messages to when no
                callback is provided.
            **kwargs: Ignored (for compatibility with registry)
        """
        if dof != 3:
            raise ValueError(f"BaseTwistAdapter requires dof=3, got {dof}")

        # Transport is stored on the instance so it can be pickled safely.
        # LCMTransport implements __reduce__, so it can cross RPC boundaries.
        self._transport: LCMTransport[Twist] | None = None

        # If no callback provided, publish Twist to LCM `/cmd_vel`
        if twist_callback is None:
            self._transport = LCMTransport(cmd_vel_topic, Twist)
            twist_callback = self._publish_twist

        super().__init__(twist_callback)

    def _publish_twist(self, twist: Twist) -> None:
        """Default callback: publish Twist to LCM `/cmd_vel`."""
        # Create transport if it doesn't exist
        if self._transport is None:
            self._transport = LCMTransport("/cmd_vel", Twist)
        self._transport.broadcast(None, twist)


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("base_twist", BaseTwistAdapterWrapper)


__all__ = ["BaseTwistAdapterWrapper"]