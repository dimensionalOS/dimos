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

"""FlowBaseDriver — a thin cmd_vel → FlowBase (Portal RPC) sink module.

Used when the ControlCoordinator drives the base through a ``transport_lcm``
adapter (so it can read SLAM odometry from a separate topic). The coordinator
publishes the final ``cmd_vel`` on an LCM topic; this module subscribes it and
forwards to the FlowBase controller via :class:`FlowBaseAdapter`, which applies
the platform's Y/yaw sign convention in ``write_velocities``. This module never
reads odometry — odometry comes from the SLAM source, not the wheels.
"""

from __future__ import annotations

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.hardware.drive_trains.flowbase.adapter import FlowBaseAdapter
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FlowBaseDriverConfig(ModuleConfig):
    """``address`` is the FlowBase Portal RPC endpoint ``host:port``; when
    ``None`` the adapter's ``DEFAULT_ADDRESS`` is used."""

    address: str | None = None


class FlowBaseDriver(Module):
    """Subscribe ``cmd_vel: In[Twist]`` → drive FlowBase via Portal RPC."""

    config: FlowBaseDriverConfig
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._adapter: FlowBaseAdapter | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._adapter = FlowBaseAdapter(dof=3, address=self.config.address)
        if not self._adapter.connect():
            logger.error("FlowBaseDriver: failed to connect to FlowBase — cmd_vel will be dropped")
        unsub = self.cmd_vel.subscribe(self._on_cmd_vel)
        self.register_disposable(Disposable(unsub))
        logger.info("FlowBaseDriver started")

    @rpc
    def stop(self) -> None:
        if self._adapter is not None:
            try:
                self._adapter.write_stop()
            except Exception:
                pass
            self._adapter.disconnect()
            self._adapter = None
        super().stop()

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._adapter is None:
            return
        self._adapter.write_velocities(
            [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        )


__all__ = ["FlowBaseDriver", "FlowBaseDriverConfig"]
