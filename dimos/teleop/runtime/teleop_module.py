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

from __future__ import annotations

from functools import singledispatchmethod
import threading
import time
from typing import Any

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.runtime.adapters import TeleopAdapterConfig, create_teleop_adapter
from dimos.teleop.runtime.types import TeleopAdapter, TeleopCommand


class TeleopModuleConfig(ModuleConfig):
    adapter: TeleopAdapterConfig = Field(default_factory=OpenArmMiniTeleopConfig)
    tick_period_s: float = 0.02
    max_publish_rate_hz: float = 50.0
    stale_command_timeout_s: float = 0.25


class TeleopModule(Module):
    """Generic teleop runtime module that routes typed command payloads to outputs."""

    config: TeleopModuleConfig  # type: ignore[assignment]

    joint_command: Out[JointState]
    coordinator_cartesian_command: Out[PoseStamped]
    twist_command: Out[Twist]

    def __init__(self, runtime_adapter: TeleopAdapter | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self.teleop_config.max_publish_rate_hz <= 0.0:
            raise ValueError("max_publish_rate_hz must be positive")
        if self.teleop_config.stale_command_timeout_s < 0.0:
            raise ValueError("stale_command_timeout_s must be non-negative")
        self._adapter = (
            runtime_adapter
            if runtime_adapter is not None
            else create_teleop_adapter(self.teleop_config.adapter)
        )
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_publish_time = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._last_publish_time = 0.0
        self._adapter.connect()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        self._adapter.disconnect()
        super().stop()

    def tick(self) -> None:
        """Run one control tick; intended for tests and synchronous drivers."""

        if self._stop_event.is_set():
            return
        command = self._adapter.get_current_command()
        if command is None:
            return
        if command.stop:
            return
        if self._is_stale(command) or self._rate_limited():
            return
        if command.payload is None:
            raise ValueError("TeleopCommand payload is missing")
        self._publish_payload(command.payload)
        self._last_publish_time = self._now()

    def _run_loop(self) -> None:
        next_tick_time = time.monotonic()
        while not self._stop_event.is_set():
            self.tick()
            next_tick_time += self.teleop_config.tick_period_s
            sleep_s = max(0.0, next_tick_time - time.monotonic())
            self._stop_event.wait(sleep_s)

    def _is_stale(self, command: TeleopCommand) -> bool:
        return self._now() - command.timestamp > self.teleop_config.stale_command_timeout_s

    def _rate_limited(self) -> bool:
        return self._now() - self._last_publish_time < 1.0 / self.teleop_config.max_publish_rate_hz

    @property
    def teleop_config(self) -> TeleopModuleConfig:
        return self.config

    def _now(self) -> float:
        return time.monotonic()

    @singledispatchmethod
    def _publish_payload(self, payload: object) -> None:
        raise TypeError(f"unsupported teleop payload type: {type(payload).__name__}")

    @_publish_payload.register
    def _(self, payload: JointState) -> None:
        self.joint_command.publish(payload)

    @_publish_payload.register
    def _(self, payload: PoseStamped) -> None:
        self.coordinator_cartesian_command.publish(payload)

    @_publish_payload.register
    def _(self, payload: Twist) -> None:
        self.twist_command.publish(payload)
