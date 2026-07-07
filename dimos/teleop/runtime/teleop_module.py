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

from collections.abc import Mapping
import threading
import time
from typing import Any, cast

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.runtime.types import TeleopAdapter, TeleopCommand, TeleopPrimaryOutput


class TeleopModuleConfig(ModuleConfig):
    tick_period_s: float = 0.02
    max_publish_rate_hz: float = 50.0
    stale_command_timeout_s: float = 0.25


class TeleopModule(Module):
    """Generic teleop runtime module that routes command envelopes to outputs."""

    config: TeleopModuleConfig  # type: ignore[assignment]

    joint_command: Out[JointState]
    coordinator_cartesian_command: Out[PoseStamped]
    twist_command: Out[Twist]

    def __init__(self, adapter: TeleopAdapter, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if adapter.primary_output not in ("joint", "cartesian", "twist"):
            raise ValueError(f"unsupported teleop primary output: {adapter.primary_output!r}")
        if self.teleop_config.max_publish_rate_hz <= 0.0:
            raise ValueError("max_publish_rate_hz must be positive")
        if self.teleop_config.stale_command_timeout_s < 0.0:
            raise ValueError("stale_command_timeout_s must be non-negative")
        self._adapter = adapter
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_publish_time = 0.0

    @classmethod
    def resolve_config(cls, config_args: Mapping[str, Any]) -> ModuleConfig:
        filtered_args = {k: v for k, v in config_args.items() if k != "adapter"}
        return super().resolve_config(filtered_args)

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
        if command.metadata.primary_output != self._adapter.primary_output:
            raise ValueError(
                "TeleopCommand primary output does not match adapter primary output: "
                f"{command.metadata.primary_output!r} != {self._adapter.primary_output!r}"
            )
        if command.stop:
            return
        if self._is_stale(command) or self._rate_limited():
            return
        self._publish(command)
        self._last_publish_time = self._now()

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            self.tick()
            time.sleep(self.teleop_config.tick_period_s)

    def _is_stale(self, command: TeleopCommand) -> bool:
        return self._now() - command.metadata.timestamp > self.teleop_config.stale_command_timeout_s

    def _rate_limited(self) -> bool:
        return self._now() - self._last_publish_time < 1.0 / self.teleop_config.max_publish_rate_hz

    @property
    def teleop_config(self) -> TeleopModuleConfig:
        return cast("TeleopModuleConfig", self.config)

    def _now(self) -> float:
        return time.monotonic()

    def _publish(self, command: TeleopCommand) -> None:
        primary_output = command.metadata.primary_output
        if primary_output == "joint" and command.joint is not None:
            self.joint_command.publish(command.joint)
            return
        if primary_output == "cartesian" and command.cartesian is not None:
            self.coordinator_cartesian_command.publish(command.cartesian)
            return
        if primary_output == "twist" and command.twist is not None:
            self.twist_command.publish(command.twist)
            return
        self._raise_inconsistent_command(primary_output)

    def _raise_inconsistent_command(self, primary_output: TeleopPrimaryOutput) -> None:
        raise ValueError(f"TeleopCommand is inconsistent for primary output {primary_output!r}")
