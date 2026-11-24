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

"""The M20 controller as a board service (like the vendor's rl_deploy.service), and the run's client of it.

The service is the controller's native process under systemd on AOS (``rust/service/m20-controller.service``). It reads
its launch line from ``launch.json``, which ``python -m dimos.robot.deeprobotics.m20.controller.service`` writes; its
streams live under ``dimos/m20_controller/`` whatever run is up, so the robot stays held (and standing) when a run
stops. ``M20ControllerClient`` is the run's side: the same ports and RPCs as ``M20Controller``, forwarded to the
service; its stop() zeroes the velocity only, as M20Connection's does.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
import sys
from typing import Any, get_args, get_type_hints

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.core.transport_factory import session_config
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.deeprobotics.m20.controller.module import (
    M20Controller,
    M20ControllerApi,
    M20ControllerConfig,
    NavigationTerrain,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# the service's streams, outside every run's domain
SERVICE_PREFIX = "dimos/m20_controller/"


def ports() -> tuple[dict[str, type], dict[str, type]]:
    """M20Controller's inputs and outputs: name -> message type."""
    inputs: dict[str, type] = {}
    outputs: dict[str, type] = {}
    for name, hint in get_type_hints(M20Controller).items():
        origin = getattr(hint, "__origin__", None)
        if origin is In:
            inputs[name] = get_args(hint)[0]
        elif origin is Out:
            outputs[name] = get_args(hint)[0]
    return inputs, outputs


def service_transport(port: str, msg_type: type) -> ZenohTransport[Any]:
    return ZenohTransport(SERVICE_PREFIX + port, msg_type)


def launch_line(config: M20ControllerConfig) -> str:
    """The JSON line the service's native process reads on stdin (NativeModule's, with the service's topics)."""
    inputs, outputs = ports()
    topics = {name: service_transport(name, t).channel for name, t in {**inputs, **outputs}.items()}
    blob = {
        "topics": topics,
        "config": config.to_config_dict(),
        "session": session_config().to_wire(),
    }
    return json.dumps(blob)


class M20ControllerClientConfig(ModuleConfig):
    navigation_terrain: NavigationTerrain = "stairs"
    stand_timeout_s: float = Field(default=12.0, gt=0.0)
    rl_control_timeout_s: float = Field(default=5.0, gt=0.0)
    gait_timeout_s: float = Field(default=5.0, gt=0.0)
    navigation_mode_attempts: int = Field(default=3, ge=1)
    control_ready_timeout_s: float = Field(default=15.0, gt=0.0)
    vendor_gait_wait_s: float = Field(default=3.0, gt=0.0)
    gait_command_attempts: int = Field(default=2, ge=1)


class M20ControllerClient(M20ControllerApi, Module):
    """M20Controller's ports and RPCs, served by the m20-controller service on AOS (which keeps holding the robot)."""

    config: M20ControllerClientConfig

    @rpc
    def start(self) -> None:
        inputs, outputs = ports()
        self._service = {
            name: service_transport(name, t) for name, t in {**inputs, **outputs}.items()
        }
        super().start()
        # the run's inputs go to the service, the service's outputs come into the run
        for name in inputs:
            stream = getattr(self, name)
            self.register_disposable(Disposable(stream.subscribe(self._service[name].publish)))
        for name in outputs:
            out = getattr(self, name)
            self.register_disposable(Disposable(self._service[name].subscribe(out.publish)))

    @rpc
    def stop(self) -> None:
        # M20Connection's stop: a zero velocity; the service keeps the robot as it is
        self._send("cmd_vel", Twist.zero())
        for transport in self._service.values():
            transport.stop()
        super().stop()

    def _can_sequence(self, what: str) -> bool:
        return True

    def _send(self, port: str, msg: Any) -> None:
        self._service[port].publish(msg)

    def _watch(self, port: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        return self._service[port].subscribe(callback)  # type: ignore[no-any-return]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Write the m20-controller service's launch line (launch.json)."
    )
    parser.add_argument(
        "--allow-gains", action="store_true", help="let commands with kp / kd reach the motors"
    )
    parser.add_argument("--policy-dir", default="", help="the extracted rl_deploy ONNX policies")
    parser.add_argument(
        "--controller-config", default="", help="an m20_controller.toml ('' = built in)"
    )
    parser.add_argument(
        "--command-timeout-s", type=float, default=0.4, help="the vendor bridge's cmd_vel timeout"
    )
    parser.add_argument(
        "--zenoh-connect", default="tcp/127.0.0.1:7447", help="the board's zenoh router"
    )
    args = parser.parse_args(argv)
    global_config.update(zenoh_mode="client", zenoh_connect=args.zenoh_connect, zenoh_domain="")
    config = M20ControllerConfig(
        mode="owner",
        policy=True,
        allow_gains=args.allow_gains,
        policy_dir=args.policy_dir,
        controller_config=args.controller_config,
        command_timeout_s=args.command_timeout_s,
    )
    sys.stdout.write(launch_line(config) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
