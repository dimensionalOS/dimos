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

"""A single ping-pong example that runs over either transport.

Run with:
    python examples/native-modules/rust_ping_pong.py --transport lcm
    python examples/native-modules/rust_ping_pong.py --transport zenoh
    python examples/native-modules/rust_ping_pong.py --transport zenoh --topology

With --topology pong is opened as router and ping is opened as client.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.service.zenohservice import ZenohConfig

_RUST_DIR = Path(__file__).parent / "rust"
# The crate is a workspace member, so cargo builds into the repo-root target dir.
_EXAMPLES = DIMOS_PROJECT_ROOT / "target" / "release"
_BUILD = "cargo build --release"

# Where pong listens when it runs as the router.
_ROUTER_ENDPOINT = "tcp/127.0.0.1:17450"


class PingConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "ping")
    build_command: str = _BUILD
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class PongConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "pong")
    build_command: str = _BUILD
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True
    sample_config: int = 42


class PingModule(NativeModule):
    """Publishes Twist messages at 5 Hz on `data` and logs echoes from `confirm`."""

    config: PingConfig
    data: Out[Twist]
    confirm: In[Twist]


class PongModule(NativeModule):
    """Echoes every received Twist message back."""

    config: PongConfig
    data: In[Twist]
    confirm: Out[Twist]


def blueprint(topology: bool = False) -> Blueprint:
    ping_zenoh = None
    pong_zenoh = None
    if topology:
        pong_zenoh = ZenohConfig(mode="router", listen=[_ROUTER_ENDPOINT], connect=[])
        ping_zenoh = ZenohConfig(
            mode="client",
            connect=[_ROUTER_ENDPOINT],
            # The client dials until the router native is up, however slowly it
            # cold-starts. A client with no router fails hard at open.
            connect_timeout=10.0,
        )
    return autoconnect(
        PingModule.blueprint(session=ping_zenoh),
        PongModule.blueprint(session=pong_zenoh),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["lcm", "zenoh"], default="lcm")
    parser.add_argument(
        "--topology",
        action="store_true",
        help="run pong as a zenoh router and ping as its client",
    )
    args = parser.parse_args()
    if args.topology and args.transport != "zenoh":
        parser.error("--topology requires --transport zenoh")

    bp = blueprint(args.topology).global_config(viewer="none", transport=args.transport)
    ModuleCoordinator.build(bp).loop()
