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

"""Two Rust NativeModules for a simple ping-pong example, over LCM or Zenoh.

Two equivalent blueprints do the same ping-pong: one whose binaries use the LCM
transport (native_ping / native_pong), one whose binaries use the Zenoh
transport (zenoh_ping / zenoh_pong). Pick one per run; run both at once in two
terminals (they use different transports, so they stay independent).

Run with:
    python examples/native-modules/rust_ping_pong.py lcm
    python examples/native-modules/rust_ping_pong.py zenoh
"""

from __future__ import annotations

from pathlib import Path
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist

_RUST_DIR = Path(__file__).parent / "rust"
_EXAMPLES = _RUST_DIR / "target" / "release"
_BUILD_LCM = "cargo build --release"
_BUILD_ZENOH = "cargo build --release --features zenoh"


class PingConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "native_ping")
    build_command: str = _BUILD_LCM
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class PongConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "native_pong")
    build_command: str = _BUILD_LCM
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True
    sample_config: int = 42


class ZenohPingConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "zenoh_ping")
    build_command: str = _BUILD_ZENOH
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class ZenohPongConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "zenoh_pong")
    build_command: str = _BUILD_ZENOH
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


class ZenohPingModule(NativeModule):
    """Ping over the Zenoh transport."""

    config: ZenohPingConfig
    data: Out[Twist]
    confirm: In[Twist]


class ZenohPongModule(NativeModule):
    """Pong over the Zenoh transport."""

    config: ZenohPongConfig
    data: In[Twist]
    confirm: Out[Twist]


def lcm_blueprint():
    return autoconnect(PingModule.blueprint(), PongModule.blueprint())


def zenoh_blueprint():
    return autoconnect(ZenohPingModule.blueprint(), ZenohPongModule.blueprint())


BLUEPRINTS = {"lcm": lcm_blueprint, "zenoh": zenoh_blueprint}


if __name__ == "__main__":
    transport = sys.argv[1] if len(sys.argv) > 1 else "lcm"
    if transport not in BLUEPRINTS:
        sys.exit(f"usage: {Path(sys.argv[0]).name} [{'|'.join(BLUEPRINTS)}]")
    ModuleCoordinator.build(BLUEPRINTS[transport]().global_config(viewer="none")).loop()
