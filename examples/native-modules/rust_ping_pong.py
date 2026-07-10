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
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist

_RUST_DIR = Path(__file__).parent / "rust"
_EXAMPLES = _RUST_DIR / "target" / "release"
_BUILD = "cargo build --release"


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


def blueprint():
    return autoconnect(PingModule.blueprint(), PongModule.blueprint())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["lcm", "zenoh"], default="lcm")
    args = parser.parse_args()

    bp = blueprint().global_config(viewer="none", transport=args.transport)
    ModuleCoordinator.build(bp).loop()
