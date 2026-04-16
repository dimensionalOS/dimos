"""Two Rust NativeModules for a simple ping-pong example.

PingModule and PongModule both declare a `data` port (Twist) and a `confirm` port (Twist).
Ping publishes to data (received by pong), and Pong published to confirm (received by ping).
Topics and module configs are sent through stdin to modules.

Run with:
    python dimos/native/rust/examples/rust_ping_pong.py
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist

_RUST_DIR = Path(__file__).parents[1]
_EXAMPLES = _RUST_DIR / "target" / "release" / "examples"


class PingConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "native_ping")
    build_command: str = "cargo build --examples --release"
    cwd: str = str(_RUST_DIR)


class PongConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "native_pong")
    build_command: str = "cargo build --examples --release"
    cwd: str = str(_RUST_DIR)
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


if __name__ == "__main__":
    ModuleCoordinator.build(
        autoconnect(
            PingModule.blueprint(),
            PongModule.blueprint(),
        ).global_config(viewer="none")
    ).loop()
