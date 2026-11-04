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

"""Native (Rust) replay of a recording: one ``Out`` port per recorded stream.

A drop-in for :func:`dimos.memory.replay_module.replay_module`, configured with ``db`` in
place of ``dataset``. Python only decides the ports (from the recording, exactly as
``ReplayModule`` does); the binary reads the ``.db`` or ``.mcap`` and publishes each
stream's stored LCM bytes on its port's topic, so nothing is decoded or re-encoded on the
way through. With no ``db`` the module is a no-op: its ports exist, nothing is launched.
"""

from __future__ import annotations

from functools import partial
import os
import sys
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.memory.cli.dataset import resolve_dataset
from dimos.memory.replay import _LOOP_GAP
from dimos.memory.replay_module import _port_names, stream_types_of
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ReplayNativeConfig(NativeModuleConfig):
    executable: str = "result/bin/memory_replay"
    build_command: str | None = "nix build -L path:."
    source_dir: str | None = "dimos/memory/replay_native/rust"
    stdin_config: bool = True

    # The recording (.db or .mcap, or a dataset name); empty makes the module a no-op.
    db: str = ""
    # Chooses the ports in Python; the binary gets the result as ``ports``.
    topics: str = Field(default="*", exclude=True)
    speed: float = Field(default=1.0, gt=0.0)
    loop: bool = False
    # Once the whole recording has played (loop off), the run shuts down.
    exit_on_complete: bool = False
    # Filled at start with exit_on_complete: the pid of this run's coordinator, which the binary signals when done.
    shutdown_pid: int = 0
    seek: float | None = Field(default=None, ge=0.0)
    duration: float | None = Field(default=None, gt=0.0)
    # Recorded-time gap between the end of one loop pass and the start of the next.
    loop_gap: float = Field(default=_LOOP_GAP, ge=0.0)
    # Period at which a single-timestamp stream (e.g. camera_info) is republished.
    static_period: float = Field(default=1.0, gt=0.0)
    # Filled from the module's Out ports at start.
    ports: list[str] = Field(default_factory=list)

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The binary takes a file path, and 0 for "from the first message" / "to the end".
        config["db"] = str(resolve_dataset(self.db)) if self.db else ""
        config["seek"] = self.seek or 0.0
        config["duration"] = self.duration or 0.0
        return config


class ReplayNative(NativeModule):
    """Publishes every ``Out`` port from the recording at recorded timing, from Rust."""

    config: ReplayNativeConfig
    stream_types: dict[str, type] = {}  # set by replay_native()
    topics: str = "*"  # set by replay_native(); carried to workers via the blueprint kwargs

    @classproperty
    def blueprint(cls) -> Any:  # noqa: N805
        from dimos.core.coordination.blueprints import Blueprint

        return partial(Blueprint.create, cls, topics=cls.topics)  # type: ignore[arg-type]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self.config.db:
            try:
                self._add_ports()
            except Exception:
                self.stop()  # Module.__init__ already spun up its threads
                raise

    def _add_ports(self) -> None:
        types = stream_types_of(self.config.db)
        existing = self.outputs
        for name in _port_names(self.config.topics, types, owner=ReplayNative):
            if name not in existing:
                setattr(self, name, Out(types[name], name, self))

    @rpc
    def start(self) -> None:
        if not self.config.db:
            # No recording: the ports exist but nothing publishes, and no binary is launched.
            logger.info("ReplayNative has no db; not replaying")
            Module.start(self)
            return
        self.config.ports = sorted(self.outputs)
        if self.config.exit_on_complete and not self.config.loop:
            self.config.shutdown_pid = _run_coordinator_pid() or 0
        super().start()

    def _argv(self, _topics: dict[str, str]) -> list[str]:
        """The binary reads its topics and config from the stdin line only."""
        return [self._executable]


def _run_coordinator_pid() -> int | None:
    """The coordinator registered under this process's run id, which the binary SIGTERMs as ``dimos stop`` does.
    The bus-wide ``Coordinator`` RPC name is not used: with two runs on one bus it reaches whichever answers."""
    from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
    from dimos.core.run_registry import list_runs

    run_id = os.environ.get(DIMOS_RUN_ID_ENV)
    entry = next((e for e in list_runs() if e.run_id == run_id), None) if run_id else None
    if entry is None:
        logger.warning("exit_on_complete: not under a registered `dimos run`; it will keep running")
        return None
    return entry.pid


def replay_native(db: str, topics: str = "*", name: str = "ReplayNative") -> type[ReplayNative]:
    """Build a :class:`ReplayNative` subclass with an ``Out`` per stream in *db*.

    Assign the result to *name* at module level: deploying to a worker pickles the
    class by that path. An empty *db* yields a port-less class.
    """
    ports: dict[str, Any] = {}
    types: dict[str, type] = {}
    if db:
        types = stream_types_of(db)
        for n in _port_names(topics, types, owner=ReplayNative):
            ports[n] = Out[types[n]]  # type: ignore[valid-type]
    caller = sys._getframe(1).f_globals.get("__name__", __name__)
    namespace = {
        "__annotations__": ports,
        "__module__": caller,
        "stream_types": {n: types[n] for n in ports},
        "topics": topics,
    }
    return type(name, (ReplayNative,), namespace)
