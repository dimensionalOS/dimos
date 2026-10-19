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

"""Record a blueprint that is already running, from a process of its own, until SIGINT/SIGTERM.

    python -m dimos.evals.sandbox.record <memory.db> run <blueprint>... [--config.flags]

The sandbox runner starts one beside the robot around each run of the agent's system that the
human tries or evaluate grades (``Conversation.record`` / ``SandboxOutcome.record``), so each of
those runs gets a store of its own and the hours the agent spends writing code get none. It taps
the same transports ``dimos run`` gives the blueprint's modules, worked out from the same
arguments and environment (transport, zenoh settings and namespace), with the recorder
``dimos --record`` uses (dimos.memory.tap). It prints ``recording`` once subscribed.
"""

from __future__ import annotations

from collections.abc import Sequence
import os
from pathlib import Path
import signal
import sys
import threading
import time
from types import FrameType
from typing import Any

from dimos.core.coordination.blueprint_config.parser import (
    BlueprintConfigParser,
    split_run_arguments,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import (
    _get_transport_for,
    _is_name_unique,
    _materialize_transports,
    _resolve_blueprint_config,
    stream_name_types,
)
from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.robot.get_all_blueprints import get_by_name_or_exit
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Seconds subscriptions get to reach the blueprint's sessions before this says it is recording.
SETTLE_S = 2.0


def blueprint_transports(dimos_args: Sequence[str]) -> dict[tuple[str, type], Transport[Any]]:
    """Every stream ``dimos <dimos_args>`` wires, and the transport its modules use for it.

    Global options before ``run`` are not read; the environment and the run's own flags are.
    A stream whose name the blueprint has twice gets a random topic at build, and is skipped.
    """
    run_args = list(dimos_args)[list(dimos_args).index("run") + 1 :]
    names, tokens = split_run_arguments(run_args)
    global_config.update(**BlueprintConfigParser.preparse_global_config(tokens, environ=os.environ))
    blueprint = autoconnect(*map(get_by_name_or_exit, names))
    parsed = BlueprintConfigParser(blueprint).parse(tokens, environ=os.environ)
    global_values, _, overrides = _resolve_blueprint_config(blueprint, parsed)
    global_config.update(**global_values)
    transports = _materialize_transports(blueprint, overrides)
    for name, stream_type in stream_name_types(blueprint):
        if (name, stream_type) in transports:
            continue
        if _is_name_unique(blueprint, name):
            transports[name, stream_type] = _get_transport_for(blueprint, name, stream_type)
        else:
            logger.warning("not recording %s: its topic is picked at random at build", name)
    return transports


def record(path: Path, dimos_args: Sequence[str], stop: threading.Event) -> None:
    """Record the running blueprint of ``dimos <dimos_args>`` into *path* until *stop*."""
    transports = blueprint_transports(dimos_args)
    path.parent.mkdir(parents=True, exist_ok=True)
    store = SqliteStore(path=str(path))
    store.start()
    recorder = TransportRecorder(store, global_config.record_topics)
    unsubscribes = [
        unsubscribe
        for (name, stream_type), transport in transports.items()
        if (unsubscribe := recorder.tap(name, stream_type, transport)) is not None
    ]
    try:
        time.sleep(SETTLE_S)
        print("recording", path, flush=True)
        stop.wait()
    finally:
        for unsubscribe in unsubscribes:
            unsubscribe()
        recorder.close()
        store.stop()
    logger.info("recorded", path=str(path), dropped=recorder.dropped)


def main(argv: Sequence[str]) -> int:
    stop = threading.Event()

    def on_signal(signum: int, frame: FrameType | None) -> None:
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    record(Path(argv[0]), argv[1:], stop)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
