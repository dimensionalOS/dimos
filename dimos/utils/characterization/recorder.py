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

"""Memory2-backed recorder for characterization runs.

Drops into any ControlCoordinator blueprint. Two input ports, one
SQLite stream each:

    commanded: In[Twist]        → sqlite stream "commanded"
    measured:  In[PoseStamped]  → sqlite stream "measured"

The ``BmsLogger`` helper is separate: the runner calls it at 1 Hz to
write battery SOC/voltage/current samples into the same DB. It only
works on a live UnitreeWebRTCConnection; in mujoco it's a no-op.
"""

from __future__ import annotations

import logging
import sqlite3
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist

if TYPE_CHECKING:
    from dimos.memory2.store.sqlite import SqliteStore

logger = logging.getLogger(__name__)


class CharacterizationRecorder(Recorder):
    """Records the commanded Twist and measured pose streams to a SQLite DB.

    Port names are used as the SQLite stream names — keep them stable,
    analysis scripts look them up by name.

    Overrides ``Recorder.start`` only to wrap each port's append callback
    with a shutdown-safe wrapper that drops in-flight LCM deliveries
    arriving after the SQLite store has closed (which happens routinely
    during coordinator teardown).
    """

    commanded: In[Twist]
    measured: In[PoseStamped]

    @rpc
    def start(self) -> None:
        # Re-implement Recorder.start() with a tolerant append wrapper so
        # the noisy "Cannot operate on a closed database" traceback at
        # shutdown is suppressed. Same effect as the parent for normal ops.
        from dimos.core.module import Module

        Module.start(self)

        db_path = Path(self.config.db_path)
        if db_path.exists():
            if self.config.overwrite:
                db_path.unlink()
                logger.info("Deleted existing recording %s", db_path)
            else:
                raise FileExistsError(f"Recording already exists: {db_path}")

        if not self.inputs:
            logger.warning(
                "CharacterizationRecorder has no In ports — nothing to record"
            )
            return

        self._shutting_down = threading.Event()
        for name, port in self.inputs.items():
            stream: Stream[Any] = self.store.stream(name, port.type)
            self.register_disposable(_port_to_stream_safe(port, stream, self._shutting_down))
            logger.info("Recording %s (%s)", name, port.type.__name__)

    @rpc
    def stop(self) -> None:
        # Flag in-flight callbacks to drop quietly; then let the parent
        # tear down disposables and close the store.
        if hasattr(self, "_shutting_down"):
            self._shutting_down.set()
        super().stop()


def _port_to_stream_safe(in_, stream, shutting_down: threading.Event):
    """Like ``memory2.module.port_to_stream``, but drops late-arriving
    appends when the store has been closed during shutdown."""

    def _append(value):
        if shutting_down.is_set():
            return
        try:
            stream.append(value)
        except sqlite3.ProgrammingError:
            # DB closed underneath us — happens when a final LCM packet is
            # delivered between the recorder.stop() call and the LCM thread
            # noticing the unsubscribe. The data already written is safe.
            pass

    return Disposable(in_.subscribe(_append))


class BmsLogger:
    """Thread-safe 1 Hz battery sampler for run metadata.

    Construct with the Unitree WebRTC connection (``go2_conn.connection``).
    If the connection doesn't expose ``lowstate_stream`` (mujoco, replay),
    the logger is instantiated as a no-op — every method returns ``None``.

    The runner owns the calling cadence; we don't spawn a thread here.
    """

    def __init__(self, webrtc_connection: Any | None) -> None:
        self._reader = None
        if webrtc_connection is None or not hasattr(webrtc_connection, "lowstate_stream"):
            logger.info("BmsLogger: lowstate_stream unavailable — BMS capture disabled")
            return

        try:
            from dimos.utils.reactive import getter_hot

            # nonblocking=True: returns None until the first sample arrives
            # rather than blocking our main busy-wait loop.
            self._reader = getter_hot(
                webrtc_connection.lowstate_stream(),
                timeout=5.0,
                nonblocking=True,
            )
        except Exception as e:  # pragma: no cover — best effort
            logger.warning("BmsLogger: failed to attach to lowstate_stream: %s", e)
            self._reader = None

    @property
    def available(self) -> bool:
        return self._reader is not None

    def snapshot(self) -> dict[str, float | int | None]:
        """Return ``{soc, power_v, current_ma}`` or all-None if unavailable."""
        if self._reader is None:
            return {"soc": None, "power_v": None, "current_ma": None}
        try:
            msg = self._reader()
        except Exception as e:  # pragma: no cover
            logger.warning("BmsLogger: lowstate read failed: %s", e)
            return {"soc": None, "power_v": None, "current_ma": None}

        if not isinstance(msg, dict):
            return {"soc": None, "power_v": None, "current_ma": None}

        data = msg.get("data", {}) if isinstance(msg, dict) else {}
        bms = data.get("bms_state", {}) if isinstance(data, dict) else {}
        return {
            "soc": bms.get("soc"),
            "power_v": data.get("power_v"),
            "current_ma": bms.get("current"),
        }

    def log_sample(self, store: SqliteStore, *, ts: float | None = None) -> None:
        """Append one BMS sample (if available) to three SQLite streams."""
        if self._reader is None:
            return
        snap = self.snapshot()
        if snap["soc"] is None:
            return
        _ts = ts if ts is not None else time.time()
        store.stream("bms_soc", int).append(int(snap["soc"]), ts=_ts)
        if snap["power_v"] is not None:
            store.stream("bms_power_v", float).append(float(snap["power_v"]), ts=_ts)
        if snap["current_ma"] is not None:
            store.stream("bms_current_ma", int).append(int(snap["current_ma"]), ts=_ts)


__all__ = ["BmsLogger", "CharacterizationRecorder"]
