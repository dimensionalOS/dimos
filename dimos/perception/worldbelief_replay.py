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

"""Replay a WorldBelief recording through the same live streams as the physical stack."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any, ClassVar

from pydantic import Field
from reactivex.disposable import CompositeDisposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_REQUIRED_STREAMS = frozenset({"color_image", "depth_image", "camera_info", "tf"})
_REPLAY_ORDER = (
    "tf",
    "camera_info",
    "depth_camera_info",
    "coordinator_joint_state",
    "depth_image",
    "color_image",
)
_RESUME_EPSILON_S = 1e-6


def _resolve_recording_path(dataset: str | Path) -> Path:
    """Resolve an explicit DB or a named LFS dataset containing exactly one DB."""
    requested = Path(dataset).expanduser()
    resolved = requested if requested.is_absolute() or requested.exists() else get_data(requested)
    if resolved.is_dir():
        databases = sorted(resolved.glob("*.db"))
        if len(databases) != 1:
            raise ValueError(
                f"WorldBelief replay directory must contain exactly one .db file: {resolved} "
                f"(found {len(databases)})"
            )
        resolved = databases[0]
    if resolved.suffix != ".db":
        raise ValueError(f"WorldBelief replay expects a Memory2 .db recording, got: {resolved}")
    return resolved


class _WorldBeliefReplaySourceConfig(ModuleConfig):
    dataset: str | Path = Field(default_factory=lambda data: data["g"].replay_db)
    speed: float = Field(default=1.0, gt=0.0)
    seek: float = Field(default=0.0, ge=0.0)
    duration: float | None = Field(default=None, gt=0.0)
    autoplay: bool = True


class _WorldBeliefReplaySource(Module):
    """Publish a recorded RGB-D/TF session with its original relative timing."""

    config: _WorldBeliefReplaySourceConfig
    dedicated_worker: ClassVar[bool] = True

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    coordinator_joint_state: Out[JointState]

    @rpc
    def build(self) -> None:
        self._recording_path = _resolve_recording_path(self.config.dataset)
        logger.info("WorldBelief replay source: %s", self._recording_path)

    @rpc
    def start(self) -> None:
        super().start()
        self._lock = threading.RLock()
        self._playback: CompositeDisposable | None = None
        self._playback_generation = 0
        self._started_once = False
        self._error: str | None = None

        path = getattr(self, "_recording_path", None)
        if path is None:
            path = _resolve_recording_path(self.config.dataset)
            self._recording_path = path

        self._store = self.register_disposable(
            SqliteStore(path=str(path), must_exist=True),
        )
        self._store.start()
        available = set(self._store.list_streams())
        missing = sorted(_REQUIRED_STREAMS - available)
        if missing:
            raise RuntimeError(
                f"WorldBelief replay recording is missing required stream(s): {missing}; "
                f"available: {sorted(available)}"
            )
        stream_names: list[str] = []
        first_timestamps: list[float] = []
        last_timestamps: list[float] = []
        for name in _REPLAY_ORDER:
            if name not in available:
                continue
            stream = self._store.stream(name)
            try:
                first_timestamps.append(float(stream.first().ts))
                last_timestamps.append(float(stream.last().ts))
            except LookupError:
                if name in _REQUIRED_STREAMS:
                    raise RuntimeError(
                        f"WorldBelief replay required stream is empty: {name}"
                    ) from None
                logger.warning("Skipping empty optional replay stream: %s", name)
                continue
            stream_names.append(name)
        self._stream_names = tuple(stream_names)

        self._timeline_start_ts = min(first_timestamps)
        self._timeline_end_ts = max(last_timestamps)
        self._window_start_ts = self._timeline_start_ts + self.config.seek
        if self._window_start_ts >= self._timeline_end_ts:
            raise ValueError(
                f"Replay seek {self.config.seek:.3f}s is outside the "
                f"{self._timeline_end_ts - self._timeline_start_ts:.3f}s recording"
            )
        self._window_end_ts = self._timeline_end_ts
        if self.config.duration is not None:
            self._window_end_ts = min(
                self._window_end_ts,
                self._window_start_ts + self.config.duration,
            )
        self._cursor_ts = self._window_start_ts
        self._active_streams: set[str] = set()
        self._state = "ready"
        logger.info(
            "WorldBelief replay ready: %.3fs..%.3fs at %.2fx",
            self._window_start_ts - self._timeline_start_ts,
            self._window_end_ts - self._timeline_start_ts,
            self.config.speed,
        )

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        """Autoplay only after Recorder, WorldBelief, Rerun, and MCP have started."""
        if self.config.autoplay:
            self._begin_playback()

    def _message_timestamp(self, msg: Any) -> float | None:
        ts = getattr(msg, "ts", None)
        if ts is not None:
            return float(ts)
        transforms = getattr(msg, "transforms", None)
        if transforms:
            return max(float(transform.ts) for transform in transforms)
        return None

    def _publish(self, name: str, msg: Any, generation: int) -> None:
        with self._lock:
            if generation != self._playback_generation or self._state != "playing":
                return
            ts = self._message_timestamp(msg)
            if ts is not None:
                self._cursor_ts = max(self._cursor_ts, min(ts, self._window_end_ts))

        if name == "tf":
            self.tf.publish(*msg.transforms)
        else:
            getattr(self, name).publish(msg)

    def _stream_completed(self, name: str, generation: int) -> None:
        with self._lock:
            if generation != self._playback_generation:
                return
            self._active_streams.discard(name)
            if self._active_streams:
                return
            self._cursor_ts = self._window_end_ts
            self._state = "ended"
            self._playback = None
        logger.info("WorldBelief replay ended")

    def _stream_failed(self, name: str, error: Exception, generation: int) -> None:
        with self._lock:
            if generation != self._playback_generation:
                return
            self._error = f"{name}: {error}"
            self._state = "error"
            self._active_streams.clear()
            playback = self._playback
            self._playback = None
        if playback is not None:
            playback.dispose()
        logger.error("WorldBelief replay failed on %s: %s", name, error)

    def _begin_playback(self) -> None:
        with self._lock:
            if self._state == "playing":
                return
            if self._state == "error":
                raise RuntimeError(f"WorldBelief replay is in error: {self._error}")
            start_ts = self._cursor_ts
            if self._started_once:
                start_ts += _RESUME_EPSILON_S
            if start_ts >= self._window_end_ts:
                self._cursor_ts = self._window_end_ts
                self._state = "ended"
                return

            duration = self._window_end_ts - start_ts
            replay = self._store.replay(
                speed=self.config.speed,
                from_timestamp=start_ts,
                duration=duration,
            )
            self._playback_generation += 1
            generation = self._playback_generation
            self._active_streams = set(self._stream_names)
            self._state = "playing"
            self._error = None
            self._started_once = True
            playback = CompositeDisposable()
            self._playback = playback

            for name in self._stream_names:
                disposable = (
                    replay.stream(name)
                    .observable()
                    .subscribe(
                        on_next=lambda msg, stream=name: self._publish(stream, msg, generation),
                        on_error=lambda error, stream=name: self._stream_failed(
                            stream, error, generation
                        ),
                        on_completed=lambda stream=name: self._stream_completed(stream, generation),
                    )
                )
                playback.add(disposable)

    def _status_locked(self) -> dict[str, Any]:
        timeline_start = getattr(self, "_timeline_start_ts", 0.0)
        timeline_end = getattr(self, "_timeline_end_ts", timeline_start)
        window_start = getattr(self, "_window_start_ts", timeline_start)
        window_end = getattr(self, "_window_end_ts", timeline_end)
        cursor = getattr(self, "_cursor_ts", window_start)
        window_duration = max(0.0, window_end - window_start)
        window_progress = 0.0
        if window_duration > 0.0:
            window_progress = min(1.0, max(0.0, (cursor - window_start) / window_duration))
        return {
            "state": getattr(self, "_state", "not_started"),
            "dataset": str(self.config.dataset),
            "recording": str(getattr(self, "_recording_path", "")),
            "position_s": round(max(0.0, cursor - timeline_start), 3),
            "recording_duration_s": round(max(0.0, timeline_end - timeline_start), 3),
            "window_start_s": round(max(0.0, window_start - timeline_start), 3),
            "window_end_s": round(max(0.0, window_end - timeline_start), 3),
            "window_progress": round(window_progress, 4),
            "speed": self.config.speed,
            "autoplay": self.config.autoplay,
            "error": getattr(self, "_error", None),
        }

    @skill
    def replay_status(self) -> dict[str, Any]:
        """Report replay position; WorldBelief scan sees evidence through this point."""
        with self._lock:
            return self._status_locked()

    @skill
    def replay_pause(self) -> dict[str, Any]:
        """Pause replay without resetting Recorder or WorldBelief state."""
        with self._lock:
            if self._state == "playing":
                self._playback_generation += 1
                playback = self._playback
                self._playback = None
                self._active_streams.clear()
                self._state = "paused"
            else:
                playback = None
        if playback is not None:
            playback.dispose()
        with self._lock:
            return self._status_locked()

    @skill
    def replay_resume(self) -> dict[str, Any]:
        """Resume replay from the last published timestamp."""
        self._begin_playback()
        with self._lock:
            return self._status_locked()

    @rpc
    def stop(self) -> None:
        lock = getattr(self, "_lock", None)
        if lock is not None:
            with lock:
                self._playback_generation += 1
                playback = self._playback
                self._playback = None
                getattr(self, "_active_streams", set()).clear()
                self._state = "stopped"
        else:
            playback = None
        if playback is not None:
            playback.dispose()
        super().stop()
