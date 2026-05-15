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

"""Standalone Rerun-backed data recorder for robot-learning data collection.

`RerunDataRecorder` owns its own ``rr.RecordingStream`` + ``rr.FileSink`` and
writes one ``.rrd`` per demonstration *episode*. It subscribes to the same
pubsubs the live-viewer bridge does, runs the same converter contract
(``visual_override`` chaining, ``entity_prefix`` derivation, ``to_rerun()``),
and is otherwise independent of the bridge — a viewer failure does not affect
recording, a recorder failure does not affect viewing.

Bridge ``bridge.py`` is **not** modified by this module. Only already
module-level types (``RerunMulti``, ``RerunData``, ``RerunConvertible``,
``is_rerun_multi``) are imported. The small composition logic
(``_get_entity_path`` / ``_compose_converter``) is reimplemented locally —
~20 lines, structurally mirroring the bridge — guarded by a drift-detector
unit test in ``test_recorder.py``.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import field
from pathlib import Path
import threading
from typing import Any, cast

from reactivex.disposable import Disposable
import rerun as rr
from rerun._baseclasses import Archetype
from toolz import pipe  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.utils.logging_config import setup_logger

# Types only — bridge.py is intentionally not edited by this change.
from dimos.visualization.rerun.bridge import (
    RerunConvertible,
    RerunData,
    is_rerun_multi,
)

logger = setup_logger()


class RerunDataRecorderConfig(ModuleConfig):
    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(default_factory=lambda: [LCM()])

    visual_override: dict[Glob | str, Callable[[Any], Archetype]] = field(default_factory=dict)
    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None

    record_path_factory: Callable[[], Path] | None = None
    recording_id_factory: Callable[[Path], str] | None = None
    episode_metadata: Callable[[int], dict[str, str]] | None = None

    app_id: str = "dimos_recorder"


RerunDataRecorderConfig.model_rebuild(_types_namespace={"Archetype": Archetype})


class RerunDataRecorder(Module):
    """Records pubsub messages to per-episode ``.rrd`` files via a standalone
    ``rr.RecordingStream``.

    One recorder instance writes a sequence of episodes to disk under a session
    directory. Episode boundaries are operator-driven via ``rotate_recording()``.
    """

    config: RerunDataRecorderConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._record_stream: rr.RecordingStream | None = None
        self._current_path: Path | None = None
        self._current_nonempty: bool = False
        self._episode_index: int = 0
        self._record_lock: threading.Lock = threading.Lock()
        self._converter_cache: dict[str, Callable[[Any], RerunData | None]] = {}

    # ── Composition logic (mirrors bridge; do not import bridge logic) ──

    def _get_entity_path(self, topic: Any) -> str:
        if self.config.topic_to_entity:
            return self.config.topic_to_entity(topic)
        topic_str = getattr(topic, "name", None) or str(topic)
        topic_str = topic_str.split("#")[0]
        return f"{self.config.entity_prefix}{topic_str}"

    def _compose_converter(self, entity_path: str) -> Callable[[Any], RerunData | None]:
        cached = self._converter_cache.get(entity_path)
        if cached is not None:
            return cached

        matches = [
            fn
            for pattern, fn in self.config.visual_override.items()
            if pattern_matches(pattern, entity_path)
        ]

        if any(fn is None for fn in matches):

            def suppressed(_msg: Any) -> RerunData | None:
                return None

            self._converter_cache[entity_path] = suppressed
            return suppressed

        def final_convert(msg: Any) -> RerunData | None:
            if isinstance(msg, Archetype):
                return msg
            if is_rerun_multi(msg):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        def composed(msg: Any) -> RerunData | None:
            return cast("RerunData | None", pipe(msg, *matches, final_convert))

        self._converter_cache[entity_path] = composed
        return composed

    # ── Episode plumbing ──

    def _open_episode(
        self,
        *,
        path: Path | None = None,
        recording_id: str | None = None,
    ) -> Path:
        if path is None:
            if self.config.record_path_factory is None:
                raise RuntimeError(
                    "RerunDataRecorder: no record_path_factory configured and no path provided"
                )
            path = self.config.record_path_factory()
        path = Path(path)
        if path.exists():
            raise FileExistsError(f"Recorder refusing to overwrite existing file: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)

        if recording_id is None and self.config.recording_id_factory is not None:
            recording_id = self.config.recording_id_factory(path)

        stream = rr.RecordingStream(self.config.app_id, recording_id=recording_id)
        rr.set_sinks(rr.FileSink(str(path)), recording=stream)

        self._record_stream = stream
        self._current_path = path
        self._current_nonempty = False

        self._log_episode_metadata()
        logger.info("RerunDataRecorder: recording started → %s", path)
        return path

    def _close_episode(self) -> None:
        with self._record_lock:
            stream = self._record_stream
            path = self._current_path
            nonempty = self._current_nonempty
            self._record_stream = None
            self._current_path = None
            self._current_nonempty = False

        if stream is not None:
            try:
                stream.flush(timeout_sec=2.0)
            except Exception:
                logger.exception("RerunDataRecorder: flush failed during episode close")
            # Dropping the stream finalizes the FileSink.
            del stream

        if path is None:
            return
        if nonempty:
            logger.info("RerunDataRecorder: recording stopped → %s", path)
            return
        try:
            path.unlink(missing_ok=True)
            logger.info("RerunDataRecorder: recording stopped (empty, discarded %s)", path.name)
        except OSError:
            logger.exception("RerunDataRecorder: failed to delete empty episode file %s", path)

    def _log_episode_metadata(self) -> None:
        stream = self._record_stream
        if stream is None:
            return
        if self.config.episode_metadata is not None:
            meta = self.config.episode_metadata(self._episode_index)
        else:
            meta = {"episode_index": str(self._episode_index)}

        for key in ("episode_id", "episode_index", "session_id", "operator"):
            value = meta.get(key)
            if value is None:
                continue
            rr.log(
                f"/meta/{key}",
                rr.TextDocument(str(value)),
                static=True,
                recording=stream,
            )

    # ── Module lifecycle / RPCs ──

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.record_path_factory is None:
            raise RuntimeError("RerunDataRecorder requires `record_path_factory` to be configured")

        # Recorder starts in IDLE. The operator must press the toggle button
        # once to open episode_001.rrd — so warmup motion, scene setup, and
        # gripper calibration before the first demo never end up baked into a
        # recording. See design.md Decision 7.
        self._episode_index = 0
        self._record_stream = None
        self._current_path = None
        self._current_nonempty = False

        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "start"):
                pubsub.start()
            unsub = pubsub.subscribe_all(self._on_message)
            self.register_disposable(Disposable(unsub))

        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "stop"):
                self.register_disposable(Disposable(pubsub.stop))  # type: ignore[union-attr]

        logger.info("RerunDataRecorder: idle — press the toggle button to start recording")

    def _on_message(self, msg: Any, topic: Any) -> None:
        entity_path = self._get_entity_path(topic)
        rerun_data = self._compose_converter(entity_path)(msg)
        if not rerun_data:
            return

        with self._record_lock:
            stream = self._record_stream
            if stream is None:
                return

            if is_rerun_multi(rerun_data):
                for path, archetype in rerun_data:
                    rr.log(path, archetype, recording=stream)
            else:
                rr.log(entity_path, cast("Archetype", rerun_data), recording=stream)
            self._current_nonempty = True

    @rpc
    def toggle_recording(
        self,
        *,
        recording_id: str | None = None,
        path: Path | None = None,
    ) -> Path | None:
        """Toggle between RECORDING and IDLE.

        - In RECORDING (``_record_stream`` is not None): flush + close the
          current episode, delete the file if it's empty, return to IDLE.
          Returns ``None``.
        - In IDLE: open the next episode and return to RECORDING. Returns the
          new path.

        Empty episodes are discarded so a debounced or bounced trigger does not
        leave zero-content artifacts on disk.
        """
        if self._record_stream is not None:
            self._close_episode()
            return None
        if self.config.record_path_factory is None and path is None:
            logger.warning(
                "RerunDataRecorder.toggle_recording called without a configured "
                "record_path_factory; staying idle"
            )
            return None
        self._episode_index += 1
        return self._open_episode(path=path, recording_id=recording_id)

    @rpc
    def stop(self) -> None:
        self._close_episode()
        self._converter_cache.clear()
        super().stop()


__all__ = ["RerunDataRecorder", "RerunDataRecorderConfig"]
