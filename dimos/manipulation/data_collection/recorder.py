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
writes one ``.rrd`` per demonstration *episode*. Inputs arrive on typed
``In[T]`` stream slots — the same idiom used by ``PolicyModule`` — so a
blueprint connects each producer to the corresponding slot at wiring time:

* ``image: In[Image]`` — camera frame, logged under
  ``/observation/camera/{camera_key}``.
* ``joint_state: In[JointState]`` — measured joint state, logged as
  per-joint scalars under ``/observation/state/<joint>``.
* ``desired_joint_action: In[JointState]`` — commanded joint trajectory,
  logged as per-joint scalars under ``/action/<joint>``.

The recorder is independent of the live-viewer bridge: a viewer failure
does not affect recording, a recorder failure does not affect viewing.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
from typing import Any

from reactivex.disposable import Disposable
import rerun as rr
from rerun._baseclasses import Archetype

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.data_collection.piper_blueprint_config import (
    joint_state_to_rerun_scalars,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RerunDataRecorderConfig(ModuleConfig):
    # Camera frames arriving on the `image` slot are logged under
    # `f"/observation/camera/{camera_key}"`. Mirrors
    # `PolicyModuleConfig.camera_key` so producer-side wiring is symmetric.
    camera_key: str = "usb"

    record_path_factory: Callable[[], Path] | None = None
    recording_id_factory: Callable[[Path], str] | None = None
    episode_metadata: Callable[[int], dict[str, str]] | None = None

    app_id: str = "dimos_recorder"


class RerunDataRecorder(Module):
    """Records typed stream inputs to per-episode ``.rrd`` files via a
    standalone ``rr.RecordingStream``.

    One recorder instance writes a sequence of episodes to disk under a
    session directory. Episode boundaries are operator-driven via
    ``toggle_recording()``.
    """

    config: RerunDataRecorderConfig

    image: In[Image]
    joint_state: In[JointState]
    desired_joint_action: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._record_stream: rr.RecordingStream | None = None
        self._current_path: Path | None = None
        self._current_nonempty: bool = False
        self._episode_index: int = 0
        self._record_lock: threading.Lock = threading.Lock()
        self._measured_scalars = joint_state_to_rerun_scalars("measured")
        self._commanded_scalars = joint_state_to_rerun_scalars("commanded")

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

    # ── Typed input handlers ──

    def _camera_entity_path(self) -> str:
        return f"/observation/camera/{self.config.camera_key}"

    def _on_image(self, image: Image) -> None:
        """Log a camera frame under ``/observation/camera/{camera_key}``.

        A frame received while the recorder is IDLE is dropped on the floor.
        """
        archetype = image.to_rerun()
        if archetype is None:
            return
        with self._record_lock:
            stream = self._record_stream
            if stream is None:
                return
            rr.log(self._camera_entity_path(), archetype, recording=stream)
            self._current_nonempty = True

    def _on_joint_state(self, msg: JointState) -> None:
        """Log measured joint state as per-joint scalars."""
        self._log_joint_scalars(self._measured_scalars(msg))

    def _on_desired_joint_action(self, msg: JointState) -> None:
        """Log commanded joint trajectory as per-joint scalars."""
        self._log_joint_scalars(self._commanded_scalars(msg))

    def _log_joint_scalars(self, scalars: list[tuple[str, Archetype]]) -> None:
        if not scalars:
            return
        with self._record_lock:
            stream = self._record_stream
            if stream is None:
                return
            for path, archetype in scalars:
                rr.log(path, archetype, recording=stream)
            self._current_nonempty = True

    # ── Module lifecycle / RPCs ──

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.record_path_factory is None:
            raise RuntimeError("RerunDataRecorder requires `record_path_factory` to be configured")

        # Recorder starts in IDLE. The operator must press the toggle button
        # once to open episode_001.rrd — so warmup motion, scene setup, and
        # gripper calibration before the first demo never end up baked into a
        # recording.
        self._episode_index = 0
        self._record_stream = None
        self._current_path = None
        self._current_nonempty = False

        # Typed slots may not be wired to a transport in standalone unit
        # tests (no blueprint, no autoconnect). Drive the handlers directly
        # via `_on_*(...)` in that case.
        for slot_name, handler in (
            ("image", self._on_image),
            ("joint_state", self._on_joint_state),
            ("desired_joint_action", self._on_desired_joint_action),
        ):
            slot = getattr(self, slot_name, None)
            if slot is None:
                continue
            try:
                self.register_disposable(Disposable(slot.subscribe(handler)))
            except AttributeError:
                logger.debug(
                    "RerunDataRecorder: %s not connected to a transport — "
                    "typed path will only fire via direct handler calls",
                    slot_name,
                )

        logger.info("RerunDataRecorder: idle — press the toggle button to start recording")

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
        super().stop()


__all__ = ["RerunDataRecorder", "RerunDataRecorderConfig"]
