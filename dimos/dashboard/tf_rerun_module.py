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

"""TF Rerun Module - Snapshot TF visualization in Rerun.

This module polls the TF buffer at a configurable rate and logs the latest
transform for each edge to Rerun. This provides stable, rate-limited TF
visualization without subscribing to the /tf transport from here.

Usage:
    # In blueprints:
    from dimos.dashboard.tf_rerun_module import tf_rerun

    def my_robot():
        return (
            robot_connection()
            + tf_rerun()  # Add TF visualization
            + other_modules()
        )
"""

from collections.abc import Sequence
import threading
import time
from typing import Any, cast

import rerun as rr

from dimos.core import Module, rpc
from dimos.core.blueprints import ModuleBlueprintSet, autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.dashboard.rerun_init import connect_rerun
from dimos.dashboard.rerun_scene_wiring import rerun_scene_wiring
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TFRerunModule(Module):
    """Polls TF buffer and logs snapshot transforms to Rerun.

    This module automatically visualizes the TF tree in Rerun by:
    - Using `self.tf` (the system TF service) to maintain the TF buffer
    - Polling at a configurable rate and logging the latest transform per edge
    """

    _global_config: GlobalConfig
    _poll_thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None
    _poll_hz: float
    _last_ts_by_edge: dict[tuple[str, str], float]

    def __init__(
        self,
        global_config: GlobalConfig | None = None,
        poll_hz: float = 30.0,
        **kwargs: Any,
    ) -> None:
        """Initialize TFRerunModule.

        Args:
            global_config: Optional global configuration for viewer backend settings
            **kwargs: Additional arguments passed to parent Module
        """
        super().__init__(**kwargs)
        self._global_config = global_config or GlobalConfig()
        self._poll_hz = poll_hz
        self._last_ts_by_edge = {}

    @rpc
    def start(self) -> None:
        """Start the TF visualization module."""
        super().start()

        # Only connect if Rerun backend is selected
        if self._global_config.viewer_backend.startswith("rerun"):
            connect_rerun(global_config=self._global_config)

            # Ensure TF transport is started so its internal subscription populates the buffer.
            self.tf.start(sub=True)

            self._stop_event = threading.Event()
            self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll_thread.start()
            logger.info("TFRerunModule: started TF snapshot polling", poll_hz=self._poll_hz)

    def _poll_loop(self) -> None:
        assert self._stop_event is not None
        period_s = 1.0 / max(self._poll_hz, 0.1)

        while not self._stop_event.is_set():
            # Snapshot keys to avoid concurrent modification while TF buffer updates.
            items = list(self.tf.buffers.items())  # type: ignore[attr-defined]
            for (parent, child), buffer in items:
                latest = buffer.get()
                if latest is None:
                    continue
                last_ts = self._last_ts_by_edge.get((parent, child))
                if last_ts is not None and latest.ts == last_ts:
                    continue

                # Log under `world/tf/...` so it is visible under the default 3D view origin.
                rr.log(f"world/tf/{child}", latest.to_rerun())  # type: ignore[no-untyped-call]
                self._last_ts_by_edge[(parent, child)] = latest.ts

            time.sleep(period_s)

    @rpc
    def stop(self) -> None:
        """Stop the TF visualization module and cleanup LCM subscription."""
        if self._stop_event is not None:
            self._stop_event.set()
            self._stop_event = None

        if self._poll_thread is not None and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        self._poll_thread = None

        super().stop()


def tf_rerun(
    *,
    poll_hz: float = 30.0,
    scene: bool = True,
    # Scene wiring kwargs (only used if scene=True)
    world_entity: str = "world",
    robot_entity: str = "world/robot",
    robot_axes_entity: str = "world/robot/axes",
    world_frame: str = "world",
    robot_frame: str = "base_link",
    urdf_path: str | None = None,
    axes_size: float | None = 0.5,
    cameras: Sequence[tuple[str, str, Any]] = (),
    camera_rgb_suffix: str = "rgb",
) -> ModuleBlueprintSet:
    """Convenience blueprint: TF snapshot polling + (optional) static scene wiring.

    - TF visualization stays in `TFRerunModule` (poll TF buffer, log to `world/tf/*`).
    - Scene wiring is handled by `RerunSceneWiringModule` (view coords, attachments, URDF, pinholes).
    """
    tf_bp = cast("ModuleBlueprintSet", TFRerunModule.blueprint(poll_hz=poll_hz))
    if not scene:
        return tf_bp

    scene_bp = cast(
        "ModuleBlueprintSet",
        rerun_scene_wiring(
            world_entity=world_entity,
            robot_entity=robot_entity,
            robot_axes_entity=robot_axes_entity,
            world_frame=world_frame,
            robot_frame=robot_frame,
            urdf_path=urdf_path,
            axes_size=axes_size,
            cameras=cameras,
            camera_rgb_suffix=camera_rgb_suffix,
        ),
    )

    return autoconnect(tf_bp, scene_bp)
