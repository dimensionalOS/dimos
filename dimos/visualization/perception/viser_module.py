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

"""Loopback Viser inspection for RGB-D perception. No motion controls or robot model."""

from __future__ import annotations

import threading
from typing import Any, Literal

import cv2
import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.runtime import ViserRuntime
from dimos.visualization.perception.scene_spec import PerceptionSceneSpec


class PerceptionViserConfig(ModuleConfig):
    host: Literal["127.0.0.1"] = "127.0.0.1"
    port: int = Field(default=8096, ge=1024, le=65535)
    poll_hz: float = Field(default=2.0, gt=0, le=10)
    max_points: int = Field(default=10000, ge=1, le=50000)


class PerceptionViser(Module):
    config: PerceptionViserConfig
    _scene: PerceptionSceneSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime = ViserRuntime(
            ViserVisualizationConfig(host=self.config.host, port=self.config.port)
        )
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture_ts: float | None = None
        self._objects: dict[str, dict[str, Any]] = {}
        self._scan_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("Previous perception viewer thread is still running")
        super().start()
        server = self._runtime.start()
        server.scene.set_up_direction("-y")
        self._root = server.scene.add_frame("/perception", show_axes=True)
        self._status = server.gui.add_markdown(
            "No scan. Coordinates are camera optical metres, not arm targets."
        )
        self._prompts = server.gui.add_text(
            "Object prompts", initial_value="water bottle, cup, box"
        )
        self._scan_button = server.gui.add_button("Scan scene")
        self._selection = server.gui.add_dropdown("Inspect object", options=["None"])
        self._details = server.gui.add_markdown("No object selected")
        empty = np.zeros((16, 16, 3), np.uint8)
        self._rgb = server.gui.add_image(empty, label="RGB / segmentation", visible=False)
        self._depth = server.gui.add_image(
            empty, label="Depth 0.3-3 m, black invalid", visible=False
        )

        @self._scan_button.on_click
        def scan(_: Any) -> None:
            self.scan(self._prompts.value)

        @self._selection.on_update
        def select(_: Any) -> None:
            obj = self._objects.get(self._selection.value)
            self._details.content = (
                "No object selected"
                if obj is None
                else f"{obj['name']} / {obj['object_id']}\n\nVisible-surface center (m): {obj['center']}\n\nAABB size (m): {obj['size']}\n\nPoints: {obj['point_count']}"
            )

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll, name="PerceptionViser", daemon=True)
        self._thread.start()

    @rpc
    def scan(self, prompts: str) -> bool:
        """Request perception only. A second request while busy is rejected."""
        labels = [label.strip() for label in prompts.split(",") if label.strip()]
        if not labels:
            raise ValueError("At least one object prompt is required")
        if not self._scan_lock.acquire(blocking=False):
            return False
        try:
            self._status.content = "Scanning. This does not move the robot."
            self._scene.scan_scene(labels)
            self._capture_ts = None
            return True
        except Exception as error:
            self._status.content = f"Scan failed: {error}"
            self._hide()
            return False
        finally:
            self._scan_lock.release()

    def _hide(self) -> None:
        self._root.visible = False
        self._rgb.visible = self._depth.visible = False
        self._objects.clear()
        self._selection.options = ["None"]
        self._details.content = "No fresh object geometry"

    def _poll(self) -> None:
        while not self._stop_event.wait(1 / self.config.poll_hz):
            try:
                self.refresh()
            except Exception as error:
                self._status.content = f"Perception unavailable: {error}"
                self._hide()

    def refresh(self) -> None:
        status = self._scene.get_scan_status()
        if self._stop_event.is_set():
            return
        self._status.content = f"Frame: {status['frame_id']} | capture age: {status['age_s']} s | inference: {status['inference_s']} s\n\nQuality: {status['quality']}\n\n{status['error'] or status['reason']}"
        if not status["available"]:
            self._hide()
            return
        if status["capture_ts"] == self._capture_ts:
            return
        snapshot = self._scene.get_scan_visualization(self.config.max_points)
        if self._stop_event.is_set():
            return
        if not snapshot["status"]["available"]:
            self._hide()
            return
        server = self._runtime.server
        assert server is not None
        server.scene.remove_by_name("/perception")
        self._root = server.scene.add_frame("/perception", show_axes=True)
        self._objects.clear()
        self._rgb.image = snapshot["overlay"].to_rgb().data
        depth = snapshot["depth"].data
        scale = np.clip((depth - 0.3) / 2.7, 0, 1)
        image = cv2.applyColorMap(((1 - scale) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        image[(depth <= 0) | ~np.isfinite(depth)] = 0
        self._depth.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self._rgb.visible = self._depth.visible = True
        server.scene.add_point_cloud(
            "/perception/scene", points=snapshot["points"], colors=(100, 110, 120), point_size=0.005
        )
        for i, obj in enumerate(snapshot["objects"]):
            name = f"{obj['name']} / {obj['object_id']}"
            self._objects[name] = obj
            color = [(50, 220, 100), (255, 180, 40), (80, 160, 255)][i % 3]
            prefix = f"/perception/object-{i}"
            server.scene.add_point_cloud(
                prefix + "/points", points=obj["points"], colors=color, point_size=0.008
            )
            server.scene.add_box(
                prefix + "/bounds",
                dimensions=tuple(max(s, 0.001) for s in obj["size"]),
                position=tuple(obj["center"]),
                color=color,
                wireframe=True,
            )
            server.scene.add_label(
                prefix + "/label", text=obj["name"], position=tuple(obj["center"])
            )
        self._selection.options = ["None", *self._objects]
        self._capture_ts = status["capture_ts"]

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        pending = False
        if self._thread is not None:
            self._thread.join(timeout=5)
            pending = self._thread.is_alive()
            if not pending:
                self._thread = None
        self._runtime.close()
        super().stop()
        if pending:
            raise RuntimeError(
                "Perception viewer RPC is still pending; listener closed, do not restart viewer"
            )
