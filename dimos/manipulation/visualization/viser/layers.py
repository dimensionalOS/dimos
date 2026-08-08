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

"""Latest-wins Viser registry for generic visualization layers."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Condition, Thread
import time
from typing import Any

from dimos.manipulation.visualization.layers import VisualizationLayer
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _display_name(segment: str) -> str:
    return segment.replace("-", " ").replace("_", " ").title()


@dataclass
class _LayerState:
    visible: bool
    generation: int = 0
    warning: str | None = None


@dataclass(frozen=True)
class _LayerOperation:
    sequence: int
    layer: VisualizationLayer | None


class ViserLayerManager:
    """Own Viser layer state, controls, and asynchronous scene reconciliation."""

    def __init__(self, server: Any, scene: ViserManipulationScene) -> None:
        self._server = server
        self._scene = scene
        self._condition = Condition()
        self._states: dict[str, _LayerState] = {}
        self._pending: dict[str, _LayerOperation] = {}
        self._sequence = 0
        self._processing = False
        self._closed = False
        self._leaf_handles: dict[str, Any] = {}
        self._group_folders: dict[str, Any] = {}
        self._group_handles: dict[str, Any] = {}
        self._control_sync_depth = 0
        self._root_folder = server.gui.add_folder("Visualization Layers", expand_by_default=True)
        self._worker = Thread(
            target=self._run,
            name="viser-visualization-layers",
            daemon=True,
        )
        self._worker.start()

    @property
    def layer_ids(self) -> tuple[str, ...]:
        with self._condition:
            return tuple(sorted(self._states))

    def visibility(self, layer_id: str) -> bool | None:
        with self._condition:
            state = self._states.get(layer_id)
            return None if state is None else state.visible

    def warning(self, layer_id: str) -> str | None:
        with self._condition:
            state = self._states.get(layer_id)
            return None if state is None else state.warning

    def set_layer(self, layer: VisualizationLayer) -> None:
        """Queue a complete replacement without waiting for scene rendering."""
        with self._condition:
            if self._closed:
                return
            if layer.id not in self._states:
                self._states[layer.id] = _LayerState(visible=layer.default_visible)
                self._rebuild_controls()
            self._sequence += 1
            self._pending[layer.id] = _LayerOperation(self._sequence, layer)
            self._condition.notify()

    def clear_layer(self, layer_id: str) -> None:
        """Queue a clear for a known layer while retaining its viewer state."""
        with self._condition:
            if self._closed or layer_id not in self._states:
                return
            self._sequence += 1
            self._pending[layer_id] = _LayerOperation(self._sequence, None)
            self._condition.notify()

    def set_visible(self, layer_id: str, visible: bool) -> None:
        with self._condition:
            state = self._states.get(layer_id)
            if self._closed or state is None:
                return
            state.visible = bool(visible)
            handle = self._leaf_handles.get(layer_id)
            if handle is not None:
                self._set_control_value(handle, state.visible)
            self._sync_parent_controls(layer_id)
        self._scene.set_visualization_layer_visible(layer_id, bool(visible))

    def set_group_visible(self, group_id: str, visible: bool) -> None:
        descendants = [
            layer_id for layer_id in self.layer_ids if layer_id.startswith(f"{group_id}/")
        ]
        for layer_id in descendants:
            self.set_visible(layer_id, visible)

    def wait_idle(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._pending or self._processing:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
        self._worker.join(timeout=2.0)
        self._scene.clear_visualization_layers()
        for handle in (
            *self._leaf_handles.values(),
            *self._group_handles.values(),
            *reversed(self._group_folders.values()),
            self._root_folder,
        ):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
        self._leaf_handles.clear()
        self._group_handles.clear()
        self._group_folders.clear()

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._pending and not self._closed:
                    self._condition.wait()
                if self._closed:
                    self._processing = False
                    self._condition.notify_all()
                    return
                layer_id = next(iter(self._pending))
                operation = self._pending.pop(layer_id)
                state = self._states[layer_id]
                state.generation += 1
                generation = state.generation
                visible = state.visible
                self._processing = True
            try:
                if operation.layer is None:
                    self._scene.clear_visualization_layer(layer_id)
                else:
                    self._scene.replace_visualization_layer(
                        operation.layer,
                        generation=generation,
                        visible=visible,
                    )
                warning = None
            except Exception as error:
                warning = str(error)
                logger.warning(
                    "Visualization layer update failed for '%s': %s",
                    layer_id,
                    error,
                    exc_info=True,
                )
            with self._condition:
                state.warning = warning
                self._processing = False
                self._condition.notify_all()

    def _add_layer_controls(self, layer_id: str) -> None:
        segments = layer_id.split("/")
        parent = self._root_folder
        for index, segment in enumerate(segments[:-1], start=1):
            group_id = "/".join(segments[:index])
            folder = self._group_folders.get(group_id)
            if folder is None:
                with parent:
                    folder = self._server.gui.add_folder(
                        _display_name(segment), expand_by_default=True
                    )
                self._group_folders[group_id] = folder
                with folder:
                    group_handle = self._server.gui.add_checkbox("All", initial_value=True)
                group_handle.on_update(
                    lambda event, selected_group=group_id: self._on_group_update(
                        selected_group,
                        bool(event.target.value),
                    )
                )
                self._group_handles[group_id] = group_handle
            parent = folder
        state = self._states[layer_id]
        with parent:
            leaf = self._server.gui.add_checkbox(
                _display_name(segments[-1]), initial_value=state.visible
            )
        leaf.on_update(
            lambda event, selected_layer=layer_id: self._on_leaf_update(
                selected_layer,
                bool(event.target.value),
            )
        )
        self._leaf_handles[layer_id] = leaf
        self._sync_parent_controls(layer_id)

    def _rebuild_controls(self) -> None:
        for handle in (
            *self._leaf_handles.values(),
            *self._group_handles.values(),
            *reversed(self._group_folders.values()),
        ):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
        self._leaf_handles.clear()
        self._group_handles.clear()
        self._group_folders.clear()
        for layer_id in sorted(self._states):
            self._add_layer_controls(layer_id)

    def _sync_parent_controls(self, layer_id: str) -> None:
        segments = layer_id.split("/")
        for index in range(1, len(segments)):
            group_id = "/".join(segments[:index])
            handle = self._group_handles.get(group_id)
            if handle is None:
                continue
            descendants = [
                state.visible
                for candidate, state in self._states.items()
                if candidate.startswith(f"{group_id}/")
            ]
            self._set_control_value(handle, bool(descendants) and all(descendants))

    def _set_control_value(self, handle: Any, value: bool) -> None:
        """Update Viser state without treating its callback as a user action."""
        self._control_sync_depth += 1
        try:
            handle.value = value
        finally:
            self._control_sync_depth -= 1

    def _on_leaf_update(self, layer_id: str, visible: bool) -> None:
        if self._control_sync_depth == 0:
            self.set_visible(layer_id, visible)

    def _on_group_update(self, group_id: str, visible: bool) -> None:
        if self._control_sync_depth == 0:
            self.set_group_visible(group_id, visible)
