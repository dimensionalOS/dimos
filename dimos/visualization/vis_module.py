#!/usr/bin/env python3
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

"""Shared visualization module factory for all robot blueprints."""

from typing import Any

from dimos.core.blueprints import Blueprint, autoconnect
from dimos.core.global_config import ViewerBackend
from dimos.protocol.pubsub.impl.lcmpubsub import LCM


def vis_module(
    viewer_backend: ViewerBackend,
    rerun_config: dict[str, Any] | None = None,
    foxglove_config: dict[str, Any] | None = None,
) -> Blueprint:
    """Create a visualization blueprint based on the selected viewer backend.

    Bundles the appropriate viewer module (Rerun or Foxglove) together with
    the ``RerunWebSocketServer`` so that the dimos-viewer keyboard/click
    events work out of the box.
    """
    from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

    if foxglove_config is None:
        foxglove_config = {}
    if rerun_config is None:
        rerun_config = {}
    rerun_config = {**rerun_config}
    rerun_config.setdefault("pubsubs", [LCM()])

    match viewer_backend:
        case "foxglove":
            from dimos.robot.foxglove_bridge import FoxgloveBridge

            return autoconnect(
                FoxgloveBridge.blueprint(**foxglove_config),
                RerunWebSocketServer.blueprint(),
            )
        case "rerun" | "rerun-web":
            from dimos.visualization.rerun.bridge import _BACKEND_TO_MODE, RerunBridgeModule

            viewer_mode = _BACKEND_TO_MODE.get(viewer_backend, "native")
            return autoconnect(
                RerunBridgeModule.blueprint(viewer_mode=viewer_mode, **rerun_config),
                RerunWebSocketServer.blueprint(),
            )
        case "rerun-connect":
            from dimos.visualization.rerun.bridge import RerunBridgeModule

            return autoconnect(
                RerunBridgeModule.blueprint(viewer_mode="connect", **rerun_config),
                RerunWebSocketServer.blueprint(),
            )
        case _:
            return autoconnect(RerunWebSocketServer.blueprint())
