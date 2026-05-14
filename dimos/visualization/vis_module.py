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

from typing import Any, get_args

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.visualization.rerun.constants import ViewerBackend
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def vis_module(
    viewer_backend: ViewerBackend,
    rerun_config: dict[str, Any] | None = None,
    foxglove_config: dict[str, Any] | None = None,
) -> Blueprint:
    """Create a visualization blueprint based on the selected viewer backend.

    Bundles the appropriate viewer module (Rerun or Foxglove) together with
    the ``WebsocketVisModule`` and ``RerunWebSocketServer`` so that the web
    dashboard and remote viewer connections work out of the box.

    Example usage::

        from dimos.core.global_config import global_config
        viz = vis_module(
            global_config.viewer,
            rerun_config={
                "visual_override": {
                    "world/camera_info": lambda ci: ci.to_rerun(...),
                },
                "static": {
                    "world/tf/base_link": lambda rr: [rr.Boxes3D(...)],
                },
            },
        )
    """
    if foxglove_config is None:
        foxglove_config = {}
    if rerun_config is None:
        rerun_config = {}

    match viewer_backend:
        case "foxglove":
            from dimos.robot.foxglove_bridge import FoxgloveBridge

            return autoconnect(
                FoxgloveBridge.blueprint(**foxglove_config),
                RerunWebSocketServer.blueprint(),
                WebsocketVisModule.blueprint(),
            )
        case "rerun":
            from dimos.core.global_config import global_config
            from dimos.protocol.pubsub.impl.lcmpubsub import LCM
            from dimos.visualization.rerun.bridge import RerunBridgeModule

            rerun_config = {**rerun_config}  # copy (avoid mutation)
            rerun_config.setdefault("pubsubs", [LCM()])
            rerun_config.setdefault("rerun_open", global_config.rerun_open)
            rerun_config.setdefault("rerun_web", global_config.rerun_web)
            return autoconnect(
                RerunBridgeModule.blueprint(
                    **rerun_config,
                ),
                RerunWebSocketServer.blueprint(),
                WebsocketVisModule.blueprint(),
            )
        case "none":
            return autoconnect(WebsocketVisModule.blueprint())
        case _:
            valid = ", ".join(get_args(ViewerBackend))
            raise ValueError(f"Unknown viewer_backend {viewer_backend!r}. Expected one of: {valid}")
