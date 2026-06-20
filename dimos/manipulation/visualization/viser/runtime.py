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

from __future__ import annotations

import webbrowser

from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

VISER_INSTALL_HINT = "Viser manipulation visualization requires Viser with URDF support. Install it with: uv sync --extra manipulation"
VISER_URDF_INSTALL_HINT = VISER_INSTALL_HINT

try:
    from viser import ViserServer
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

__all__ = ["VISER_INSTALL_HINT", "VISER_URDF_INSTALL_HINT", "ViserRuntime", "ViserServer"]


class ViserRuntime:
    """Owns the Viser server lifecycle."""

    def __init__(self, config: ViserVisualizationConfig) -> None:
        self.config = config
        self.server: ViserServer | None = None

    @property
    def url(self) -> str | None:
        if self.server is None:
            return None
        return f"http://{self.config.host}:{self.config.port}"

    def start(self) -> ViserServer:
        if self.server is None:
            self.server = ViserServer(host=self.config.host, port=self.config.port)
            if self.config.open_browser and self.url:
                webbrowser.open_new_tab(self.url)
        return self.server

    def close(self) -> None:
        server = self.server
        self.server = None
        if server is not None:
            server.stop()
