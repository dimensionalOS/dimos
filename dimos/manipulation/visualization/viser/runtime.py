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

from typing import TYPE_CHECKING, Protocol, cast
import webbrowser

from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

if TYPE_CHECKING:
    import viser
    from viser.extras import ViserUrdf as ViserUrdfType

VISER_INSTALL_HINT = (
    "Viser manipulation visualization requires Viser. Install it with: uv sync --extra manipulation"
)
VISER_URDF_INSTALL_HINT = (
    "Viser URDF support requires yourdfpy. Install it with: uv sync --extra manipulation"
)


class _Stoppable(Protocol):
    def stop(self) -> None: ...


def import_viser() -> type[viser.ViserServer]:
    """Import Viser with a feature-specific install hint."""
    try:
        from viser import ViserServer
    except ModuleNotFoundError as e:
        if e.name != "viser":
            raise
        raise ModuleNotFoundError(VISER_INSTALL_HINT) from e
    return ViserServer


def import_viser_urdf() -> type[ViserUrdfType]:
    """Import ViserUrdf with a feature-specific install hint."""
    try:
        from viser.extras import ViserUrdf
    except ModuleNotFoundError as e:
        if e.name not in {"viser", "viser.extras", "yourdfpy"}:
            raise
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
    except ImportError as e:
        if "ViserUrdf" not in str(e):
            raise
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
    return ViserUrdf


class ViserRuntime:
    """Owns the Viser server lifecycle."""

    def __init__(self, config: ViserVisualizationConfig) -> None:
        self.config = config
        self.server: viser.ViserServer | None = None

    @property
    def url(self) -> str | None:
        if self.server is None:
            return None
        return f"http://{self.config.host}:{self.config.port}"

    def start(self) -> viser.ViserServer:
        if self.server is None:
            ViserServer = import_viser()
            self.server = ViserServer(host=self.config.host, port=self.config.port)
            if self.config.open_browser and self.url:
                webbrowser.open_new_tab(self.url)
        return self.server

    def close(self) -> None:
        server = self.server
        self.server = None
        if server is not None and hasattr(server, "stop"):
            cast("_Stoppable", server).stop()
