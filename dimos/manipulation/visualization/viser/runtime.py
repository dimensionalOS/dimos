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

import importlib
from types import ModuleType
from typing import Protocol, cast
import webbrowser

from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

VISER_INSTALL_HINT = "Viser manipulation visualization requires Viser. Install it with: uv sync --extra manipulation-viser"
VISER_URDF_INSTALL_HINT = (
    "Viser URDF support requires yourdfpy. Install it with: uv sync --extra manipulation-viser"
)


class _ViserModule(Protocol):
    def ViserServer(self, *, host: str, port: int) -> object: ...


class _ViserExtrasModule(Protocol):
    ViserUrdf: object


class _Stoppable(Protocol):
    def stop(self) -> None: ...


def import_viser() -> ModuleType:
    """Import Viser with a feature-specific install hint."""
    try:
        return importlib.import_module("viser")
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(VISER_INSTALL_HINT) from e


def import_viser_urdf() -> object:
    """Import ViserUrdf with a feature-specific install hint."""
    try:
        viser_extras = importlib.import_module("viser.extras")
    except (ImportError, ModuleNotFoundError) as e:
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
    try:
        return cast("_ViserExtrasModule", viser_extras).ViserUrdf
    except AttributeError as e:
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e


class ViserRuntime:
    """Owns the Viser server lifecycle."""

    def __init__(self, config: ViserVisualizationConfig) -> None:
        self.config = config
        self.server: object | None = None

    @property
    def url(self) -> str | None:
        if self.server is None:
            return None
        return f"http://{self.config.host}:{self.config.port}"

    def start(self) -> object:
        if self.server is None:
            viser = cast("_ViserModule", import_viser())
            self.server = viser.ViserServer(host=self.config.host, port=self.config.port)
            if self.config.open_browser and self.url:
                webbrowser.open_new_tab(self.url)
        return self.server

    def close(self) -> None:
        server = self.server
        self.server = None
        if server is not None and hasattr(server, "stop"):
            cast("_Stoppable", server).stop()
