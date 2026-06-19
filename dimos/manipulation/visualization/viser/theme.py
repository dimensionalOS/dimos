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

import base64
from pathlib import Path

from dimos.manipulation.visualization.viser.runtime import VISER_INSTALL_HINT
from dimos.utils.logging_config import setup_logger

try:
    from viser import ViserServer
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

try:
    from viser.theme import TitlebarConfig
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

DIMOS_THEME_TITLE = "DimOS Manipulation"
DIMOS_THEME_URL = "https://github.com/dimensionalOS/dimos"
DIMOS_BRAND_COLOR = (22, 130, 163)
DIMOS_LOGO_PATH = Path(__file__).with_name("assets") / "dimensional-logo.svg"

logger = setup_logger()


def apply_dimos_theme(server: ViserServer) -> bool:
    """Apply the default DimOS Viser theme without blocking visualization startup."""
    titlebar_content = _dimos_titlebar_content()
    if _configure_theme(server, titlebar_content):
        return True
    return titlebar_content is not None and _configure_theme(server, None)


def _dimos_titlebar_content() -> TitlebarConfig | None:
    try:
        from viser.theme import TitlebarButton, TitlebarConfig, TitlebarImage

        logo_data_url = _dimos_logo_data_url()
        image = TitlebarImage(
            image_url_light=logo_data_url,
            image_url_dark=logo_data_url,
            image_alt="Dimensional",
            href=DIMOS_THEME_URL,
        )
        return TitlebarConfig(
            buttons=(
                TitlebarButton(
                    text=DIMOS_THEME_TITLE,
                    icon=None,
                    href=DIMOS_THEME_URL,
                ),
            ),
            image=image,
        )
    except (ImportError, AttributeError, OSError, TypeError):
        logger.warning("Skipping Viser titlebar content; theme API unavailable", exc_info=True)
        return None


def _dimos_logo_data_url() -> str:
    logo = DIMOS_LOGO_PATH.read_bytes()
    encoded = base64.b64encode(logo).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _configure_theme(server: ViserServer, titlebar_content: TitlebarConfig | None) -> bool:
    try:
        server.gui.configure_theme(
            titlebar_content=titlebar_content,
            control_layout="collapsible",
            control_width="medium",
            dark_mode=True,
            show_logo=False,
            show_share_button=False,
            brand_color=DIMOS_BRAND_COLOR,
        )
    except (TypeError, AttributeError):
        logger.warning("Skipping DimOS Viser theme; theme API unavailable", exc_info=True)
        return False
    return True
