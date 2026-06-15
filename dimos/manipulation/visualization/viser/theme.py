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
from collections.abc import Callable
import importlib
from pathlib import Path
from typing import Protocol, cast

from dimos.utils.logging_config import setup_logger

DIMOS_THEME_TITLE = "DimOS Manipulation"
DIMOS_THEME_URL = "https://github.com/dimensionalOS/dimos"
DIMOS_BRAND_COLOR = (22, 130, 163)
DIMOS_LOGO_PATH = Path(__file__).with_name("assets") / "dimensional-logo.svg"

logger = setup_logger()


class _ThemeGui(Protocol):
    configure_theme: Callable[..., object]


class _GuiServer(Protocol):
    gui: _ThemeGui


class _ViserThemeModule(Protocol):
    TitlebarButton: Callable[..., object]
    TitlebarConfig: Callable[..., object]
    TitlebarImage: Callable[..., object]


def apply_dimos_theme(server: object) -> bool:
    """Apply the default DimOS Viser theme without blocking visualization startup."""
    try:
        gui = cast("_GuiServer", server).gui
        configure_theme = gui.configure_theme
    except AttributeError:
        return False
    if not callable(configure_theme):
        return False

    kwargs: dict[str, object] = {
        "control_layout": "collapsible",
        "control_width": "medium",
        "dark_mode": True,
        "show_logo": False,
        "show_share_button": False,
        "brand_color": DIMOS_BRAND_COLOR,
    }
    titlebar_content = _dimos_titlebar_content()
    if titlebar_content is not None:
        kwargs["titlebar_content"] = titlebar_content

    if _configure_theme(configure_theme, kwargs):
        return True
    if "titlebar_content" in kwargs:
        kwargs.pop("titlebar_content")
        return _configure_theme(configure_theme, kwargs)
    return False


def _dimos_titlebar_content() -> object | None:
    try:
        viser_theme = cast("_ViserThemeModule", importlib.import_module("viser.theme"))
        logo_data_url = _dimos_logo_data_url()
        image = (
            viser_theme.TitlebarImage(
                image_url_light=logo_data_url,
                image_url_dark=logo_data_url,
                image_alt="Dimensional",
                href=DIMOS_THEME_URL,
            )
            if logo_data_url is not None
            else None
        )
        return viser_theme.TitlebarConfig(
            buttons=(
                viser_theme.TitlebarButton(
                    text=DIMOS_THEME_TITLE,
                    icon=None,
                    href=DIMOS_THEME_URL,
                ),
            ),
            image=image,
        )
    except Exception:
        logger.debug("Skipping Viser titlebar content; theme API unavailable", exc_info=True)
        return None


def _dimos_logo_data_url() -> str | None:
    try:
        logo = DIMOS_LOGO_PATH.read_bytes()
    except OSError:
        logger.debug("Skipping DimOS Viser logo; asset unavailable", exc_info=True)
        return None
    encoded = base64.b64encode(logo).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _configure_theme(configure_theme: Callable[..., object], kwargs: dict[str, object]) -> bool:
    try:
        configure_theme(**kwargs)
    except Exception:
        logger.debug("Skipping DimOS Viser theme; theme API unavailable", exc_info=True)
        return False
    return True
