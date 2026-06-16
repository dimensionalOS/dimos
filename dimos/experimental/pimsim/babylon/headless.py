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

"""Headless Chromium runner for pimsim's BabylonSceneViewerModule.

Pimsim's interactive runtime gates physics, lidar, and entity broadcasts
on having at least one connected browser tab (`module.py:_clients`).
For CI we want that tab without a human — this wrapper does exactly
that: it launches a small-viewport Chromium against the running pimsim
viewer URL and waits for ``window.__pimsimReady`` before returning.

Requires the optional ``pimsim`` extra (``pip install
'dimos[pimsim]'``) plus a one-time ``playwright install chromium``.

The Playwright argument list intentionally uses ``--headless=new`` with
``headless=False`` — old-headless Chromium has WebGL/Havok issues, but
Playwright's ``headless=True`` flag selects old-headless. Passing the
flag explicitly via ``args`` is the documented workaround.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from playwright.sync_api import Browser, BrowserContext, Page, Playwright

logger = setup_logger()

DEFAULT_URL = "http://localhost:8091/"
DEFAULT_VIEWPORT = (320, 240)
READY_TIMEOUT_MS = 60_000

RenderMode = Literal["cpu", "gpu"]


class HeadlessBrowser:
    """Drives a hidden Chromium tab against a running pimsim viewer."""

    def __init__(
        self,
        url: str = DEFAULT_URL,
        *,
        render: RenderMode = "cpu",
        viewport: tuple[int, int] = DEFAULT_VIEWPORT,
        ready_timeout_ms: int = READY_TIMEOUT_MS,
    ) -> None:
        self._url = url
        self._render = render
        self._viewport = viewport
        self._ready_timeout_ms = ready_timeout_ms
        self._pw: Playwright | None = None
        self._browser: Browser | None = None
        self._context: BrowserContext | None = None
        self._page: Page | None = None

    def start(self) -> None:
        """Launch Chromium and wait for ``window.__pimsimReady``."""
        try:
            from playwright.sync_api import sync_playwright
        except ModuleNotFoundError as exc:  # pragma: no cover — import guard
            raise RuntimeError(
                "playwright is required for HeadlessBrowser; install with "
                "`pip install 'dimos[pimsim]'` and run `playwright install chromium`"
            ) from exc

        args = [
            "--headless=new",
            "--no-sandbox",
            "--disable-background-timer-throttling",
            "--disable-renderer-backgrounding",
            "--disable-features=AudioServiceOutOfProcess",
        ]
        if self._render == "cpu":
            args += ["--use-gl=angle", "--use-angle=swiftshader"]

        self._pw = sync_playwright().start()
        # See class docstring on why headless=False + --headless=new.
        self._browser = self._pw.chromium.launch(headless=False, args=args)
        self._context = self._browser.new_context(
            viewport={"width": self._viewport[0], "height": self._viewport[1]},
        )
        self._page = self._context.new_page()
        self._page.goto(self._url, wait_until="load")
        self._page.wait_for_function(
            "window.__pimsimReady === true",
            timeout=self._ready_timeout_ms,
        )
        logger.info("pimsim headless browser ready at %s", self._url)

    def stop(self) -> None:
        if self._page is not None:
            try:
                self._page.close()
            except Exception:
                pass
            self._page = None
        if self._context is not None:
            try:
                self._context.close()
            except Exception:
                pass
            self._context = None
        if self._browser is not None:
            try:
                self._browser.close()
            except Exception:
                pass
            self._browser = None
        if self._pw is not None:
            try:
                self._pw.stop()
            except Exception:
                pass
            self._pw = None

    def __enter__(self) -> HeadlessBrowser:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        del exc_type, exc, tb
        self.stop()


__all__ = ["DEFAULT_URL", "HeadlessBrowser", "RenderMode"]
