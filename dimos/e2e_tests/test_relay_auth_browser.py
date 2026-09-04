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

"""Relay auth browser e2e (the T12d cockpit login, in CI).

A hand-started relay with --auth-file and one RelayBridgeModule attached
through relay_url with the robot's key: the hosted-relay shape, on loopback.
Nothing publishes data: the panel renders in its "waiting for data" state,
enough to prove that the token-bearing session adopted the manifest. The
page must show the token form (no token stored), connect and render the
panel once the token is typed, and return to the form on "log out".

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_relay_auth_browser.py`.
"""

from collections.abc import Iterator
import json

import pytest

from dimos.web.cockpit import Video, cockpit
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_web_dist

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

ROBOT_KEY = "robot-key-e2e-0123456789abcdef"
VIEWER_TOKEN = "viewer-token-e2e-0123456789abcdef"


@pytest.fixture(scope="module")
def auth_relay_url(tmp_path_factory: pytest.TempPathFactory) -> Iterator[str]:
    ensure_web_dist(find_web_dir())
    auth_file = tmp_path_factory.mktemp("auth") / "auth.json"
    auth_file.write_text(
        json.dumps({"robots": {"go2-lab": ROBOT_KEY}, "viewers": {"tester": VIEWER_TOKEN}})
    )
    with RelayProcess(auth_file=auth_file) as ready:
        (atom,) = cockpit(layout=Video("color_image")).blueprints
        module = atom.module(
            relay_url=ready.open_url,
            relay_key=ROBOT_KEY,
            open_browser=False,
            web_build=False,
            robot_id="go2-lab",
            **atom.kwargs,
        )
        module.start()  # returns only after hello/welcome: the key was accepted
        try:
            yield ready.open_url
        finally:
            stop_module(module)


@pytest.fixture
def chromium_page() -> Iterator[Page]:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def test_token_form_connects_and_logs_out(auth_relay_url: str, chromium_page: Page) -> None:
    page = chromium_page
    page.goto(auth_relay_url)
    # No token stored: the relay rejects the session and the form shows why.
    expect(page.get_by_test_id("token-message")).to_have_text(
        "missing viewer token", timeout=120_000
    )
    expect(page.get_by_test_id("log-out")).to_have_count(0)

    page.get_by_test_id("token-input").fill(VIEWER_TOKEN)
    page.get_by_test_id("token-connect").click()
    expect(page.get_by_test_id("status")).to_have_attribute(
        "data-phase", "connected", timeout=60_000
    )
    expect(page.get_by_test_id("panel-p0")).to_be_visible(timeout=30_000)

    page.get_by_test_id("log-out").click()
    expect(page.get_by_test_id("token-message")).to_have_text(
        "missing viewer token", timeout=60_000
    )
