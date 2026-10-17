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

"""Robot picker browser e2e (the T12b acceptance demo, in CI).

Two RelayBridgeModules attach to one hand-started relay through relay_url
(two robots sharing a relay), each with its own cockpit manifest: "Alpha"
has a video panel, "Bravo" adds a Stats page. Nothing publishes data: the
panels render in their "waiting for data" state, which is enough to prove
that the picked robot's own manifest was adopted (data flow through the
relay is covered by the other browser e2e files). The page must list both
robots sorted by name whatever the registration order, render the picked
robot's panels, and switch to the other one from the status bar.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_robot_picker_browser.py`.
"""

from collections.abc import Iterator
from contextlib import ExitStack

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.web.cockpit import Stats, Video, cockpit
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_web_dist

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser


def _attach(
    cleanup: ExitStack,
    relay_url: str,
    robot_id: str,
    name: str,
    blueprint: Blueprint,
) -> None:
    """A bridge registered on the relay at relay_url (the --relay-url path)."""
    (atom,) = blueprint.blueprints
    module = atom.module(
        relay_url=relay_url,
        open_browser=False,
        web_build=False,
        robot_id=robot_id,
        robot_name=name,
        **atom.kwargs,
    )
    cleanup.callback(stop_module, module)
    module.start()


@pytest.fixture(scope="module")
def two_robots_url() -> Iterator[str]:
    # A bare RelayProcess serves whatever cockpit dist exists: build (or
    # refresh) it first, the way a spawned local relay would.
    ensure_web_dist(find_web_dir())
    with RelayProcess() as ready, ExitStack() as cleanup:
        # Bravo registers first, so the relay's list is [bravo, alpha] and the
        # sorted order on the page is the picker's doing.
        bravo = cockpit(layout=Video("color_image"), pages=[Stats()])
        _attach(cleanup, ready.open_url, "bravo", "Bravo", bravo)
        _attach(
            cleanup,
            ready.open_url,
            "alpha",
            "Alpha",
            cockpit(layout=Video("color_image")),
        )
        yield ready.open_url


@pytest.fixture
def chromium_page() -> Iterator[Page]:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def test_picker_lists_both_robots_and_switches(two_robots_url: str, chromium_page: Page) -> None:
    page = chromium_page
    page.goto(two_robots_url)
    entries = page.locator('[data-testid^="robot-pick-"]')
    # Count and order: sorted by name, not the registration order.
    expect(entries).to_contain_text(["Alpha", "Bravo"], timeout=120_000)
    expect(page.get_by_test_id("robot")).to_have_text("no robot")

    page.get_by_test_id("robot-pick-bravo").click()
    expect(page.get_by_test_id("robot")).to_contain_text("Bravo", timeout=30_000)
    expect(page.get_by_test_id("robot-picker")).to_have_count(0)
    expect(page.get_by_test_id("tab-page-p1")).to_have_text("Stats", timeout=30_000)
    expect(page.get_by_test_id("panel-p0")).to_be_visible()

    page.get_by_test_id("switch-robot").click()
    expect(page.get_by_test_id("robot-pick-bravo")).to_have_attribute("aria-current", "true")
    page.get_by_test_id("robot-pick-alpha").click()
    expect(page.get_by_test_id("robot")).to_contain_text("Alpha", timeout=30_000)
    expect(page.get_by_test_id("panel-p0")).to_be_visible(timeout=30_000)
    expect(page.get_by_test_id("tab-page-p1")).to_have_count(0)
    expect(page.get_by_test_id("switch-robot")).to_be_visible()
