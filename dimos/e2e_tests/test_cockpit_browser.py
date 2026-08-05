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

"""Cockpit browser e2e (the T3 acceptance demo, in CI).

Starts `dimos --robot-ip fake run unitree-go2-basic --local-relay` (the go2
blueprint on the go2_short replay dataset: real recorded odom + camera, no
MuJoCo and no hardware) and drives the served Cockpit with a real headless
browser: live odom must tick, the session must stay connected (Firefox's
tight QUIC stream credit found the relay's stream-per-frame overflow the
first time), and killing + restarting the dimos process must take the page
through reconnecting and back to live data without a reload.

Runs in both Chromium and Firefox: their WebTransport implementations differ
enough that one browser staying green says little about the other.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers and the LFS dataset);
the CI `web` job runs it against a freshly-built cockpit dist. Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_cockpit_browser.py`.
"""

from collections.abc import Callable, Iterator
import time

import pytest
import requests

from dimos.e2e_tests.dimos_cli_call import DimosCliCall

# Playwright lives in the `browser-tests` dependency group, which only the CI
# web job installs; collection elsewhere must skip, not fail.
pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

COCKPIT_URL = "http://127.0.0.1:7780/"

# The first start may download Deno and, outside CI, build the cockpit dist.
START_TIMEOUT_S = 300

# Long enough to catch periodic disconnects (the Firefox stream-credit
# overflow kicked the viewer every ~8 s).
STABILITY_WINDOW_S = 12


@pytest.fixture
def start_go2_replay() -> Iterator[Callable[[], DimosCliCall]]:
    calls: list[DimosCliCall] = []

    def start() -> DimosCliCall:
        call = DimosCliCall()
        call.simulator = None
        # --viewer none: no rerun viewer spawn (headless CI); the replay
        # connection means no MuJoCo and no hardware.
        call.global_args = ["--robot-ip", "fake", "--viewer", "none"]
        call.extra_env = {"RELAYBRIDGEMODULE__OPEN_BROWSER": "false"}
        call.demo_args = ["run", "unitree-go2-basic", "--local-relay"]
        call.start()
        calls.append(call)
        return call

    yield start
    for call in calls:
        call.stop()


@pytest.fixture(params=["chromium", "firefox"])
def page(request: pytest.FixtureRequest) -> Iterator[Page]:
    with sync_playwright() as p:
        browser = getattr(p, request.param).launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def _wait_for_relay(call: DimosCliCall, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            requests.get(f"{COCKPIT_URL}api/info", timeout=2).raise_for_status()
            return
        except requests.RequestException:
            assert call.process is not None and call.process.poll() is None, (
                f"dimos exited with {call.process and call.process.returncode} before the relay came up"
            )
            if time.monotonic() > deadline:
                raise TimeoutError(f"relay not reachable after {timeout_s} s") from None
            time.sleep(0.5)


def _assert_odom_ticks(page: Page) -> None:
    seq = page.get_by_test_id("ch-odom-seq")
    expect(seq).not_to_have_text("-", timeout=60_000)
    first = seq.inner_text()
    expect(seq).not_to_have_text(first, timeout=30_000)
    expect(page.get_by_test_id("ch-odom-value")).to_contain_text("yaw")


def test_cockpit_live_data_and_reconnect(
    start_go2_replay: Callable[[], DimosCliCall], page: Page
) -> None:
    call = start_go2_replay()
    _wait_for_relay(call, START_TIMEOUT_S)

    page.goto(COCKPIT_URL)
    status = page.get_by_test_id("status")
    expect(status).to_have_attribute("data-phase", "connected", timeout=120_000)
    expect(page.get_by_test_id("robot")).not_to_have_text("no robot", timeout=30_000)
    _assert_odom_ticks(page)

    # Stability: record every phase change and require none for the window.
    page.evaluate("""() => {
      window.__phases = [];
      const el = document.querySelector('[data-testid="status"]');
      new MutationObserver(() => window.__phases.push(el.dataset.phase))
        .observe(el, { attributes: true, attributeFilter: ["data-phase"] });
    }""")
    page.wait_for_timeout(STABILITY_WINDOW_S * 1000)
    flaps = page.evaluate("() => window.__phases")
    assert flaps == [], f"session flapped during the stability window: {flaps}"
    _assert_odom_ticks(page)

    # Kill dimos (relay dies with it): the page must notice on its own.
    call.stop()
    expect(status).to_have_attribute("data-phase", "reconnecting", timeout=60_000)

    # Restart: the page reattaches by itself and data resumes, no reload.
    restarted = start_go2_replay()
    _wait_for_relay(restarted, START_TIMEOUT_S)
    expect(status).to_have_attribute("data-phase", "connected", timeout=120_000)
    _assert_odom_ticks(page)
