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

"""Shared-publish browser e2e (the W7 acceptance demo, in CI).

Starts a cockpit(channels=[...]) bridge with
Channel("human_input", str, dir="tx", encoding="text.json.v1",
publish="shared") - a generated RelayBridgeModule subclass with a typed
human_input Out port and the registry-resolved text.json.v1 decoder - serving
web/examples/chat-input from its own local relay. The page calls
session.publish() and renders the settled outcome: an ack proves the bridge
called Out.publish() (so the DimOS consumer already holds the exact string by
the time the browser promise resolves), and a bridge-rejected value renders
its correlated error code without touching anything else.

The bridge runs in-process (like the relay_bridge e2e) on an ephemeral port,
so it cannot clash with test_sdk_browser's stack on :7780 in the same CI job.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_publish_browser.py`.
"""

from collections.abc import Iterator

import pytest

from dimos.web.cockpit import Channel, cockpit
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.locate import find_web_dir

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

TEXT = "salut din browser β"


@pytest.fixture(scope="module")
def chat_bridge() -> Iterator[tuple[str, list[str]]]:
    blueprint = cockpit(
        channels=[Channel("human_input", str, dir="tx", encoding="text.json.v1", publish="shared")]
    )
    (atom,) = blueprint.blueprints
    module = atom.module(
        local_port=0,
        open_browser=False,
        robot_id="chat-input-e2e",
        serve_dir=str(find_web_dir() / "examples" / "chat-input"),
        **atom.kwargs,
    )
    received: list[str] = []
    module.human_input.subscribe(received.append)
    try:
        module.start()  # builds web dists if stale, spawns the relay, registers
        relay = module._relay
        assert relay is not None and relay.info is not None
        yield relay.info.open_url, received
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


def test_publish_acks_and_rejections_render(
    chat_bridge: tuple[str, list[str]], chromium_page: Page
) -> None:
    url, received = chat_bridge
    chromium_page.goto(url)
    expect(chromium_page.locator("#status")).to_contain_text("connected", timeout=120_000)

    chromium_page.fill("#text", TEXT)
    chromium_page.click("#send")
    # The ack proves the whole chain: relay validation, the carrier tx frame,
    # the registry-resolved decoder, Out.publish(), and the @control ack.
    expect(chromium_page.locator("#log")).to_contain_text("acked human_input", timeout=15_000)
    # The bridge acks only after Out.publish() returned, so the DimOS
    # consumer already held the exact string when the promise resolved.
    assert received == [TEXT]

    # A value the decoder refuses settles as a definite rejection with the
    # bridge's correlated code, and the accepted publish above is unaffected.
    chromium_page.click("#send-bad")
    expect(chromium_page.locator("#log")).to_contain_text("rejected decode_failed", timeout=15_000)
    assert received == [TEXT]
