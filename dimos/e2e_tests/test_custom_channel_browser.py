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

"""Custom-channel browser e2e (the W6 acceptance demo, in CI).

Starts a cockpit(channels=[...]) bridge - a generated RelayBridgeModule
subclass with a nav_path In[Path] port whose path.points.v1 encoder was
resolved from the codec registry at blueprint definition time - serving
web/examples/custom-path from its own local relay. The page registers the
matching JavaScript decoder, subscribes manually (no cockpit panel anywhere in
the chain), and must render the decoded points: Python @web_encoder -> relay
-> browser decoder, end to end.

The bridge runs in-process (like the relay_bridge e2e) on an ephemeral port,
so it cannot clash with test_sdk_browser's stack on :7780 in the same CI job.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_custom_channel_browser.py`.
"""

from collections.abc import Iterator
import struct
import threading
import time

import pytest

from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.web.cockpit import Channel, cockpit
from dimos.web.codecs import EncodedPayload, web_encoder
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.locate import find_web_dir

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

TOPIC = "/custom_path_e2e/nav_path"

PATH_MSG = Path(
    ts=42.5,
    frame_id="world",
    poses=[
        PoseStamped(ts=42.5, position=[1.5, -2.5, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]),
        PoseStamped(ts=42.5, position=[2.0, 0.25, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]),
        PoseStamped(ts=42.5, position=[3.5, 4.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]),
    ],
)


# The codec pair documented in web/README.md: little-endian float32 (x, y)
# pairs, meta.n = point count; web/examples/custom-path decodes it.
@web_encoder("path.points.v1")
def encode_path_points(msg: Path) -> EncodedPayload:
    payload = b"".join(struct.pack("<ff", p.position.x, p.position.y) for p in msg.poses)
    return EncodedPayload(payload, {"n": len(msg.poses)})


@pytest.fixture(scope="module")
def custom_path_page_url() -> Iterator[str]:
    blueprint = cockpit(
        channels=[Channel("nav_path", Path, encoding="path.points.v1", max_hz=20.0)]
    )
    (atom,) = blueprint.blueprints
    module = atom.module(
        local_port=0,
        open_browser=False,
        robot_id="custom-path-e2e",
        serve_dir=str(find_web_dir() / "examples" / "custom-path"),
        **atom.kwargs,
    )
    bridge_transport = pLCMTransport(TOPIC)
    bridge_transport.start()
    module.nav_path.transport = bridge_transport
    publisher = pLCMTransport(TOPIC)
    publisher.start()
    stop = threading.Event()

    def republish() -> None:
        # Frames flow only once the bridge lazily subscribes, so a single
        # publish is never enough - keep them coming like a robot would.
        while not stop.is_set():
            publisher.publish(PATH_MSG)
            time.sleep(0.1)

    thread = threading.Thread(target=republish, daemon=True)
    try:
        module.start()  # builds web dists if stale, spawns the relay, registers
        thread.start()
        relay = module._relay
        assert relay is not None and relay.info is not None
        yield relay.info.open_url
    finally:
        stop.set()
        thread.join(timeout=2)
        stop_module(module)
        publisher.stop()
        bridge_transport.stop()


@pytest.fixture
def chromium_page() -> Iterator[Page]:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def test_custom_path_page_shows_decoded_points(
    custom_path_page_url: str, chromium_page: Page
) -> None:
    chromium_page.goto(custom_path_page_url)
    # Decoded coordinates prove the whole chain: the generated bridge class
    # started, the registry-resolved encoder ran on demand, the relay
    # forwarded the opaque payload, and the page's registered decoder
    # reconstructed the points - with no cockpit panel involved.
    expect(chromium_page.locator("#count")).to_contain_text("points: 3", timeout=120_000)
    expect(chromium_page.locator("#points")).to_contain_text("1.5")
    expect(chromium_page.locator("#points")).to_contain_text("-2.5")
    expect(chromium_page.locator("#status")).to_contain_text("connected")
