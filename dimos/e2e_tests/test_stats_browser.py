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

"""Stats page browser e2e (the T9 acceptance demo, in CI).

Starts a cockpit(pages=[Stats()]) bridge - a generated RelayBridgeModule
subclass with a resource_stats In[dict] port and the registry-resolved
stats.json.v1 encoder - serving the real cockpit dist from its own local
relay, and plays the resource monitor: pickled dicts on the port's topic at
the monitor's 1 Hz (no coordinator runs in-process, so no StatsMonitor). The
Stats tab must show a row per process from those dicts, dtop's way: pickled
dict -> bridge encoder -> relay -> page.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_stats_browser.py`.
"""

from collections.abc import Iterator
from dataclasses import asdict, replace
import threading

import pytest

from dimos.core.resource_monitor.stats import ChildProcessStats, ProcessStats, WorkerStats
from dimos.core.transport import pLCMTransport
from dimos.web.cockpit import Stats, Video, cockpit
from dimos.web.relay_bridge.e2e_support import stop_module

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

TOPIC = "/stats_e2e/resource_stats"

COORDINATOR = ProcessStats(
    pid=4242, alive=True, cpu_percent=12.5, pss=47_400_000, num_threads=4, num_fds=32
)
WORKERS = [
    WorkerStats(
        pid=4243,
        alive=True,
        cpu_percent=34.5,
        pss=125_829_120,
        num_threads=8,
        num_children=1,
        num_fds=64,
        worker_id=0,
        modules=["Navigation", "Lidar"],
        children=[ChildProcessStats(pid=4300, name="ffmpeg", cpu_percent=5.5)],
    ),
    WorkerStats(pid=0, alive=False, worker_id=1, modules=["Vision"], dedicated=True),
]


@pytest.fixture(scope="module")
def stats_page_url() -> Iterator[str]:
    (atom,) = cockpit(layout=Video("color_image"), pages=[Stats()]).blueprints
    module = atom.module(local_port=0, open_browser=False, robot_id="stats-e2e", **atom.kwargs)
    bridge_transport = pLCMTransport(TOPIC)
    bridge_transport.start()
    module.resource_stats.transport = bridge_transport
    publisher = pLCMTransport(TOPIC)
    publisher.start()
    stop = threading.Event()

    def monitor() -> None:
        # The monitor's cadence; frames flow only once the bridge lazily
        # subscribes, so keep them coming. A wobbling CPU moves the sparkline.
        tick = 0
        while not stop.is_set():
            coordinator = replace(COORDINATOR, cpu_percent=10.0 + tick % 5)
            workers = [asdict(w) for w in WORKERS]
            publisher.publish({"coordinator": asdict(coordinator), "workers": workers})
            tick += 1
            stop.wait(1.0)

    thread = threading.Thread(target=monitor, daemon=True)
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


def test_stats_tab_lists_every_process(stats_page_url: str, chromium_page: Page) -> None:
    chromium_page.goto(stats_page_url)
    # p0 is the video panel on the overview; p1 the page.
    tab = chromium_page.get_by_test_id("tab-p1")
    expect(tab).to_have_text("Stats", timeout=120_000)
    tab.click()
    worker = chromium_page.get_by_test_id("stats-resource_stats-row-worker-0")
    expect(worker).to_contain_text("Navigation, Lidar", timeout=60_000)
    expect(worker).to_contain_text("120.0 MB")
    coordinator = chromium_page.get_by_test_id("stats-resource_stats-row-coordinator")
    expect(coordinator).to_contain_text("[4242]")
    child = chromium_page.get_by_test_id("stats-resource_stats-row-child-4300")
    expect(child).to_contain_text("ffmpeg")
    dead = chromium_page.get_by_test_id("stats-resource_stats-row-worker-1")
    expect(dead).to_have_attribute("data-dead", "true")
    expect(dead).to_contain_text("dead")
    badge = chromium_page.get_by_test_id("stats-resource_stats-badge")
    expect(badge).to_contain_text("Hz", timeout=30_000)
