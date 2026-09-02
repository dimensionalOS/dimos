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

"""SDK browser e2e (the W2 acceptance demos, in CI).

Starts the go2 replay blueprint with `--local-relay --serve-dir
web/examples/minimal` and drives the zero-build page with a real browser: the
page imports the relay-served /sdk.js bundle, connects same-origin, rides the
lone-robot auto-watch, and must render decoded odom (this also proves the
bundle is self-contained - a bare react import would fail the module graph).
The cross-origin case serves a page from a second loopback origin importing
the absolute http://127.0.0.1:7780/sdk.js and passing that base to
connect({url}) - the Vite-dev-server scenario - and the file: case loads the
same page straight from disk; both work only because the local relay answers
/api/info and /sdk.js with wildcard CORS.

One dimos stack serves the whole module (nothing here kills it, unlike the
cockpit e2e's restart case, so module scope is safe and saves ~2 min/start).

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers and the LFS dataset).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_sdk_browser.py`.
"""

from collections.abc import Iterator
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall, wait_for_http
from dimos.web.relay_bridge.locate import find_web_dir

# Playwright lives in the `browser-tests` dependency group, which only the CI
# web job installs; collection elsewhere must skip, not fail.
pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

RELAY_URL = "http://127.0.0.1:7780/"

# The first start may download Deno and, outside CI, build the web dists.
START_TIMEOUT_S = 300

# A page whose SDK and relay live on another origin (what a Vite dev server
# or a double-clicked local file would show): both the module import and
# connect() use absolute URLs.
ABSOLUTE_URL_PAGE = f"""<!DOCTYPE html>
<html>
  <body>
    <pre id="odom">odom: (no data)</pre>
    <script type="module">
      import {{ connect }} from "{RELAY_URL}sdk.js";
      const session = connect({{ url: "{RELAY_URL}" }});
      session.subscribe("odom", (snapshot) => {{
        document.querySelector("#odom").textContent =
          `odom: ${{JSON.stringify(snapshot.slot?.value ?? null)}}`;
      }});
    </script>
  </body>
</html>
"""


@pytest.fixture(scope="module")
def go2_replay_serve_dir() -> Iterator[DimosCliCall]:
    call = DimosCliCall()
    call.simulator = None
    # --viewer none: no rerun viewer spawn (headless CI); the replay
    # connection means no MuJoCo and no hardware.
    call.global_args = ["--robot-ip", "fake", "--viewer", "none"]
    call.extra_env = {"RELAYBRIDGEMODULE__OPEN_BROWSER": "false"}
    call.demo_args = [
        "run",
        "unitree-go2-basic",
        "--local-relay",
        "--serve-dir",
        str(find_web_dir() / "examples" / "minimal"),
    ]
    call.start()
    try:
        wait_for_http(call, f"{RELAY_URL}api/info", START_TIMEOUT_S)
        yield call
    finally:
        call.stop()


@pytest.fixture(params=["chromium", "firefox"])
def page(request: pytest.FixtureRequest, playwright_browsers: None) -> Iterator[Page]:
    with sync_playwright() as p:
        browser = getattr(p, request.param).launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


@pytest.fixture
def chromium_page(playwright_browsers: None) -> Iterator[Page]:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


@pytest.fixture
def second_origin(tmp_path: Path) -> Iterator[str]:
    """A second loopback HTTP origin serving tmp_path (the Vite stand-in)."""
    (tmp_path / "index.html").write_text(ABSOLUTE_URL_PAGE)
    handler = partial(SimpleHTTPRequestHandler, directory=str(tmp_path))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_address[1]}/"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_minimal_page_shows_decoded_odom(go2_replay_serve_dir: DimosCliCall, page: Page) -> None:
    page.goto(RELAY_URL)
    # A decoded pose proves the whole chain: --serve-dir served the page,
    # /sdk.js resolved, the session connected and auto-watched, and the
    # json decoder ran on a live frame.
    expect(page.locator("#odom")).to_contain_text("yaw", timeout=120_000)
    expect(page.locator("#status")).to_contain_text("connected")


def test_cross_origin_bootstrap_from_second_origin(
    go2_replay_serve_dir: DimosCliCall, second_origin: str, chromium_page: Page
) -> None:
    chromium_page.goto(second_origin)
    expect(chromium_page.locator("#odom")).to_contain_text("yaw", timeout=120_000)


def test_file_page_connects(
    go2_replay_serve_dir: DimosCliCall, tmp_path: Path, chromium_page: Page
) -> None:
    # The zero-config extreme: a page double-clicked from disk. Both supported
    # browsers permit the cross-origin module import, /api/info fetch, and
    # WebTransport from a file: context (probed in Chromium and Firefox);
    # exercised in one engine here to bound CI time.
    (tmp_path / "index.html").write_text(ABSOLUTE_URL_PAGE)
    chromium_page.goto(f"file://{tmp_path / 'index.html'}")
    expect(chromium_page.locator("#odom")).to_contain_text("yaw", timeout=120_000)
