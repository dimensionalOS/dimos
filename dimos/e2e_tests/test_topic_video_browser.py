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

"""Direct topic video in a real browser.

A local relay without Cloudflare, the standalone topic provider with video
enabled as a subprocess, a synthetic 320x240 camera on a loopback Zenoh
session, and a page that plays the camera through `session.topics.video`.
Chromium and Firefox must each see `requestVideoFrameCallback` fire at least
25 times in two seconds with `mode()` reporting `rtc`.

Playwright's Firefox ships without the OpenH264 plugin; the prefs below let it
decode H.264 through its platform decoder, and the test skips when that is
not available on the host.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group and Playwright browsers). Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_topic_video_browser.py`.
"""

from collections.abc import Iterator
from contextlib import ExitStack
from dataclasses import dataclass
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

import pytest
import zenoh

from dimos.protocol.service.zenoh_wire import make_zenoh_config
from dimos.web.relay_bridge.relay_process import RelayProcess, RelayReadyInfo
from dimos.web.relay_bridge.topic_test_support import camera_frame

pytest.importorskip("playwright")

from playwright.sync_api import Page, sync_playwright

pytestmark = pytest.mark.web_browser

CAMERA_KEY = "demo/camera"
START_TIMEOUT_S = 60
FIREFOX_PREFS = {
    "media.navigator.mediadatadecoder_h264_enabled": True,
    "media.webrtc.hw.h264.enabled": True,
    "media.gmp-gmpopenh264.enabled": True,
    "media.gmp-gmpopenh264.autoupdate": True,
}

PAGE = """<!DOCTYPE html>
<html>
  <body>
    <video id="v" autoplay muted playsinline></video>
    <pre id="status">connecting</pre>
    <script type="module">
      import { connect } from "/sdk.js";
      const video = document.querySelector("#v");
      const session = connect({ url: location.origin });
      window.frames_seen = 0;
      window.tracks_seen = 0;
      window.statuses = [];
      const tick = () => {
        window.frames_seen++;
        video.requestVideoFrameCallback(tick);
      };
      const handle = session.topics.video("%(key)s", { maxHz: Number(new URLSearchParams(location.search).get("hz") || 30) }, (track) => {
        window.tracks_seen++;
        video.srcObject = track === null ? null : new MediaStream([track]);
        if (track !== null) video.requestVideoFrameCallback(tick);
      });
      handle.onStatus((status) => {
        window.statuses.push(status.phase + (status.error ? ":" + status.error.code : ""));
        document.querySelector("#status").textContent = status.phase;
      });
      window.mode = () => handle.mode();
      window.change = (options) => handle.update(options);
      window.track = () => handle.track();
      window.stats = async () => {
        const receiver = handle.receiver();
        if (receiver === null) return null;
        const report = {};
        for (const stat of (await receiver.getStats()).values()) {
          if (stat.type === "inbound-rtp") report.inbound = stat;
        }
        return report;
      };
    </script>
  </body>
</html>
"""


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@dataclass
class Stack:
    relay: RelayReadyInfo
    publisher: zenoh.Session
    frames_published: list[int]


@pytest.fixture(scope="module")
def stack(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Stack]:
    root = tmp_path_factory.mktemp("topic-video")
    (root / "index.html").write_text(PAGE % {"key": CAMERA_KEY})
    # Each resource registers its cleanup as it comes up, so a failure in the
    # middle of the setup still tears down what was started before it.
    with ExitStack() as cleanup:
        yield from _stack(root, cleanup)


def _stack(root: Path, cleanup: ExitStack) -> Iterator[Stack]:
    relay = RelayProcess(serve_dir=root)
    info = relay.start()
    cleanup.callback(relay.stop)
    port = free_port()
    publisher = zenoh.open(
        make_zenoh_config(
            {
                "mode": "peer",
                "listen": [f"tcp/127.0.0.1:{port}"],
                "multicast": False,
                "gossip": False,
            }
        )
    )
    cleanup.callback(publisher.close)
    config = root / "provider.json"
    config.write_text(
        json.dumps(
            {
                "source": "workspace",
                "relay_url": f"http://127.0.0.1:{info.http_port}",
                "video": {"enabled": True},
                "zenoh": {
                    "mode": "peer",
                    "connect": [f"tcp/127.0.0.1:{port}"],
                    "listen": [],
                    "multicast": False,
                    "gossip": False,
                },
            }
        )
    )
    provider = subprocess.Popen(
        [sys.executable, "-m", "dimos.web.relay_bridge.topic_service", "--config", str(config)],
        stdin=subprocess.PIPE,
    )

    def stop_provider() -> None:
        if provider.stdin is not None:
            provider.stdin.close()
        provider.terminate()
        try:
            provider.wait(10)
        except subprocess.TimeoutExpired:
            provider.kill()
            provider.wait()

    cleanup.callback(stop_provider)
    stop = threading.Event()
    published: list[int] = [0]

    def publish() -> None:
        seq = 0
        while not stop.is_set():
            publisher.put(CAMERA_KEY, camera_frame(seq))
            published[0] = seq = seq + 1
            stop.wait(1 / 30)

    def stop_publishing() -> None:
        stop.set()
        thread.join(2)

    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    cleanup.callback(stop_publishing)
    yield Stack(info, publisher, published)


@pytest.fixture(params=["chromium", "firefox"])
def page(request: pytest.FixtureRequest, playwright_browsers: None) -> Iterator[Page]:
    with sync_playwright() as p:
        if request.param == "firefox":
            browser = p.firefox.launch(firefox_user_prefs=FIREFOX_PREFS)
        else:
            browser = p.chromium.launch()
        try:
            page = browser.new_page()
            page.goto("about:blank")
            deadline = time.monotonic() + 20
            while not page.evaluate(
                "() => RTCRtpReceiver.getCapabilities('video').codecs.some(c => c.mimeType === 'video/H264')"
            ):
                if time.monotonic() > deadline:
                    pytest.skip(f"{request.param} has no WebRTC H.264 decoder on this host")
                time.sleep(0.5)
            yield page
        finally:
            browser.close()


def test_camera_plays_as_a_direct_track(stack: Stack, page: Page) -> None:
    console: list[str] = []
    page.on("console", lambda message: console.append(message.text))
    page.on("pageerror", lambda error: console.append(f"pageerror: {error}"))
    page.goto(f"http://127.0.0.1:{stack.relay.http_port}/")
    page.wait_for_function(
        "() => typeof window.mode === 'function'", timeout=START_TIMEOUT_S * 1000
    )
    deadline = time.monotonic() + START_TIMEOUT_S
    while page.evaluate("() => window.mode()") != "rtc":
        statuses = page.evaluate("() => window.statuses")
        assert time.monotonic() < deadline, f"no track; statuses {statuses}; console {console}"
        assert not any(s.startswith("rejected") for s in statuses), statuses
        time.sleep(0.2)
    # Let the first frames land, then count two seconds of presented frames.
    time.sleep(1)
    before = page.evaluate("() => window.frames_seen")
    time.sleep(2)
    after = page.evaluate("() => window.frames_seen")
    assert after - before >= 25, (before, after, page.evaluate("() => window.statuses"))
    assert page.evaluate("() => window.mode()") == "rtc"
    stats = page.evaluate("() => window.stats()")
    inbound = stats["inbound"]
    assert inbound["framesDecoded"] >= 25
    assert inbound["keyFramesDecoded"] >= 1
    assert inbound["frameWidth"] == 320 and inbound["frameHeight"] == 240
    assert inbound["bytesReceived"] > 0


def test_sole_viewer_settings_change_keeps_the_track_and_presented_frames(stack, page):
    page.goto(f"http://127.0.0.1:{stack.relay.http_port}/?hz=24")
    page.wait_for_function("() => window.frames_seen >= 10", timeout=START_TIMEOUT_S * 1000)
    page.evaluate("() => { window.original_track = window.track(); }")
    before = page.evaluate("() => window.frames_seen")
    settings = page.evaluate(
        "() => window.change({ maxResolution: [160, 120], maxBitrate: 200000 })"
    )
    assert settings["maxResolution"] == [160, 120]
    assert settings["maxBitrate"] == 200000
    page.wait_for_function("n => window.frames_seen >= n + 10", arg=before, timeout=5000)
    assert page.evaluate("() => window.track() === window.original_track")
    assert page.evaluate("() => window.tracks_seen") == 1
    page.wait_for_function(
        "async () => (await window.stats()).inbound.frameWidth === 160", timeout=5000
    )
