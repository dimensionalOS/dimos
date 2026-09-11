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

"""WebRTC video browser e2e against the live Cloudflare Realtime SFU.

Skipped without Cloudflare credentials: set CF_TELEOP_APP_ID and
CF_TELEOP_APP_SECRET (optionally CF_TURN_KEY_ID and CF_TURN_API_TOKEN). Run by
hand, never in CI (it spends Cloudflare egress):
`CF_TELEOP_APP_ID=... CF_TELEOP_APP_SECRET=... uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_rtc_video_browser.py`.
"""

from collections.abc import Callable, Iterator
import json
import os
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.web.cockpit import Video, cockpit
from dimos.web.relay_bridge.relay_bridge_module import RTC_AVAILABLE

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = [
    pytest.mark.web_browser,
    pytest.mark.skipif(
        not (os.environ.get("CF_TELEOP_APP_ID") and os.environ.get("CF_TELEOP_APP_SECRET")),
        reason="needs CF_TELEOP_APP_ID and CF_TELEOP_APP_SECRET (a Cloudflare Realtime app)",
    ),
    pytest.mark.skipif(not RTC_AVAILABLE, reason="aiortc (the webrtc extra) not installed"),
]

TOPIC = "/rtc_video_e2e/color_image"
# A static picture still yields a steady stream: aiortc encodes every fed frame.
_ROWS = np.linspace(0, 255, 240, dtype=np.uint8)[:, None]
_COLS = np.linspace(0, 255, 320, dtype=np.uint8)[None, :]
IMAGE = Image(
    data=np.stack(
        [
            np.broadcast_to(_ROWS, (240, 320)),
            np.broadcast_to(_COLS, (240, 320)),
            np.full((240, 320), 96, np.uint8),
        ],
        axis=-1,
    ).copy(),
    format=ImageFormat.RGB,
)


@pytest.fixture(scope="module")
def rtc_file(tmp_path_factory: pytest.TempPathFactory) -> str:
    config = {
        "appId": os.environ["CF_TELEOP_APP_ID"],
        "appSecret": os.environ["CF_TELEOP_APP_SECRET"],
    }
    if os.environ.get("CF_TURN_KEY_ID") and os.environ.get("CF_TURN_API_TOKEN"):
        config["turnKeyId"] = os.environ["CF_TURN_KEY_ID"]
        config["turnToken"] = os.environ["CF_TURN_API_TOKEN"]
    path = tmp_path_factory.mktemp("relay") / "rtc.json"
    path.write_text(json.dumps(config))
    return str(path)


@pytest.fixture(scope="module")
def cockpit_url(serve_channel: Callable[..., str], rtc_file: str) -> str:
    return serve_channel(
        cockpit(Video("color_image", max_hz=10.0)),
        stream="color_image",
        topic=TOPIC,
        message=IMAGE,
        robot_id="rtc-video-e2e",
        rtc_file=rtc_file,
    )


@pytest.fixture(params=["chromium", "firefox"])
def page(request: pytest.FixtureRequest, playwright_browsers: None) -> Iterator[Page]:
    with sync_playwright() as p:
        browser = getattr(p, request.param).launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def _rtc_stats(page: Page, url: str) -> dict[str, int]:
    rtc: dict[str, int] = page.request.get(f"{url}api/stats").json()["rtc"]
    return rtc


def _wait_pulls(page: Page, url: str, pulls: int, timeout_s: float = 20.0) -> None:
    deadline = time.monotonic() + timeout_s
    while _rtc_stats(page, url)["pulls"] != pulls:
        assert time.monotonic() < deadline, (
            f"rtc.pulls never reached {pulls}: {_rtc_stats(page, url)}"
        )
        time.sleep(0.5)


def _assert_track_plays(page: Page) -> None:
    video = page.get_by_test_id("video-color_image-track")
    expect(video).to_be_attached(timeout=60_000)
    # The SFU pull landed and the first frame decoded...
    page.wait_for_function(
        """() => {
          const video = document.querySelector('[data-testid="video-color_image-track"]');
          return video !== null && video.videoWidth === 320 && video.videoHeight === 240;
        }""",
        timeout=60_000,
    )
    # ...and frames keep coming (the badge reads them off the element too).
    frames = page.evaluate(
        """() => document.querySelector('[data-testid="video-color_image-track"]')
          .getVideoPlaybackQuality().totalVideoFrames"""
    )
    page.wait_for_function(
        """(before) => document.querySelector('[data-testid="video-color_image-track"]')
          .getVideoPlaybackQuality().totalVideoFrames > before + 5""",
        arg=frames,
        timeout=30_000,
    )
    expect(page.get_by_test_id("video-color_image-badge")).to_contain_text("fps", timeout=30_000)


def test_video_plays_as_a_webrtc_track_and_the_pull_follows_the_panel(
    cockpit_url: str, page: Page
) -> None:
    page.goto(cockpit_url)
    expect(page.get_by_test_id("status")).to_have_attribute(
        "data-phase", "connected", timeout=120_000
    )
    _assert_track_plays(page)
    _wait_pulls(page, cockpit_url, 1)
    # Nothing rode the relay for the camera: the SFU carried it.
    assert _rtc_stats(page, cockpit_url)["framesOnTrack"] == 0

    # The channels tab unmounts the panel: the subscription and the pull go.
    page.get_by_test_id("view-channels").click()
    _wait_pulls(page, cockpit_url, 0)
    # Back to the panels: a fresh pull, playing again.
    page.get_by_test_id("view-panels").click()
    _assert_track_plays(page)
    _wait_pulls(page, cockpit_url, 1)
