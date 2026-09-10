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

"""Chat-mic push-to-talk browser e2e (the T14 acceptance demo, in CI).

Starts a cockpit(layout=Chat()) bridge - a generated RelayBridgeModule
subclass with a typed audio_in Out port and the registry-resolved
audio.json.v1 decoder - serving the real cockpit dist from its own local
relay. Both engines run with a fake microphone (T14's named risk is
MediaRecorder container variance, so Chromium's webm is not enough);
holding the chat composer's mic records real audio, ships it as chunked
publishes, and the test asserts the reassembled byte stream (one sid,
contiguous seqs, empty final frame) lands on the module's port AND survives
the real ffmpeg decode into the 16-kHz PCM the Whisper pipeline assumes.
VoiceInput/Whisper stay out of scope: chunk delivery and decodability are
the wire contract under test; transcription wiring is pytest-covered.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, built web dists, and
the ffmpeg binary). Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_voice_browser.py`.
"""

from collections.abc import Iterator
import time

import pytest

from dimos.stream.audio.decode import decode_audio_bytes
from dimos.web.cockpit import Chat, cockpit
from dimos.web.relay_bridge.audio_codec import AudioChunk
from dimos.web.relay_bridge.e2e_support import stop_module

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser


@pytest.fixture(scope="module")
def voice_bridge() -> Iterator[tuple[str, list[AudioChunk]]]:
    (atom,) = cockpit(layout=Chat()).blueprints
    module = atom.module(local_port=0, open_browser=False, robot_id="voice-e2e", **atom.kwargs)
    chunks: list[AudioChunk] = []
    module.audio_in.subscribe(chunks.append)
    try:
        module.start()  # builds web dists if stale, spawns the relay, registers
        relay = module._relay
        assert relay is not None and relay.info is not None
        yield relay.info.open_url, chunks
    finally:
        stop_module(module)


@pytest.fixture(params=["chromium", "firefox"])
def fake_mic_page(request: pytest.FixtureRequest, playwright_browsers: None) -> Iterator[Page]:
    with sync_playwright() as p:
        if request.param == "chromium":
            browser = p.chromium.launch(
                args=["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"]
            )
            context = browser.new_context(permissions=["microphone"])
        else:
            # Firefox has no permissions API in Playwright; the prefs both
            # fake the capture device and suppress the permission prompt.
            browser = p.firefox.launch(
                firefox_user_prefs={
                    "media.navigator.streams.fake": True,
                    "media.navigator.permission.disabled": True,
                }
            )
            context = browser.new_context()
        try:
            yield context.new_page()
        finally:
            browser.close()


def test_hold_to_talk_ships_a_decodable_recording(
    voice_bridge: tuple[str, list[AudioChunk]], fake_mic_page: Page
) -> None:
    url, chunks = voice_bridge
    chunks.clear()  # the module fixture is shared across both engines
    fake_mic_page.goto(url)
    mic = fake_mic_page.get_by_test_id("chat-audio_in-mic")
    # Enabled == transport connected; the manifest already placed the panel.
    expect(mic).to_be_enabled(timeout=120_000)
    expect(mic).to_have_attribute("data-state", "idle")

    mic.hover()
    fake_mic_page.mouse.down()
    expect(mic).to_have_attribute("data-state", "recording", timeout=15_000)
    fake_mic_page.wait_for_timeout(1_500)
    fake_mic_page.mouse.up()
    expect(mic).to_have_attribute("data-state", "idle", timeout=30_000)

    # The bridge acks each frame only after Out.publish() returned, so by the
    # time the page settled back to idle the port has (about) everything;
    # tolerate the last callback still crossing threads.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not any(chunk.final for chunk in chunks):
        time.sleep(0.1)

    assert chunks and chunks[-1].final and chunks[-1].data == b""
    assert {chunk.sid for chunk in chunks} == {chunks[0].sid}
    assert [chunk.seq for chunk in chunks] == list(range(len(chunks)))
    assert chunks[0].mime.startswith("audio/")
    audio = b"".join(chunk.data for chunk in chunks)
    assert len(audio) > 1_000  # ~1.5 s of compressed audio, not an empty shell
    # Whatever container this engine produced, the owned ffmpeg decode turns
    # it into the PCM shape the Whisper pipeline assumes.
    event = decode_audio_bytes(audio)
    assert event is not None and event.sample_rate == 16_000
    assert event.data.ndim == 1 and event.data.shape[0] > 8_000  # > 0.5 s
