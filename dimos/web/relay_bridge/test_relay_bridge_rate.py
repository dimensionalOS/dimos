# Copyright 2025-2026 Dimensional Inc.
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

"""The relay's rate feedback at the bridge (protocol v8): the sampling
interval follows the cap and a capped JPEG channel lowers its quality."""

from __future__ import annotations

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.relay_bridge.module_test_support import image_transport, push, wait_until
from dimos.web.relay_bridge.protocol import Rate, Subs


def test_rate_feedback_caps_the_sampling_rate(bridge) -> None:
    # The relay's rate message (v8) lowers the channel's sampling interval to
    # what its slowest viewer keeps up with, never above max_hz or under the
    # floor; a channel it does not know is ignored.
    module, clients = bridge
    client = clients[0]
    close = lambda interval: abs(module._min_interval["color_image"] - interval) < 1e-9  # noqa: E731
    assert close(1 / 30)
    push(module, client, Rate(ch="color_image", hz=5))
    assert wait_until(lambda: close(0.2))
    push(module, client, Rate(ch="nope", hz=1))
    push(module, client, Rate(ch="color_image", hz=100))
    assert wait_until(lambda: close(1 / 30))
    push(module, client, Rate(ch="color_image", hz=0.01))
    assert wait_until(lambda: close(2.0))


def test_rate_feedback_lowers_jpeg_quality(bridge, monkeypatch) -> None:
    # A rate cap at a quarter of max_hz halves the JPEG quality (square root
    # of the share), never under the floor; back at max_hz the channel's own
    # quality returns.
    module, clients = bridge
    qualities: list[int] = []
    real = Image.to_jpeg_bytes

    def spy(self: Image, quality: int = 75) -> bytes:
        qualities.append(quality)
        return real(self, quality=quality)

    monkeypatch.setattr(Image, "to_jpeg_bytes", spy)
    spec = next(s for s in module._channel_specs if s.ch == "color_image")
    base = spec.params["quality"]
    push(module, clients[0], Subs(chs=["color_image"], n=1))
    assert wait_until(lambda: image_transport(module).subscribers)
    push(module, clients[0], Rate(ch="color_image", hz=spec.max_hz / 4))
    assert wait_until(lambda: module._rate_scale.get("color_image") == 0.25)
    image = Image.from_numpy(np.zeros((8, 12, 3), dtype=np.uint8))
    image_transport(module).publish(image)
    assert wait_until(lambda: len(qualities) == 1)
    assert qualities[-1] == max(30, round(base * 0.5))
    push(module, clients[0], Rate(ch="color_image", hz=spec.max_hz))
    assert wait_until(lambda: module._rate_scale.get("color_image") == 1.0)
    # The gate's next due time was set under the slower cap: publish until
    # a frame passes it.
    for _ in range(40):
        image_transport(module).publish(image)
        if wait_until(lambda: len(qualities) == 2, timeout=0.05):
            break
    assert len(qualities) == 2
    assert qualities[-1] == base
