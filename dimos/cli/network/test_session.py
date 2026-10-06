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

import threading

from dimos.cli.network.model import Settings
from dimos.cli.network.session import HEADER, Endpoint, Receiver


def packet(phase, sequence):
    return HEADER.pack(phase, sequence) + bytes(1024 - HEADER.size)


def test_unique_delivery_reorder_and_window_excludes_drain():
    receiver = Receiver()
    receiver.reset(2, 1)
    receiver.accept(packet(1, 99), 0)  # Previous phase is ignored.
    receiver.accept(packet(2, 0), 1)
    receiver.accept(packet(2, 2), 1.1)
    receiver.accept(packet(2, 2), 1.2)
    receiver.accept(packet(2, 1), 1.3)
    receiver.accept(packet(2, 3), 2.5)  # Count for delivery, exclude from goodput.
    stats = receiver.summary(5)
    assert stats["unique_received"] == 4
    assert stats["missing"] == 1
    assert stats["duplicates"] == 1
    assert stats["out_of_order"] == 1
    assert stats["window_bytes"] == 3072
    assert stats["goodput_mbps"] == 3072 * 8 / 1e6
    assert stats["max_gap_ms"] == 1200


def test_sender_cancel_and_byte_cap_are_obeyed(mocker):
    stop = threading.Event()
    publisher = mocker.Mock()
    session = mocker.Mock()
    session.declare_publisher.return_value = publisher
    mocker.patch("dimos.cli.network.session.zenoh.open", return_value=session)
    endpoint = Endpoint("a" * 32, "local", "tcp/127.0.0.1:1", stop)
    try:
        settings = Settings(payload_bytes=1024)
        sent = endpoint.send(1, 1, 0.1, settings, 1024)
        assert sent["messages"] == 1
        assert sent["budget_exhausted"]
        assert sent["offered_mbps"] <= 1
        stop.set()
        cancelled = endpoint.send(2, 1, 0.1, settings, 2048)
        assert cancelled["messages"] == 0
    finally:
        endpoint.close()
