# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Generated snapshots retain source headers while seeking on the archive clock."""

import base64
from copy import deepcopy
from pathlib import Path
import pickle
from typing import Any

import pytest

from dimos.core.stream import Stream, Transport
from dimos.message_utils import time as message_time
from dimos.messages.builtin_interfaces.msg import Time
from dimos.utils.testing.moment import OutputMoment, SensorMoment
from dimos.utils.testing.test_pickle_messages import FIXTURES


class CapturingTransport(Transport[Any]):
    def __init__(self) -> None:
        self.values: list[Any] = []
        self.stopped = False

    def broadcast(self, stream: Stream[Any] | None, value: Any) -> None:
        self.values.append(value)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stopped = True


@pytest.mark.parametrize("fixture", ["image", "pose", "cloud"])
def test_generated_moments_seek_archive_clock_and_preserve_headers(
    tmp_path: Path, fixture: str
) -> None:
    (tmp_path / "000.pickle").write_bytes(base64.b64decode(FIXTURES[fixture]))
    transport = CapturingTransport()
    sensor = SensorMoment[Any](str(tmp_path), transport)
    sensor.seek(0.0)
    first = sensor.value
    assert first is not None
    assert message_time.to_nanoseconds(first.header.stamp) == 1_250_000_000
    original_frame = first.header.frame_id
    sensor.publish()
    assert transport.values[-1] is first

    second = deepcopy(first)
    second.header.stamp = Time(sec=55, nanosec=987654321)
    with (tmp_path / "001.pickle").open("wb") as file:
        pickle.dump((100.0, second), file)
    sensor.seek(99.9)
    selected = sensor.value
    assert selected is not None
    assert message_time.to_nanoseconds(selected.header.stamp) == 55_987_654_321
    assert selected.header.frame_id == original_frame
    sensor.publish()
    assert transport.values[-1] is selected

    output = OutputMoment[Any](transport)
    output.set(selected)
    output.publish()
    assert transport.values[-1] is selected
    assert message_time.to_nanoseconds(selected.header.stamp) == 55_987_654_321
    sensor.stop()
    output.stop()
    assert sensor.value is None and output.value is None
    assert transport.stopped
