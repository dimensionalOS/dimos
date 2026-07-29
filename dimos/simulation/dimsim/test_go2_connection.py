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

from threading import Thread
import time

from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.dimsim.go2_connection import DimSimGO2Connection


def _connection_without_runtime() -> DimSimGO2Connection:
    connection = object.__new__(DimSimGO2Connection)
    connection._init_fresh_frame_state()
    return connection


def test_wait_for_frame_rejects_pre_request_image() -> None:
    connection = _connection_without_runtime()
    stale = Image(ts=99.0)
    fresh = Image(ts=101.0)
    connection._on_dimsim_frame(stale)

    publisher = Thread(
        target=lambda: (
            time.sleep(0.02),
            connection._on_dimsim_frame(fresh),
        )
    )
    publisher.start()
    try:
        assert connection._wait_for_frame_after(100.0, timeout=0.5) is fresh
    finally:
        publisher.join()


def test_wait_for_frame_rejects_late_delivery_with_old_sensor_timestamp() -> None:
    connection = _connection_without_runtime()
    delayed_stale = Image(ts=99.0)
    fresh = Image(ts=101.0)

    def publish_frames() -> None:
        time.sleep(0.01)
        connection._on_dimsim_frame(delayed_stale)
        time.sleep(0.01)
        connection._on_dimsim_frame(fresh)

    publisher = Thread(target=publish_frames)
    publisher.start()
    try:
        assert connection._wait_for_frame_after(100.0, timeout=0.5) is fresh
    finally:
        publisher.join()


def test_wait_for_frame_times_out_without_post_request_image() -> None:
    connection = _connection_without_runtime()
    connection._on_dimsim_frame(Image(ts=99.0))

    assert connection._wait_for_frame_after(100.0, timeout=0.01) is None
