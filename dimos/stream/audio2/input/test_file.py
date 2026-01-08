#!/usr/bin/env python3
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

"""Tests for file input source."""

import threading
import time

import pytest
from reactivex import operators as ops

from dimos.stream.audio2.input.file import file_input
from dimos.utils.data import get_data


def test_file_input_completes():
    """Test that file_input emits events and properly completes the observable."""

    # Track events and completion
    event_count = 0
    completed = False

    def on_next(value):
        nonlocal event_count
        event_count += 1

    def on_completed():
        nonlocal completed
        completed = True

    # Subscribe and wait with run() - blocks until completion
    file_input(
        file_path=str(get_data("audio_bender") / "out_of_date.wav"),
        realtime=False,  # Fast playback for testing
    ).pipe(ops.do_action(on_next=on_next, on_completed=on_completed)).run()

    # Check that we received events
    assert event_count > 0, f"Expected events but got {event_count}"

    # Check that the observable completed
    assert completed, "Observable did not complete"

    # Wait for threads to clean up
    import time

    max_wait = 2.0
    start = time.time()
    while time.time() - start < max_wait:
        file_threads = [
            t
            for t in threading.enumerate()
            if "FileInput" in t.name or "GStreamerMainLoop" in t.name
        ]
        if not file_threads:
            break
        time.sleep(0.1)
