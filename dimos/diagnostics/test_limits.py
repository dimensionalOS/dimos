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

"""Observer limits apply to metadata and every collector sharing a host root."""

from contextlib import ExitStack
import json
import os

import pytest

from dimos.diagnostics.collector import bounded_metadata
from dimos.diagnostics.limits import HOST_BYTES, MemoryReservation


def test_process_reservations_are_cumulative_and_release_on_close(tmp_path):
    with ExitStack() as cleanup:
        first = MemoryReservation(tmp_path)
        cleanup.callback(first.close)
        second = MemoryReservation(tmp_path)
        cleanup.callback(second.close)
        assert first.bytes == 64 * 1024 * 1024
        assert second.bytes == 0
        first.close()
        third = MemoryReservation(tmp_path, requested=16 * 1024 * 1024)
        cleanup.callback(third.close)
        assert third.bytes == 16 * 1024 * 1024


def test_invalid_reservations_do_not_silently_overcommit(tmp_path):
    (tmp_path / ".memory.json").write_text(
        json.dumps({"bad": {"pid": os.getpid(), "bytes": HOST_BYTES + 1}})
    )
    with pytest.raises(ValueError, match="reservations"):
        MemoryReservation(tmp_path)


def test_metadata_never_retains_mutable_application_objects():
    fields = {"event": "received", "context": {"parents": ["one"]}}
    copy = bounded_metadata(fields)
    fields["context"]["parents"].append("two")
    assert copy["context"]["parents"] == ["one"]
    with pytest.raises(ValueError):
        bounded_metadata({"image": bytearray(100)})
    with pytest.raises(ValueError):
        bounded_metadata({"label": "x" * 100000})
