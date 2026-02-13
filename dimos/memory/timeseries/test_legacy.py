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
"""Tests specific to LegacyPickleStore."""

from dimos.memory.timeseries.legacy import LegacyPickleStore


class TestLegacyPickleStoreRealData:
    """Test LegacyPickleStore with real recorded data."""

    def test_read_lidar_recording(self) -> None:
        """Test reading from unitree_go2_bigoffice/lidar recording."""
        store = LegacyPickleStore("unitree_go2_bigoffice/lidar")

        # Check first timestamp exists
        first_ts = store.first_timestamp()
        assert first_ts is not None
        assert first_ts > 0

        # Check first data
        first = store.first()
        assert first is not None
        assert hasattr(first, "ts")

        # Check find_closest_seek works
        data_at_10s = store.find_closest_seek(10.0)
        assert data_at_10s is not None

        # Check iteration returns monotonically increasing timestamps
        prev_ts = None
        for i, item in enumerate(store.iterate()):
            assert item.ts is not None
            if prev_ts is not None:
                assert item.ts >= prev_ts, "Timestamps should be monotonically increasing"
            prev_ts = item.ts
            if i >= 10:  # Only check first 10 items
                break
