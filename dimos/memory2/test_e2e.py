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

"""E2E test: import legacy pickle replays into memory2 SqliteStore."""

from __future__ import annotations

import bisect
from typing import TYPE_CHECKING, Any

import pytest

from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.memory2.embed import EmbedImages
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import QualityWindow
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data_dir

if TYPE_CHECKING:
    from collections.abc import Iterator

DB_PATH = get_data_dir() / "go2_short.db"


@pytest.fixture(scope="module")
def session() -> Iterator[SqliteStore]:
    store = SqliteStore(path=str(DB_PATH))
    with store:
        yield store
    store.stop()


class PoseIndex:
    """Preloaded odom data with O(log n) closest-timestamp lookup."""

    def __init__(self, replay: LegacyPickleStore[Any]) -> None:
        self._timestamps: list[float] = []
        self._data: list[Any] = []
        for ts, data in replay.iterate_ts():
            self._timestamps.append(ts)
            self._data.append(data)

    def find_closest(self, ts: float) -> Any | None:
        if not self._timestamps:
            return None
        idx = bisect.bisect_left(self._timestamps, ts)
        # Compare the two candidates around the insertion point
        if idx == 0:
            return self._data[0]
        if idx >= len(self._timestamps):
            return self._data[-1]
        if ts - self._timestamps[idx - 1] <= self._timestamps[idx] - ts:
            return self._data[idx - 1]
        return self._data[idx]

    def __iter__(self) -> Iterator[tuple[float, Any]]:
        return iter(zip(self._timestamps, self._data, strict=False))


@pytest.fixture(scope="module")
def video_replay() -> LegacyPickleStore[Image]:
    return LegacyPickleStore("go2_short/video")


@pytest.fixture(scope="module")
def odom_index() -> PoseIndex:
    return PoseIndex(LegacyPickleStore("go2_short/odom"))


@pytest.fixture(scope="module")
def lidar_replay() -> LegacyPickleStore[PointCloud2]:
    return LegacyPickleStore("go2_short/lidar")


@pytest.mark.tool
class TestImportReplay:
    """Import legacy pickle replay data into a memory2 SqliteStore.

    Lidar/odom are trimmed to start at video's first ts. The Memory2ReplayAdapter
    scheduler anchors each stream to its own first_ts on subscribe, so aligning
    first_ts across streams keeps replay synchronized.
    """

    def test_import_odom(
        self,
        session: SqliteStore,
        odom_index: PoseIndex,
        video_replay: LegacyPickleStore[Any],
    ) -> None:
        threshold = video_replay.first_timestamp()
        with session.stream("odom", Odometry) as odom:
            count = 0
            skipped = 0
            for ts, data in odom_index:
                if ts < threshold:
                    skipped += 1
                    continue
                odom.append(data, ts=ts, pose=data)
                count += 1

            assert count > 0
            assert odom.count() == count
            print(f"Imported {count} odom frames (skipped {skipped} before {threshold:.2f})")

    def test_import_video(
        self,
        session: SqliteStore,
        video_replay: LegacyPickleStore[Any],
        odom_index: PoseIndex,
    ) -> None:
        with session.stream("color_image", Image) as video:
            count = 0
            for ts, frame in video_replay.iterate_ts():
                pose = odom_index.find_closest(ts)
                video.append(frame, ts=ts, pose=pose)
                count += 1
                print(f"import [{count}] ts={ts:.2f} {frame}")

            assert count > 0
            assert video.count() == count
            print(f"Imported {count} video frames")

    def test_import_lidar(
        self,
        session: SqliteStore,
        lidar_replay: LegacyPickleStore[Any],
        odom_index: PoseIndex,
        video_replay: LegacyPickleStore[Any],
    ) -> None:
        threshold = video_replay.first_timestamp()
        lidar = session.stream("lidar", PointCloud2, codec="lz4+lcm")

        count = 0
        skipped = 0
        for ts, frame in lidar_replay.iterate_ts():
            if ts < threshold:
                skipped += 1
                continue
            pose = odom_index.find_closest(ts)
            lidar.append(frame, ts=ts, pose=pose)
            count += 1
            print(f"import [{count}] ts={ts:.2f} {frame}")

        assert count > 0
        assert lidar.count() == count
        print(f"Imported {count} lidar frames (skipped {skipped} before {threshold:.2f})")


@pytest.mark.tool
class TestEmbed:
    def test_embed_and_save(self, session: SqliteStore, clip: CLIPModel) -> None:
        """Embed video frames at 1Hz and persist to an embedded stream."""
        video = session.stream("color_image", Image)

        # Clear any prior run so the test is idempotent
        if "color_image_embedded" in session.list_streams():
            session.delete_stream("color_image_embedded")

        embedded = session.stream("color_image_embedded", Image)

        # Downsample to 2Hz, then embed
        pipeline = (
            video.filter(lambda obs: obs.data.brightness > 0.1)
            .tap(print)
            .transform(QualityWindow(lambda img: img.sharpness, window=0.5))
            .transform(EmbedImages(clip))
            .save(embedded)
        )

        count = 0
        for obs in pipeline:
            count += 1
            print(f"  [{count}] ts={obs.ts:.2f} pose={obs.pose}")

        assert count > 0
        print(f"Embedded {count} frames (1Hz from {video.count()} total)")

    def test_query_imported_data(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        lidar = session.stream("lidar", PointCloud2)

        assert video.exists()
        assert lidar.exists()

        first_frame = video.first()
        last_frame = video.last()
        assert first_frame.ts < last_frame.ts

        mid_ts = (first_frame.ts + last_frame.ts) / 2
        subset = video.time_range(first_frame.ts, mid_ts).to_list()
        assert 0 < len(subset) < video.count()

        streams = session.list_streams()
        assert "color_image" in streams
        assert "lidar" in streams


@pytest.mark.tool
class TestE2EQuery:
    """Query operations against real robot replay data."""

    def test_list_streams(self, session: SqliteStore) -> None:
        streams = session.list_streams()
        print(streams)

        assert "color_image" in streams
        assert "lidar" in streams
        assert session.streams.color_image
        assert session.streams.lidar

        print(session.streams.lidar)

    def test_video_count(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        assert video.count() > 1000

    def test_lidar_count(self, session: SqliteStore) -> None:
        lidar = session.stream("lidar", PointCloud2)
        assert lidar.count() > 1000

    def test_first_last_timestamps(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        first = video.first()
        last = video.last()
        assert first.ts < last.ts
        duration = last.ts - first.ts
        assert duration > 10.0  # at least 10s of data

    def test_time_range_filter(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        first = video.first()

        # Grab first 5 seconds
        window = video.time_range(first.ts, first.ts + 5.0).to_list()
        assert len(window) > 0
        assert len(window) < video.count()
        assert all(first.ts <= obs.ts <= first.ts + 5.0 for obs in window)

    def test_limit_offset_pagination(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        page1 = video.limit(10).to_list()
        page2 = video.offset(10).limit(10).to_list()

        assert len(page1) == 10
        assert len(page2) == 10
        assert page1[-1].ts < page2[0].ts  # no overlap

    def test_order_by_desc(self, session: SqliteStore) -> None:
        video = session.stream("color_image", Image)
        last_10 = video.order_by("ts", desc=True).limit(10).to_list()

        assert len(last_10) == 10
        assert all(last_10[i].ts >= last_10[i + 1].ts for i in range(9))

    def test_lazy_data_loads_correctly(self, session: SqliteStore) -> None:
        """Verify lazy blob loading returns valid Image data."""
        from dimos.memory2.type.observation import _Unloaded

        video = session.stream("color_image", Image)
        obs = next(iter(video.limit(1)))

        # Should start lazy
        assert isinstance(obs._data, _Unloaded)

        # Trigger load
        frame = obs.data
        assert isinstance(frame, Image)
        assert frame.width > 0
        assert frame.height > 0

    def test_iterate_window_decodes_all(self, session: SqliteStore) -> None:
        """Iterate a time window and verify every frame decodes."""
        video = session.stream("color_image", Image)
        first_ts = video.first().ts

        window = video.time_range(first_ts, first_ts + 2.0)
        count = 0
        for obs in window:
            frame = obs.data
            assert isinstance(frame, Image)
            count += 1
        assert count > 0

    def test_lidar_data_loads(self, session: SqliteStore) -> None:
        """Verify lidar blobs decode to PointCloud2."""
        lidar = session.stream("lidar", PointCloud2)
        frame = lidar.first().data
        assert isinstance(frame, PointCloud2)

    def test_poses_present(self, session: SqliteStore) -> None:
        """Verify poses were stored during import."""
        video = session.stream("color_image", Image)
        obs = video.first()
        assert obs.pose is not None

    def test_cross_stream_time_alignment(self, session: SqliteStore) -> None:
        """Video and lidar should overlap in time."""
        video = session.stream("color_image", Image)
        lidar = session.stream("lidar", PointCloud2)

        v_first, v_last = video.first().ts, video.last().ts
        l_first, l_last = lidar.first().ts, lidar.last().ts

        # Overlap: max of starts < min of ends
        overlap_start = max(v_first, l_first)
        overlap_end = min(v_last, l_last)
        assert overlap_start < overlap_end, "Video and lidar should overlap in time"


@pytest.mark.tool
class TestEmbedImages:
    """CLIP-embed imported video frames and search by text."""

    def test_search_by_text(self, session: SqliteStore, clip: CLIPModel) -> None:
        """Search embedded frames with a text query."""
        embedded = session.stream("color_image_embedded", Image)
        query = clip.embed_text("a door")

        results = embedded.search(query, k=5).to_list()
        assert len(results) > 0
        for obs in results:
            assert obs.similarity is not None
            assert obs.pose is not None
            print(f"sim={obs.similarity:.3f} ts={obs.ts} pose={obs.pose}")
