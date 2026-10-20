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

"""Tests for refactored temporal memory components."""

from __future__ import annotations

from contextlib import ExitStack
import json
import threading
from typing import TYPE_CHECKING
from unittest.mock import create_autospec, patch

from dimos_cdrgen.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
import rerun as rr

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.models.vl.base import VlModel
from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds
from dimos.perception.experimental.temporal_memory.clip_filter import _get_image_data
from dimos.perception.experimental.temporal_memory.entity_graph_db import EntityGraphDB
from dimos.perception.experimental.temporal_memory.frame_window_accumulator import (
    Frame,
    FrameWindowAccumulator,
)
from dimos.perception.experimental.temporal_memory.temporal_memory import (
    TemporalMemory,
)
from dimos.perception.experimental.temporal_memory.temporal_state import TemporalState
from dimos.perception.experimental.temporal_memory.temporal_utils.graph_utils import (
    extract_time_window,
)
from dimos.perception.experimental.temporal_memory.temporal_utils.helpers import is_scene_stale
from dimos.perception.experimental.temporal_memory.window_analyzer import WindowAnalyzer
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.cdr import entity_points

if TYPE_CHECKING:
    from pathlib import Path


logger = setup_logger()


# ── Helpers ──────────────────────────────────────────────────────────


def _make_image(value: int = 128, shape: tuple[int, ...] = (64, 64, 3)) -> Image:
    data = np.full(shape, value, dtype=np.uint8)
    return image_from_array(data, encoding="rgb8")


class TestFrameWindowAccumulator:
    def test_bounded_buffer(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=5, window_s=1.0, stride_s=1.0, fps=1.0)
        acc.set_start_time(0.0)
        for i in range(10):
            img = _make_image(i * 25)
            img.header.stamp = time_from_seconds(float(i))
            acc.add_frame(img, float(i))
        assert acc.buffer_size == 5
        assert acc.frame_count == 10

    def test_window_extraction(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=50, window_s=2.0, stride_s=2.0, fps=1.0)
        acc.set_start_time(0.0)
        # Not enough frames
        assert acc.try_extract_window() is None
        # Add 3 frames
        for i in range(3):
            img = _make_image()
            img.header.stamp = time_from_seconds(float(i))
            acc.add_frame(img, float(i))
        frames = acc.try_extract_window()
        assert frames is not None
        assert len(frames) == 2  # fps=1 * window_s=2 = 2 frames needed

    def test_stride_guard(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=50, window_s=1.0, stride_s=5.0, fps=1.0)
        acc.set_start_time(0.0)
        for i in range(3):
            img = _make_image()
            img.header.stamp = time_from_seconds(float(i))
            acc.add_frame(img, float(i))
        # First extraction should succeed
        frames = acc.try_extract_window()
        assert frames is not None
        # Second extraction should fail (stride not elapsed)
        assert acc.try_extract_window() is None

    def test_empty_buffer(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=50, window_s=1.0, stride_s=1.0, fps=1.0)
        acc.set_start_time(0.0)
        assert acc.try_extract_window() is None
        assert acc.latest_frame() is None
        assert acc.frames_list() == []

    def test_clear(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=50, window_s=1.0, stride_s=1.0, fps=1.0)
        acc.set_start_time(0.0)
        img = _make_image()
        img.header.stamp = time_from_seconds(0.0)
        acc.add_frame(img, 0.0)
        assert acc.buffer_size == 1
        acc.clear()
        assert acc.buffer_size == 0


class TestTemporalState:
    def test_update_and_snapshot(self) -> None:
        state = TemporalState(next_summary_at_s=10.0)
        parsed = {
            "window": {"start_s": 0.0, "end_s": 2.0},
            "caption": "A person walks",
            "entities_present": [],
            "new_entities": [{"id": "E1", "type": "person", "descriptor": "walking person"}],
            "relations": [],
            "on_screen_text": [],
        }
        needs_summary = state.update_from_window(parsed, w_end=2.0, summary_interval_s=10.0)
        assert needs_summary is False
        snap = state.snapshot()
        assert len(snap.entity_roster) == 1
        assert snap.entity_roster[0]["id"] == "E1"
        assert len(snap.chunk_buffer) == 1

    def test_summary_trigger(self) -> None:
        state = TemporalState(next_summary_at_s=5.0)
        parsed = {
            "window": {"start_s": 0.0, "end_s": 5.0},
            "caption": "test",
            "entities_present": [],
            "new_entities": [],
            "relations": [],
        }
        needs = state.update_from_window(parsed, w_end=5.0, summary_interval_s=5.0)
        assert needs is True

    def test_apply_summary(self) -> None:
        state = TemporalState(next_summary_at_s=5.0)
        state.chunk_buffer.append({"caption": "test"})
        state.apply_summary("Summary text", w_end=5.0, summary_interval_s=5.0)
        assert state.rolling_summary == "Summary text"
        assert state.chunk_buffer == []
        assert state.next_summary_at_s == 10.0

    def test_error_skipped(self) -> None:
        state = TemporalState()
        parsed = {"_error": "bad parse"}
        needs = state.update_from_window(parsed, w_end=1.0, summary_interval_s=10.0)
        assert needs is False
        assert len(state.entity_roster) == 0

    def test_thread_safety(self) -> None:
        state = TemporalState(next_summary_at_s=1000.0)
        errors: list[Exception] = []

        def writer(n: int) -> None:
            try:
                for i in range(50):
                    parsed = {
                        "new_entities": [
                            {"id": f"E{n}_{i}", "type": "object", "descriptor": f"obj{n}_{i}"}
                        ],
                        "entities_present": [],
                        "relations": [],
                    }
                    state.update_from_window(parsed, w_end=float(i), summary_interval_s=1000.0)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(t,)) for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert len(errors) == 0
        snap = state.snapshot()
        # 4 threads * 50 entities each = 200
        assert len(snap.entity_roster) == 200

    def test_clear(self) -> None:
        state = TemporalState()
        state.entity_roster.append({"id": "E1"})
        state.rolling_summary = "test"
        state.clear(summary_interval_s=30.0)
        assert state.entity_roster == []
        assert state.rolling_summary == ""
        assert state.next_summary_at_s == 30.0

    def test_auto_add_referenced(self) -> None:
        """Entities referenced in relations but not in roster are auto-added."""
        state = TemporalState(next_summary_at_s=100.0)
        parsed = {
            "new_entities": [],
            "entities_present": [{"id": "E1"}],
            "relations": [{"type": "holds", "subject": "E1", "object": "E2"}],
        }
        state.update_from_window(parsed, w_end=1.0, summary_interval_s=100.0)
        ids = {e["id"] for e in state.entity_roster}
        assert "E1" in ids
        assert "E2" in ids


class TestExtractTimeWindow:
    def test_keyword_patterns(self) -> None:
        assert extract_time_window("just now") == 60
        assert extract_time_window("recently") == 600
        assert extract_time_window("yesterday") == 86400
        assert extract_time_window("last week") == 7 * 86400

    def test_quantity_patterns(self) -> None:
        assert extract_time_window("3 hours ago") == 10800
        assert extract_time_window("last 2 days") == 172800
        assert extract_time_window("past 5 minutes") == 300

    def test_no_time_reference(self) -> None:
        assert extract_time_window("what entities are visible?") is None
        assert extract_time_window("is there a person?") is None


class TestEntityGraphDB:
    @pytest.fixture
    def db(self, tmp_path: Path) -> EntityGraphDB:
        return EntityGraphDB(db_path=tmp_path / "test.db")

    def test_upsert_and_get(self, db: EntityGraphDB) -> None:
        db.upsert_entity("E1", "person", "walking person", 0.0)
        entity = db.get_entity("E1")
        assert entity is not None
        assert entity["entity_type"] == "person"
        assert entity["descriptor"] == "walking person"

    def test_relations(self, db: EntityGraphDB) -> None:
        db.upsert_entity("E1", "person", "person", 0.0)
        db.upsert_entity("E2", "object", "cup", 0.0)
        db.add_relation("holds", "E1", "E2", 0.9, 1.0)
        rels = db.get_relations_for_entity("E1")
        assert len(rels) == 1
        assert rels[0]["relation_type"] == "holds"

    def test_distances(self, db: EntityGraphDB) -> None:
        db.upsert_entity("E1", "person", "person", 0.0)
        db.upsert_entity("E2", "object", "table", 0.0)
        db.add_distance("E1", "E2", 2.0, "medium", 0.8, 1.0, "vlm")
        dist = db.get_distance("E1", "E2")
        assert dist is not None
        assert dist["distance_meters"] == 2.0
        # Test normalized ordering
        dist2 = db.get_distance("E2", "E1")
        assert dist2 is not None

    def test_no_semantic_relations_table(self, db: EntityGraphDB) -> None:
        """semantic_relations table should NOT exist after init."""
        conn = db._get_connection()
        cursor = conn.cursor()
        cursor.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='semantic_relations'"
        )
        assert cursor.fetchone() is None

    def test_save_window_data(self, db: EntityGraphDB) -> None:
        parsed = {
            "new_entities": [{"id": "E1", "type": "person", "descriptor": "standing person"}],
            "entities_present": [],
            "relations": [{"type": "looks_at", "subject": "E1", "object": "E2", "confidence": 0.7}],
        }
        # E2 not yet in DB but referenced in relation — should still save
        db.upsert_entity("E2", "object", "screen", 0.0)
        db.save_window_data(parsed, 1.0)
        assert db.get_entity("E1") is not None
        rels = db.get_recent_relations(limit=5)
        assert len(rels) == 1

    def test_stats(self, db: EntityGraphDB) -> None:
        db.upsert_entity("E1", "person", "person", 0.0)
        stats = db.get_stats()
        assert stats["entities"] == 1
        assert stats["relations"] == 0
        assert stats["distances"] == 0
        assert "semantic_relations" not in stats


class TestPersistence:
    def test_new_memory_clears_db(self, tmp_path: Path) -> None:
        db_dir = tmp_path / "memory" / "temporal"
        db_dir.mkdir(parents=True)
        db_path = db_dir / "entity_graph.db"
        # Create and populate DB
        db = EntityGraphDB(db_path=db_path)
        db.upsert_entity("E1", "person", "test", 0.0)
        db.commit()
        db.close()
        assert db_path.exists()

        # new_memory should delete it
        with patch(
            "dimos.perception.experimental.temporal_memory.temporal_memory.get_run_log_dir",
            return_value=None,
        ):
            tm = TemporalMemory(
                use_clip_filtering=False,
                vlm=create_autospec(VlModel, spec_set=True, instance=True),
                db_dir=str(db_dir),
                new_memory=True,
            )
            # DB should be empty since we cleared it
            stats = tm._graph_db.get_stats()
            assert stats["entities"] == 0
            tm.stop()

    def test_persistent_memory_survives(self, tmp_path: Path) -> None:
        db_dir = tmp_path / "memory" / "temporal"
        db_dir.mkdir(parents=True)
        db_path = db_dir / "entity_graph.db"
        # Populate
        db = EntityGraphDB(db_path=db_path)
        db.upsert_entity("E1", "person", "test", 0.0)
        db.commit()
        db.close()

        # new_memory=False (default) should keep data
        with patch(
            "dimos.perception.experimental.temporal_memory.temporal_memory.get_run_log_dir",
            return_value=None,
        ):
            tm = TemporalMemory(
                use_clip_filtering=False,
                vlm=create_autospec(VlModel, spec_set=True, instance=True),
                db_dir=str(db_dir),
                new_memory=False,
            )
            stats = tm._graph_db.get_stats()
            assert stats["entities"] == 1
            tm.stop()


class TestJSONLLogging:
    def test_log_entries(self, tmp_path: Path) -> None:
        db_dir = tmp_path / "db"
        db_dir.mkdir()
        log_dir = tmp_path / "logs" / "run1"
        log_dir.mkdir(parents=True)

        with patch(
            "dimos.perception.experimental.temporal_memory.temporal_memory.get_run_log_dir",
            return_value=log_dir,
        ):
            tm = TemporalMemory(
                use_clip_filtering=False,
                vlm=create_autospec(VlModel, spec_set=True, instance=True),
                db_dir=str(db_dir),
            )

        jsonl_path = log_dir / "temporal_memory" / "temporal_memory.jsonl"
        assert tm._jsonl_path == jsonl_path

        # Log a test entry
        tm._log_jsonl(
            {
                "ts": 1234.5,
                "type": "window_analysis",
                "window": [0.0, 2.0],
                "raw_vlm_response": "test raw response",
                "parsed": {"caption": "test"},
            }
        )

        assert jsonl_path.exists()
        with open(jsonl_path) as f:
            line = json.loads(f.readline())
        assert line["type"] == "window_analysis"
        assert line["raw_vlm_response"] == "test raw response"
        tm.stop()


class TestEntityMarkers:
    def test_publish_entity_markers(self, tmp_path: Path) -> None:
        db_dir = tmp_path / "db"
        db_dir.mkdir()

        with patch(
            "dimos.perception.experimental.temporal_memory.temporal_memory.get_run_log_dir",
            return_value=None,
        ):
            tm = TemporalMemory(
                use_clip_filtering=False,
                vlm=create_autospec(VlModel, spec_set=True, instance=True),
                db_dir=str(db_dir),
                visualize=True,
            )

        with ExitStack() as cleanup:
            wire = CdrTransport("entity_markers", EntityMarkers, backend="lcm", url="memq://")
            cleanup.callback(wire.stop)
            cleanup.callback(tm.stop)
            tm.entity_markers.transport = wire
            tm._graph_db.upsert_entity(
                "E1",
                "person",
                "walking person",
                1,
                metadata={"world_x": 1.0, "world_y": 2.0, "world_z": 0.0},
            )
            tm._graph_db.upsert_entity(
                "E2",
                "object",
                "table",
                1,
                metadata={"world_x": 3.0, "world_y": 4.0, "world_z": 0.0},
            )
            received = []
            ready = threading.Event()

            def receive(value):
                received.append(value)
                ready.set()

            cleanup.callback(wire.subscribe(receive))
            tm._publish_entity_markers()
            assert ready.wait(2)
            assert len(received) == 1
            msg = received[0]
            assert msg.header.frame_id == "world"
            assert {m.entity_id for m in msg.markers} == {"E1", "E2"}
            e1 = next(m for m in msg.markers if m.entity_id == "E1")
            assert (e1.position.x, e1.position.y, e1.position.z) == (1, 2, 0.3)

    def test_markers_to_rerun(self) -> None:
        markers = EntityMarkers(
            markers=[
                EntityMarker(
                    entity_id="E1",
                    label="person walking",
                    entity_type="person",
                    position=Point(x=1.0, y=2.0, z=0.3),
                ),
                EntityMarker(
                    entity_id="E2",
                    label="wooden table",
                    entity_type="object",
                    position=Point(x=3.0, y=4.0, z=0.3),
                ),
            ],
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
        archetype = entity_points(cdr_decode(cdr_encode(markers), EntityMarkers))
        # Should return rr.Points3D

        assert isinstance(archetype, rr.Points3D)


class TestWindowAnalyzer:
    def test_analyze_window_calls_vlm(self) -> None:
        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.return_value = json.dumps(
            {
                "window": {"start_s": 0.0, "end_s": 2.0},
                "caption": "test caption",
                "entities_present": [],
                "new_entities": [{"id": "E1", "type": "person", "descriptor": "a person"}],
                "relations": [],
            }
        )

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()
        img.header.stamp = time_from_seconds(0.0)
        frame = Frame(frame_index=0, timestamp_s=0.0, image=img)
        state_dict = {"entity_roster": [], "rolling_summary": ""}

        result = analyzer.analyze_window([frame], state_dict, 0.0, 2.0)
        assert result is not None
        assert result.parsed["caption"] == "test caption"
        assert result.raw_vlm_response != ""
        mock_vlm.query.assert_called_once()

    def test_analyze_window_vlm_error(self) -> None:
        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.side_effect = RuntimeError("VLM error")

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()
        img.header.stamp = time_from_seconds(0.0)
        frame = Frame(frame_index=0, timestamp_s=0.0, image=img)

        result = analyzer.analyze_window([frame], {}, 0.0, 2.0)
        assert result is None

    def test_update_summary(self) -> None:
        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.return_value = "Updated summary text"

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()

        result = analyzer.update_summary(img, "old summary", [{"caption": "chunk"}])
        assert result is not None
        assert result.summary_text == "Updated summary text"

    def test_answer_query(self) -> None:
        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.return_value = "The answer is 42"

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()

        result = analyzer.answer_query("What is the answer?", {}, img)
        assert result is not None
        assert result.answer == "The answer is 42"


def test_scene_staleness_compares_pixels_not_padding():
    first = Image(
        width=2,
        height=2,
        encoding="rgb8",
        step=8,
        data=np.asarray([10] * 6 + [0, 0] + [10] * 6 + [0, 0], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_bigendian=0,
    )
    last = cdr_decode(cdr_encode(first), Image)
    last.data = np.asarray([10] * 6 + [255, 255] + [10] * 6 + [255, 255], dtype=np.uint8)
    assert is_scene_stale([Frame(0, 0, first), Frame(1, 1, last)], stale_threshold=1)
    last.data = np.asarray([30] * 6 + [255, 255] + [30] * 6 + [255, 255], dtype=np.uint8)
    assert not is_scene_stale([Frame(0, 0, first), Frame(1, 1, last)], stale_threshold=1)


def test_clip_frame_comparison_uses_rgb_colors_for_bgr_wire_images():
    pixels = np.zeros((4, 4, 3), dtype=np.uint8)
    pixels[:, :, 0] = 255
    rgb = image_from_array(pixels, encoding="rgb8")
    bgr = image_from_array(pixels[:, :, ::-1], encoding="bgr8")
    bgr = cdr_decode(cdr_encode(bgr), Image)
    np.testing.assert_array_equal(_get_image_data(rgb), _get_image_data(bgr))
    np.testing.assert_array_equal(_get_image_data(bgr)[0, 0], [255, 0, 0])
