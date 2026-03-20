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

import json
import os
import threading
import time
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock, create_autospec, patch

from dotenv import load_dotenv
import numpy as np
import pytest
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs.Image import Image
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
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

load_dotenv()

logger = setup_logger()


# ── Helpers ──────────────────────────────────────────────────────────


def _make_image(value: int = 128, shape: tuple[int, ...] = (64, 64, 3)) -> Image:
    data = np.full(shape, value, dtype=np.uint8)
    return Image.from_numpy(data)


class TestFrameWindowAccumulator:
    def test_bounded_buffer(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=5, window_s=1.0, stride_s=1.0, fps=1.0)
        acc.set_start_time(0.0)
        for i in range(10):
            img = _make_image(i * 25)
            img.ts = float(i)
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
            img.ts = float(i)
            acc.add_frame(img, float(i))
        frames = acc.try_extract_window()
        assert frames is not None
        assert len(frames) == 2  # fps=1 * window_s=2 = 2 frames needed

    def test_stride_guard(self) -> None:
        acc = FrameWindowAccumulator(max_buffer_frames=50, window_s=1.0, stride_s=5.0, fps=1.0)
        acc.set_start_time(0.0)
        for i in range(3):
            img = _make_image()
            img.ts = float(i)
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
        img.ts = 0.0
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
                vlm=create_autospec(VlModel, spec_set=True, instance=True),
                db_dir=str(db_dir),
                visualize=True,
            )

        # Populate DB with world positions
        tm._graph_db.upsert_entity(
            "E1",
            "person",
            "walking person",
            1.0,
            metadata={"world_x": 1.0, "world_y": 2.0, "world_z": 0.0},
        )
        tm._graph_db.upsert_entity(
            "E2",
            "object",
            "table",
            1.0,
            metadata={"world_x": 3.0, "world_y": 4.0, "world_z": 0.0},
        )

        # Mock the output stream publish
        tm.entity_markers = MagicMock()
        tm._publish_entity_markers()

        # Should have published EntityMarkers
        tm.entity_markers.publish.assert_called_once()
        msg = tm.entity_markers.publish.call_args[0][0]
        assert len(msg.markers) == 2
        ids = {m.entity_id for m in msg.markers}
        assert ids == {"E1", "E2"}
        e1 = next(m for m in msg.markers if m.entity_id == "E1")
        assert e1.x == 1.0
        assert e1.y == 2.0
        tm.stop()

    def test_markers_to_rerun(self) -> None:
        from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker

        markers = EntityMarkers(
            markers=[
                Marker("E1", "person walking", "person", 1.0, 2.0, 0.3),
                Marker("E2", "wooden table", "object", 3.0, 4.0, 0.3),
            ]
        )
        archetype = markers.to_rerun()
        # Should return rr.Points3D
        import rerun as rr

        assert isinstance(archetype, rr.Points3D)


class TestWindowAnalyzer:
    def test_analyze_window_calls_vlm(self) -> None:
        from dimos.perception.experimental.temporal_memory.window_analyzer import WindowAnalyzer

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
        img.ts = 0.0
        frame = Frame(frame_index=0, timestamp_s=0.0, image=img)
        state_dict = {"entity_roster": [], "rolling_summary": ""}

        result = analyzer.analyze_window([frame], state_dict, 0.0, 2.0)
        assert result is not None
        assert result.parsed["caption"] == "test caption"
        assert result.raw_vlm_response != ""
        mock_vlm.query.assert_called_once()

    def test_analyze_window_vlm_error(self) -> None:
        from dimos.perception.experimental.temporal_memory.window_analyzer import WindowAnalyzer

        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.side_effect = RuntimeError("VLM error")

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()
        img.ts = 0.0
        frame = Frame(frame_index=0, timestamp_s=0.0, image=img)

        result = analyzer.analyze_window([frame], {}, 0.0, 2.0)
        assert result is None

    def test_update_summary(self) -> None:
        from dimos.perception.experimental.temporal_memory.window_analyzer import WindowAnalyzer

        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.return_value = "Updated summary text"

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()

        result = analyzer.update_summary(img, "old summary", [{"caption": "chunk"}])
        assert result is not None
        assert result.summary_text == "Updated summary text"

    def test_answer_query(self) -> None:
        from dimos.perception.experimental.temporal_memory.window_analyzer import WindowAnalyzer

        mock_vlm = create_autospec(VlModel, spec_set=True, instance=True)
        mock_vlm.query.return_value = "The answer is 42"

        analyzer = WindowAnalyzer(mock_vlm)
        img = _make_image()

        result = analyzer.answer_query("What is the answer?", {}, img)
        assert result is not None
        assert result.answer == "The answer is 42"


class VideoReplayModule(Module):
    """Module that replays synthetic video data for tests."""

    video_out: Out[Image]

    def __init__(self, num_frames: int = 5, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.num_frames = num_frames

    @rpc
    def start(self) -> None:
        import reactivex

        def emit_frames(observer, scheduler):  # type: ignore[no-untyped-def]
            for i in range(self.num_frames):
                img = _make_image(value=min(50 + i * 30, 255))  # Varying brightness
                img.ts = time.time()
                observer.on_next(img)
                time.sleep(0.5)
            observer.on_completed()

        self._disposables.add(
            reactivex.create(emit_frames)
            .pipe(
                ops.observe_on(reactivex.scheduler.NewThreadScheduler()),
            )
            .subscribe(self.video_out.publish)
        )

    @rpc
    def stop(self) -> None:
        for stream in list(self.outputs.values()):
            if stream.transport is not None and hasattr(stream.transport, "stop"):
                stream.transport.stop()
                stream._transport = None
        super().stop()


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.slow
class TestTemporalMemoryIntegration:
    @pytest.fixture(scope="function")
    def dimos_cluster(self):
        dimos = ModuleCoordinator()
        dimos.start()
        try:
            yield dimos
        finally:
            dimos.stop()

    @pytest.fixture(scope="function")
    def video_module(self, dimos_cluster):
        video_module = dimos_cluster.deploy(VideoReplayModule, num_frames=8)
        video_module.video_out.transport = LCMTransport("/test_video_refactored", Image)
        yield video_module
        try:
            video_module.stop()
        except Exception:
            pass

    @pytest.fixture(scope="function")
    def temporal_memory_module(self, dimos_cluster, tmp_path):
        from dimos.models.vl.openai import OpenAIVlModel

        api_key = os.getenv("OPENAI_API_KEY")
        vlm = OpenAIVlModel(api_key=api_key)

        db_dir = tmp_path / "memory" / "temporal"
        log_dir = tmp_path / "logs" / "run1"
        log_dir.mkdir(parents=True)

        with patch(
            "dimos.perception.experimental.temporal_memory.temporal_memory.get_run_log_dir",
            return_value=log_dir,
        ):
            tm = dimos_cluster.deploy(
                TemporalMemory,
                vlm=vlm,
                fps=1.0,
                window_s=2.0,
                stride_s=2.0,
                summary_interval_s=10.0,
                max_frames_per_window=3,
                db_dir=str(db_dir),
            )
        yield tm
        try:
            tm.stop()
        except Exception:
            pass

    def test_frames_flow_and_query(
        self, dimos_cluster, video_module, temporal_memory_module, tmp_path
    ):
        temporal_memory_module.color_image.connect(video_module.video_out)
        temporal_memory_module.start()  # subscribe first (consumer)
        time.sleep(1)  # let subscription establish
        video_module.start()  # then emit frames (producer)

        timeout = 15.0
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            state = temporal_memory_module.get_state()
            if state["frame_count"] >= 3:
                break
            time.sleep(0.5)
        else:
            state = temporal_memory_module.get_state()
            raise AssertionError(
                f"No frames processed within {timeout}s. Count: {state['frame_count']}"
            )

        time.sleep(3)

        state = temporal_memory_module.get_state()
        assert state["frame_count"] >= 3

        answer = temporal_memory_module.query("What entities are visible?")
        assert len(answer) > 0

        entities = temporal_memory_module.get_entity_roster()
        assert isinstance(entities, list)

        summary = temporal_memory_module.get_rolling_summary()
        assert isinstance(summary, str)

        video_module.stop()
        temporal_memory_module.stop()
