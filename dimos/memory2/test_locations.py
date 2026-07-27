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

"""Tests for durable named locations. Run against both store backends."""

from __future__ import annotations

import math
import tempfile
import time
from typing import TYPE_CHECKING, cast

import pytest

from dimos.memory2.locations import (
    FRAME_MAP,
    FRAME_WORLD,
    LocationStore,
    MapMismatchError,
    NotRelocalizedError,
    RelocStatus,
    RunLocalError,
    StaleAnchorError,
    UnknownLocationError,
    normalize_name,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

    from dimos.memory2.store.base import Store

RUN_A = "run_a"
RUN_B = "run_b"
MAP_V1 = "map_v1"
MAP_V2 = "map_v2"


@pytest.fixture
def sqlite_location_store() -> Iterator[SqliteStore]:
    """SQLite store without conftest's blanket Darwin skip.

    conftest skips SQLite everywhere sqlite-vec might not load, but that extension
    only backs vector search. Locations rely on the tag ``json_extract`` push-down,
    the R*Tree pose index, and SQL ``ORDER BY`` — all of which work regardless, and
    all of which this suite would otherwise never exercise.
    """
    with tempfile.NamedTemporaryFile(suffix=".db") as f:
        with SqliteStore(path=f.name) as store:
            yield store


@pytest.fixture(params=["memory_store", "sqlite_location_store"])
def session(request: pytest.FixtureRequest) -> Store:
    """Overrides conftest's ``session`` so both backends really run here."""
    return cast("Store", request.getfixturevalue(request.param))


class FakeTF:
    """Minimal ``TFLookup``: a fixed table of ``(parent, child) -> Transform``."""

    def __init__(self, transforms: dict[tuple[str, str], Transform] | None = None) -> None:
        self.transforms = transforms or {}

    def set(self, parent: str, child: str, transform: Transform) -> None:
        self.transforms[(parent, child)] = transform
        self.transforms[(child, parent)] = transform.inverse()

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        if parent_frame == child_frame:
            return Transform.identity()
        return self.transforms.get((parent_frame, child_frame))


def yaw_quat(theta: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))


def world_to_map(x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> FakeTF:
    """A tf where ``map`` sits at (x, y, theta) relative to ``world``."""
    tf = FakeTF()
    tf.set(
        FRAME_WORLD,
        FRAME_MAP,
        Transform(
            translation=Vector3(x, y, 0.0),
            rotation=yaw_quat(theta),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        ),
    )
    return tf


@pytest.fixture
def unanchored(session: Store) -> LocationStore:
    """No map, no tf — everything saves run-local."""
    return LocationStore(session, run_id=RUN_A)


@pytest.fixture
def anchored(session: Store) -> LocationStore:
    """Relocalized against MAP_V1, with map offset 10m in x from world."""
    return LocationStore(
        session,
        run_id=RUN_A,
        map_lineage=(MAP_V1,),
        tf=world_to_map(x=10.0),
        status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time(), fitness=0.9),
    )


class TestNames:
    def test_normalization(self) -> None:
        assert normalize_name("Kitchen") == "kitchen"
        assert normalize_name("  the  Front   Door ") == "the front door"

    def test_empty_name_rejected(self) -> None:
        with pytest.raises(ValueError, match="must not be empty"):
            normalize_name("   ")

    def test_lookup_is_case_insensitive(self, unanchored: LocationStore) -> None:
        unanchored.save("Kitchen", (1.0, 2.0, 0.0))
        assert unanchored.get("KITCHEN") is not None
        assert unanchored.get("  kitchen ") is not None


class TestSaveAndGet:
    def test_round_trip(self, unanchored: LocationStore) -> None:
        saved = unanchored.save("kitchen", (1.0, 2.0, 0.5))
        got = unanchored.get("kitchen")
        assert got is not None
        assert got.position == pytest.approx((1.0, 2.0, 0.5))
        assert got.location_id == saved.location_id

    def test_unanchored_without_map(self, unanchored: LocationStore) -> None:
        loc = unanchored.save("kitchen", (1.0, 2.0, 0.0))
        assert loc.anchored is False
        assert loc.frame_id == FRAME_WORLD
        assert loc.map_id is None

    def test_orientation_survives_round_trip(self, unanchored: LocationStore) -> None:
        """Regression for the legacy path, which stored quaternion (x, y, z) into a
        roll/pitch/yaw field and read it back through from_euler — compressing every
        heading into a ±57° cone. 90° must come back as 90°, not 40.5°."""
        heading = math.pi / 2
        q = yaw_quat(heading)
        unanchored.save("kitchen", (1.0, 2.0, 0.0, q.x, q.y, q.z, q.w))

        got = unanchored.get("kitchen")
        assert got is not None
        assert got.yaw == pytest.approx(heading)

    def test_orientation_survives_full_circle(self, unanchored: LocationStore) -> None:
        """The legacy bug is invisible near 0° — check headings it could not represent."""
        for heading in (-3.0, -2.0, 2.0, 3.0):
            q = yaw_quat(heading)
            unanchored.save("spot", (0.0, 0.0, 0.0, q.x, q.y, q.z, q.w))
            got = unanchored.get("spot")
            assert got is not None
            assert got.yaw == pytest.approx(heading, abs=1e-6)

    def test_missing_returns_none(self, unanchored: LocationStore) -> None:
        assert unanchored.get("nowhere") is None

    def test_pose_is_required(self, unanchored: LocationStore) -> None:
        with pytest.raises(ValueError, match="needs a pose"):
            unanchored.save("kitchen", None)


class TestUpsert:
    def test_second_save_replaces(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        unanchored.save("kitchen", (5.0, 5.0, 0.0))

        got = unanchored.get("kitchen")
        assert got is not None
        assert got.position == pytest.approx((5.0, 5.0, 0.0))
        assert len(unanchored.list_all()) == 1

    def test_identity_and_creation_time_are_stable(self, unanchored: LocationStore) -> None:
        first = unanchored.save("kitchen", (1.0, 1.0, 0.0))
        second = unanchored.save("kitchen", (5.0, 5.0, 0.0))

        assert second.location_id == first.location_id
        assert second.created_ts == first.created_ts
        assert second.updated_ts >= first.updated_ts

    def test_history_retains_versions(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        unanchored.save("kitchen", (5.0, 5.0, 0.0))

        history = unanchored.history("kitchen")
        assert len(history) == 2
        assert history[0].position == pytest.approx((5.0, 5.0, 0.0))
        assert history[1].position == pytest.approx((1.0, 1.0, 0.0))

    def test_distinct_names_coexist(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        unanchored.save("office", (2.0, 2.0, 0.0))
        assert {loc.name for loc in unanchored.list_all()} == {"kitchen", "office"}


class TestDelete:
    def test_tombstone_hides_location(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        assert unanchored.delete("kitchen") is True

        assert unanchored.get("kitchen") is None
        assert unanchored.list_all() == []

    def test_delete_is_recorded_not_erased(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        unanchored.delete("kitchen")

        assert len(unanchored.history("kitchen")) == 2
        assert len(unanchored.list_all(include_deleted=True)) == 1

    def test_delete_missing_is_false(self, unanchored: LocationStore) -> None:
        assert unanchored.delete("nowhere") is False

    def test_resave_after_delete(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 1.0, 0.0))
        unanchored.delete("kitchen")
        unanchored.save("kitchen", (9.0, 9.0, 0.0))

        got = unanchored.get("kitchen")
        assert got is not None
        assert got.position == pytest.approx((9.0, 9.0, 0.0))


class TestAnchoring:
    def test_save_converts_world_to_map(self, anchored: LocationStore) -> None:
        loc = anchored.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_WORLD)

        assert loc.anchored is True
        assert loc.frame_id == FRAME_MAP
        assert loc.map_id == MAP_V1
        assert loc.position == pytest.approx((-9.0, 2.0, 0.0))

    def test_map_frame_pose_stored_as_is(self, anchored: LocationStore) -> None:
        loc = anchored.save("kitchen", (3.0, 4.0, 0.0), frame_id=FRAME_MAP)
        assert loc.position == pytest.approx((3.0, 4.0, 0.0))
        assert loc.anchored is True

    def test_round_trip_through_map_is_identity(self, anchored: LocationStore) -> None:
        anchored.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_WORLD)
        resolved = anchored.resolve("kitchen", FRAME_WORLD)

        assert (resolved.position.x, resolved.position.y) == pytest.approx((1.0, 2.0))
        assert resolved.frame_id == FRAME_WORLD

    def test_round_trip_survives_rotation(self, session: Store) -> None:
        """The map origin is rotated relative to world, not just translated."""
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=3.0, y=-2.0, theta=math.pi / 3),
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time()),
        )
        store.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_WORLD)
        resolved = store.resolve("kitchen", FRAME_WORLD)

        assert (resolved.position.x, resolved.position.y) == pytest.approx((1.0, 2.0))


class TestPersistenceAcrossSessions:
    """The closest thing to the acceptance test that needs no hardware.

    Every other cross-run test here keeps one store open, which exercises the frame
    maths but not persistence. "Tomorrow" means the process exited: the SQLite file
    is closed and reopened, streams and their codecs are rebuilt from the registry,
    and the run id is different. That is a genuinely separate failure surface.
    """

    def _store(self, db: str, run_id: str, tf: FakeTF) -> tuple[SqliteStore, LocationStore]:
        store = SqliteStore(path=db)
        return store, LocationStore(
            store,
            run_id=run_id,
            map_lineage=(MAP_V1,),
            tf=tf,
            status=lambda: RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=0.9, map_id=MAP_V1
            ),
        )

    def test_anchored_location_survives_a_restart(self, tmp_path: Path) -> None:
        db = str(tmp_path / "locations.db")

        store_a, run_a = self._store(db, RUN_A, world_to_map(x=10.0))
        with store_a:
            saved = run_a.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_WORLD)
            assert saved.anchored is True

        store_b, run_b = self._store(db, RUN_B, world_to_map(x=-40.0, y=7.0, theta=math.pi / 2))
        with store_b:
            reloaded = run_b.get("kitchen")
            assert reloaded is not None
            assert reloaded.map_id == MAP_V1
            assert reloaded.position == pytest.approx((-9.0, 2.0, 0.0))

            resolved = run_b.resolve("kitchen", FRAME_WORLD)
            assert (resolved.position.x, resolved.position.y) == pytest.approx((-42.0, -2.0))

    def test_orientation_survives_a_restart(self, tmp_path: Path) -> None:
        db = str(tmp_path / "locations.db")
        heading = 2.5
        q = yaw_quat(heading)

        store_a, run_a = self._store(db, RUN_A, world_to_map())
        with store_a:
            run_a.save("kitchen", (1.0, 2.0, 0.0, q.x, q.y, q.z, q.w), frame_id=FRAME_WORLD)

        store_b, run_b = self._store(db, RUN_B, world_to_map())
        with store_b:
            reloaded = run_b.get("kitchen")
            assert reloaded is not None
            assert reloaded.yaw == pytest.approx(heading)

    def test_run_local_location_does_not_survive_a_restart(self, tmp_path: Path) -> None:
        """It was honestly labelled session-only when saved, and it stays that way."""
        db = str(tmp_path / "locations.db")

        with SqliteStore(path=db) as store:
            LocationStore(store, run_id=RUN_A).save("kitchen", (1.0, 2.0, 0.0))

        with SqliteStore(path=db) as store:
            run_b = LocationStore(store, run_id=RUN_B)
            assert run_b.get("kitchen") is None
            with pytest.raises(RunLocalError):
                run_b.resolve("kitchen")

    def test_promotion_makes_a_session_tag_durable(self, tmp_path: Path) -> None:
        """The full unanchored -> promoted -> next-run path, which is what happens when
        a user tags something before relocalization has converged."""
        db = str(tmp_path / "locations.db")

        with SqliteStore(path=db) as store:
            run_a = LocationStore(store, run_id=RUN_A)
            run_a.save("kitchen", (1.0, 2.0, 0.0))

            run_a.set_map_lineage((MAP_V1,))
            run_a._tf = world_to_map(x=10.0)
            run_a._status = lambda: RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=0.9, map_id=MAP_V1
            )
            assert len(run_a.promote_pending()) == 1

        store_b, run_b = self._store(db, RUN_B, world_to_map(x=-40.0, y=7.0, theta=math.pi / 2))
        with store_b:
            resolved = run_b.resolve("kitchen", FRAME_WORLD)
            assert (resolved.position.x, resolved.position.y) == pytest.approx((-42.0, -2.0))


class TestCrossRun:
    def test_anchored_location_resolves_in_a_later_run(self, session: Store) -> None:
        """The headline feature, in miniature: save in run A, resolve in run B where
        the robot booted somewhere else entirely."""
        run_a = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=10.0),
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time()),
        )
        run_a.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_WORLD)

        run_b_tf = world_to_map(x=-40.0, y=7.0, theta=math.pi / 2)
        run_b = LocationStore(
            session,
            run_id=RUN_B,
            map_lineage=(MAP_V1,),
            tf=run_b_tf,
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time()),
        )
        resolved = run_b.resolve("kitchen", FRAME_WORLD)

        stored = run_b.get("kitchen")
        assert stored is not None
        assert stored.position == pytest.approx((-9.0, 2.0, 0.0))

        assert (resolved.position.x, resolved.position.y) == pytest.approx((-42.0, -2.0))

        reanchored, frame, anchored = run_b._anchor(
            (resolved.position.x, resolved.position.y, resolved.position.z),
            FRAME_WORLD,
            time.time(),
        )
        assert anchored and frame == FRAME_MAP
        assert reanchored[:3] == pytest.approx((-9.0, 2.0, 0.0))

    def test_run_local_location_refuses_in_a_later_run(self, session: Store) -> None:
        LocationStore(session, run_id=RUN_A).save("kitchen", (1.0, 2.0, 0.0))
        run_b = LocationStore(session, run_id=RUN_B)

        with pytest.raises(RunLocalError, match="restarted"):
            run_b.resolve("kitchen")

    def test_run_local_still_works_within_its_own_run(self, unanchored: LocationStore) -> None:
        unanchored.save("kitchen", (1.0, 2.0, 0.0))
        resolved = unanchored.resolve("kitchen", FRAME_WORLD)
        assert (resolved.position.x, resolved.position.y) == pytest.approx((1.0, 2.0))


class TestResolveFailures:
    def test_unknown_name(self, anchored: LocationStore) -> None:
        with pytest.raises(UnknownLocationError):
            anchored.resolve("nowhere")

    def test_not_relocalized_without_tf(self, session: Store) -> None:
        anchored_store = LocationStore(
            session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map()
        )
        anchored_store.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP)

        cold = LocationStore(session, run_id=RUN_B, map_lineage=(MAP_V1,), tf=None)
        with pytest.raises(NotRelocalizedError):
            cold.resolve("kitchen", FRAME_WORLD)

    def test_not_relocalized_when_edge_missing(self, session: Store) -> None:
        anchored_store = LocationStore(
            session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map()
        )
        anchored_store.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP)

        cold = LocationStore(session, run_id=RUN_B, map_lineage=(MAP_V1,), tf=FakeTF())
        with pytest.raises(NotRelocalizedError, match="recognised where it is"):
            cold.resolve("kitchen", FRAME_WORLD)

    def test_status_reports_no_fix(self, session: Store) -> None:
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(),
            status=lambda: RelocStatus(relocalized=False),
        )
        store.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP)

        with pytest.raises(NotRelocalizedError):
            store.resolve("kitchen", FRAME_WORLD)

    def test_stale_anchor(self, session: Store) -> None:
        """tf still looks fresh — the module republishes its last good transform —
        so staleness can only come from the status channel."""
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(),
            status=lambda: RelocStatus(
                relocalized=True,
                last_success_ts=time.time() - 300.0,
                consecutive_failures=12,
            ),
            stale_after=30.0,
        )
        store.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP)

        with pytest.raises(StaleAnchorError, match="no longer trustworthy"):
            store.resolve("kitchen", FRAME_WORLD)


class TestMapLineage:
    def test_foreign_map_rejected(self, session: Store) -> None:
        LocationStore(session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map()).save(
            "kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP
        )

        other = LocationStore(session, run_id=RUN_B, map_lineage=("unrelated_map",))
        with pytest.raises(MapMismatchError, match="lineage"):
            other.resolve("kitchen")

    def test_ancestor_map_rejected_until_reanchoring(self, session: Store) -> None:
        """MAP_V2 extends MAP_V1, but nothing guarantees the rebuild kept the same
        origin — so an ancestor pose is confidently wrong rather than approximately
        right. Strict until M4 re-anchoring can verify it."""
        LocationStore(session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map()).save(
            "kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP
        )

        rebuilt = LocationStore(
            session,
            run_id=RUN_B,
            map_lineage=(MAP_V2, MAP_V1),
            tf=world_to_map(),
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time()),
        )
        with pytest.raises(MapMismatchError, match="ancestor"):
            rebuilt.resolve("kitchen", FRAME_WORLD)

    def test_same_name_under_two_maps_stays_separate(self, session: Store) -> None:
        """Two buildings both have a kitchen. Saving one must not rewrite the other."""
        building_a = LocationStore(session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map())
        first = building_a.save("kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP)

        building_b = LocationStore(
            session, run_id=RUN_B, map_lineage=("other_building",), tf=world_to_map()
        )
        second = building_b.save("kitchen", (50.0, 60.0, 0.0), frame_id=FRAME_MAP)

        assert second.location_id != first.location_id

        a_kitchen = building_a.get("kitchen")
        b_kitchen = building_b.get("kitchen")
        assert a_kitchen is not None and b_kitchen is not None
        assert a_kitchen.position == pytest.approx((1.0, 2.0, 0.0))
        assert b_kitchen.position == pytest.approx((50.0, 60.0, 0.0))

        assert len(building_a.history("kitchen")) == 1
        assert len(building_b.history("kitchen")) == 1

    def test_list_all_is_scoped_to_current_map(self, session: Store) -> None:
        LocationStore(session, run_id=RUN_A, map_lineage=(MAP_V1,), tf=world_to_map()).save(
            "kitchen", (1.0, 2.0, 0.0), frame_id=FRAME_MAP
        )
        building_b = LocationStore(
            session, run_id=RUN_B, map_lineage=("other_building",), tf=world_to_map()
        )
        building_b.save("garage", (5.0, 5.0, 0.0), frame_id=FRAME_MAP)

        assert {loc.name for loc in building_b.list_all()} == {"garage"}


class TestAnchorTrust:
    """The write path must be gated on relocalization health, not just on the tf edge
    existing. The module republishes its last good transform restamped to now, so a
    dead registration still presents a fresh-looking edge — and a bad anchor is
    written permanently, unlike a bad read."""

    def test_save_falls_back_to_run_local_when_not_relocalized(self, session: Store) -> None:
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=10.0),
            status=lambda: RelocStatus(relocalized=False),
        )
        loc = store.save("kitchen", (1.0, 2.0, 0.0))

        assert loc.anchored is False
        assert loc.frame_id == FRAME_WORLD
        assert loc.position == pytest.approx((1.0, 2.0, 0.0))

    def test_save_falls_back_to_run_local_when_stale(self, session: Store) -> None:
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=10.0),
            status=lambda: RelocStatus(
                relocalized=True, last_success_ts=time.time() - 300.0, fitness=0.9
            ),
            stale_after=30.0,
        )
        assert store.save("kitchen", (1.0, 2.0, 0.0)).anchored is False

    def test_save_anchors_at_the_publish_gate(self, session: Store) -> None:
        """An explicit save is one deliberate, redoable record — it does not need the
        stricter floor that bulk auto-promotion does."""
        store = LocationStore(
            session,
            run_id=RUN_A,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=10.0),
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time(), fitness=0.5),
            promote_min_fitness=0.6,
        )
        assert store.save("kitchen", (1.0, 2.0, 0.0)).anchored is True

    def test_promote_refuses_a_marginal_fix(self, session: Store) -> None:
        """0.5 clears RelocalizationModule's 0.45 publish gate but not the promote
        floor — one marginal first fix would otherwise stamp every session tag at once."""
        fitness = 0.5
        store = LocationStore(
            session,
            run_id=RUN_A,
            tf=None,
            status=lambda: RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=fitness
            ),
            promote_min_fitness=0.6,
        )
        loc = store.save("kitchen", (1.0, 2.0, 0.0))
        assert loc.anchored is False

        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)
        with pytest.raises(StaleAnchorError, match="anchoring floor"):
            store.promote(loc)

    def test_promote_accepts_a_confident_fix(self, session: Store) -> None:
        store = LocationStore(
            session,
            run_id=RUN_A,
            tf=None,
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time(), fitness=0.9),
            promote_min_fitness=0.6,
        )
        loc = store.save("kitchen", (1.0, 2.0, 0.0))

        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)
        assert store.promote(loc).anchored is True

    def test_promote_pending_skips_rather_than_throws(self, session: Store) -> None:
        """A marginal fix must leave run-local tags alone, not crash the sweep."""
        store = LocationStore(
            session,
            run_id=RUN_A,
            tf=None,
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time(), fitness=0.5),
            promote_min_fitness=0.6,
        )
        store.save("kitchen", (1.0, 2.0, 0.0))
        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)

        assert store.promote_pending() == []
        assert all(not loc.anchored for loc in store.list_all())


class TestPromotion:
    def test_promote_reanchors_into_map(self, session: Store) -> None:
        store = LocationStore(session, run_id=RUN_A)
        loc = store.save("kitchen", (1.0, 2.0, 0.0))
        assert loc.anchored is False

        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)
        promoted = store.promote(loc)

        assert promoted.anchored is True
        assert promoted.frame_id == FRAME_MAP
        assert promoted.position == pytest.approx((-9.0, 2.0, 0.0))

    def test_promote_appends_a_version(self, session: Store) -> None:
        store = LocationStore(session, run_id=RUN_A)
        loc = store.save("kitchen", (1.0, 2.0, 0.0))
        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)
        store.promote(loc)

        history = store.history("kitchen")
        assert len(history) == 2
        assert history[0].anchored is True
        assert history[1].anchored is False

    def test_promote_pending_sweeps_the_run(self, session: Store) -> None:
        store = LocationStore(session, run_id=RUN_A)
        store.save("kitchen", (1.0, 2.0, 0.0))
        store.save("office", (3.0, 4.0, 0.0))

        store._tf = world_to_map(x=10.0)
        store._map_lineage = (MAP_V1,)
        promoted = store.promote_pending()

        assert len(promoted) == 2
        assert all(loc.anchored for loc in store.list_all())

    def test_promote_without_map_raises(self, unanchored: LocationStore) -> None:
        loc = unanchored.save("kitchen", (1.0, 2.0, 0.0))
        with pytest.raises(NotRelocalizedError):
            unanchored.promote(loc)

    def test_cannot_promote_another_runs_location(self, session: Store) -> None:
        LocationStore(session, run_id=RUN_A).save("kitchen", (1.0, 2.0, 0.0))
        run_b = LocationStore(
            session,
            run_id=RUN_B,
            map_lineage=(MAP_V1,),
            tf=world_to_map(x=10.0),
            status=lambda: RelocStatus(relocalized=True, last_success_ts=time.time(), fitness=0.9),
        )

        assert run_b.get("kitchen") is None

        hidden = run_b._latest_any_scope("kitchen")
        assert hidden is not None
        with pytest.raises(RunLocalError, match="no longer exists"):
            run_b.promote(hidden)

    def test_promote_is_idempotent(self, anchored: LocationStore) -> None:
        loc = anchored.save("kitchen", (1.0, 2.0, 0.0))
        assert anchored.promote(loc) is loc
