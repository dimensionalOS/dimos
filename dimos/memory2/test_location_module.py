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

"""M2 wiring tests: spec compliance, status reporting, and agent-facing copy.

``Module.__init__`` binds transports and an event loop, so these do not construct
modules normally — the store-level behaviour is already covered by
``test_locations.py``, and what is worth testing here is the glue: does the module
satisfy the Spec the coordinator will match it against, does the status channel
report what ``LocationStore`` needs, and does each typed failure reach the user as
something they can act on.
"""

from __future__ import annotations

import time
from typing import Any, get_type_hints

import pytest

from dimos.agents.skills.navigation import _explain, _normalized, _planar_distance
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.mapping.relocalization.spec import RelocalizationSpec
from dimos.memory2.location_module import LocationMemory
from dimos.memory2.location_spec import LocationMemorySpec
from dimos.memory2.locations import (
    LocationStore,
    MapMismatchError,
    NotRelocalizedError,
    RelocStatus,
    RunLocalError,
    StaleAnchorError,
)
from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.spec.utils import spec_annotation_compliance, spec_structural_compliance


def _bare(cls: type) -> Any:
    """Instance without ``__init__`` — module construction binds real transports."""
    return object.__new__(cls)


class TestSpecCompliance:
    """The coordinator matches implementations to Specs structurally *and* by
    annotation. Both checks resolve annotations at runtime, so a Spec whose types
    are imported under TYPE_CHECKING fails at wiring time rather than import time."""

    def test_location_memory_satisfies_spec(self) -> None:
        assert spec_structural_compliance(LocationMemory, LocationMemorySpec)
        assert spec_annotation_compliance(LocationMemory, LocationMemorySpec)

    def test_relocalization_module_satisfies_spec(self) -> None:
        assert spec_structural_compliance(RelocalizationModule, RelocalizationSpec)
        assert spec_annotation_compliance(RelocalizationModule, RelocalizationSpec)

    def test_spec_annotations_resolve_at_runtime(self) -> None:
        """Regression: SavedLocation/PoseStamped were TYPE_CHECKING-only here, which
        made annotation matching raise NameError during wiring."""
        hints = get_type_hints(LocationMemorySpec)
        assert (
            get_type_hints(LocationMemorySpec.save_location)["return"].__name__ == "SavedLocation"
        )
        assert hints is not None

    def test_module_annotations_resolve_at_runtime(self) -> None:
        """The blueprint layer calls get_type_hints() on every module to discover
        ports and Spec-typed refs, so `_reloc: RelocalizationSpec | None` has to be
        importable at runtime too."""
        hints = get_type_hints(LocationMemory)
        assert "_reloc" in hints
        assert "odom" in hints and "color_image" in hints


class TestRelocalizationStatus:
    """`LocationStore` trusts nothing but this channel — the tf edge always looks
    fresh because the module republishes its last good transform on a timer."""

    def _module(self, status: RelocStatus) -> Any:
        module = _bare(RelocalizationModule)
        module._status = status
        return module

    def test_reports_current_status(self) -> None:
        status = RelocStatus(relocalized=True, fitness=0.8, map_id="abc")
        assert self._module(status).reloc_status() is status

    def test_failure_increments_counter(self) -> None:
        module = self._module(RelocStatus(map_id="abc"))
        module._record_failure(fitness=0.1)
        module._record_failure(fitness=0.2)

        status = module.reloc_status()
        assert status.consecutive_failures == 2
        assert status.map_id == "abc"

    def test_failure_preserves_last_success(self) -> None:
        """Staleness is measured from the last *success*, so a failure must not
        touch it — otherwise a failing registration would look freshly fixed."""
        succeeded_at = time.time() - 120.0
        module = self._module(
            RelocStatus(relocalized=True, last_success_ts=succeeded_at, map_id="abc")
        )
        module._record_failure()

        status = module.reloc_status()
        assert status.relocalized is True  # we did fix at some point this run
        assert status.last_success_ts == succeeded_at
        assert status.consecutive_failures == 1

    def test_failure_before_any_success_stays_unrelocalized(self) -> None:
        module = self._module(RelocStatus(map_id="abc"))
        module._record_failure()
        assert module.reloc_status().relocalized is False

    def test_stale_status_is_visible_to_the_store(self) -> None:
        """End-to-end on the contract that matters: a failing registration must make
        the store refuse, even though tf would still hand back a transform."""
        module = self._module(
            RelocStatus(relocalized=True, last_success_ts=time.time() - 600.0, map_id="abc")
        )
        with MemoryStore() as store:
            locations = LocationStore(
                store,
                run_id="run_a",
                map_lineage=("abc",),
                status=module.reloc_status,
                stale_after=30.0,
            )
            locations.save("kitchen", (1.0, 2.0, 0.0), frame_id="map")
            with pytest.raises(StaleAnchorError):
                locations.resolve("kitchen", "world")


class TestLocationMemoryStatusBridge:
    def _module(self, reloc: Any) -> Any:
        module = _bare(LocationMemory)
        module._reloc = reloc
        return module

    def test_no_relocalization_means_no_fix(self) -> None:
        """A blueprint without relocalization is a valid degraded configuration —
        every location is run-local — not an error."""
        assert self._module(None)._status() == RelocStatus()

    def test_unreachable_relocalization_degrades(self) -> None:
        class Broken:
            def reloc_status(self) -> RelocStatus:
                raise RuntimeError("rpc down")

        assert self._module(Broken())._status().relocalized is False

    def test_passes_through_a_live_status(self) -> None:
        live = RelocStatus(relocalized=True, fitness=0.9, map_id="abc")

        class Live:
            def reloc_status(self) -> RelocStatus:
                return live

        assert self._module(Live())._status() is live


class TestSyncMap:
    """`_sync_map` tracks the active map and promotes run-local tags exactly once
    per map, off a fix confident enough to stamp all of them at the same time."""

    def _module(self, store: MemoryStore, status: RelocStatus, **kwargs: Any) -> Any:
        module = _bare(LocationMemory)

        class Reloc:
            def reloc_status(self) -> RelocStatus:
                return module._reloc_status

        module._reloc_status = status
        module._reloc = Reloc()
        module._promoted_for_map = None
        module._locations = LocationStore(
            store, run_id="run_a", tf=kwargs.get("tf"), status=module._status
        )
        return module

    def test_adopts_the_active_map(self) -> None:
        with MemoryStore() as store:
            module = self._module(store, RelocStatus(relocalized=True, map_id="abc", fitness=0.9))
            module._sync_map()
            assert module._locations.map_id == "abc"

    def test_no_map_leaves_lineage_empty(self) -> None:
        with MemoryStore() as store:
            module = self._module(store, RelocStatus())
            module._sync_map()
            assert module._locations.map_id is None

    def test_promotes_run_local_tags_once(self) -> None:
        from dimos.memory2.test_locations import world_to_map

        with MemoryStore() as store:
            module = self._module(store, RelocStatus())
            module._locations.save("kitchen", (1.0, 2.0, 0.0))
            assert module._locations.get("kitchen").anchored is False

            # Relocalization converges with a confident fix.
            module._locations._tf = world_to_map(x=10.0)
            module._reloc_status = RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=0.9, map_id="abc"
            )
            module._sync_map()

            promoted = module._locations.get("kitchen")
            assert promoted.anchored is True
            assert promoted.position == pytest.approx((-9.0, 2.0, 0.0))
            assert module._promoted_for_map == "abc"

    def test_marginal_fix_does_not_consume_the_promotion(self) -> None:
        """0.5 clears the publish gate but not the promote floor. The sweep must not
        latch, or the confident fix that follows would never promote anything."""
        from dimos.memory2.test_locations import world_to_map

        with MemoryStore() as store:
            module = self._module(store, RelocStatus())
            module._locations.save("kitchen", (1.0, 2.0, 0.0))

            module._locations._tf = world_to_map(x=10.0)
            module._reloc_status = RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=0.5, map_id="abc"
            )
            module._sync_map()
            assert module._locations.get("kitchen").anchored is False
            assert module._promoted_for_map is None

            module._reloc_status = RelocStatus(
                relocalized=True, last_success_ts=time.time(), fitness=0.9, map_id="abc"
            )
            module._sync_map()
            assert module._locations.get("kitchen").anchored is True


class TestSkillCopy:
    """Each typed failure means a different thing to the user and has a different
    remedy, so each has to read differently."""

    def test_every_error_gets_its_own_wording(self) -> None:
        messages = {
            _explain("kitchen", NotRelocalizedError("x")),
            _explain("kitchen", StaleAnchorError("x")),
            _explain("kitchen", RunLocalError("x")),
            _explain("kitchen", MapMismatchError("x")),
        }
        assert len(messages) == 4

    def test_not_relocalized_reads_as_temporary(self) -> None:
        message = _explain("kitchen", NotRelocalizedError("x")).lower()
        assert "moment" in message or "yet" in message

    def test_run_local_explains_the_restart(self) -> None:
        assert "restarted" in _explain("kitchen", RunLocalError("x")).lower()

    def test_copy_names_the_place(self) -> None:
        for error in (
            NotRelocalizedError("x"),
            StaleAnchorError("x"),
            RunLocalError("x"),
            MapMismatchError("x"),
        ):
            assert "kitchen" in _explain("kitchen", error)


class TestReissueHelpers:
    def test_planar_distance_ignores_z(self) -> None:
        a = PoseStamped(position=(0.0, 0.0, 0.0), orientation=(0, 0, 0, 1))
        b = PoseStamped(position=(3.0, 4.0, 99.0), orientation=(0, 0, 0, 1))
        assert _planar_distance(a, b) == pytest.approx(5.0)

    def test_normalized_matches_the_store(self) -> None:
        assert _normalized("  The  Kitchen ") == "the kitchen"

    def test_normalized_tolerates_an_empty_name(self) -> None:
        """Used to decide "no such location" vs "unresolvable", so it must not raise
        on input the store would reject."""
        assert _normalized("   ") == ""
