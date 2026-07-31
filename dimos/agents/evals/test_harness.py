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

"""What the harness assumes about the code it wraps, checked without a recording.

Three assumptions, all of which fail silently when they stop holding:

* the recording subclass of ``navigate_with_text`` is still the *shipping* skill
  as far as the MCP server is concerned -- same capabilities, same lifecycle --
  so the eval measures the tool the robot runs rather than a lookalike;
* the ingested store the fixture attaches to actually exists and holds frames.
  Attaching to a name that does not exist succeeds and answers nothing, which
  reads exactly like a model that never found anything;
* the shape ``query_by_text`` answers in is the shape the retrieval read-back
  unpacks. That one is a diagnostic, so getting it wrong produces no error at
  all -- just an empty ``retrievals`` on every shard, or worse, a plausible
  coordinate read out of the wrong slot.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from dimos.agents.evals.conftest import _queryable_module, _top_retrieval
from dimos.agents.evals.contracts import RetrievalRecord
from dimos.agents.evals.ingest import assert_store_ingested
from dimos.agents.evals.modules import RecordingNavigationSkillContainer, RecordingStubNavigation
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.module import Module

#: The attributes ``@skill`` stamps onto a method and the MCP server reads back
#: (``dimos/agents/annotation.py``). Named here rather than spelled into the
#: assertion so a new one added over there shows up as a missing attribute.
SKILL_ATTRIBUTES = ("__skill__", "__skill_uses__", "__skill_lifecycle__")


def test_the_recording_skill_keeps_the_shipping_skills_metadata() -> None:
    """Re-applying ``@skill`` on the override must reproduce it exactly.

    ``RecordingNavigationSkillContainer`` overrides ``navigate_with_text`` to
    record the query, and an override does not inherit the decorator's marker
    attributes, so they are re-applied by hand. That hand-copy is what would
    silently rot: if the shipping skill gains a capability or becomes
    ``background``, the eval would keep exercising an ``instant``,
    movement-only tool -- the MCP server's capability holds would differ from
    production, and nothing in a shard would say so.
    """
    shipping = NavigationSkillContainer.navigate_with_text
    recording = RecordingNavigationSkillContainer.navigate_with_text

    assert {name: getattr(recording, name) for name in SKILL_ATTRIBUTES} == {
        name: getattr(shipping, name) for name in SKILL_ATTRIBUTES
    }
    # The docstring is the tool description the model reads, and it is inherited
    # rather than copied; assert that it is the same object, not merely equal.
    assert recording.__doc__ is shipping.__doc__


# The ingested store.


def test_a_bogus_collection_name_fails_before_anything_is_built(tmp_path: Path) -> None:
    """The misconfiguration that otherwise scores as a model failure.

    ``SpatialMemory(new_memory=False)`` reaches chroma through
    ``get_or_create_collection``, so a typo'd collection name -- or a ``db_path``
    pointing at the directory *above* the real ``chroma/`` -- yields an empty
    collection rather than an error, and every question of the sweep answers
    nothing. The guard says which name in which directory, and how to build one.
    """
    with pytest.raises(RuntimeError) as excinfo:
        assert_store_ingested(tmp_path, "go2_bigoffice_typo")

    message = str(excinfo.value)
    assert "go2_bigoffice_typo" in message
    assert str(tmp_path) in message
    assert "no_prediction" in message
    assert "dimos.agents.evals.ingest" in message


def test_an_empty_collection_is_refused_as_loudly_as_a_missing_one(tmp_path: Path) -> None:
    """A collection that exists and holds nothing answers nothing either.

    This is the shape a half-finished or clobbered ingest leaves behind, and it
    is the case a plain existence check would wave through.
    """
    import chromadb

    client = chromadb.PersistentClient(path=str(tmp_path))
    client.create_collection("go2_bigoffice", embedding_function=None)

    with pytest.raises(RuntimeError, match="is empty"):
        assert_store_ingested(tmp_path, "go2_bigoffice")


def test_a_db_path_that_is_not_a_store_is_refused_without_creating_one(tmp_path: Path) -> None:
    """Pointing one directory above the real ``chroma/`` is the common typo.

    ``PersistentClient`` would happily create the directory, so the guard checks
    first: a run that mistyped its path must not leave a plausible-looking empty
    store behind for the next one to find.
    """
    missing = tmp_path / "spatial-eval"  # the dir above, i.e. no chroma/ here
    with pytest.raises(RuntimeError, match="is not a directory"):
        assert_store_ingested(missing, "go2_bigoffice")
    assert not missing.exists()


def test_a_populated_collection_passes_and_reports_its_size(tmp_path: Path) -> None:
    import chromadb

    client = chromadb.PersistentClient(path=str(tmp_path))
    collection = client.create_collection("go2_bigoffice", embedding_function=None)
    collection.add(ids=["frame-0", "frame-1"], embeddings=[[0.1, 0.2], [0.3, 0.4]])

    assert assert_store_ingested(tmp_path, "go2_bigoffice") == 2


# The retrieval read-back.


class MemoryLikeModule(Module):
    """A blueprint-able module that answers ``query_by_text``.

    Stands in for ``SpatialMemory`` so the resolution test does not import it:
    that import costs ~4 s of CLIP and torch, which is exactly why
    ``conftest.py`` defers it.
    """

    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        return []


#: One ``query_by_text`` answer, in the shape ``SpatialVectorDB`` actually
#: returns: *one* result object per query text, with the hits packed into
#: ``metadata`` and ``distance``/``id`` belonging to the top one. Copied from a
#: live call against the ingested store, trimmed to the keys that are read.
CHROMA_SHAPE = [
    {
        "id": "frame_20260729_120747_214cba8f",
        "metadata": [
            {"pos_x": -12.835005, "pos_y": 13.495942, "pos_z": 0.306406, "rot_z": -2.107393},
            {"pos_x": -12.834929, "pos_y": 13.495897, "pos_z": 0.306402, "rot_z": -2.107982},
        ],
        "distance": 0.7412,
    }
]


def test_top_retrieval_reads_the_pose_out_of_the_first_metadata_entry() -> None:
    """The subtle part: the pose is in ``metadata[0]``, not in the result dict.

    ``_process_query_results`` enumerates chroma's per-*query* lists, so a
    one-text query yields one result whose ``metadata`` is the list of hits.
    Reading it as one-result-per-hit would silently pick the pose of the wrong
    frame the moment ``limit`` rose above 1 -- which is how
    ``navigate_with_text`` reads it too (``_get_goal_pose_from_result``).
    """
    record = _top_retrieval("elevator door", CHROMA_SHAPE)
    assert record == RetrievalRecord(
        query="elevator door", x=-12.835005, y=13.495942, distance=0.7412
    )


def test_top_retrieval_reports_nothing_rather_than_guessing() -> None:
    """Every shape it does not recognize is ``None``, not a coordinate.

    This is a diagnostic, so an unfamiliar answer has to be recorded as "nothing
    retrieved" -- a fabricated or mis-sloted coordinate would be worse than a
    hole, because a reader would take it for a measurement.
    """
    assert _top_retrieval("q", []) is None
    assert _top_retrieval("q", None) is None
    assert _top_retrieval("q", [{"id": "frame-0", "distance": 0.4}]) is None
    assert _top_retrieval("q", [{"metadata": [], "distance": 0.4}]) is None
    assert _top_retrieval("q", [{"metadata": [{"pos_y": 1.0}], "distance": 0.4}]) is None
    assert _top_retrieval("q", ["not a mapping"]) is None


def test_top_retrieval_accepts_a_bare_metadata_mapping() -> None:
    """A memory that hands back one metadata dict instead of a list of them.

    Not the shipping shape, but it is one line of ``_process_query_results`` away
    (``metadatas`` is keyed differently by ``get`` than by ``query``), and the
    read-back should survive a substitute memory rather than record nothing.
    """
    record = _top_retrieval("q", [{"metadata": {"pos_x": 1.5, "pos_y": -2.5}, "distance": 0.5}])
    assert record == RetrievalRecord(query="q", x=1.5, y=-2.5, distance=0.5)


def test_top_retrieval_treats_a_missing_distance_the_way_the_shipping_skill_does() -> None:
    """``1.0``, i.e. "no usable similarity" -- not ``0.0``, i.e. a perfect match.

    ``_get_goal_pose_from_result`` computes ``1.0 - (result.get("distance") or 1)``,
    so a hole reads as similarity 0 there and has to read the same here.
    """
    record = _top_retrieval("q", [{"metadata": [{"pos_x": 1.0, "pos_y": 2.0}]}])
    assert record is not None
    assert record.distance == 1.0


def test_the_queryable_module_is_resolved_from_whatever_blueprint_is_in_use() -> None:
    """The memory module is a fixture parameter, so it cannot be hard-coded.

    The hermetic lanes substitute their own memory blueprint. One that answers
    ``query_by_text`` is found; one that does not is reported as ``None`` so the
    read-back skips with a logged note instead of raising -- a substitute without
    the RPC is a legitimate configuration, not a broken run.
    """
    assert _queryable_module(MemoryLikeModule.blueprint()) is MemoryLikeModule
    assert _queryable_module(RecordingStubNavigation.blueprint()) is None
