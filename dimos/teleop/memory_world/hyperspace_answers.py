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

"""Hyperspace's answers, drawn in the memory world.

NOT the `hyperspace_answers.py` that used to sit here on `jeff/feat/memworld_clip`; that
one owned Navigate and the whole /ask route and is `answers.py` now. This file does one
thing: it listens to what the Hyperspace module publishes and turns it into the result
the viewer already knows how to draw.

**Why a subscription rather than a call.** The agent asks Hyperspace directly -- its
`start_item_query` / `start_heatmap_query` / `start_area_query` are skills on the MCP
server, beside this module's `navigate_to_place` -- and Hyperspace publishes every
answer on `found` whether or not anyone called it. So the world lights up because the
answer happened, not because MemoryWorld asked for it, and nothing here has to hold a
handle to another module or re-run the query to find out what it said.

**What changes in the wording, and why it is not cosmetic.** The SigLIP path answers with
the camera pose of a matching frame: where a thing was SEEN FROM, because one vector per
image genuinely cannot say more. Hyperspace measures the object: `centre` is the thing's
own position in the world frame, from OWLv2's box and that frame's depth. So an answer
here says where the thing IS, and the two must not be described the same way -- the
whole point of this branch is the difference.

Three limits kept in the vocabulary, from hyperspace's own author:
- A heatmap or area answer has **no extent**, deliberately: nothing measured a size, and
  a box drawn around it would be invented. Those fall back to the configured radius.
- `depth_m` is how far the object was from the camera that saw it, which `centre` does
  not tell you -- a box eight metres out is worth less than the same box at one metre.
- `confidence` is the detector's own calibrated scale, and only an item query has one.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

from dimos.core.module import In
from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects
from dimos.teleop.memory_world.query import ClusterSummary, HighlightPoint, MemoryQueryResult
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _label_for(found: FoundObject, query: str, index: int, kind: str) -> str:
    """One place's caption, said in the units the KIND actually measured.

    `confidence` means two different things on this stream and the field does not say
    which: on an item answer it is OWLv2's own calibrated score for a box it drew, and on
    a heatmap or area answer it is the SCORE OF A CELL -- a different quantity on a
    different scale, comparable only within its kind. Printing a cell score as
    "0.42 confidence" would have the reader trust a number nothing measured, so the word
    changes with the kind.

    The distance is named when there is one, because the centre cannot imply it and a
    reader has no other way to tell a confident near thing from a confident far one. The
    patch path chose no photograph, so it has none.
    """
    parts = [f"{query[:80]} #{index + 1}"]
    if found.confidence:
        parts.append(
            f"{found.confidence:.2f}" if kind == "item" else f"score {found.confidence:.2f}"
        )
    if found.depth_m:
        parts.append(f"{found.depth_m:.1f} m out")
    return "  ".join(parts)


def _one_per_place(objects: list[FoundObject]) -> list[FoundObject]:
    """One entry per distinct thing, keeping the look that saw it best.

    Two answers sharing a `place_id` are two looks at ONE object -- hyperspace says so
    itself, and that is what the field is for. Drawing both puts two markers on one chair
    and, worse, makes the numbering lie: the sentence would say "2 places" while
    `navigate_to_place(2)` walked to the second LOOK at the first chair. The text, the
    markers and the number the agent passes have to mean the same thing.

    A `place_id` of 0 means unplaced -- the heatmap and area paths do not assign one --
    so those are never merged with each other.
    """
    best: dict[Any, FoundObject] = {}
    order: list[Any] = []
    for index, obj in enumerate(objects):
        key = obj.place_id or ("unplaced", index)
        if key not in best:
            best[key] = obj
            order.append(key)
        elif _is_better_look(obj, best[key]):
            best[key] = obj
    return [best[key] for key in order]


def _is_better_look(candidate: FoundObject, held: FoundObject) -> bool:
    """Whether *candidate* is the look worth keeping for a place.

    VIEWS first, confidence only to break a tie. `views > 1` is the signal that a box was
    REFINED from more than one look, and a refined box is sharper than either look that
    made it -- so ranking on confidence alone would keep the better score and throw away
    the better geometry, which is precisely the case this function exists for.
    """
    if int(candidate.views) != int(held.views):
        return int(candidate.views) > int(held.views)
    return float(candidate.confidence) > float(held.confidence)


class HyperspaceAnswers:
    """Draw what Hyperspace found, without having asked it.

    Needs, from the module: ``config``, ``_publish_query_result``, ``_markers_near`` and
    ``register_disposable``.
    """

    config: Any
    _clients_lock: threading.Lock
    _last_answer: tuple[Any | None, str | None]

    # Declared HERE rather than on `MemoryWorldModule`: `get_type_hints` resolves the
    # whole MRO, so the framework sees this exactly as if it were on the module -- and
    # `module.py` is at the repository's 75 KB ceiling with a couple of hundred bytes to
    # spare, so a stream that belongs to this file is also the only place it fits.
    hyperspace_found: In[FoundObjects]

    if TYPE_CHECKING:

        def _publish_query_result(self, result: MemoryQueryResult) -> str: ...
        def _markers_near(self, positions: list[Any]) -> list[int]: ...
        def _query_is_current(self, query_id: str) -> bool: ...
        def register_disposable(self, disposable: Any) -> None: ...

    def _watch_hyperspace(self) -> None:
        """Start drawing Hyperspace's answers, if there is a Hyperspace to hear.

        `memory-world-module` and `memory-world-agent` run this same module with no
        Hyperspace in the blueprint, which leaves this stream with no transport. Reaching
        for it anyway raises `AttributeError: 'NoneType' object has no attribute
        'subscribe'` out of `stream.py` -- measured, not guessed -- so every start of the
        siglip blueprints would print a traceback for a module that is working correctly.
        An unconnected stream is the normal case for two of the three blueprints, so it
        is said once at debug and skipped.
        """
        stream = getattr(self, "hyperspace_found", None)
        if stream is None or getattr(stream, "transport", None) is None:
            logger.debug("no hyperspace connected; the world draws its own answers only")
            return
        self.register_disposable(stream.observable().subscribe(self._on_hyperspace_found))
        # Read by `_index_status`: with Hyperspace answering, the ask box must not wait
        # for a siglip index this recording may never hold.
        self._hyperspace_live = True
        logger.info("memory world: drawing whatever hyperspace answers")

    def _on_hyperspace_found(self, found: FoundObjects) -> None:
        """One hyperspace answer, in the shape the viewer already draws."""
        try:
            self._draw_hyperspace_answer(found)
        except Exception:
            # A malformed answer must not take the subscription down with it: the next
            # question would then light nothing and the viewer would look merely idle.
            logger.exception("could not draw the hyperspace answer for %r", found.query)

    def _draw_hyperspace_answer(self, found: FoundObjects) -> None:
        query = found.query or "that"
        # `timings` carries the detector's own split, including `passes` -- the number of
        # forward passes the answer actually cost. Logged because the cost of an answer is
        # not visible from the answer: two queries can return one place each and differ
        # several-fold in work, and without this the only way to tell "more work" from
        # "dearer work" is inside hyperspace's own loop.
        if found.timings:
            logger.info(
                "hyperspace %s %r: %s",
                found.kind or "item",
                query,
                "  ".join(f"{name}={value:.0f}" for name, value in sorted(found.timings.items())),
            )
        # Read BEFORE `confidence` or `extent`: both mean different things per kind, and
        # neither field says which. It also rides out on the answer, because `engine`
        # alone no longer says what scale the scores are on.
        kind = found.kind or "item"
        objects = list(found.objects)
        if not objects:
            # Refused and empty are different facts, and the one an agent most needs
            # told: a detector that refused every episode looked at the pictures and
            # declined, where an empty answer searched and found nothing.
            answer = (
                f"Nothing matching {query!r} was found"
                if not found.refused
                else f"Nothing matching {query!r}: the detector refused {found.refused} episode(s)"
            )
            self._publish_query_result(
                MemoryQueryResult(
                    engine="hyperspace",
                    kind=kind,
                    query_text=query,
                    clusters=[],
                    answer=answer,
                    focus_point=None,
                    points=[],
                    observation_ids=[],
                )
            )
            return

        objects = _one_per_place(objects)
        fallback_radius = float(self.config.place_radius_m)
        clusters: list[ClusterSummary] = []
        points: list[HighlightPoint] = []
        for index, obj in enumerate(objects):
            centre = tuple(float(value) for value in obj.centre)
            # Half the longest side, so the marker covers the thing rather than a corner
            # of it. Zero extent is a heatmap or area answer -- nothing measured a size,
            # so the configured radius stands in and is not presented as a measurement.
            measured = max(float(value) for value in obj.extent) if any(obj.extent) else 0.0
            radius = measured / 2.0 if measured else fallback_radius
            clusters.append(
                ClusterSummary(
                    index=index,
                    centre=centre,
                    radius=radius,
                    score=float(obj.confidence),
                    peak=float(obj.confidence),
                    # `views` is how many viewpoints agreed on this place, which is a
                    # real count here -- unlike the siglip path, where clustering throws
                    # the near-identical frames away and there is nothing left to count.
                    n_views=int(obj.views),
                    n_evidence=1 if obj.image is not None else 0,
                    label=_label_for(obj, query, index, kind),
                )
            )
            points.append(
                HighlightPoint(
                    position=centre,
                    label=_label_for(obj, query, index, kind),
                    radius=radius if measured else None,
                )
            )

        strongest = max(objects, key=lambda obj: float(obj.confidence))
        answer = self._hyperspace_sentence(query, objects, found.refused)
        query_id = self._publish_query_result(
            MemoryQueryResult(
                engine="hyperspace",
                kind=kind,
                query_text=query,
                clusters=clusters,
                answer=answer,
                focus_point=tuple(float(value) for value in strongest.centre),
                points=points,
                observation_ids=self._markers_near([point.position for point in points]),
            )
        )
        # `_publish_query_result` CLEARS `_last_answer`, and whoever published is
        # expected to set it again -- `visual_answers` does. Without this the answer
        # draws and `navigate_to_place` has nothing to walk to: the Navigate button goes
        # inert and /answer describes nothing, which is the failure mode this package has
        # already had once. The same-answer check is `visual_answers`' too: a slow query
        # can land after a newer one replaced it, and writing unconditionally would put
        # these places behind that newer answer.
        with self._clients_lock:
            if self._query_is_current(query_id):
                self._last_answer = (
                    SimpleNamespace(
                        clusters=clusters,
                        text=answer,
                        frame=found.frame or self.config.world_frame,
                        stats={"places": len(points), "refused": found.refused},
                        seconds=found.ms / 1000.0,
                    ),
                    query_id,
                )

    @staticmethod
    def _hyperspace_sentence(query: str, objects: list[FoundObject], refused: int) -> str:
        """What was found, said as a location rather than as a vantage point.

        "Found X in N places it was seen from" is the siglip path's sentence and would be
        wrong here: these are the things themselves. Counts the places actually drawn,
        which `_one_per_place` has already made one per thing.
        """
        count = len(objects)
        noun = "place" if count == 1 else "places"
        best = max(float(obj.confidence) for obj in objects)
        sentence = f"Found {query} at {count} {noun} in the world"
        if best:
            sentence += f", best {best:.2f}"
        if refused:
            sentence += f" ({refused} episode(s) the detector refused)"
        return sentence
