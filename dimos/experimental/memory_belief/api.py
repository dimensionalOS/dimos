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

"""One query entry point, because a round trip costs far more than a query.

Splitting a question across six tool calls wraps milliseconds of work in seconds
of inference, and gives the model six chances to go astray instead of one.

The store already solved this: its filters are frozen dataclasses composed
lazily, and because they are data rather than code the same algebra can be driven
from JSON. So this is a front end onto the existing filter chain, not a second
query engine. **Filters narrow; projections compute** -- that split is what lets
a projection be added without adding a tool.

Importing this module pulls in nothing heavier than the store. Reaching it
through ``skills.py`` does pull torch, by way of ``MemoryModule``; that is the
memory module base's dependency, not this layer's.

Every answer carries how much to trust it, because the observed failure was not a
wrong tool but a list returned with nothing to choose on.
"""

from __future__ import annotations

import math
import statistics
from typing import TYPE_CHECKING, Any

from dimos.experimental.memory_belief.entity import COHERENT
from dimos.experimental.memory_belief.write import derived_stream

if TYPE_CHECKING:
    from collections.abc import Sequence

#: Read verbatim into the agent's tool description, so a name listed here that
#: cannot succeed is an invitation to a failing query. One value: an entity is a
#: thing, and a question about what is where is a question about things. Raw
#: sightings are still stored -- they are what entities are folded from -- but
#: answering from them would report one chair seen three hundred times as three
#: hundred chairs.
SELECTS = ("entities",)
#: One projection: where the things are. Counting, timelines and distances were
#: removed rather than left half-served -- each needed a stream this layer stopped
#: producing, and a projection that always returns nothing is worse than an
#: absent one, because a caller reads it as an answer.
PROJECTIONS = ("locate",)

#: How many rows one question brings back when it does not say. The bound has to
#: live in the stream chain: capping the projection instead let a broad query
#: read every matching row out of the store and then throw most of them away.
DEFAULT_LIMIT = 50

#: Streams backing each ``select``. Raw streams are queried directly; the rest
#: are materialised views built by the producers in this package.
#: Imported lazily and by path because the producers pull in numpy and scipy,
#: and a query-only process should not pay for them to answer about a stream
#: that may not exist.
_STREAM_FOR = {
    "entities": ("belief_entity", "dimos.experimental.memory_belief.entity", "Entity"),
}


class QueryError(ValueError):
    """A query that cannot be built. Distinct from one that finds nothing."""


class MissingStreamError(LookupError):
    """The stream backing a ``select`` is not in this store.

    Separate from :class:`QueryError` because the two need opposite answers: a
    malformed query is the caller's to fix, while an absent stream is ordinary
    ignorance and belongs in an envelope as NEVER_COVERED. A robot that just
    booted hits this on its first question, and raising there would turn "I have
    not seen anything yet" into a traceback.
    """

    def __init__(self, select: str, stream_name: str) -> None:
        super().__init__(
            f"select {select!r} needs stream {stream_name!r}, which this store has none of"
        )
        self.select = select
        self.stream_name = stream_name


def _open(store: Any, select: str) -> Any:
    if select in _STREAM_FOR:
        name, module_path, type_name = _STREAM_FOR[select]
        if name not in store.list_streams():
            raise MissingStreamError(select, name)
        module = __import__(module_path, fromlist=[type_name])
        return derived_stream(store, name, getattr(module, type_name))
    # A projection passed as a select. The model confused the two, so the error
    # says which field the value belongs in rather than only listing what was
    # expected.
    if select in PROJECTIONS:
        raise QueryError(
            f'{select!r} is a projection, not a stream: pass it as "project", '
            f'and choose a "select" from {SELECTS}'
        )
    raise QueryError(f"unknown select {select!r}; expected one of {SELECTS}")


def _before_asking_time(stream: Any, as_of: float) -> Any:
    """Bound a stream at the asking time.

    One function rather than one expression per call site: the diagnostic
    reopened the stream and left this off, so it went on counting rows the
    answer beside it could not see.
    """
    return stream.before(as_of + 1e-9)


def _payload(observation: Any) -> Any:
    return getattr(observation, "data", observation)


def _check_shape(where: Any) -> None:
    """Reject a malformed ``where`` as a fixable error, not an AttributeError.

    Runs before anything reads a clause. A model that sends ``["chair"]``
    instead of ``[{"op": "label", ...}]`` should get a message saying so and
    retry; a traceback out of the middle of validation tells it nothing and
    ends the turn.
    """
    if isinstance(where, (str, bytes)) or not isinstance(where, (list, tuple)):
        raise QueryError(
            f"where must be a list of clause objects, got {type(where).__name__}; "
            'example: [{"op": "label", "value": "chair"}]'
        )
    for clause in where:
        if not isinstance(clause, dict):
            raise QueryError(
                f"each where clause must be an object, got {type(clause).__name__}: {clause!r}"
            )


def _apply(stream: Any, where: Sequence[dict[str, Any]]) -> Any:
    """Build the filter chain. Composition happens here, not across turns.

    Unknown operators raise rather than being skipped: a filter silently dropped
    widens the result set, and a caller reading the answer has no way to tell
    that the constraint it asked for was never applied.
    """
    for clause in where or ():
        op = clause.get("op")
        if op == "label":
            stream = stream.tags(label=clause["value"])
        elif op == "time_range":
            stream = stream.time_range(float(clause["t1"]), float(clause["t2"]))
        else:
            raise QueryError(f"unknown filter op {op!r}")
    return stream


def _position_of(payload: Any) -> tuple[float, float, float] | None:
    position = getattr(payload, "position", None)
    if position is None:
        position = getattr(payload, "target_pose", None)
    return tuple(position) if position is not None else None


def _quality(rows: Sequence[Any]) -> dict[str, Any]:
    """What a reader needs to decide whether to believe the result.

    ``deduplicated`` is reported rather than assumed. It is true only when every
    row is an entity, because a count over sightings is not a count of things,
    and the identity layer is currently too weak for the distinction to be
    academic.
    """
    payloads = [_payload(r) for r in rows]
    supports = [n for p in payloads if (n := getattr(p, "support", None)) is not None]
    coherences = [c for p in payloads if (c := getattr(p, "coherence", None)) is not None]
    positions = [xyz for p in payloads if (xyz := _position_of(p)) is not None]
    dispersion = 0.0
    if len(positions) > 1:
        centre = tuple(sum(p[i] for p in positions) / len(positions) for i in range(3))
        dispersion = statistics.median(math.dist(centre, p) for p in positions)
    return {
        "rows": len(rows),
        "support": sum(supports) if supports else None,
        "dispersion_m": round(dispersion, 3),
        "coherence": round(statistics.median(coherences), 3) if coherences else None,
        "deduplicated": bool(payloads) and all(hasattr(p, "entity_id") for p in payloads),
    }


def _limit(query: dict[str, Any]) -> int:
    """The row bound, validated, before any row is read.

    ``limit`` was previously read for truth, so ``0`` and ``-1`` both fell
    through to "no limit" -- the opposite of what either asks for. Neither is a
    sensible number of answers, so both are refused rather than reinterpreted.
    """
    if query.get("limit") is None:
        return DEFAULT_LIMIT
    try:
        limit = int(query["limit"])
    except (TypeError, ValueError) as exc:
        raise QueryError(f"limit must be a whole number, got {query['limit']!r}") from exc
    if limit < 1:
        raise QueryError(f"limit must be at least 1, got {limit}")
    return limit


def _project(kind: str, rows: Sequence[Any]) -> dict[str, Any]:
    payloads = [_payload(r) for r in rows]

    if kind == "locate":
        positioned = [(p, xyz) for p in payloads if (xyz := _position_of(p)) is not None]
        if not positioned:
            return {}
        return {
            "positions": [
                {
                    "entity_id": getattr(p, "entity_id", None),
                    "label": getattr(p, "label", None),
                    "position": [round(v, 3) for v in xyz],
                    "extent": list(getattr(p, "extent", None) or ()),
                    "support": getattr(p, "support", None),
                    "dispersion_m": round(getattr(p, "dispersion_m", 0.0) or 0.0, 3),
                    "place_ref": getattr(p, "place_ref", None),
                }
                for p, xyz in positioned
            ]
        }

    raise QueryError(f"unknown projection {kind!r}; expected one of {PROJECTIONS}")


#: Which tag a filter needs, for the filters that read one. A stream that does not
#: carry the tag cannot answer the clause, and saying so is the whole point: the
#: alternative is a filter that matches nothing and an envelope that blames the
#: robot's coverage for the caller's mistake.
_TAG_FOR_OP = {"label": "label", "in_region": "place_ref"}


def _tags_of(store: Any, select: str) -> set[str]:
    """The tag keys this stream's observations actually carry.

    Read from the data rather than declared in a table here, so a stream that
    gains a tag does not need this module edited to admit it.
    """
    try:
        first = _open(store, select).first()
    except (LookupError, KeyError, TypeError, QueryError):
        return set()
    return set(getattr(first, "tags", None) or ())


def _place_refs(store: Any) -> set[str]:
    """Place references that exist, for telling a typo from an unvisited place.

    Read off the entities themselves rather than from a partition stream. Named
    regions were derived geometrically and removed: the partition could split
    space but never name a room, so every question that named one stayed
    unanswerable while the stream implied otherwise. A place reference is a
    lattice cell today, and the honest list of them is whatever the sightings
    actually resolved to.
    """
    from dimos.experimental.memory_belief.entity import ENTITY_STREAM_NAME

    if ENTITY_STREAM_NAME not in store.streams:
        return set()
    return {
        ref for o in store.streams[ENTITY_STREAM_NAME] if (ref := (o.tags or {}).get("place_ref"))
    }


def _vocabulary(store: Any) -> set[str]:
    """The vocabulary the detector was actually run with, from the data itself."""
    from dimos.experimental.memory_belief.types import STREAM_NAME as BELIEF_STREAM_NAME

    try:
        first = store.streams[BELIEF_STREAM_NAME].first()
    except (KeyError, AttributeError, LookupError):
        return set()
    return set(getattr(first.data, "vocabulary", None) or ())


def describe_store(store: Any) -> dict[str, Any]:
    """The constants a caller has to know to write a valid query.

    Every argument rejection in the first traced run was a guess at one of these
    -- a time written as ``"09:00"``, a place invented as ``"Room 3"``. They are
    not secrets and they are not stable across recordings, so they are read from
    the store and handed over rather than written into a prompt by hand.
    """
    spans = []
    for name in ("belief_observation", "belief_entity", "odom"):
        # A store legitimately lacks some of these -- a recording with no
        # detections has no entity stream -- and the span of what is absent is
        # simply not reported. An empty stream has no first record either.
        if name not in store.streams:
            continue
        stream = store.streams[name]
        first, last = stream.first(), stream.last()
        if first is not None and last is not None:
            spans.append((first.ts, last.ts))
    vocab = _vocabulary(store)
    return {
        "time_unit": "unix epoch seconds (float). Not a clock string, not ISO 8601.",
        "t_start": min((a for a, _ in spans), default=None),
        "t_end": max((b for _, b in spans), default=None),
        "place_refs": sorted(_place_refs(store)),
        "vocabulary_size": len(vocab),
        "labelled_streams": sorted(s for s in SELECTS if "label" in _tags_of(store, s)),
        "streams": sorted(s for s in SELECTS if _tags_of(store, s) or s in SELECTS),
    }


def _validate(store: Any, select: str, where: Sequence[dict[str, Any]]) -> str | None:
    """Reject a clause the stream cannot answer; report an unknown term as unknown.

    The distinction is the point. A ``label`` clause on a stream with no labels is
    the caller's error and is raised. A label the detector was never asked about is
    the *world's* limit -- the frames are stored and re-running detection resolves
    it -- so it returns ``OUT_OF_VOCABULARY`` and travels as an answer, not an
    exception.
    """
    tags = _tags_of(store, select)
    if not tags:
        # No tags means no records, not a stream that cannot carry the clause.
        # Rejecting here would tell a robot that has simply not written this
        # belief yet that it asked the wrong question, sending it to rewrite a
        # query that was correct. Absence is reported downstream as
        # NEVER_COVERED, where it belongs.
        return None
    places = _place_refs(store)
    vocab = _vocabulary(store)
    for clause in where:
        op = str(clause.get("op") or "")
        needed = _TAG_FOR_OP.get(op)
        if needed and not (set(needed) & tags if isinstance(needed, tuple) else {needed} & tags):
            usable = sorted(
                s
                for s in SELECTS
                if _tags_of(store, s) & (set(needed) if isinstance(needed, tuple) else {needed})
            )
            raise QueryError(
                f"select {select!r} carries no {op!r} information, so that clause "
                f"can only ever match nothing. Streams that do: {usable or 'none'}."
            )
        if op == "in_region" and places and clause.get("region") not in places:
            raise QueryError(
                f"no place {clause.get('region')!r}; this recording has "
                f"{sorted(places)}. These are lattice cells, not room names."
            )
        if op == "label" and vocab and clause.get("value") not in vocab:
            import difflib

            near = difflib.get_close_matches(str(clause.get("value")), vocab, n=4, cutoff=0.6)
            return (
                f"{clause.get('value')!r} is not in the vocabulary the detector ran "
                f"with ({len(vocab)} terms)" + (f"; nearest: {near}" if near else "")
            )
    return None


def execute(store: Any, query: dict[str, Any]) -> dict[str, Any]:
    """Run one query and return the envelope.

    ``as_of`` is required rather than defaulted. The asking time determines the
    answer -- "where is the backpack" differs at t=30 s and t=400 s -- and a
    default would silently pick one, which is the failure the wall-clock
    staleness bug produced in every recorded trace.

    ``limit`` is defaulted, at ``DEFAULT_LIMIT``, because the alternative is
    reading a whole recording's worth of rows to answer a question about a
    handful of them.
    """
    select = query.get("select")
    if select is None:
        raise QueryError("select is required")
    if "as_of" not in query:
        raise QueryError("as_of is required: the asking time determines the answer")
    try:
        as_of = float(query["as_of"])
    except (TypeError, ValueError) as exc:
        raise QueryError(f"as_of must be a number, got {query['as_of']!r}") from exc
    # `inf` passes every `before` comparison, so it does not merely widen the
    # window -- it removes it, and the answer covers the whole recording while
    # still reporting a time base. `nan` fails every comparison instead and
    # reads as an empty world. Neither is a moment.
    if not math.isfinite(as_of):
        raise QueryError(f"as_of must be a finite time, got {as_of}")
    # "locate" is the only projection, and the previous default named one that
    # does not exist -- so a query that simply omitted `project` was rejected as
    # if it had asked for something.
    projection = query.get("project") or PROJECTIONS[0]
    limit = _limit(query)

    where = query.get("where") or []
    _check_shape(where)
    # Validated before the stream is touched: an argument the store cannot honour
    # must not come back looking like an absence of observation.
    out_of_vocab = _validate(store, select, where)
    if out_of_vocab is not None:
        return {
            "status": "unknown",
            "reason": "OUT_OF_VOCABULARY",
            "result": None,
            "quality": {"rows": 0},
            "time_base": {"as_of": as_of, "unit": "seconds"},
            "diagnostic": {"note": out_of_vocab, "remedy": "redetect_with_vocabulary"},
            "query": query,
        }

    try:
        stream = _open(store, select)
    except MissingStreamError as exc:
        # Not an error the caller can fix by rewriting the query: the robot has
        # simply never written this kind of belief. A fresh store on a robot that
        # just booted is in exactly this state, and the first question asked of
        # it deserves an answer rather than a traceback. `QueryError` stays an
        # exception because a malformed query *is* the caller's to fix, and the
        # skill layer converts that one for the agent.
        return {
            "status": "unknown",
            "reason": "NEVER_COVERED",
            "result": None,
            "quality": {"rows": 0},
            "time_base": {"as_of": as_of, "unit": "seconds"},
            "diagnostic": {
                "note": str(exc),
                "remedy": f"write the {exc.stream_name!r} stream before asking about {exc.select!r}",
            },
            "query": query,
        }
    stream = _apply(stream, where)
    # The asking time bounds every query. Nothing after it is knowable yet, which
    # is true of a live stream and must stay true of a replayed one.
    stream = _before_asking_time(stream, as_of)
    # Bounded here rather than in the projection: `to_list` below reads every row
    # the stream still holds, and rows discarded after that were paid for.
    stream = stream.limit(limit)

    rows = stream.to_list()
    if not rows:
        return {
            "status": "unknown",
            "reason": "NEVER_COVERED",
            "result": None,
            "quality": {"rows": 0},
            "time_base": {"as_of": as_of, "unit": "seconds"},
            "diagnostic": _diagnose(store, query, as_of),
            "query": query,
        }

    result = _project(projection, rows)
    quality = _quality(rows)
    status, reason = "ok", None
    if projection == "locate" and not result:
        status, reason = "unknown", "NO_CAPABILITY"
    elif quality.get("coherence") is not None and quality["coherence"] < COHERENT:
        status, reason = "unknown", "INCOHERENT"

    return {
        "status": status,
        "reason": reason,
        "result": result,
        "quality": quality,
        "time_base": {"as_of": as_of, "unit": "seconds"},
        "query": query,
    }


def _diagnose(store: Any, query: dict[str, Any], as_of: float) -> dict[str, Any]:
    """Which clause emptied the result, so a retry is informed rather than blind.

    Compressing six steps into one call costs the model its chance to see
    intermediate results. Returning an empty list without saying why would make
    that trade a bad one, so an empty result reports what each clause cost.

    Bounded at the asking time like the query it explains. A count is still an
    answer: unbounded, "your label clause matched nothing" came back beside a
    row count drawn from observations the query itself had refused to look at.

    Counted rather than listed. These rows are never read, only tallied, and
    ``count()`` pushes the tally into the store instead of building a list per
    clause to take its length.
    """
    where = list(query.get("where") or [])
    if not where:
        return {"note": "stream is empty"}
    stream = _before_asking_time(_open(store, query["select"]), as_of)
    remaining = stream.count()
    steps = []
    for clause in where:
        try:
            stream = _apply(stream, [clause])
            after = stream.count()
        except QueryError as exc:  # pragma: no cover - reported, not raised
            steps.append({"clause": clause, "error": str(exc)})
            break
        steps.append({"clause": clause, "rows_before": remaining, "rows_after": after})
        remaining = after
        if after == 0:
            break
    return {"per_clause": steps, "hint": "relax the clause whose rows_after is 0"}
