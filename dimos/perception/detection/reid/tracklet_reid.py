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

"""Merging tracklets a tracker lost track of.

Two chairs of one model are identical to a similarity model, so a threshold high
enough to merge one chair's fragments also merges two different chairs -- and a
wrong merge invents an object where fragments only under-report. What separates
identical things is where and when they were, so geometry vetoes first and
appearance only decides what survives.

Pairs score 100% precision; the 99% recall figure predates the reachability veto
moving from run centroids to the sightings either side of the gap and wants one
rerun. Grouping is unmeasured: the numbers
that once stood here were taken before closure applied the overlap veto to the
group it forms, and the benchmark's negative class is that same veto, so it
cannot referee the fix. Hence :func:`find_merges` returns pairs and
:func:`connected` is opt-in. Measurements and method:
``docs/capabilities/perception/tracklet_reid.md``.

numpy only -- embeddings arrive already computed, positions and times as plain
numbers -- so whoever has the crops keeps that half.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
import itertools
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from numpy.typing import NDArray


@dataclass(frozen=True, slots=True, eq=False)
class Tracklet:
    """One run of sightings a tracker believed was a single thing.

    The unit is the tracklet, not the sighting: the tracker has already done the
    easy grouping, and thirty views decide better than thirty decisions of one.
    """

    key: str
    t_start: float
    t_end: float

    #: One row per positioned sighting, oldest first. The order matters: the
    #: reachability veto reads the two rows either side of the gap, not the mean.
    #: None leaves geometry unable to veto.
    positions: NDArray[np.float64] | None = None

    #: ``(N, D)``, L2-normalised. None means this cannot be compared at all.
    embeddings: NDArray[np.float64] | None = None

    label: str | None = None

    def centroid(self) -> NDArray[np.float64] | None:
        """Where the whole run sat. For how far a group spans, not for reachability."""
        if self.positions is None or not len(self.positions):
            return None
        mean: NDArray[np.float64] = np.asarray(self.positions, float).mean(axis=0)
        return mean

    def first_point(self) -> NDArray[np.float64] | None:
        """Where this run began."""
        if self.positions is None or not len(self.positions):
            return None
        point: NDArray[np.float64] = np.asarray(self.positions[0], float)
        return point

    def last_point(self) -> NDArray[np.float64] | None:
        """Where this run ended."""
        if self.positions is None or not len(self.positions):
            return None
        point: NDArray[np.float64] = np.asarray(self.positions[-1], float)
        return point

    def overlaps(self, other: Tracklet) -> bool:
        return self.t_start <= other.t_end and other.t_start <= self.t_end


@dataclass(frozen=True, slots=True)
class ReidPolicy:
    """What counts as the same thing. Every field is a claim about the world."""

    #: Belongs to the embedding model and to ``top_frac``, not to this code: a
    #: threshold read under one aggregation is wrong under another. Measured
    #: together; picking either alone cost 15 points of recall.
    similarity: float = 0.63

    #: Placement error, not motion. The veto that keeps two identical chairs
    #: apart.
    max_move_m: float = 2.0

    #: Travel allowed across the gap. Without it the distance check treats a
    #: chair and a walking person alike, and 30 objects on the measured capture
    #: could never be rejoined. Zero restores a fixed radius.
    max_speed_mps: float = 1.5

    #: Ceiling on that allowance. Unbounded it stops being a constraint: a
    #: minute of walking reaches past every wall of the room.
    max_reach_m: float = 8.0

    #: Metres a merged group may span. Pairwise checks cannot see this -- a-b
    #: and b-c both in range puts a and c twice as far -- and an unchecked
    #: closure produced a "chair" spread over 4.28 m.
    max_spread_m: float = 4.0

    #: A chair's back and its front are not similar; one view either side of a
    #: merge is not evidence.
    min_views: int = 10

    #: Fraction of pairwise similarities averaged. A fraction, not a count: a
    #: count degenerates into the mean on small tracklets, which is what the
    #: aggregation exists to avoid.
    top_frac: float = 0.12

    #: A sighting with no depth could be anywhere, so nothing vetoes it and
    #: appearance decides alone. False refuses instead.
    merge_without_position: bool = False

    #: Off by default: labels flicker between synonyms far more than appearance
    #: does.
    require_same_label: bool = False


@dataclass(frozen=True, slots=True)
class Merge:
    """A claim that two tracklets are one thing, with what supports it."""

    keys: tuple[str, ...]
    similarity: float
    #: Across the gap: the earlier run's last sighting to the later run's first.
    distance_m: float | None
    #: Not disqualifying -- a long gap is the case a tracker cannot handle --
    #: but it is what a reader needs to judge the claim.
    gap_s: float


@dataclass
class Report:
    """What a pass did, including what it refused.

    Merging nothing because the constraints held and merging nothing because no
    tracklet carried an embedding look identical from outside. Only the counts
    tell them apart.
    """

    merges: list[Merge] = field(default_factory=list)
    rejected: Counter[str] = field(default_factory=Counter)

    def summary(self) -> str:
        parts = ", ".join(f"{k}={v}" for k, v in sorted(self.rejected.items()))
        return f"{len(self.merges)} merges; rejected: {parts or 'none'}"


def group_similarity(a: NDArray[np.float64], b: NDArray[np.float64], *, top_frac: float) -> float:
    """Mean of the best ``top_frac`` of pairwise cosines. Inputs L2-normalised."""
    sims = np.asarray(a, float) @ np.asarray(b, float).T
    flat = sims.ravel()
    k = max(1, min(flat.size, round(top_frac * flat.size)))
    return float(np.mean(np.partition(flat, -k)[-k:]))


def any_overlap(spans: Iterable[tuple[float, float]]) -> bool:
    """Does any pair of these windows share an instant?

    Sorted by start, it is enough to compare neighbours: anything between two
    overlapping windows starts inside the earlier one and so overlaps it too.
    """
    windows = sorted(spans)
    return any(later[0] <= earlier[1] for earlier, later in itertools.pairwise(windows))


def gap_between(a: Tracklet, b: Tracklet) -> float:
    """Seconds between one tracklet ending and the other beginning."""
    earlier, later = (a, b) if a.t_end <= b.t_start else (b, a)
    return max(0.0, later.t_start - earlier.t_end)


def _veto(a: Tracklet, b: Tracklet, policy: ReidPolicy) -> tuple[str | None, float | None]:
    """The cheap checks, most conclusive first."""
    if a.overlaps(b):
        # Two tracks visible at one instant are two things, whatever they look
        # like. No score outweighs this.
        return "time_overlap", None
    if policy.require_same_label and a.label != b.label:
        return "label_mismatch", None

    # The budget is what the gap allows, so measure what the gap spans: where the
    # earlier run ended and where the later one resumed. A whole-run centroid
    # answers a different question -- it charges a continuation for movement that
    # happened inside its own tracklet, and forgives a jump when two runs happen to
    # average to the same place. Non-overlapping by the check above, so the order
    # is well defined.
    earlier, later = (a, b) if a.t_end <= b.t_start else (b, a)
    left, right = earlier.last_point(), later.first_point()
    if left is None or right is None:
        if not policy.merge_without_position:
            return "no_position", None
        return None, None

    distance = float(np.linalg.norm(left - right))
    reachable = min(
        policy.max_move_m + policy.max_speed_mps * gap_between(a, b), policy.max_reach_m
    )
    if distance > reachable:
        return "too_far", distance
    return None, distance


def find_merges(
    tracklets: Iterable[Tracklet], policy: ReidPolicy | None = None
) -> tuple[list[Merge], Report]:
    """Every pair that survives the vetoes and clears the threshold.

    Pairs, not clusters: "a matches b, b matches c" asserts something about a
    and c that nothing here checked, and on identical objects it is often false.
    """
    policy = policy or ReidPolicy()
    items = list(tracklets)
    report = Report()

    # The online path normalises what it is given; this one is handed arrays it
    # cannot re-check per pair. Un-normalised vectors do not lower the score,
    # they inflate it -- orthogonal tracklets scored 3.29 and merged -- so this
    # fails loudly rather than merging on a cosine that is not one.
    for t in items:
        if t.embeddings is None or not len(t.embeddings):
            continue
        norms = np.linalg.norm(np.asarray(t.embeddings, float), axis=1)
        if not np.allclose(norms, 1.0, atol=1e-3):
            raise ValueError(
                f"tracklet {t.key!r} carries embeddings that are not L2-normalised "
                f"(norms {float(norms.min()):.3f}..{float(norms.max()):.3f})"
            )

    # ponytail: every pair, no spatial index -- fine for one room, revisit if a
    # capture ever holds enough tracklets that the vetoes stop being the cheap part.
    for a, b in itertools.combinations(items, 2):
        reason, distance = _veto(a, b, policy)
        if reason is not None:
            report.rejected[reason] += 1
            continue
        if a.embeddings is None or b.embeddings is None:
            report.rejected["no_embeddings"] += 1
            continue
        if len(a.embeddings) < policy.min_views or len(b.embeddings) < policy.min_views:
            report.rejected["too_few_views"] += 1
            continue

        score = group_similarity(a.embeddings, b.embeddings, top_frac=policy.top_frac)
        # NaN in, merge out: `nan < threshold` is False, so one bad row of
        # embedding would pass the check the whole module rests on.
        if not np.isfinite(score):
            report.rejected["undefined_similarity"] += 1
            continue
        if score < policy.similarity:
            report.rejected["below_threshold"] += 1
            continue

        report.merges.append(
            Merge(
                keys=(a.key, b.key),
                similarity=score,
                distance_m=distance,
                gap_s=gap_between(a, b),
            )
        )
    return report.merges, report


def connected(
    merges: Iterable[Merge],
    tracklets: Iterable[Tracklet] | None = None,
    policy: ReidPolicy | None = None,
) -> Iterator[frozenset[str]]:
    """Close pairwise merges into groups, strongest link first.

    A weaker claim than the pairs it is built from, and an unmeasured one -- see
    the module docstring. Given ``tracklets``, a link is refused if the group it
    would create holds two tracklets seen at one instant, or overruns
    ``max_spread_m``; refusing the link rather than the finished group keeps one
    bad pair from costing every good merge around it. Descending similarity
    means what gets refused is the weakest evidence.

    ``tracklets`` must cover every key in ``merges``: a veto that silently
    skipped half of them would read from outside exactly like one that held.
    """
    policy = policy or ReidPolicy()
    centroids: dict[str, NDArray[np.float64] | None] = {}
    spans: dict[str, tuple[float, float]] = {}
    if tracklets is not None:
        for t in tracklets:
            centroids[t.key] = t.centroid()
            spans[t.key] = (t.t_start, t.t_end)
        missing = {k for m in merges for k in m.keys} - spans.keys()
        if missing:
            # A veto that silently covers half the keys is worse than none: it
            # reads as checked.
            raise ValueError(f"no tracklet given for {sorted(missing)}")

    parent: dict[str, str] = {}
    members: dict[str, set[str]] = {}

    def find(x: str) -> str:
        parent.setdefault(x, x)
        members.setdefault(x, {x})
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def spread(keys: set[str]) -> float:
        points = [c for c in (centroids.get(k) for k in keys) if c is not None]
        if len(points) < 2:
            return 0.0
        stacked = np.stack(points)
        return float(np.linalg.norm(stacked.max(axis=0) - stacked.min(axis=0)))

    # ponytail: greedy, strongest link first, no backtracking -- a link refused
    # here is gone. A global objective only if grouping is ever trusted enough.
    for merge in sorted(merges, key=lambda m: -m.similarity):
        roots = {find(k) for k in merge.keys}
        if len(roots) < 2:
            continue
        joined: set[str] = set().union(*(members[r] for r in roots))
        # Pairwise, "seen at one instant" is the veto no score outweighs. A
        # group asserts the same thing about every pair it contains, including
        # the ones no merge ever proposed, so it has to clear the same bar.
        if spans and any_overlap(spans[k] for k in joined if k in spans):
            continue
        if centroids and spread(joined) > policy.max_spread_m:
            continue
        keep = roots.pop()
        for other in roots:
            parent[other] = keep
        members[keep] = joined

    for root in {find(k) for k in parent}:
        yield frozenset(members[root])


@dataclass(frozen=True, slots=True)
class OnlinePolicy:
    """Bounds a batch pass does not need, because a robot runs for a shift."""

    #: The comparison reads the best few anyway, so what a cap drops is the
    #: middle of the distribution rather than its shape.
    max_views: int = 200

    #: A track last seen an hour ago is not a candidate for something visible
    #: now -- the speed veto would refuse it regardless.
    max_tracks: int = 500


class OnlineAssociator:
    """Assign long-term ids to tracks as their sightings arrive.

    The same decisions as :func:`find_merges`, one observation at a time: state
    accumulates into :class:`Tracklet` views that the same vetoes and similarity
    run against, so the two halves cannot disagree about what counts as one
    thing.

    **Deciding late is allowed; deciding twice is not.** :meth:`resolve` returns
    None while evidence is thin rather than inventing an id it would take back.

    An id is a group, so this inherits whatever grouping is worth -- which the
    module docstring says is not yet known. It is here because the shape suits a
    robot, not because anything measured justifies deploying it.
    """

    def __init__(self, policy: ReidPolicy | None = None, online: OnlinePolicy | None = None):
        self.policy = policy or ReidPolicy()
        self.online = online or OnlinePolicy()
        self._views: dict[str, list[NDArray[np.float64]]] = {}
        self._points: dict[str, list[tuple[float, float, float]]] = {}
        self._span: dict[str, tuple[float, float]] = {}
        self._label: dict[str, str | None] = {}
        self._assigned: dict[str, str] = {}
        self._excluded: dict[str, set[str]] = {}
        #: Insertion-ordered, rewritten on every sighting: least recently seen
        #: first. Keyed by track, the value is unused.
        self._order: dict[str, None] = {}
        self._counter = 0

    def observe(
        self,
        track: str,
        *,
        embedding: NDArray[np.float64] | None = None,
        position: tuple[float, float, float] | None = None,
        ts: float,
        label: str | None = None,
    ) -> None:
        """Add one sighting's evidence to a track."""
        self._order.pop(track, None)
        self._order[track] = None
        self._forget_old()
        first, last = self._span.get(track, (ts, ts))
        self._span[track] = (min(first, ts), max(last, ts))
        if label is not None:
            self._label.setdefault(track, label)
        if embedding is not None:
            vector = np.asarray(embedding, float).ravel()
            norm = float(np.linalg.norm(vector))
            if norm:
                views = self._views.setdefault(track, [])
                views.append(vector / norm)
                if len(views) > self.online.max_views:
                    # From the middle: the first views establish the track and
                    # the newest reflect it now.
                    del views[len(views) // 2]
        if position is not None:
            x, y, z = (float(v) for v in position)
            self._points.setdefault(track, []).append((x, y, z))

    def co_occurring(self, tracks: Iterable[str]) -> None:
        """Record that these tracks shared a frame, so they are not one thing.

        The batch path derives this from overlapping spans; live, a caller that
        knows gets the veto before either track is long enough to compare.
        """
        keys = list(tracks)
        for a in keys:
            self._excluded.setdefault(a, set()).update(k for k in keys if k != a)

    def _tracklet(self, track: str) -> Tracklet:
        first, last = self._span[track]
        points = self._points.get(track)
        views = self._views.get(track)
        return Tracklet(
            key=track,
            t_start=first,
            t_end=last,
            positions=np.asarray(points, float) if points else None,
            embeddings=np.stack(views) if views else None,
            label=self._label.get(track),
        )

    def resolve(self, track: str) -> str | None:
        """The long-term id for ``track``, or None while the evidence is thin.

        Once given, an id does not change: revising one a caller already acted
        on is worse than having been slow.
        """
        if track in self._assigned:
            return self._assigned[track]
        views = self._views.get(track)
        if views is None or len(views) < self.policy.min_views:
            return None

        query = self._tracklet(track)
        if query.embeddings is None:
            return None
        best_key, best_score = None, self.policy.similarity
        centroid = query.centroid()

        entities: dict[str, list[str]] = {}
        for other in self._order:
            entity = self._assigned.get(other)
            if entity is not None and other != track:
                entities.setdefault(entity, []).append(other)

        # ponytail: re-derives every member's tracklet per resolve; cache per entity
        # if resolve ever runs hot enough to show up.
        for entity, group in entities.items():
            # Joining an id claims sameness with everything already under it,
            # not just with the track that scored. So every member gets a veto,
            # a thin one included: "seen at one instant" needs no embedding.
            if any(m in self._excluded.get(track, ()) for m in group):
                continue
            if any(_veto(query, self._tracklet(m), self.policy)[0] is not None for m in group):
                continue
            # Without this, 388 pieces of one capture collapsed into a single
            # id -- every link legal, the chain across the room.
            if centroid is not None and self._spread_with(entity, centroid) > (
                self.policy.max_spread_m
            ):
                continue
            for member in group:
                candidate = self._tracklet(member)
                if (
                    candidate.embeddings is None
                    or len(candidate.embeddings) < self.policy.min_views
                ):
                    continue
                score = group_similarity(
                    query.embeddings, candidate.embeddings, top_frac=self.policy.top_frac
                )
                if score >= best_score:
                    best_key, best_score = member, score

        if best_key is not None:
            assigned = self._assigned[best_key]
        else:
            self._counter += 1
            assigned = f"entity-{self._counter}"
        self._assigned[track] = assigned
        return assigned

    def _spread_with(self, entity: str, point: NDArray[np.float64]) -> float:
        """How far a group would span with ``point`` added to it."""
        points = [point]
        for track, assigned in self._assigned.items():
            if assigned != entity:
                continue
            centre = self._tracklet(track).centroid()
            if centre is not None:
                points.append(centre)
        if len(points) < 2:
            return 0.0
        stacked = np.stack(points)
        return float(np.linalg.norm(stacked.max(axis=0) - stacked.min(axis=0)))

    def _forget_old(self) -> None:
        """Drop the oldest tracks entirely, assignment included.

        An id outliving its evidence is an answer nothing supports, and the
        group it names still measured itself against a span already dropped.
        """
        while len(self._order) > self.online.max_tracks:
            gone = next(iter(self._order))
            del self._order[gone]
            # A tracker reuses ids. Leaving the dropped key inside surviving
            # exclusion sets would bar the next track to carry it from a match it
            # never co-occurred with.
            for excluded in self._excluded.values():
                excluded.discard(gone)
            for store in (
                self._views,
                self._points,
                self._span,
                self._label,
                self._excluded,
                self._assigned,
            ):
                store.pop(gone, None)
