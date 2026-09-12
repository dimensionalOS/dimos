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

from __future__ import annotations

import numpy as np
import pytest

from dimos.perception.detection.reid.tracklet_reid import (
    Merge,
    OnlineAssociator,
    OnlinePolicy,
    ReidPolicy,
    Tracklet,
    connected,
    find_merges,
    group_similarity,
)


def views(direction: tuple[float, float], n: int = 10, jitter: float = 0.02) -> np.ndarray:
    """``n`` unit vectors clustered around one direction in 2-D embedding space."""
    rng = np.random.default_rng(0)
    base = np.asarray(direction, float)
    raw = base + rng.normal(0.0, jitter, size=(n, 2))
    return raw / np.linalg.norm(raw, axis=1, keepdims=True)


def tracklet(key, t0, t1, xy=(0.0, 0.0), look=(1.0, 0.0), n=10, label="chair") -> Tracklet:
    return Tracklet(
        key=key,
        t_start=t0,
        t_end=t1,
        positions=np.array([[xy[0], xy[1], 0.0]] * n, float),
        embeddings=views(look, n=n),
        label=label,
    )


class TestGeometryVetoesAppearance:
    """Two chairs of one model score near-perfect, correctly. Only where and
    when they were separates them, so no score overrides those checks."""

    def test_two_things_seen_at_once_never_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 5.0, 15.0, xy=(0.2, 0.0))  # identical look, overlapping time

        merges, report = find_merges([a, b])

        assert merges == []
        assert report.rejected == {"time_overlap": 1}

    def test_identical_things_far_apart_never_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(50.0, 0.0))

        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0))

        assert merges == []
        assert report.rejected == {"too_far": 1}

    def test_the_same_thing_seen_again_nearby_merges(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(0.5, 0.0))

        merges, _ = find_merges([a, b])

        assert len(merges) == 1
        assert merges[0].keys == ("a", "b")
        assert merges[0].gap_s == 10.0
        assert merges[0].distance_m is not None


class TestAThingIsAllowedToHaveMoved:
    """A fixed radius refuses every person who walked away and came back."""

    def test_a_walk_within_reach_of_the_gap_merges(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), label="person")
        b = tracklet("b", 40.0, 50.0, xy=(6.0, 0.0), label="person")

        # 30 s at 1.5 m/s reaches 47 m, capped to 8 m; 6 m is inside it.
        merges, _ = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert len(merges) == 1
        assert merges[0].gap_s == 30.0

    def test_the_allowance_stops_growing_at_the_ceiling(self):
        """A budget that grows without limit stops being a constraint."""
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 300.0, 310.0, xy=(30.0, 0.0))

        # 290 s of walking reaches 437 m, further than any room.
        merges, report = find_merges(
            [a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5, max_reach_m=8.0)
        )

        assert merges == []
        assert report.rejected == {"too_far": 1}

    def test_a_walk_beyond_reach_of_the_gap_does_not(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), label="person")
        b = tracklet("b", 12.0, 22.0, xy=(20.0, 0.0), label="person")

        # 2 s reaches 5 m. Twenty metres in two seconds is not one person.
        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert merges == []
        assert report.rejected == {"too_far": 1}

    def test_zero_speed_restores_a_fixed_radius(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 40.0, 50.0, xy=(20.0, 0.0))

        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=0.0))

        assert merges == []
        assert report.rejected == {"too_far": 1}


def walk(start: float, stop: float, n: int = 10) -> np.ndarray:
    """A run that moves from ``start`` to ``stop`` along x, oldest row first."""
    return np.array([[x, 0.0, 0.0] for x in np.linspace(start, stop, n)], float)


class TestTheBudgetIsSpentOnTheGap:
    """The allowance is what the gap permits, so it is measured across the gap:
    where the earlier run ended, where the later one resumed. A whole-run mean
    answers a different question and gets both directions wrong."""

    def test_movement_inside_a_run_is_not_charged_to_the_gap(self):
        a = Tracklet("a", 0.0, 10.0, positions=walk(0.0, 14.0), embeddings=views((1.0, 0.0)))
        b = Tracklet(
            "b",
            12.0,
            22.0,
            positions=np.array([[14.0, 0.0, 0.0]] * 10),
            embeddings=views((1.0, 0.0)),
        )

        # a ends at 14 m and b resumes at 14 m: nothing crossed the gap. The
        # whole-run means are 7 m apart, which 2 s of budget would have refused.
        merges, _ = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert len(merges) == 1
        assert merges[0].distance_m == 0.0

    def test_a_jump_across_the_gap_is_refused_though_the_runs_average_alike(self):
        a = Tracklet("a", 0.0, 10.0, positions=walk(0.0, 20.0), embeddings=views((1.0, 0.0)))
        b = Tracklet("b", 12.0, 22.0, positions=walk(0.0, 20.0), embeddings=views((1.0, 0.0)))

        # Both runs average to 10 m, so their means coincide -- but a ended at
        # 20 m and b resumed at 0 m. Twenty metres in two seconds is not one thing.
        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert merges == []
        assert report.rejected == {"too_far": 1}


class TestAppearanceStillHasToAgree:
    def test_a_different_looking_thing_nearby_does_not_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), look=(1.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(0.5, 0.0), look=(0.0, 1.0))

        merges, report = find_merges([a, b])

        assert merges == []
        assert report.rejected == {"below_threshold": 1}


class TestEvidenceHasToBeEnough:
    def test_a_tracklet_with_too_few_views_is_refused_not_guessed(self):
        a = tracklet("a", 0.0, 10.0, n=2)
        b = tracklet("b", 20.0, 30.0, n=20)

        merges, report = find_merges([a, b], ReidPolicy(min_views=5))

        assert merges == []
        assert report.rejected == {"too_few_views": 1}

    def test_a_tracklet_with_no_position_is_refused_by_default(self):
        a = Tracklet("a", 0.0, 10.0, positions=None, embeddings=views((1.0, 0.0)))
        b = tracklet("b", 20.0, 30.0)

        merges, report = find_merges([a, b])

        # Nothing vetoes a sighting that could be anywhere.
        assert merges == []
        assert report.rejected == {"no_position": 1}

    def test_positionless_merging_is_available_when_asked_for(self):
        a = Tracklet("a", 0.0, 10.0, positions=None, embeddings=views((1.0, 0.0)))
        b = tracklet("b", 20.0, 30.0)

        merges, _ = find_merges([a, b], ReidPolicy(merge_without_position=True))

        assert len(merges) == 1
        assert merges[0].distance_m is None


class TestTheReportSaysWhyNothingMerged:
    """Zero merges from bad input and from good input look alike."""

    def test_every_rejection_is_counted_by_reason(self):
        far = tracklet("far", 20.0, 30.0, xy=(50.0, 0.0))
        overlapping = tracklet("over", 5.0, 15.0)
        base = tracklet("base", 0.0, 10.0)

        _, report = find_merges([base, overlapping, far])

        assert sum(report.rejected.values()) == 3
        assert "time_overlap" in report.summary()


class TestAGroupIsCheckedAsAWhole:
    """Every pair in a chain can pass and the chain still be wrong: a-b and b-c
    both in range puts a and c twice as far."""

    def test_the_link_that_overruns_is_refused_not_the_whole_group(self):
        chain = [
            tracklet("a", 0.0, 10.0, xy=(0.0, 0.0)),
            tracklet("b", 20.0, 30.0, xy=(1.8, 0.0)),
            tracklet("c", 40.0, 50.0, xy=(3.6, 0.0)),
        ]
        merges = [Merge(("a", "b"), 0.99, 1.8, 10.0), Merge(("b", "c"), 0.90, 1.8, 10.0)]

        # Unchecked, the chain closes into one group spanning 3.6 m.
        assert list(connected(merges)) == [frozenset({"a", "b", "c"})]

        # Checked, the stronger link is taken and only the one that would
        # overrun is refused. Dropping the whole group instead cost several
        # hundred correct merges on real data.
        groups = sorted(connected(merges, chain, ReidPolicy(max_spread_m=2.0)), key=sorted)
        assert groups == [frozenset({"a", "b"}), frozenset({"c"})]

    def test_a_tight_chain_survives(self):
        chain = [
            tracklet("a", 0.0, 10.0, xy=(0.0, 0.0)),
            tracklet("b", 20.0, 30.0, xy=(0.3, 0.0)),
            tracklet("c", 40.0, 50.0, xy=(0.6, 0.0)),
        ]
        merges = [Merge(("a", "b"), 0.95, 0.3, 10.0), Merge(("b", "c"), 0.95, 0.3, 10.0)]

        groups = list(connected(merges, chain, ReidPolicy(max_spread_m=2.0)))

        assert groups == [frozenset({"a", "b", "c"})]


class TestGroupingIsAWeakerClaim:
    def test_chained_merges_close_into_one_group(self):
        groups = list(
            connected([Merge(("a", "b"), 0.9, 1.0, 5.0), Merge(("b", "c"), 0.9, 1.0, 5.0)])
        )

        assert groups == [frozenset({"a", "b", "c"})]

    def test_unrelated_merges_stay_separate(self):
        groups = list(
            connected([Merge(("a", "b"), 0.9, 1.0, 5.0), Merge(("c", "d"), 0.9, 1.0, 5.0)])
        )

        assert sorted(groups, key=sorted) == [frozenset({"a", "b"}), frozenset({"c", "d"})]


class TestSimilarityAggregation:
    def test_the_best_fraction_ignores_the_worst_pairs(self):
        a = np.array([[1.0, 0.0], [0.0, 1.0]])
        b = np.array([[1.0, 0.0], [0.0, 1.0]])

        # Pairs are 1, 0, 0, 1. The best half average to 1; all four average
        # to 0.5, which would hide a real match behind unhelpful views.
        assert group_similarity(a, b, top_frac=0.5) == 1.0
        assert group_similarity(a, b, top_frac=1.0) == 0.5

    def test_the_fraction_does_not_degenerate_on_small_tracklets(self):
        """A count would: five views each is 25 pairs, and the best 20 is a mean."""
        a = np.eye(5)
        b = np.eye(5)

        # Orthogonal views: every pair scores 0 except the five matching ones.
        assert group_similarity(a, b, top_frac=0.2) == 1.0
        assert group_similarity(a, b, top_frac=0.8) < 0.3


class TestAGroupIsHeldToTheVetoItsPairsWere:
    """A group asserts sameness about every pair inside it, including pairs no
    merge ever proposed. The instant-overlap veto has no exception for those."""

    def test_a_group_never_holds_two_things_seen_at_once(self):
        chain = [
            tracklet("a", 0.0, 10.0, xy=(0.0, 0.0)),
            tracklet("b", 20.0, 30.0, xy=(0.3, 0.0)),
            tracklet("c", 5.0, 15.0, xy=(0.6, 0.0)),  # overlaps a, disjoint from b
        ]
        # Both links are legal and the group is tight enough to pass spread.
        merges = [Merge(("a", "b"), 0.99, 0.3, 10.0), Merge(("b", "c"), 0.90, 0.3, 10.0)]

        groups = sorted(connected(merges, chain, ReidPolicy()), key=sorted)

        assert groups == [frozenset({"a", "b"}), frozenset({"c"})]

    def test_grouping_without_the_tracklets_it_checks_is_refused(self):
        """Half the keys checked reads from outside exactly like all of them."""
        merges = [Merge(("a", "b"), 0.9, 1.0, 5.0)]

        with pytest.raises(ValueError, match="no tracklet given"):
            list(connected(merges, [tracklet("a", 0.0, 10.0)]))


class TestBadInputIsRefusedNotMerged:
    def test_un_normalised_embeddings_are_refused_not_rescaled(self):
        """Scale does not lower a cosine, it inflates one: orthogonal tracklets
        scored 3.29 and merged."""
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = Tracklet(
            "b", 20.0, 30.0, positions=np.zeros((10, 3)), embeddings=views((0.0, 1.0)) * 10.0
        )

        with pytest.raises(ValueError, match="L2-normalised"):
            find_merges([a, b])

    def test_a_tracklet_can_be_put_in_a_set(self):
        """Frozen promises hashable, and arrays broke both that and ``==``."""
        a = tracklet("a", 0.0, 10.0)

        assert len({a, a}) == 1


class TestOnlineDecidesLateRatherThanTwice:
    """Deciding on partial evidence is paid for by declining to answer, not by
    answering and revising -- a caller cannot un-act on an identity."""

    def feed(self, assoc, track, direction, n, t0, xy=(0.0, 0.0)):
        for i, vector in enumerate(views(direction, n=n)):
            assoc.observe(
                track,
                embedding=vector,
                position=(xy[0], xy[1], 0.0),
                ts=t0 + i * 0.1,
            )

    def test_a_thin_track_gets_no_id_rather_than_a_wrong_one(self):
        assoc = OnlineAssociator(ReidPolicy(min_views=5))
        self.feed(assoc, "a", (1.0, 0.0), n=2, t0=0.0)

        assert assoc.resolve("a") is None

    def test_an_id_once_given_does_not_change(self):
        assoc = OnlineAssociator(ReidPolicy(min_views=5))
        self.feed(assoc, "a", (1.0, 0.0), n=10, t0=0.0)
        first = assoc.resolve("a")

        self.feed(assoc, "a", (0.0, 1.0), n=10, t0=5.0)

        assert assoc.resolve("a") == first

    def test_the_same_thing_seen_again_gets_the_same_id(self):
        assoc = OnlineAssociator(ReidPolicy(min_views=5, similarity=0.9))
        self.feed(assoc, "a", (1.0, 0.0), n=10, t0=0.0, xy=(0.0, 0.0))
        self.feed(assoc, "b", (1.0, 0.0), n=10, t0=30.0, xy=(0.5, 0.0))

        assert assoc.resolve("a") == assoc.resolve("b")

    def test_things_seen_at_once_never_share_an_id(self):
        assoc = OnlineAssociator(ReidPolicy(min_views=5, similarity=0.9))
        self.feed(assoc, "a", (1.0, 0.0), n=10, t0=0.0)
        self.feed(assoc, "b", (1.0, 0.0), n=10, t0=0.0)  # identical, simultaneous
        assoc.co_occurring(["a", "b"])

        assert assoc.resolve("a") != assoc.resolve("b")

    def test_a_forgotten_track_does_not_crash_the_group_it_was_in(self):
        """Reading a dropped track's span raised KeyError once the budget was
        reached -- so this worked until it had run long enough to matter."""
        assoc = OnlineAssociator(
            ReidPolicy(min_views=2, similarity=0.5), OnlinePolicy(max_tracks=2)
        )
        self.feed(assoc, "first", (1.0, 0.0), n=6, t0=0.0)
        assoc.resolve("first")
        self.feed(assoc, "second", (1.0, 0.0), n=6, t0=50.0)
        assoc.resolve("second")
        self.feed(assoc, "third", (1.0, 0.0), n=6, t0=100.0)

        assert assoc.resolve("third") is not None

    def test_a_forgotten_track_is_a_new_thing_not_a_stale_match(self):
        """Reporting the id it used to have would be a match made from evidence
        that is no longer there."""
        assoc = OnlineAssociator(
            ReidPolicy(min_views=2, similarity=0.9), OnlinePolicy(max_tracks=2)
        )
        self.feed(assoc, "first", (1.0, 0.0), n=6, t0=0.0)
        original = assoc.resolve("first")

        for n in range(3):  # pushes "first" out of the budget
            self.feed(assoc, f"later{n}", (0.0, 1.0), n=6, t0=100.0 + n * 100.0)
        self.feed(assoc, "first", (1.0, 0.0), n=6, t0=500.0)

        assert assoc.resolve("first") != original

    def test_an_id_does_not_absorb_a_track_declared_a_different_object(self):
        """The exclusion is against the id, not against the one track that
        scored: entering through a third track is the same wrong answer."""
        assoc = OnlineAssociator(ReidPolicy(min_views=2, similarity=0.5))
        self.feed(assoc, "a", (1.0, 0.0), n=6, t0=0.0, xy=(0.0, 0.0))
        self.feed(assoc, "x", (1.0, 0.0), n=6, t0=0.0, xy=(0.3, 0.0))
        assoc.co_occurring(["a", "x"])
        self.feed(assoc, "b", (1.0, 0.0), n=6, t0=30.0, xy=(0.2, 0.0))

        entity = assoc.resolve("a")

        assert assoc.resolve("b") == entity  # b rejoins a, nothing forbids it
        assert assoc.resolve("x") != entity  # x cannot enter that id through b

    def test_the_track_dropped_is_the_one_least_recently_seen(self):
        """Order of first sighting is not recency: a landmark watched all shift
        was evicted ahead of a track that went quiet a minute in."""
        assoc = OnlineAssociator(ReidPolicy(min_views=2), OnlinePolicy(max_tracks=2))
        self.feed(assoc, "landmark", (1.0, 0.0), n=6, t0=0.0)
        self.feed(assoc, "passer", (0.0, 1.0), n=6, t0=10.0)
        self.feed(assoc, "landmark", (1.0, 0.0), n=6, t0=100.0)
        self.feed(assoc, "newcomer", (0.0, 1.0), n=6, t0=110.0)  # forces a drop

        assert assoc.resolve("landmark") is not None
        assert assoc.resolve("passer") is None

    def test_a_reused_key_is_not_barred_by_the_exclusion_it_inherited(self):
        """Trackers reuse ids. An exclusion that outlives the track which earned
        it refuses a match the new holder of that key never co-occurred with."""
        assoc = OnlineAssociator(
            ReidPolicy(min_views=2, similarity=0.5), OnlinePolicy(max_tracks=4)
        )
        self.feed(assoc, "reused", (1.0, 0.0), n=6, t0=0.0, xy=(5.0, 0.0))
        self.feed(assoc, "watcher", (1.0, 0.0), n=6, t0=1.0, xy=(0.0, 0.0))
        assoc.co_occurring(["watcher", "reused"])  # the old holder of the key

        for n in range(3):  # pushes "reused" out of the budget
            self.feed(assoc, f"filler{n}", (0.0, 1.0), n=6, t0=100.0 + n * 100.0)
        self.feed(assoc, "watcher", (1.0, 0.0), n=6, t0=500.0, xy=(0.0, 0.0))  # still seen

        # The tracker hands the freed key to a different object entirely, and it
        # is the one holding an id by the time the excluded track asks for one.
        self.feed(assoc, "reused", (1.0, 0.0), n=6, t0=600.0, xy=(0.5, 0.0))
        entity = assoc.resolve("reused")

        assert assoc.resolve("watcher") == entity
