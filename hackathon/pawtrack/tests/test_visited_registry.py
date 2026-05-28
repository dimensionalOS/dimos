"""Tests for the pure visited-subject memory and target selection."""

import random

import pytest

from pawtrack.visited_registry import (
    Candidate,
    VisitedRegistry,
    select_target,
)


def _cand(x, y, bbox=(0.0, 0.0, 10.0, 10.0)):
    return Candidate(position=(x, y), bbox=bbox)


def test_marks_and_detects_visited_within_radius():
    registry = VisitedRegistry(revisit_radius_m=1.0)
    assert not registry.is_visited((0.0, 0.0))
    registry.mark_visited((0.0, 0.0))
    assert registry.is_visited((0.5, 0.0))  # within the radius -> same subject
    assert not registry.is_visited((2.0, 0.0))  # a different subject


def test_unvisited_filters_already_greeted():
    registry = VisitedRegistry(revisit_radius_m=1.0)
    registry.mark_visited((0.0, 0.0))
    candidates = [_cand(0.2, 0.0), _cand(3.0, 0.0)]
    assert registry.unvisited(candidates) == [candidates[1]]


def test_select_target_never_picks_a_visited_subject():
    registry = VisitedRegistry(revisit_radius_m=1.0)
    registry.mark_visited((0.0, 0.0))
    candidates = [_cand(0.1, 0.0), _cand(5.0, 0.0), _cand(6.0, 0.0)]
    for seed in range(20):
        chosen = select_target(candidates, registry, random.Random(seed))
        assert chosen in candidates[1:]  # never the greeted one


def test_select_target_none_when_all_visited():
    registry = VisitedRegistry(revisit_radius_m=1.0)
    registry.mark_visited((0.0, 0.0))
    registry.mark_visited((5.0, 0.0))
    candidates = [_cand(0.1, 0.0), _cand(5.1, 0.0)]
    assert select_target(candidates, registry, random.Random(0)) is None


def test_select_target_none_when_no_candidates():
    assert select_target([], VisitedRegistry(1.0), random.Random(0)) is None


def test_select_target_is_random_yet_reproducible():
    candidates = [_cand(float(i), 0.0) for i in range(10)]
    a = select_target(candidates, VisitedRegistry(0.1), random.Random(3))
    b = select_target(candidates, VisitedRegistry(0.1), random.Random(3))
    assert a == b  # same seed -> same pick

    picks = {
        select_target(
            candidates, VisitedRegistry(0.1), random.Random(seed)
        ).position
        for seed in range(20)
    }
    assert len(picks) > 1  # actually varies across seeds


def test_revisit_radius_must_be_positive():
    with pytest.raises(ValueError):
        VisitedRegistry(0.0)


def test_forget_after_s_must_be_positive_or_none():
    VisitedRegistry(1.0, forget_after_s=None)  # no forget -> ok
    VisitedRegistry(1.0, forget_after_s=30.0)  # ok
    with pytest.raises(ValueError):
        VisitedRegistry(1.0, forget_after_s=0.0)


def test_greeting_is_forgotten_after_the_forget_window():
    registry = VisitedRegistry(revisit_radius_m=1.0, forget_after_s=60.0)
    registry.mark_visited((0.0, 0.0), now=10.0)
    assert registry.is_visited((0.2, 0.0), now=40.0)  # within the window
    assert registry.is_visited((0.2, 0.0), now=70.0)  # exactly at the window
    assert not registry.is_visited((0.2, 0.0), now=71.0)  # forgotten


def test_select_target_repicks_a_subject_after_it_is_forgotten():
    registry = VisitedRegistry(revisit_radius_m=1.0, forget_after_s=60.0)
    candidates = [_cand(0.0, 0.0)]
    registry.mark_visited((0.0, 0.0), now=0.0)
    # Still remembered -> skipped.
    assert select_target(candidates, registry, random.Random(0), now=30.0) is None
    # Forgotten -> greet-eligible again.
    chosen = select_target(candidates, registry, random.Random(0), now=90.0)
    assert chosen == candidates[0]


def test_none_forget_never_expires():
    registry = VisitedRegistry(revisit_radius_m=1.0, forget_after_s=None)
    registry.mark_visited((0.0, 0.0), now=0.0)
    assert registry.is_visited((0.0, 0.0), now=1_000_000.0)  # never forgotten
