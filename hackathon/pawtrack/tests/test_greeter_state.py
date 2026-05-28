"""Tests for the pure greeter phase machine."""

import json

from pawtrack.greeter_state import (
    GreeterMachine,
    GreeterObservation,
    GreeterParams,
)


def _params():
    return GreeterParams(
        standoff_m=1.5, standoff_tolerance_m=0.3,
        greet_duration_s=2.0, cooldown_duration_s=4.0,
    )


def _obs(target=False, visible=False, distance=None, reached=False):
    return GreeterObservation(
        target_acquired=target, subject_visible=visible, distance_m=distance,
        reached_destination=reached,
    )


def test_starts_wandering():
    assert GreeterMachine().state == "wander"


def test_a_visible_subject_alone_does_not_leave_wander():
    # Only a selected (random, unvisited) target leaves wander, not mere
    # visibility -- the container does the selection.
    m = GreeterMachine(_params())
    assert m.step(_obs(visible=True, distance=1.0), 0.0).state == "wander"


def test_full_cycle_wander_to_greet_and_back():
    m = GreeterMachine(_params())  # min_safe 1.0, greet window [1.0, 1.8]
    assert m.step(_obs(), 0.0).state == "wander"

    acquired = m.step(_obs(target=True, visible=True, distance=4.0), 1.0)
    assert acquired.state == "approach" and acquired.entered

    assert m.step(_obs(visible=True, distance=3.0), 1.5).state == "approach"

    greet = m.step(_obs(visible=True, distance=1.5), 2.0)  # at a safe standoff
    assert greet.state == "greet" and greet.entered  # wave, no facing required

    assert m.step(_obs(visible=True, distance=1.5), 2.5).state == "greet"

    cool = m.step(_obs(visible=True, distance=1.5), 4.0)
    assert cool.state == "cooldown" and cool.entered  # greet_duration elapsed

    assert m.step(_obs(), 7.9).state == "cooldown"  # still cooling
    back = m.step(_obs(), 8.0)  # cooldown elapsed
    assert back.state == "wander" and back.entered  # resume patrol


def test_approach_needs_distance_to_greet():
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)
    assert m.step(_obs(visible=True, distance=None), 0.5).state == "approach"


def test_approach_holds_off_greeting_until_a_safe_distance():
    m = GreeterMachine(_params())  # min_safe_distance_m default 1.0
    m.step(_obs(target=True, visible=True, distance=1.4), 0.0)  # approach
    # Within standoff but too close (0.6 < 1.0): do not wave, keep approaching.
    assert m.step(_obs(visible=True, distance=0.6), 0.1).state == "approach"
    # Backed out to a safe standoff -> greet.
    assert m.step(_obs(visible=True, distance=1.4), 0.2).state == "greet"


def test_greets_immediately_if_already_at_a_safe_distance():
    m = GreeterMachine(_params())
    # Acquired a target already within the greet window -> approach then greet.
    m.step(_obs(target=True, visible=True, distance=1.4), 0.0)  # approach
    assert m.step(_obs(visible=True, distance=1.4), 0.1).state == "greet"


def test_losing_the_subject_keeps_approaching_not_bailing():
    # A track loss no longer abandons the engagement: the container dead-reckons
    # to the last-known spot, so the machine stays in approach while not visible
    # and not yet arrived.
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)
    assert m.step(_obs(visible=False, distance=3.0), 1.0).state == "approach"
    assert m.step(_obs(visible=False, distance=2.0), 5.0).state == "approach"


def test_arriving_at_last_known_without_seeing_subject_gives_up():
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)  # approach
    # Drove to the last-known spot (reached) but still cannot see it -> give up.
    gone = m.step(_obs(visible=False, distance=0.2, reached=True), 2.0)
    assert gone.state == "wander" and gone.reason == "lost"


def test_does_not_greet_a_lost_subject_even_at_standoff():
    # At standoff distance but not visible (lost): must not wave -- the greeting
    # only fires once the subject is (re)detected.
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)  # approach
    assert m.step(_obs(visible=False, distance=1.5), 1.0).state == "approach"
    # Re-detected at standoff -> greet.
    assert m.step(_obs(visible=True, distance=1.5), 1.5).state == "greet"


def test_approach_times_out_to_wander_marked_stuck():
    m = GreeterMachine(GreeterParams(
        standoff_m=1.5, standoff_tolerance_m=0.3, engage_timeout_s=5.0))
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)  # approach
    assert m.step(_obs(visible=True, distance=4.0), 4.9).state == "approach"
    stuck = m.step(_obs(visible=True, distance=4.0), 5.1)  # stalled too long
    assert stuck.state == "wander" and stuck.reason == "stuck"


def test_greet_and_cooldown_ignore_visibility():
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=1.4), 0.0)  # approach
    m.step(_obs(visible=True, distance=1.4), 0.1)  # greet at 0.1
    assert m.step(_obs(visible=False), 1.0).state == "greet"
    assert m.step(_obs(visible=False), 2.2).state == "cooldown"


def test_reset_returns_to_clean_wander():
    m = GreeterMachine(_params())
    m.step(_obs(target=True, visible=True, distance=4.0), 0.0)  # -> approach
    assert m.state == "approach"
    m.reset()
    assert m.state == "wander"
    # Clean start: only a freshly acquired target leaves wander, not old phase.
    assert m.step(_obs(visible=True, distance=1.0), 1.0).state == "wander"


def test_snapshot_json_round_trips():
    m = GreeterMachine(_params())
    snap = m.step(_obs(target=True, visible=True, distance=4.0), 1.0)
    data = json.loads(snap.to_json())
    assert data["state"] == "approach"
    assert data["entered"] is True
    assert "message" in data
