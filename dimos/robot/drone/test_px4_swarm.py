#!/usr/bin/env python3
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

"""Unit tests for the namespaced PX4 swarm.

Covers the three things the architecture split is supposed to guarantee:

1. Blueprint composition — N drones really do land in N namespaces, per-drone
   streams get prefixed, and the fleet bus (``drone_state``/``swarm_cmd``)
   stays global so it can cross the boundary.
2. The command bus — the coordinator's broadcasts reach the right drones and
   only the right drones.
3. The fleet logic that used to live in ``Px4SitlFleetModule`` — spacing
   guardrail, altitude caps, sweep geometry, formations — still behaves.

None of this needs a live MAVLink link or a running sim.
"""

from __future__ import annotations

from itertools import pairwise
import json
import math
import threading
import time
from types import SimpleNamespace
from typing import Any

from dimos_lcm.std_msgs import String
import pytest

from dimos.robot.drone.px4_drone_module import Px4DroneModule
from dimos.robot.drone.px4_geo import distance_3d_m, offset_latlon, pairwise_distances
from dimos.robot.drone.px4_swarm_coordinator import STATE_STALE_AFTER_SEC, SwarmCoordinator

# A reference point with easy arithmetic (Zurich Irchel is PX4's SITL default home).
HOME_LAT, HOME_LON = 47.397742, 8.545594


# ---------------------------------------------------------------------------
# 1. Blueprint composition / namespacing
# ---------------------------------------------------------------------------


def _swarm_blueprint(n: int = 3):
    from dimos.robot.drone.blueprints.basic.drone_px4_sitl_fleet_mcp import px4_sitl_swarm
    from dimos.robot.drone.px4_sitl_fleet_config import get_px4_sitl_fleet_configs

    return px4_sitl_swarm(configs=get_px4_sitl_fleet_configs(fleet_size=n))


def test_each_drone_gets_its_own_namespaced_instance():
    bp = _swarm_blueprint(3)
    names = {a.name for a in bp.blueprints}
    assert names == {
        "swarmcoordinator",
        "drone1/px4dronemodule",
        "drone2/px4dronemodule",
        "drone3/px4dronemodule",
    }


def test_fleet_size_is_dynamic():
    assert len([a for a in _swarm_blueprint(2).blueprints if "px4dronemodule" in a.name]) == 2
    assert len([a for a in _swarm_blueprint(5).blueprints if "px4dronemodule" in a.name]) == 5


def test_per_drone_streams_are_prefixed():
    """cmd_vel and odom must NOT be shared, or one tracker would fly every drone."""
    remap = _swarm_blueprint(3).remapping_map
    for i in (1, 2, 3):
        assert remap[(f"drone{i}/px4dronemodule", "cmd_vel")] == f"drone{i}/cmd_vel"
        assert remap[(f"drone{i}/px4dronemodule", "odom")] == f"drone{i}/odom"


def test_fleet_bus_streams_stay_global():
    """drone_state/swarm_cmd are exposed, so they must have no namespace remap."""
    remap = _swarm_blueprint(3).remapping_map
    for i in (1, 2, 3):
        assert (f"drone{i}/px4dronemodule", "drone_state") not in remap
        assert (f"drone{i}/px4dronemodule", "swarm_cmd") not in remap


def test_each_drone_gets_its_own_connection_string():
    bp = _swarm_blueprint(3)
    conns = {
        a.name: a.kwargs["connection_string"] for a in bp.blueprints if "px4dronemodule" in a.name
    }
    assert conns == {
        "drone1/px4dronemodule": "udp:127.0.0.1:14540",
        "drone2/px4dronemodule": "udp:127.0.0.1:14541",
        "drone3/px4dronemodule": "udp:127.0.0.1:14542",
    }


def test_config_keys_are_unique_per_instance():
    """Namespacing is what makes N instances of one class configurable separately."""
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser

    parsed = BlueprintConfigParser(_swarm_blueprint(3)).parse(
        environ={}, overrides={"g": {"viewer": "none"}}
    )
    assert {
        "drone1/px4dronemodule",
        "drone2/px4dronemodule",
        "drone3/px4dronemodule",
        "swarmcoordinator",
    } <= set(parsed.module_configs)


def test_a_single_drone_can_be_reconfigured_without_touching_the_others():
    """`-o drone2/px4dronemodule.max_altitude_m=15` must hit only drone2."""
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser

    parsed = BlueprintConfigParser(_swarm_blueprint(3)).parse(
        environ={},
        overrides={
            "g": {"viewer": "none"},
            "drone2_px4dronemodule": {"max_altitude_m": 15.0},
        },
    )
    assert parsed.module_kwargs("drone2/px4dronemodule")["max_altitude_m"] == 15.0
    assert parsed.module_kwargs("drone1/px4dronemodule").get("max_altitude_m") != 15.0


# ---------------------------------------------------------------------------
# 2. Drone identity + command bus
# ---------------------------------------------------------------------------


def _bare_drone(instance_name: str | None = "drone2/px4dronemodule", max_alt: float | None = None):
    m = object.__new__(Px4DroneModule)
    m.config = SimpleNamespace(instance_name=instance_name, max_altitude_m=max_alt)
    m._follow_enabled = False
    m._follow_lock_altitude = True
    m.connection = None
    return m


def test_drone_key_comes_from_the_namespace():
    assert _bare_drone("drone2/px4dronemodule").drone_key == "drone2"
    assert _bare_drone("drone11/px4dronemodule").drone_key == "drone11"


def test_drone_key_falls_back_when_not_namespaced():
    assert _bare_drone(None).drone_key == "drone1"


def _dispatched(drone: Px4DroneModule) -> list[tuple[str, dict[str, Any]]]:
    """Record what _on_swarm_cmd actually dispatches, without touching MAVLink."""
    seen: list[tuple[str, dict[str, Any]]] = []
    drone._dispatch = lambda action, args: (  # type: ignore[method-assign]
        seen.append((action, args)) or "ok"
    )
    return seen


def _cmd(target: str, action: str, **args: Any) -> String:
    return String(json.dumps({"seq": 1, "target": target, "action": action, "args": args}))


def test_drone_acts_on_commands_addressed_to_it():
    d = _bare_drone("drone2/px4dronemodule")
    seen = _dispatched(d)
    d._on_swarm_cmd(_cmd("drone2", "takeoff", altitude=5.0))
    assert seen == [("takeoff", {"altitude": 5.0})]


def test_drone_ignores_commands_for_other_drones():
    d = _bare_drone("drone2/px4dronemodule")
    seen = _dispatched(d)
    d._on_swarm_cmd(_cmd("drone1", "takeoff", altitude=5.0))
    assert seen == []


def test_drone_acts_on_broadcast_commands():
    d = _bare_drone("drone2/px4dronemodule")
    seen = _dispatched(d)
    d._on_swarm_cmd(_cmd("all", "rtl"))
    assert seen == [("rtl", {})]


def test_malformed_command_is_ignored_not_raised():
    d = _bare_drone("drone2/px4dronemodule")
    _dispatched(d)
    d._on_swarm_cmd(String("not json at all"))  # must not raise


def test_unknown_action_is_reported_not_raised():
    d = _bare_drone("drone2/px4dronemodule")
    assert "unknown action" in d._dispatch("fly_to_the_moon", {})


def test_altitude_cap_rejects_and_allows():
    d = _bare_drone("drone1/px4dronemodule", max_alt=30.0)
    assert d._check_altitude_cap(10.0) is None
    assert "REJECTED" in (d._check_altitude_cap(50.0) or "")
    # No cap configured -> nothing is rejected.
    assert _bare_drone("drone1/px4dronemodule", max_alt=None)._check_altitude_cap(500.0) is None


def test_takeoff_is_blocked_by_the_cap_before_touching_mavlink():
    d = _bare_drone("drone1/px4dronemodule", max_alt=30.0)
    d.connection = None  # would fail with NOT CONNECTED if the cap did not fire first
    assert "REJECTED" in d.takeoff(999.0)


# ---------------------------------------------------------------------------
# 3. Coordinator: state aggregation, guardrails, maneuvers
# ---------------------------------------------------------------------------


class _FakeOut:
    def __init__(self) -> None:
        self.published: list[dict[str, Any]] = []

    def publish(self, msg: String) -> None:
        self.published.append(json.loads(msg.data if hasattr(msg, "data") else str(msg)))


def _coordinator(
    min_separation_m: float = 2.0,
    max_altitude_m: float | None = None,
    expected: str = "",
    min_battery_pct: float = 25.0,
    max_range_m: float = 250.0,
) -> SwarmCoordinator:
    c = object.__new__(SwarmCoordinator)
    c.config = SimpleNamespace(
        min_separation_m=min_separation_m,
        max_altitude_m=max_altitude_m,
        expected_drones=expected,
        min_battery_pct=min_battery_pct,
        max_range_m=max_range_m,
    )
    c._states = {}
    c._lock = threading.RLock()
    c._seq = 0
    c._gmaps_client = False
    c.swarm_cmd = _FakeOut()

    # Delivery verification runs synchronously and instantly in tests: no
    # leaked threads, deterministic message counts, and the resend behavior
    # itself is covered by its own tests below.
    c._verify_delay_s = 0.0
    c._start_fleet_verify = lambda action, keys: c._verify_fleet_command(action, keys)
    return c


def _report(c: SwarmCoordinator, key: str, n: float, e: float, alt: float, **extra: Any) -> None:
    """Feed one drone_state message in, positioned n/e meters from HOME.

    Armed by default: the position maneuvers require an airborne aircraft, so an
    unarmed fleet is the exception rather than the norm in these tests.
    """
    lat, lon = offset_latlon(HOME_LAT, HOME_LON, n, e)
    state = {
        "key": key,
        "robot_class": "multirotor",
        "connected": True,
        "armed": True,
        "battery_pct": 95.0,
        "ned": [n, e, -alt],
        "altitude_m": alt,
        "global": {"lat": lat, "lon": lon, "rel_alt_m": alt},
        "ts": time.time(),
    }
    state.update(extra)
    c._on_drone_state(String(json.dumps(state)))


def test_coordinator_aggregates_state_from_the_bus():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 10, 0, 5)
    state = json.loads(c.fleet_state())
    assert set(state["drones"]) == {"drone1", "drone2"}
    assert state["drones"]["drone1"]["battery_pct"] == 95.0


def test_pairwise_distances_are_world_frame():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 30, 0, 5)
    pairs = json.loads(c.fleet_state())["pairwise_m"]
    assert len(pairs) == 1
    assert pairs[0]["distance"] == pytest.approx(30.0, abs=0.5)


def test_stale_drones_are_flagged_and_excluded_from_geometry():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 10, 0, 5)
    # Backdate drone2 past the staleness horizon.
    c._states["drone2"]["ts"] = time.time() - (STATE_STALE_AFTER_SEC + 10)
    snap = json.loads(c.fleet_state())
    assert snap["drones"]["drone2"]["stale"] is True
    assert snap["pairwise_m"] == []  # a stale drone can't anchor guardrail math


def test_expected_but_silent_drones_are_reported():
    c = _coordinator(expected="drone1,drone2,drone3")
    _report(c, "drone1", 0, 0, 5)
    drones = json.loads(c.fleet_state())["drones"]
    assert drones["drone3"]["never_reported"] is True
    assert drones["drone3"]["connected"] is False


def test_count_within_matches_the_pdf_demo_question():
    """"How many units are within 100 meters of whatever drone"."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 50, 0, 5)  # inside 100 m
    _report(c, "drone3", 500, 0, 5)  # outside
    out = c.count_within("drone1", 100.0)
    assert out.startswith("1 robot(s) within 100m of drone1")
    assert "drone2" in out and "drone3" not in out


def test_count_within_unknown_drone_is_reported():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    assert "no usable global position" in c.count_within("ghost", 100.0)


# --- spacing guardrail -----------------------------------------------------


def test_guardrail_rejects_a_target_too_close_to_another_drone():
    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 20, 0, 5)
    # Send drone1 to within 1 m of drone2.
    out = c.goto_drone("drone1", north=19.0, east=0.0, altitude=5.0)
    assert "REJECTED" in out and "drone2" in out
    assert c.swarm_cmd.published == []  # nothing dispatched


def test_guardrail_allows_a_well_separated_target():
    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 20, 0, 5)
    out = c.goto_drone("drone1", north=5.0, east=0.0, altitude=5.0)
    assert "REJECTED" not in out
    assert c.swarm_cmd.published[-1]["action"] == "goto_ned"
    assert c.swarm_cmd.published[-1]["target"] == "drone1"


def test_guardrail_fails_closed_without_telemetry():
    """"Cannot verify" is not "safe". No position -> refuse, do not hope."""
    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 5)
    reason = c._violates_separation("unknown-drone", 0.0, 0.0, -5.0)
    assert reason is not None and "cannot verify" in reason


def test_guardrail_refuses_to_use_stale_positions():
    """At 5 m/s a stale fix is metres of travel; deciding separation on it is
    arithmetic theatre, so the guardrail must refuse rather than answer."""
    from dimos.robot.drone.px4_swarm_coordinator import GUARDRAIL_MAX_AGE_SEC

    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 20, 0, 5)
    c._states["drone1"]["ts"] = time.time() - (GUARDRAIL_MAX_AGE_SEC + 0.5)
    reason = c._violates_separation("drone1", 19.0, 0.0, -5.0)
    assert reason is not None and "cannot verify" in reason


def test_guardrail_still_answers_on_fresh_positions():
    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 20, 0, 5)
    assert c._violates_separation("drone1", 5.0, 0.0, -5.0) is None      # clear
    assert c._violates_separation("drone1", 19.0, 0.0, -5.0) is not None  # conflict


def test_fleet_altitude_cap_rejects_before_dispatch():
    c = _coordinator(max_altitude_m=30.0)
    _report(c, "drone1", 0, 0, 5)
    assert "REJECTED" in c.takeoff_all(100.0)
    assert c.swarm_cmd.published == []


# --- aggregate commands ----------------------------------------------------


def test_takeoff_all_broadcasts_once():
    """takeoff is the one aggregate that stays single-shot: it is not a safety
    verb, and a drone that misses it just stays on the ground -- visible and
    harmless, unlike a drone that misses an rtl."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    c.takeoff_all(5.0)
    assert [m["action"] for m in c.swarm_cmd.published] == ["takeoff"]


@pytest.mark.parametrize(
    ("call", "action"),
    [
        ("land_all", "land"),
        ("rtl_all", "rtl"),
        ("hold_all", "hold"),
        ("emergency_land_all", "emergency_land"),
        ("kill_all", "kill"),
    ],
)
def test_safety_commands_broadcast_three_times_to_all(call: str, action: str):
    """Safety verbs repeat 3x on purpose: the bus is best-effort, and one lost
    rtl_all broadcast left an armed drone hovering while its twin landed. All
    of these are idempotent, so repetition is free insurance."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 10, 0, 5)
    # Make both fakes look ALREADY compliant so the (synchronous, separately
    # tested) delivery verification stays quiet and this counts only the
    # broadcast itself.
    compliant_mode = {"land": "AUTO.LAND", "rtl": "AUTO.RTL", "hold": "AUTO.LOITER",
                      "emergency_land": "AUTO.LAND", "kill": None}[action]
    with c._lock:
        for k in ("drone1", "drone2"):
            c._states[k]["mode"] = compliant_mode
            c._states[k]["armed"] = False
    getattr(c, call)()
    assert len(c.swarm_cmd.published) == 3
    seqs = [m["seq"] for m in c.swarm_cmd.published]
    assert seqs == sorted(seqs) and len(set(seqs)) == 3
    for m in c.swarm_cmd.published:
        assert m["target"] == "all" and m["action"] == action


def test_command_sequence_numbers_increase():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    c.rtl_all()
    c.hold_all()
    seqs = [m["seq"] for m in c.swarm_cmd.published]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)


# --- formations and sweeps -------------------------------------------------


def test_line_formation_spaces_drones_at_least_min_separation_apart():
    c = _coordinator(min_separation_m=4.0)
    for i in range(3):
        _report(c, f"drone{i + 1}", i * 10, 0, 5)
    c.line_formation(center_north=0, center_east=0, altitude=5, spacing_m=1.0, heading_deg=90.0)
    targets = [
        (m["args"]["north"], m["args"]["east"])
        for m in c.swarm_cmd.published
        if m["action"] == "goto_ned"
    ]
    assert len(targets) == 3
    easts = sorted(e for _n, e in targets)
    # spacing_m=1.0 was below the guardrail, so it must have been raised.
    assert all(b - a >= 4.0 for a, b in pairwise(easts))


def test_grid_sweep_gives_each_drone_its_own_strip():
    c = _coordinator(min_separation_m=2.0)
    for i in range(3):
        _report(c, f"drone{i + 1}", 0, i * 5, 5)
    c.grid_sweep(0, 0, 60, 30, altitude=8, sub_lane_spacing_m=5)
    paths = {m["target"]: m["args"]["path"] for m in c.swarm_cmd.published if m["action"] == "path"}
    assert set(paths) == {"drone1", "drone2", "drone3"}
    # Strips must not overlap: every drone's east range is disjoint from the others.
    ranges = []
    for path in paths.values():
        easts = [wp[1] for wp in path]
        ranges.append((min(easts), max(easts)))
    ranges.sort()
    for (_lo_a, hi_a), (lo_b, _hi_b) in pairwise(ranges):
        assert lo_b > hi_a, f"strips overlap: {ranges}"


def test_grid_sweep_strips_respect_the_separation_buffer():
    c = _coordinator(min_separation_m=6.0)
    for i in range(3):
        _report(c, f"drone{i + 1}", 0, i * 5, 5)
    c.grid_sweep(0, 0, 60, 60, altitude=8, sub_lane_spacing_m=5)
    ranges = sorted(
        (min(wp[1] for wp in m["args"]["path"]), max(wp[1] for wp in m["args"]["path"]))
        for m in c.swarm_cmd.published
        if m["action"] == "path"
    )
    for (_lo_a, hi_a), (lo_b, _hi_b) in pairwise(ranges):
        assert lo_b - hi_a >= 6.0 - 1e-6


def test_grid_sweep_is_a_boustrophedon_not_a_teleport():
    """Consecutive passes must reverse direction, so the drone never jumps."""
    c = _coordinator(min_separation_m=2.0)
    _report(c, "drone1", 0, 0, 5)
    c.grid_sweep(0, 0, 50, 20, altitude=8, sub_lane_spacing_m=5)
    path = next(m["args"]["path"] for m in c.swarm_cmd.published if m["action"] == "path")
    # Waypoints come in (start, end) pairs per lane; each pair flips direction.
    for i in range(0, len(path) - 2, 2):
        this_dir = path[i + 1][0] - path[i][0]
        next_dir = path[i + 3][0] - path[i + 2][0]
        assert this_dir * next_dir < 0, "consecutive lanes must run opposite ways"


def test_grid_sweep_rejects_an_area_too_narrow_for_the_buffer():
    c = _coordinator(min_separation_m=10.0)
    for i in range(3):
        _report(c, f"drone{i + 1}", 0, i * 5, 5)
    out = c.grid_sweep(0, 0, 30, 6, altitude=8)  # 2 m strips, 10 m buffer needed
    assert "REJECTED" in out
    assert c.swarm_cmd.published == []


def test_grid_sweep_altitude_is_applied_to_every_waypoint():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    c.grid_sweep(0, 0, 40, 20, altitude=12)
    path = next(m["args"]["path"] for m in c.swarm_cmd.published if m["action"] == "path")
    assert all(wp[2] == pytest.approx(-12.0) for wp in path)  # NED down is negative altitude


# --- investigate -----------------------------------------------------------


def test_investigate_sends_exactly_two_units_by_default():
    """The PDF's "Task two units to investigate this coordinate"."""
    c = _coordinator()
    for i in range(3):
        _report(c, f"drone{i + 1}", i * 30, 0, 5)
    c.investigate(north=100, east=50, altitude=6, num_drones=2)
    sent = [m for m in c.swarm_cmd.published if m["action"] == "goto_ned"]
    assert len(sent) == 2


def test_investigate_staggers_targets_beyond_min_separation():
    c = _coordinator(min_separation_m=4.0)
    for i in range(2):
        _report(c, f"drone{i + 1}", i * 40, 0, 5)
    c.investigate(north=200, east=0, altitude=6, num_drones=2)
    norths = sorted(m["args"]["north"] for m in c.swarm_cmd.published if m["action"] == "goto_ned")
    assert norths[1] - norths[0] >= 4.0


def test_investigate_honours_an_explicit_drone_list():
    c = _coordinator()
    for i in range(3):
        _report(c, f"drone{i + 1}", i * 30, 0, 5)
    c.investigate(north=100, east=0, drones="drone1,drone3")
    targets = {m["target"] for m in c.swarm_cmd.published if m["action"] == "goto_ned"}
    assert targets == {"drone1", "drone3"}


def test_investigate_rejects_unknown_drone_names():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    out = c.investigate(north=10, east=0, drones="drone1,ghost")
    assert "Unknown drone" in out and "ghost" in out
    assert c.swarm_cmd.published == []


def test_fleet_skills_report_clearly_when_no_drone_has_checked_in():
    c = _coordinator()
    assert "No aircraft" in c.grid_sweep(0, 0, 30, 30)
    assert "No aircraft" in c.line_formation()
    assert "No aircraft" in c.investigate(north=1, east=1)
    assert "No robots have reported yet" in c.list_drones()


# ---------------------------------------------------------------------------
# Mixed fleet: air maneuvers must not be handed to ground robots
# ---------------------------------------------------------------------------


def _report_ground(c: SwarmCoordinator, key: str, n: float, e: float) -> None:
    """Feed one legged-robot state message in."""
    lat, lon = offset_latlon(HOME_LAT, HOME_LON, n, e)
    c._on_drone_state(
        String(
            json.dumps(
                {
                    "key": key,
                    "robot_class": "legged",
                    "connected": True,
                    "ned": [n, e, -0.4],
                    "altitude_m": 0.4,
                    "battery_pct": 100.0,
                    "global": {"lat": lat, "lon": lon, "rel_alt_m": 0.4},
                    "ts": time.time(),
                }
            )
        )
    )


def test_grid_sweep_does_not_hand_an_air_lane_to_a_ground_robot():
    """A quadruped assigned a sweep strip silently loses that share of coverage."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 10, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    c.grid_sweep(0, 0, 60, 40, altitude=8)
    targets = {m["target"] for m in c.swarm_cmd.published if m["action"] == "path"}
    assert targets == {"drone1", "drone2"}, f"ground robot got an air lane: {targets}"


def test_investigate_counts_only_aircraft_towards_num_drones():
    """num_drones=2 must mean two *aircraft*, not one aircraft and a dog."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 40, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    c.investigate(north=100, east=0, altitude=6, num_drones=2)
    sent = {m["target"] for m in c.swarm_cmd.published if m["action"] == "goto_ned"}
    assert sent == {"drone1", "drone2"}


def test_line_formation_excludes_ground_robots():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    out = c.line_formation(altitude=6)
    targets = {m["target"] for m in c.swarm_cmd.published if m["action"] == "goto_ned"}
    assert targets == {"drone1"}
    assert "dog1" in out  # still reported as skipped, not silently dropped


def test_goto_drone_rejects_a_ground_robot_with_a_clear_reason():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    out = c.goto_drone("dog1", north=10, east=0, altitude=6)
    assert "ground robot" in out
    assert c.swarm_cmd.published == []


def test_aggregate_commands_still_reach_ground_robots():
    """rtl/land are broadcast verbs every robot class translates for itself."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    out = c.rtl_all()
    assert "dog1" in out and "drone1" in out
    assert c.swarm_cmd.published[-1]["target"] == "all"


def test_fleet_view_reports_both_classes():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", 0, 20)
    listing = c.list_drones()
    assert "1 aircraft" in listing and "1 ground" in listing
    # count_within is world-frame geometry and must span classes.
    assert "dog1" in c.count_within("drone1", 100.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def test_offset_latlon_round_trips_through_distance():
    lat, lon = offset_latlon(HOME_LAT, HOME_LON, 100.0, 0.0)
    assert distance_3d_m((HOME_LAT, HOME_LON, 0), (lat, lon, 0)) == pytest.approx(100.0, abs=0.5)


def test_distance_3d_includes_altitude():
    a = (HOME_LAT, HOME_LON, 0.0)
    b = (HOME_LAT, HOME_LON, 30.0)
    assert distance_3d_m(a, b) == pytest.approx(30.0, abs=0.01)


def test_pairwise_distances_are_symmetric_and_complete():
    lat2, lon2 = offset_latlon(HOME_LAT, HOME_LON, 10, 0)
    lat3, lon3 = offset_latlon(HOME_LAT, HOME_LON, 0, 10)
    pairs = pairwise_distances(
        {"a": (HOME_LAT, HOME_LON, 0), "b": (lat2, lon2, 0), "c": (lat3, lon3, 0)}
    )
    assert len(pairs) == 3  # 3 choose 2
    assert {(a, b) for a, b, _ in pairs} == {("a", "b"), ("a", "c"), ("b", "c")}


# ---------------------------------------------------------------------------
# Preflight check (the PDF's hardware safety checklist, as a gate)
# ---------------------------------------------------------------------------


def test_preflight_passes_on_a_healthy_fleet():
    c = _coordinator(min_separation_m=2.0)
    for i in range(3):
        _report(c, f"drone{i + 1}", i * 10, 0, 0, sys_id=i + 1, armed=False)
    assert "PASS" in c.preflight_check()


def test_preflight_fails_with_no_drones():
    assert "FAIL" in _coordinator().preflight_check()


def test_preflight_catches_duplicate_sys_ids():
    """The most common real-hardware mistake: every Pixhawk ships as MAV_SYS_ID 1."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    _report(c, "drone2", 20, 0, 0, sys_id=1, armed=False)
    out = c.preflight_check()
    assert "FAIL" in out and "MAV_SYS_ID 1 shared by" in out


def test_preflight_catches_low_battery():
    c = _coordinator(min_battery_pct=40.0)
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False, battery_pct=12.0)
    out = c.preflight_check()
    assert "FAIL" in out and "battery 12% below floor" in out


def test_preflight_catches_a_drone_already_armed():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=True)
    out = c.preflight_check()
    assert "FAIL" in out and "already ARMED" in out


def test_preflight_catches_drones_parked_too_close():
    c = _coordinator(min_separation_m=10.0)
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    _report(c, "drone2", 3, 0, 0, sys_id=2, armed=False)
    out = c.preflight_check()
    assert "FAIL" in out and "below the 10.0m minimum" in out


def test_preflight_catches_missing_gps_fix():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    c._states["drone1"]["global"] = None
    out = c.preflight_check()
    assert "FAIL" in out and "no global position fix" in out


def test_preflight_catches_stale_telemetry():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    c._states["drone1"]["ts"] = time.time() - (STATE_STALE_AFTER_SEC + 30)
    out = c.preflight_check()
    assert "FAIL" in out and "telemetry stale" in out


def test_preflight_reports_expected_but_absent_drones():
    c = _coordinator(expected="drone1,drone2,drone3")
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    out = c.preflight_check()
    assert "drone2: expected but never reported" in out
    assert "drone3: expected but never reported" in out


def test_preflight_always_lists_the_manual_checks():
    """Things software cannot verify must still be put in front of the operator."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    out = c.preflight_check()
    assert "RC transmitter bound" in out
    assert "Geofence configured" in out


def test_preflight_never_commands_anything():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, sys_id=1, armed=False)
    c.preflight_check()
    assert c.swarm_cmd.published == []


def test_position_maneuvers_refuse_when_nothing_is_airborne():
    """A grid_sweep on grounded drones engages OFFBOARD and trips a failsafe that
    then blocks the next arm, so it must be refused rather than dispatched."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 0, armed=False)
    _report(c, "drone2", 10, 0, 0, armed=False)
    for out in (
        c.grid_sweep(0, 0, 40, 30),
        c.line_formation(),
        c.investigate(north=10, east=0),
        c.goto_drone("drone1", north=5, east=0, altitude=5),
    ):
        assert "REJECTED" in out and "takeoff_all first" in out
    assert c.swarm_cmd.published == []


def test_position_maneuvers_use_only_airborne_aircraft():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 20, 0, 5)
    _report(c, "drone3", 40, 0, 0, armed=False)  # still on the ground
    c.grid_sweep(0, 0, 40, 30, altitude=8)
    targets = {m["target"] for m in c.swarm_cmd.published if m["action"] == "path"}
    assert targets == {"drone1", "drone2"}


def test_preflight_does_not_fail_on_closely_parked_ground_robots():
    """The separation floor is a flight limit; two dogs standing a metre apart
    is normal and must not FAIL preflight for the whole fleet."""
    c = _coordinator(min_separation_m=2.0)
    _report(c, "drone1", 0, 0, 0, armed=False)
    _report_ground(c, "dog1", -4.0, 0.0)
    _report_ground(c, "dog2", -5.5, 0.0)  # 1.5 m from dog1
    out = c.preflight_check()
    assert "PASS" in out, out
    assert "not a flight limit" in out


def test_preflight_still_fails_on_close_aircraft():
    c = _coordinator(min_separation_m=5.0)
    _report(c, "drone1", 0, 0, 0, armed=False)
    _report(c, "drone2", 1.0, 0, 0, armed=False)
    out = c.preflight_check()
    assert "FAIL" in out and "below the" in out


# ---------------------------------------------------------------------------
# Operating-radius boundary
# ---------------------------------------------------------------------------


def test_goto_drone_rejects_a_target_beyond_the_operating_radius():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    out = c.goto_drone("drone1", north=300.0, east=0.0, altitude=6.0)
    assert "REJECTED" in out and "250" in out
    assert c.swarm_cmd.published == []


def test_goto_drone_accepts_a_target_inside_the_radius():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    out = c.goto_drone("drone1", north=249.0, east=0.0, altitude=6.0)
    assert "REJECTED" not in out
    assert len(c.swarm_cmd.published) == 1


def test_grid_sweep_rejects_when_the_far_corner_is_outside_the_fence():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    out = c.grid_sweep(corner_b_north=400.0, corner_b_east=100.0)
    assert "REJECTED" in out and "far corner" in out
    assert c.swarm_cmd.published == []


def test_investigate_checks_the_staggered_extreme_not_the_raw_target():
    """Two drones staggered around a target near the fence: the outermost
    assignment crosses it even though the target itself does not."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 0, 10, 5)
    out = c.investigate(north=249.0, east=0.0, num_drones=2)
    assert "REJECTED" in out
    assert c.swarm_cmd.published == []


def test_boundaries_reports_every_active_limit():
    c = _coordinator(max_altitude_m=40.0)
    out = c.boundaries()
    assert "250 m" in out
    assert "40 m" in out
    assert "2.0 m" in out
    assert "geofence" in out


# ---------------------------------------------------------------------------
# Staged sweep-then-ground mission
# ---------------------------------------------------------------------------


def test_sweep_then_ground_needs_a_ground_robot():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    out = c.sweep_then_ground()
    assert "no ground robot" in out


def test_sweep_then_ground_inherits_grid_sweep_gates():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", -4, 0)
    out = c.sweep_then_ground(corner_b_north=400.0, corner_b_east=100.0)
    assert "REJECTED" in out and "far corner" in out
    assert c.swarm_cmd.published == []


def test_sweep_then_ground_dispatches_air_now_and_ground_after_delay():
    c = _coordinator()
    c._ground_wait_timeout_s = 0.05  # dog never "arrives"; move on fast
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", -4, 0)
    out = c.sweep_then_ground(ground_delay_s=0.0)
    assert "Staged sweep started" in out
    # The air sweep is on the bus immediately (published messages arrive parsed).
    assert "path" in [m["action"] for m in c.swarm_cmd.published]
    # The ground legs appear as the thread times through its waypoints.
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if [m["action"] for m in c.swarm_cmd.published].count("ground_goto") >= 4:
            break
        time.sleep(0.05)
    gg = [m for m in c.swarm_cmd.published if m["action"] == "ground_goto"]
    assert len(gg) == 4
    assert gg[0]["target"] == "dog1"
    assert [g["args"]["north"] for g in gg][-1] == -4.0  # last leg is home
    assert "complete" in c.mission_status() or "transect" in c.mission_status()


def test_emergency_aborts_the_ground_stage():
    c = _coordinator()
    c._ground_wait_timeout_s = 30.0
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", -4, 0)
    c.sweep_then_ground(ground_delay_s=30.0)  # long delay: still in air stage
    c.emergency_land_all()
    deadline = time.time() + 3.0
    while time.time() < deadline and "aborted" not in c.mission_status():
        time.sleep(0.05)
    assert "aborted" in c.mission_status()


def test_legged_module_parses_ground_goto_from_the_coordinator_wire_format():
    """The bug this pins: the handler read north/east from the TOP level of the
    message, but _send() nests kwargs under "args" -- so the dog walked to the
    (0, 0) defaults instead of its waypoint and the staged mission hung. The
    coordinator-side tests asserted the message shape; nothing checked the
    consumer parsed it. This does, end to end across the wire format."""
    from unittest.mock import MagicMock

    from dimos.robot.legged.legged_sim_module import LeggedSimModule

    # Build the exact payload _send() produces.
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report_ground(c, "dog1", -4, 0)
    c._send("dog1", "ground_goto", north=12.5, east=-7.0)
    wire = json.dumps(c.swarm_cmd.published[-1])

    dog = object.__new__(LeggedSimModule)
    # robot_key is a property over config.instance_name
    dog.config = SimpleNamespace(instance_name="dog1")
    dog.goto = MagicMock(return_value="ok")
    dog.stand = MagicMock()
    dog.halt = MagicMock()
    dog.crouch = MagicMock()

    class _Msg:
        data = wire

    dog._on_swarm_cmd(_Msg())
    dog.goto.assert_called_once_with(north=12.5, east=-7.0)


def test_legged_move_is_relative_to_the_robots_live_pose_and_heading():
    """Pins the frame math AND the reason `move` exists: an external caller
    computing 'forward 2 m' from a stale position sends the robot back to
    where it used to be. `_move_target` reads live state at call time; `move`
    executes it with the heading HELD (goto would turn the robot, silently
    redefining "forward" for the next command)."""
    from unittest.mock import MagicMock

    from dimos.robot.legged.legged_sim_module import LeggedSimModule

    dog = object.__new__(LeggedSimModule)
    dog.config = SimpleNamespace(instance_name="dog1", max_range_m=250.0)

    def set_pose(n, e, yaw):
        dog._state_dict = MagicMock(
            return_value={"connected": True, "ned": [n, e, 0.0], "yaw_ned_rad": yaw}
        )

    # Facing NORTH at (10, 5): forward -> +north, left 5 (right_m=-5) -> WEST.
    set_pose(10.0, 5.0, 0.0)
    tn, te, yaw0 = dog._move_target(2.0, -5.0)
    assert abs(tn - 12.0) < 1e-6 and abs(te - 0.0) < 1e-6 and yaw0 == 0.0

    # Facing EAST (yaw pi/2): forward -> +east, right -> SOUTH.
    set_pose(0.0, 0.0, math.pi / 2)
    tn, te, _ = dog._move_target(3.0, 1.0)
    assert abs(tn - (-1.0)) < 1e-6 and abs(te - 3.0) < 1e-6

    # The chaining property the user's bug was about: second move computes
    # from the NEW pose, not the original one.
    set_pose(0.0, -5.0, 0.0)  # after "left 5"
    tn, te, _ = dog._move_target(2.0, 0.0)
    assert abs(tn - 2.0) < 1e-6 and abs(te - (-5.0)) < 1e-6


def test_legged_move_rejects_targets_beyond_the_operating_radius():
    from unittest.mock import MagicMock

    from dimos.robot.legged.legged_sim_module import LeggedSimModule

    dog = object.__new__(LeggedSimModule)
    dog.config = SimpleNamespace(instance_name="dog1", max_range_m=250.0)
    dog._state_dict = MagicMock(
        return_value={"connected": True, "ned": [249.0, 0.0, 0.0], "yaw_ned_rad": 0.0}
    )
    out = dog.move(forward_m=10.0)
    assert "REJECTED" in out and "250" in out


def test_bridge_move_controller_math():
    """Pins _move_cmds: frame conversions and the deadband bumps, in the NWU
    bridge frame with NED-signed outputs."""
    from dimos.simulation.px4_hil.fleet_bridge import _move_cmds

    # Target dead ahead (north), facing north: pure forward, no strafe/turn.
    vx, vy, wz = _move_cmds(5.0, 0.0, 0.0, 0.0)
    assert vx == 0.7 and vy == 0.0 and wz == 0.0

    # Target to the WEST (NWU +y), facing north: west is LEFT -> vy negative.
    vx, vy, wz = _move_cmds(0.0, 5.0, 0.0, 0.0)
    assert vx == 0.0 and vy == -0.3 and wz == 0.0

    # Heading disturbed LEFT of hold (NWU yaw +0.2): correction turns RIGHT
    # (NED wz positive).
    _, _, wz = _move_cmds(0.0, 0.0, 0.2, 0.0)
    assert wz > 0.0

    # Deadband bump: small forward error still above half the arrive radius
    # commands at least 0.3, never a sub-deadband creep.
    vx, _, _ = _move_cmds(0.30, 0.0, 0.0, 0.0)
    assert abs(vx) >= 0.3


def test_drone_aero_helpers():
    """Pins the aero layer's pure math: spool lag, ground effect, H-drag."""
    import numpy as np

    from dimos.simulation.px4_hil.hil_bridge import (
        GROUND_EFFECT_MAX,
        _ground_effect_boost,
        _motor_lag_step,
        _rotor_hdrag_force,
    )

    # Spool: converges toward the command, never overshoots, monotone.
    st = np.zeros(4)
    cmd = np.full(4, 0.8)
    prev = st.copy()
    for _ in range(200):
        st = _motor_lag_step(st, cmd, 0.004)
        assert np.all(st >= prev - 1e-12) and np.all(st <= 0.8 + 1e-12)
        prev = st.copy()
    assert np.all(st > 0.79)  # settled after ~0.8 s at tau=0.06

    # Ground effect: capped in the clamped near-ground region, present but
    # modest at half a metre, gone at altitude, monotonically decreasing.
    assert _ground_effect_boost(0.05) == GROUND_EFFECT_MAX
    assert 0.0 < _ground_effect_boost(0.5) < _ground_effect_boost(0.2) < GROUND_EFFECT_MAX
    assert _ground_effect_boost(10.0) < 0.001

    # H-drag opposes airspeed, scales with thrust, no vertical component.
    f = _rotor_hdrag_force(np.array([2.0, -1.0, 3.0]), 20.0)
    assert f[0] < 0 and f[1] > 0 and f[2] == 0.0
    f2 = _rotor_hdrag_force(np.array([2.0, -1.0, 0.0]), 40.0)
    assert abs(f2[0]) > abs(f[0])
    # Motors off, falling: no phantom damping.
    assert np.allclose(_rotor_hdrag_force(np.array([5.0, 5.0, -8.0]), 0.0), 0.0)


def test_fleet_bridge_constructs_without_vehicles():
    """Pins the whole Px4HilFleet.__init__ path (wind setup included) without
    needing PX4 or sockets. A NameError here shipped once: the wind code used
    `np` that was only imported inside Go1Link, so EVERY sim start died at
    construction while all unit tests stayed green -- none of them built the
    fleet."""
    from dimos.simulation.px4_hil.fleet_bridge import Px4HilFleet

    fleet = Px4HilFleet(n_drones=0, n_dogs=0)
    assert fleet.links == [] and fleet.dogs == []
    assert fleet._wind_mean.shape == (3,)


def test_drone_link_apply_runs_the_full_aero_path():
    """Constructs a real DroneLink on a real model (no sockets, no PX4) and
    runs apply() with wind -- the path where a missing import shipped TWICE
    (np in the fleet ctor, MAX_THRUST_PER_ROTOR_N in apply) while the pure-
    math helper tests stayed green. Steps the world to prove forces are
    finite and the drone does not explode at rest."""
    import mujoco
    import numpy as np

    from dimos.simulation.px4_hil.hil_bridge import DroneLink
    from dimos.simulation.px4_hil.scene import build_model

    model, _ = build_model(1, 0)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link = DroneLink(mujoco, model, data, 0, "127.0.0.1", 4560)
    link.controls[:] = 0.5
    wind = np.array([3.0, 0.0, 0.0])
    for _ in range(50):
        link.apply(wind)
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    assert np.all(link.motor_state > 0.1)  # spool actually progressed


def test_fleet_verify_resends_to_the_straggler_only():
    """The stranded-drone incident, mechanized: after rtl_all, a drone still
    reporting AUTO.LOITER gets an individually addressed re-send; the one that
    complied gets nothing."""
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    _report(c, "drone2", 10, 0, 5)
    with c._lock:
        c._states["drone1"]["mode"] = "AUTO.LOITER"   # never heard the rtl
        c._states["drone2"]["mode"] = "AUTO.RTL"      # complied
    c._verify_rounds = 1
    c._verify_fleet_command("rtl", ["drone1", "drone2"])
    targeted = [m for m in c.swarm_cmd.published if m["target"] == "drone1"]
    assert targeted and all(m["action"] == "rtl" for m in targeted)
    assert not any(m["target"] == "drone2" for m in c.swarm_cmd.published)


def test_fleet_verify_is_quiet_when_everyone_complied():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    with c._lock:
        c._states["drone1"]["mode"] = "AUTO.RTL"
    c._verify_fleet_command("rtl", ["drone1"])
    assert c.swarm_cmd.published == []


def test_fleet_verify_kill_checks_armed_not_mode():
    c = _coordinator()
    _report(c, "drone1", 0, 0, 5)
    with c._lock:
        c._states["drone1"]["armed"] = True   # kill did not land
    c._verify_rounds = 1
    c._verify_fleet_command("kill", ["drone1"])
    assert any(
        m["target"] == "drone1" and m["action"] == "kill" for m in c.swarm_cmd.published
    )
