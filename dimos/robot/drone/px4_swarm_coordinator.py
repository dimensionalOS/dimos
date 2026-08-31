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

"""Fleet-level brain for the PX4 swarm: one shared instance, N namespaced drones.

This is the other half of the ``Px4SitlFleetModule`` split. Where
[Px4DroneModule][dimos.robot.drone.px4_drone_module.Px4DroneModule] owns one
vehicle inside its own namespace, this module sits *outside* all namespaces and
owns everything that is inherently about the fleet as a whole:

* the shared world-frame picture (``fleet_state``, ``count_within``),
* the minimum-separation guardrail, which is only meaningful across drones,
* the multi-drone demo maneuvers (``grid_sweep``, ``line_formation``,
  ``investigate``), and
* the aggregate commands (``takeoff_all``, ``rtl_all``, ``kill_all``, ...).

It never holds a MAVLink connection. Data crosses the namespace boundary on two
exposed streams: it *listens* on ``drone_state`` (every drone publishes its own
snapshot there) and *broadcasts* on ``swarm_cmd`` (each drone acts only on
messages addressed to its key, or to ``all``).

That indirection is what makes the fleet size dynamic: adding a fourth drone is
one more namespaced blueprint, with no change here.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.drone.px4_geo import (
    GlobalPoint,
    distance_3d_m,
    offset_latlon,
    pairwise_distances,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A drone whose last snapshot is older than this is reported as stale rather
# than silently trusted for guardrail math.
STATE_STALE_AFTER_SEC = 5.0

# Separate, much tighter bound for the separation guardrail. Reporting a robot
# as "present" can tolerate seconds of lag; deciding two vehicles will not
# collide cannot. At a 5 m/s closing speed, 5 s of staleness is 25 m of travel
# against a 2 m floor -- the check would be arithmetic theatre. The guardrail
# refuses to answer on data older than this rather than answering wrongly.
GUARDRAIL_MAX_AGE_SEC = 0.75


class SwarmCoordinatorConfig(ModuleConfig):
    """Fleet-wide policy. Not per-drone — these apply across the whole swarm."""

    # Minimum allowed world-frame separation between any two drones, in meters.
    # Position commands predicted to violate this are rejected.
    min_separation_m: float = 2.0
    # Optional fleet-wide altitude ceiling (m above home, positive up). Each
    # drone also enforces its own cap; this one catches fleet maneuvers early.
    max_altitude_m: float | None = None
    # Expected drone keys, CSV (e.g. "drone1,drone2,drone3"). Used only to
    # report drones that have never reported in. Empty = infer from traffic.
    expected_drones: str = ""
    # Battery floor for preflight_check, in percent. PX4 has its own low-battery
    # failsafe; this is the "don't even take off" gate above it.
    min_battery_pct: float = 25.0
    # UDP port where PX4 expects its ground control station. Every PX4
    # instance unicasts its GCS-mode MAVLink stream at this ONE port (14550,
    # the QGroundControl convention -- verified: instances 0 and 1 both target
    # 14550), and only credits "connected to ground control station" for
    # heartbeats sourced FROM it. That check becomes an arming blocker once the
    # datalink-loss failsafe (NAV_DLL_ACT) is configured, so the coordinator --
    # one per fleet, like a real GCS -- owns this socket and answers every
    # vehicle from it. When DimOS dies these heartbeats stop and every drone
    # returns and lands on its own authority, which is exactly the failsafe
    # story wanted on hardware. 0 disables (e.g. when a real QGC is attached).
    gcs_port: int = 14550
    # Operating radius: maximum horizontal distance (m) of any commanded
    # waypoint from the drone's home. Dispatch-time gate; the PX4-side
    # geofence (GF_ACTION=hold at GF_MAX_HOR_DIST, written by
    # sim_params.py) is the in-flight backstop for velocity commands and
    # drift, which no dispatch check can see. The sim ground plane ends at
    # 300 m, hence the default.
    max_range_m: float = 250.0


class SwarmCoordinator(Module):
    """Fleet state, spacing guardrails, and multi-drone maneuvers."""

    config: SwarmCoordinatorConfig

    # Exposed streams (global): telemetry in from every drone, commands out.
    drone_state: In[String]
    swarm_cmd: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._states: dict[str, dict[str, Any]] = {}
        self._lock = threading.RLock()
        self._seq = 0
        # Google Maps client is built lazily on first map skill so a missing
        # GOOGLE_MAPS_API_KEY doesn't break startup. None = not tried,
        # False = tried and unavailable.
        self._gmaps_client: Any = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        if getattr(self.drone_state, "transport", None):
            self.drone_state.subscribe(self._on_drone_state)
            logger.info("SwarmCoordinator subscribed to drone_state")
        if self.config.gcs_port:
            self._gcs_stop = threading.Event()
            self._gcs_thread = threading.Thread(
                target=self._gcs_presence_loop, name="fleet-gcs-presence", daemon=True
            )
            self._gcs_thread.start()

    @rpc
    def stop(self) -> None:
        ev = getattr(self, "_gcs_stop", None)
        if ev is not None:
            ev.set()
        super().stop()

    def _gcs_presence_loop(self) -> None:
        """Be the fleet's ground station, the way QGroundControl is.

        Binds the shared GCS port, discovers every PX4 instance from the
        source address of its stream, and answers each with a 2 Hz GCS
        heartbeat FROM that port. Hard-won details, learned by strace:

        * Every instance targets the ONE port; a per-drone socket cannot work
          (only one process can bind it) and letting pymavlink reply to
          "whoever sent last" splits the heartbeat stream across instances --
          each then sees ~0.5 Hz with gaps beyond the 2.5 s heartbeat validity
          window, so the GCS flag flaps ("connection regained" once a second)
          and arming fails at random.
        * Heartbeats PUSHED at an instance's local port from an ephemeral
          source are received (they show in `mavlink status` rx counters) but
          never credited. Only traffic sourced from the GCS port counts.
        * 2 Hz leaves margin under the 2.5 s window even when this thread
          jitters under load.
        """
        try:
            from pymavlink import mavutil  # deferred: pymavlink is heavy
        except Exception as e:
            logger.warning(f"fleet GCS presence disabled: pymavlink unavailable ({e})")
            return
        try:
            link = mavutil.mavlink_connection(
                f"udp:0.0.0.0:{self.config.gcs_port}",
                source_system=255,
                source_component=190,
            )
        except Exception as e:
            logger.warning(
                f"fleet GCS presence disabled: cannot bind :{self.config.gcs_port} ({e}). "
                "Is QGroundControl running? Set gcs_port=0 to silence this."
            )
            return
        logger.info(f"fleet GCS presence up on :{self.config.gcs_port}")
        stop = self._gcs_stop
        ticks = drained = sent = send_err = 0
        while not stop.is_set():
            ticks += 1
            # Drain a slice of the flood. In pymavlink's server mode this is
            # what maintains `link.clients` -- every vehicle whose stream we
            # receive is registered there with a freshness stamp.
            for _ in range(400):
                try:
                    if link.recv_match(blocking=False) is None:
                        break
                    drained += 1
                except Exception:
                    break
            try:
                # Server-mode write() fans out to EVERY live client, so one
                # heartbeat_send reaches the whole fleet.
                link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                sent += 1
            except Exception as e:
                send_err += 1
                if send_err <= 3:
                    logger.warning(f"fleet GCS heartbeat failed: {type(e).__name__}: {e}")
            if ticks % 120 == 0:
                clients = sorted(getattr(link, "clients", []) or [])
                logger.debug(
                    f"fleet GCS presence: clients={clients} sent={sent} "
                    f"send_err={send_err} drained={drained}"
                )
            stop.wait(0.5)
        try:
            link.close()
        except Exception:
            pass

    def _on_drone_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data if hasattr(msg, "data") else str(msg))
        except (ValueError, AttributeError) as e:
            logger.debug(f"malformed drone_state: {e}")
            return
        key = state.get("key")
        if not key:
            return
        with self._lock:
            self._states[key] = state

    # ------------------------------------------------------------------
    # Fleet view
    # ------------------------------------------------------------------

    @property
    def _expected_keys(self) -> list[str]:
        raw = self.config.expected_drones or ""
        return [k.strip() for k in raw.split(",") if k.strip()]

    def _snapshot(self) -> dict[str, dict[str, Any]]:
        """Copy of the latest per-drone states, with staleness annotated."""
        now = time.time()
        with self._lock:
            snap = {k: dict(v) for k, v in self._states.items()}
        for state in snap.values():
            age = now - float(state.get("ts", 0.0))
            state["age_sec"] = round(age, 2)
            state["stale"] = age > STATE_STALE_AFTER_SEC
        for key in self._expected_keys:
            if key not in snap:
                snap[key] = {"key": key, "connected": False, "stale": True, "never_reported": True}
        return snap

    def _globals(
        self,
        snap: dict[str, dict[str, Any]] | None = None,
        max_age_s: float | None = None,
    ) -> dict[str, GlobalPoint]:
        """World-frame (lat, lon, rel_alt) per robot, skipping stale/missing ones.

        ``max_age_s`` tightens the freshness bound beyond the reporting default;
        the guardrail passes GUARDRAIL_MAX_AGE_SEC so it never reasons about
        collisions using positions that have had time to become wrong.
        """
        snap = snap if snap is not None else self._snapshot()
        out: dict[str, GlobalPoint] = {}
        for key, state in snap.items():
            if state.get("stale") or not state.get("connected"):
                continue
            if max_age_s is not None and float(state.get("age_sec", 0.0)) > max_age_s:
                continue
            g = state.get("global")
            if not g:
                continue
            out[key] = (float(g["lat"]), float(g["lon"]), float(g["rel_alt_m"]))
        return out

    def _local_ned(self, key: str) -> tuple[float, float, float] | None:
        with self._lock:
            state = self._states.get(key)
        if not state:
            return None
        ned = state.get("ned")
        if not ned:
            return None
        return (float(ned[0]), float(ned[1]), float(ned[2]))

    @skill
    def list_drones(self) -> str:
        """List every robot that has reported in, with its class and state."""
        snap = self._snapshot()
        if not snap:
            return (
                "No robots have reported yet. The fleet modules may still be "
                "connecting — retry in a few seconds."
            )
        n_air = len(self._flying_keys())
        n_ground = len(snap) - n_air
        header = f"Fleet of {len(snap)} robot(s): {n_air} aircraft"
        if n_ground:
            header += f", {n_ground} ground"
        lines = [header + ":"]
        for key in sorted(snap):
            st = snap[key]
            cls = st.get("robot_class", "multirotor")
            parts = [f"class={cls}", f"connected={st.get('connected', False)}"]
            # sys_id and armed are aircraft concepts; showing "?" for a ground
            # robot reads like missing telemetry rather than "not applicable".
            if st.get("sys_id") is not None:
                parts.append(f"sys_id={st['sys_id']}")
            if st.get("armed") is not None:
                parts.append(f"armed={st['armed']}")
            parts.append(f"stale={st.get('stale', False)}")
            lines.append(f"  {key} — " + ", ".join(parts))
        return "\n".join(lines)

    @skill
    def fleet_state(self) -> str:
        """Report position, armed state, battery %, and pairwise distances for the fleet.

        Call this before commanding motion to confirm the fleet is where you
        think it is. ``pairwise_m`` is world-frame 3-D distance (Haversine plus
        altitude difference), so the values are real inter-drone separations
        regardless of each drone's own NED origin.
        """
        snap = self._snapshot()
        distances = pairwise_distances(self._globals(snap))
        return json.dumps(
            {
                "drones": snap,
                "pairwise_m": [
                    {"a": a, "b": b, "distance": round(d, 2)} for a, b, d in distances
                ],
                "min_separation_m": self.config.min_separation_m,
            },
            indent=2,
        )

    @skill
    def count_within(self, drone: str, radius_m: float = 100.0) -> str:
        """Count how many other drones are within ``radius_m`` of ``drone``.

        Uses real world-frame 3-D distance (Haversine plus altitude difference).

        Args:
            drone: Reference drone key (e.g. ``drone1``).
            radius_m: Search radius in meters.
        """
        globals_ = self._globals()
        if drone not in globals_:
            return (
                f"{drone}: no usable global position yet "
                f"(known drones: {', '.join(sorted(globals_)) or 'none'})"
            )
        ref = globals_[drone]
        within: list[str] = []
        for key, point in globals_.items():
            if key == drone:
                continue
            d = distance_3d_m(ref, point)
            if d <= radius_m:
                within.append(f"{key} ({d:.1f}m)")
        return (
            f"{len(within)} robot(s) within {radius_m:.0f}m of {drone}: "
            + (", ".join(sorted(within)) if within else "(none)")
        )

    @skill
    def preflight_check(self) -> str:
        """Check whether the fleet is safe to fly, and say exactly what is wrong.

        This is the PDF's "hardware safety checklist" as a callable gate. Run it
        before every flight, and especially before the first outdoor multi-drone
        test. It is read-only — it never commands anything.

        Checks, per drone:
          * reporting at all, and not stale
          * MAVLink connected
          * a global position fix (no fix -> no position commands, no RTL)
          * battery at or above ``min_battery_pct``
          * not already armed (an armed drone before takeoff is a surprise)

        And across the fleet:
          * every expected drone present
          * unique MAV_SYS_IDs — colliding ids make drones unaddressable, and it
            is the single most common real-hardware bring-up mistake
          * current pairwise separation at or above ``min_separation_m``

        Returns a PASS/FAIL report. FAIL means do not fly.
        """
        snap = self._snapshot()
        problems: list[str] = []
        notes: list[str] = []

        if not snap:
            return "FAIL: no robots have reported at all. Is the sim or the telemetry link up?"

        sys_ids: dict[int, list[str]] = {}
        for key in sorted(snap):
            s = snap[key]
            if s.get("never_reported"):
                problems.append(f"{key}: expected but never reported")
                continue
            if s.get("stale"):
                problems.append(f"{key}: telemetry stale ({s.get('age_sec')}s old)")
                continue
            if not s.get("connected"):
                problems.append(f"{key}: MAVLink not connected")
                continue
            if not s.get("global"):
                problems.append(f"{key}: no global position fix (GPS) — RTL and goto unavailable")
            battery = s.get("battery_pct")
            if battery is None:
                notes.append(f"{key}: no battery telemetry (placeholder in SITL)")
            elif battery < self.config.min_battery_pct:
                problems.append(
                    f"{key}: battery {battery:.0f}% below floor {self.config.min_battery_pct:.0f}%"
                )
            if s.get("armed"):
                problems.append(f"{key}: already ARMED before preflight")
            sid = s.get("sys_id")
            if sid is not None:
                sys_ids.setdefault(int(sid), []).append(key)

        for sid, keys in sorted(sys_ids.items()):
            if len(keys) > 1:
                problems.append(
                    f"MAV_SYS_ID {sid} shared by {', '.join(sorted(keys))} — "
                    f"set a unique MAV_SYS_ID on each airframe"
                )

        # The separation floor exists for things that fly -- rotor wash and
        # in-flight position uncertainty. Two parked ground robots standing a
        # metre apart is normal, and failing preflight over it would make this
        # check useless on any mixed fleet.
        air = set(self._flying_keys())
        air_positions = {k: v for k, v in self._globals(snap).items() if k in air}
        for a, b, dist in pairwise_distances(air_positions):
            if dist < self.config.min_separation_m:
                problems.append(
                    f"{a} and {b} are {dist:.1f}m apart, below the "
                    f"{self.config.min_separation_m:.1f}m minimum"
                )
        ground_pairs = [
            (a, b, dist)
            for a, b, dist in pairwise_distances(self._globals(snap))
            if (a not in air or b not in air) and dist < self.config.min_separation_m
        ]
        for a, b, dist in ground_pairs:
            notes.append(f"{a} and {b} are {dist:.1f}m apart (ground robot, not a flight limit)")

        header = f"Preflight: {len(snap)} robot(s), separation floor "
        header += f"{self.config.min_separation_m:.1f}m, battery floor "
        header += f"{self.config.min_battery_pct:.0f}%, operating radius "
        header += f"{self.config.max_range_m:.0f}m"
        lines = [header]
        if notes:
            lines += ["", "NOTES:"] + [f"  - {n}" for n in notes]
        if problems:
            lines += ["", "FAIL — do not fly:"] + [f"  - {p}" for p in problems]
        else:
            lines += ["", "PASS — all checks clear."]
        lines += [
            "",
            "Not checkable from here — confirm by hand before a real flight:",
            "  - RC transmitter bound, and you can flip out of OFFBOARD instantly",
            "  - PX4 failsafes set: RC loss, datalink loss, low battery, geofence",
            "  - Geofence configured (GF_ACTION, GF_MAX_HOR_DIST)",
            "  - Props secured, area clear, spotter briefed",
        ]
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Guardrails
    # ------------------------------------------------------------------

    def _check_altitude_cap(self, altitude: float) -> str | None:
        cap = self.config.max_altitude_m
        if cap is None:
            return None
        if altitude > cap:
            return (
                f"REJECTED: altitude {altitude:.1f}m exceeds fleet cap {cap:.1f}m. "
                f"Stay below the cap or raise max_altitude_m."
            )
        return None

    def _violates_separation(
        self, issuing_key: str, target_n: float, target_e: float, target_d: float
    ) -> str | None:
        """Predict where ``issuing_key`` would land and check world-frame spacing.

        The target lives in the *issuing drone's* local NED frame, so it is
        converted to lat/lon using that drone's current global position plus the
        NED delta from its current local position. The prediction is then
        Haversine-compared against every other drone's current global position.

        Returns ``"{key} @ {dist}m"`` if the target lands within
        ``min_separation_m`` of another drone, otherwise None.
        """
        globals_ = self._globals(max_age_s=GUARDRAIL_MAX_AGE_SEC)
        issuing_global = globals_.get(issuing_key)
        issuing_local = self._local_ned(issuing_key)
        # Fail closed. Without a *fresh* position for the issuing vehicle this
        # check cannot be performed, and "cannot verify" is not "safe" -- the
        # previous behaviour let the command through and hoped PX4 would nack
        # it. Telemetry arrives within a second, so refusing costs a retry.
        if issuing_global is None or issuing_local is None:
            return (
                f"no position for {issuing_key} fresher than "
                f"{GUARDRAIL_MAX_AGE_SEC:.2f}s — cannot verify separation"
            )
        dn = target_n - issuing_local[0]
        de = target_e - issuing_local[1]
        pred_lat, pred_lon = offset_latlon(issuing_global[0], issuing_global[1], dn, de)
        pred_world = (pred_lat, pred_lon, -target_d)  # NED down -> altitude up
        for key, point in globals_.items():
            if key == issuing_key:
                continue
            dist = distance_3d_m(pred_world, point)
            if dist < self.config.min_separation_m:
                return f"{key} @ {dist:.2f}m"
        return None

    # ------------------------------------------------------------------
    # Command bus
    # ------------------------------------------------------------------

    def _send(self, target: str, action: str, repeats: int = 1, **args: Any) -> None:
        """Broadcast one command on the swarm bus.

        ``repeats`` exists for the safety verbs: the bus is best-effort, and a
        single lost broadcast once left one of two drones hovering armed after
        ``rtl_all`` while its twin landed (the module's handler never saw the
        message; an individual retry worked instantly). rtl/land/hold/
        emergency/kill are all idempotent, so repeating them is free insurance
        against exactly that. Non-idempotent commands keep repeats=1.
        """
        for i in range(max(1, repeats)):
            with self._lock:
                self._seq += 1
                seq = self._seq
            payload = {"seq": seq, "target": target, "action": action, "args": args}
            self.swarm_cmd.publish(String(json.dumps(payload)))
            if i + 1 < repeats:
                time.sleep(0.15)

    # What each fleet safety verb must produce in a drone's telemetry. A drone
    # matching none of these a few seconds after the broadcast did not hear it.
    _VERB_OUTCOME = {
        "rtl": lambda st: st.get("mode") in ("AUTO.RTL", "AUTO.LAND") or st.get("armed") is False,
        "land": lambda st: st.get("mode") == "AUTO.LAND" or st.get("armed") is False,
        "hold": lambda st: st.get("mode") == "AUTO.LOITER" or st.get("armed") is False,
        "emergency_land": lambda st: st.get("mode") == "AUTO.LAND" or st.get("armed") is False,
        "kill": lambda st: st.get("armed") is False,
    }
    # Wall-clock settle time before checking; telemetry streams at wall rate
    # regardless of the sim's speed, so this is the correct clock.
    _verify_delay_s = 4.0
    _verify_rounds = 2

    def _verify_fleet_command(self, action: str, keys: list[str]) -> None:
        """Background delivery check for a broadcast safety verb.

        The bus is best-effort: a lost rtl_all once left one of two drones
        hovering armed while its twin landed, and the coordinator had already
        reported "dispatched". Dispatched is not delivered -- so after each
        safety broadcast this waits, reads every aircraft's reported mode, and
        RE-SENDS, individually addressed, to any that did not comply. Stops
        after _verify_rounds; a drone still non-compliant then is logged as
        needing the operator.
        """
        outcome = self._VERB_OUTCOME.get(action)
        if outcome is None:
            return
        pending = list(keys)
        for round_no in range(1, self._verify_rounds + 1):
            time.sleep(self._verify_delay_s)
            snap = self._snapshot()
            still = []
            for key in pending:
                st = snap.get(key) or {}
                # kill verifies on `armed` alone; every other verb needs the
                # mode field. Without it we cannot distinguish "did not hear"
                # from "mid-descent", so we say we cannot verify rather than
                # spam re-sends at a healthy drone.
                needs_mode = action != "kill"
                if (needs_mode and st.get("mode") is None) or (
                    not needs_mode and st.get("armed") is None
                ):
                    logger.warning(
                        f"[verify:{action}] {key}: telemetry lacks the field needed to "
                        "verify delivery -- cannot confirm"
                    )
                    continue
                if not outcome(st):
                    still.append(key)
            if not still:
                if round_no > 1:
                    logger.info(f"[verify:{action}] all stragglers complied after re-send")
                return
            for key in still:
                logger.warning(
                    f"[verify:{action}] {key} did not comply (mode="
                    f"{(snap.get(key) or {}).get('mode')}) -- re-sending individually"
                )
                self._send(key, action, repeats=2)
            pending = still
        logger.error(
            f"[verify:{action}] STILL non-compliant after {self._verify_rounds} rounds: "
            f"{', '.join(pending)} -- operator action needed (try the per-drone skill)"
        )

    def _start_fleet_verify(self, action: str, keys: list[str]) -> None:
        threading.Thread(
            target=self._verify_fleet_command,
            args=(action, keys),
            name=f"verify-{action}",
            daemon=True,
        ).start()

    def _known_keys(self, robot_class: str | None = None) -> list[str]:
        """Connected robots, optionally restricted to one class.

        Robots that predate the ``robot_class`` field are treated as multirotors,
        which is what they were.
        """
        snap = self._snapshot()
        out = []
        for key, state in snap.items():
            if not state.get("connected"):
                continue
            if robot_class is not None and state.get("robot_class", "multirotor") != robot_class:
                continue
            out.append(key)
        return sorted(out)

    def _check_range(self, north: float, east: float, what: str) -> str | None:
        """Reject a waypoint outside the operating radius, or None if fine."""
        r = math.hypot(north, east)
        if r > self.config.max_range_m:
            return (
                f"REJECTED {what}: ({north:.0f}, {east:.0f}) is {r:.0f} m out, "
                f"beyond the {self.config.max_range_m:.0f} m operating radius. "
                "Pick a closer target."
            )
        return None

    def _flying_keys(self) -> list[str]:
        """Robots that can actually hold an altitude.

        Maneuvers that assign a 3-D lane -- grid_sweep, line_formation,
        investigate -- must use this rather than every reporting robot. Handing a
        quadruped an air lane silently loses that share of the coverage, and in
        `investigate` it consumes one of the requested units.
        """
        return self._known_keys(robot_class="multirotor")

    def _airborne_keys(self) -> list[str]:
        """Aircraft that are actually armed and flying.

        Position maneuvers engage OFFBOARD. Doing that to a disarmed vehicle
        sitting on the ground is not just useless: PX4 trips an OFFBOARD
        failsafe and then refuses the *next* arm request with "Resolve system
        health failures first", so a stray grid_sweep before takeoff silently
        breaks takeoff afterwards.
        """
        snap = self._snapshot()
        return sorted(
            k
            for k in self._flying_keys()
            if snap.get(k, {}).get("armed")
        )

    def _needs_airborne(self, maneuver: str) -> str | None:
        """Refuse an air maneuver when nothing is flying, and say what to do."""
        if self._airborne_keys():
            return None
        if not self._flying_keys():
            return "No aircraft have reported in yet."
        return (
            f"REJECTED: no aircraft are armed, so {maneuver} would engage OFFBOARD on "
            f"grounded vehicles and trip a failsafe that blocks the next arm. "
            f"Call takeoff_all first."
        )

    def _ground_note(self) -> str:
        """Trailing note naming ground robots skipped by an air maneuver."""
        ground = [k for k in self._known_keys() if k not in set(self._flying_keys())]
        return f"\n  (ground robots not assigned an air lane: {', '.join(ground)})" if ground else ""

    # ------------------------------------------------------------------
    # Aggregate skills
    # ------------------------------------------------------------------

    @skill
    def takeoff_all(self, altitude: float = 3.0) -> str:
        """Arm and take off every drone to ``altitude`` (m above home) simultaneously."""
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        # Dispatch stays a broadcast: a drone whose telemetry is momentarily
        # stale still hears it, where a filtered list would silently skip it.
        # Ground robots ignore air actions. The REPORT, though, must name only
        # aircraft -- listing a quadruped under "takeoff" reads as though it
        # left the ground, which is the kind of false success that wastes an
        # hour of debugging.
        keys = self._flying_keys()
        if not keys:
            return "REJECTED: no aircraft in the fleet to take off." + self._ground_note()
        self._send("all", "takeoff", altitude=altitude)
        return (
            f"takeoff_all to {altitude}m dispatched to {len(keys)} aircraft: "
            f"{', '.join(keys)}" + self._ground_note()
        )

    @skill
    def land_all(self) -> str:
        """Land every drone where it is, simultaneously."""
        keys = self._flying_keys()
        if not keys:
            return "REJECTED: no aircraft in the fleet to land." + self._ground_note()
        self._send("all", "land", repeats=3)
        self._start_fleet_verify("land", self._flying_keys())
        return (
            f"land_all dispatched to {len(keys)} aircraft: {', '.join(keys)}"
            + self._ground_note()
        )

    @skill
    def rtl_all(self) -> str:
        """Return every drone to launch and land — the PDF's "Return to launch"."""
        keys = self._flying_keys()
        if not keys:
            return "REJECTED: no aircraft in the fleet to return." + self._ground_note()
        self._send("all", "rtl", repeats=3)
        self._start_fleet_verify("rtl", self._flying_keys())
        return (
            f"rtl_all dispatched to {len(keys)} aircraft: {', '.join(keys)}"
            + self._ground_note()
        )

    # ------------------------------------------------------------------
    # Staged missions
    # ------------------------------------------------------------------

    def _legged_keys(self) -> list[str]:
        return self._known_keys(robot_class="legged")

    def _dog_at(self, key: str, north: float, east: float, radius_m: float = 1.5) -> bool:
        st = self._snapshot().get(key) or {}
        ned = st.get("ned")
        if not ned:
            return False
        return math.hypot(north - ned[0], east - ned[1]) <= radius_m

    def _set_mission(self, stage: str, detail: str) -> None:
        self.__dict__["_mission"] = {"stage": stage, "detail": detail, "ts": time.time()}
        logger.info(f"[mission] {stage}: {detail}")

    @skill
    def sweep_then_ground(
        self,
        corner_b_north: float = 40.0,
        corner_b_east: float = 30.0,
        altitude: float = 8.0,
        ground_delay_s: float = 30.0,
        dog: str = "",
    ) -> str:
        """Air sweep first, ground robot in after a delay -- the staged demo.

        The drones fly a boustrophedon over (0,0)..(corner_b) immediately. After
        ``ground_delay_s`` (wall-clock seconds -- at ~10x realtime that is far
        longer in sim time, deliberately, since the operator watches the wall
        clock) the ground robot walks a straight transect up the middle of the
        swept area -- entry, centre, far edge -- then returns to where it
        started. Progress is readable at any time via ``mission_status``, and
        ``emergency_land_all`` / ``kill_all`` abort the ground stage.

        The default 40 x 30 m rectangle matches the marked field in the sim
        world, so the whole mission is legible in the viewer.
        """
        prior = self.__dict__.get("_mission_thread")
        if prior is not None and prior.is_alive():
            return "REJECTED: a staged mission is already running. mission_status to watch it."
        # Reuse grid_sweep wholesale: its altitude, range, and airborne gates
        # are the authority, and its rejection strings say what to fix.
        sweep = self.grid_sweep(
            corner_b_north=corner_b_north, corner_b_east=corner_b_east, altitude=altitude
        )
        if sweep.startswith("REJECTED"):
            return sweep
        dogs = self._legged_keys()
        if dog:
            if dog not in dogs:
                return f"Unknown ground robot '{dog}'. Known: {', '.join(dogs) or 'none'}"
            picked = dog
        elif dogs:
            picked = dogs[0]
        else:
            return (
                "REJECTED: no ground robot in the fleet -- sweep dispatched to nobody. "
                "Use grid_sweep for an air-only sweep."
            )
        start = (self._snapshot().get(picked) or {}).get("ned") or [0.0, 0.0, 0.0]
        mid_e = corner_b_east / 2.0
        route = [
            (0.0, mid_e, "entry"),
            (corner_b_north / 2.0, mid_e, "centre"),
            (corner_b_north, mid_e, "far edge"),
            (start[0], start[1], "home"),
        ]
        abort = threading.Event()
        self.__dict__["_mission_abort"] = abort
        timeout_s = float(self.__dict__.get("_ground_wait_timeout_s", 180.0))

        def _run() -> None:
            self._set_mission(
                "air sweep", f"drones sweeping; ground in {ground_delay_s:.0f}s"
            )
            if abort.wait(timeout=max(0.0, ground_delay_s)):
                self._set_mission("aborted", "before ground stage")
                return
            for wn, we, label in route:
                if abort.is_set():
                    self._set_mission("aborted", f"during ground stage at {label}")
                    return
                self._set_mission("ground transect", f"{picked} -> {label} ({wn:.0f}, {we:.0f})")
                self._send(picked, "ground_goto", north=wn, east=we)
                deadline = time.time() + timeout_s
                while time.time() < deadline and not abort.is_set():
                    if self._dog_at(picked, wn, we):
                        break
                    if (self._snapshot().get(picked) or {}).get("fallen"):
                        self._set_mission("failed", f"{picked} fell en route to {label}")
                        return
                    time.sleep(0.5)
                else:
                    if not abort.is_set():
                        logger.warning(f"[mission] {picked} timed out reaching {label}; continuing")
            self._set_mission("complete", f"{picked} back at home; drones hold at lane ends")

        t = threading.Thread(target=_run, daemon=True, name="staged-mission")
        self.__dict__["_mission_thread"] = t
        t.start()
        return (
            f"Staged sweep started over (0,0)..({corner_b_north:.0f},{corner_b_east:.0f}):\n"
            f"  now   : {len(self._flying_keys())} aircraft sweeping at {altitude:.0f} m\n"
            f"  +{ground_delay_s:.0f}s : {picked} walks the centre transect "
            f"(entry -> centre -> far edge -> home)\n"
            "  drones hold at their lane ends when done -- rtl_all to bring them back.\n"
            "  mission_status shows the current stage."
        )

    @skill
    def mission_status(self) -> str:
        """Current stage of the staged mission, if any. Read-only."""
        m = self.__dict__.get("_mission")
        if not m:
            return "No staged mission has run."
        age = time.time() - m["ts"]
        return f"[{m['stage']}] {m['detail']} (stage entered {age:.0f}s ago)"

    @skill
    def boundaries(self) -> str:
        """Report every active control boundary and which layer enforces it.

        Read-only. Two layers exist and they catch different things:
        dispatch-time checks reject a bad *command* before anything moves;
        the PX4 geofence catches what dispatch cannot see -- velocity-command
        drift, wind in a future HITL setup, or a maneuver mid-flight.
        """
        cap = (
            f"{self.config.max_altitude_m:.0f} m"
            if self.config.max_altitude_m is not None
            else "none set (each drone still enforces its own)"
        )
        return (
            "Fleet control boundaries:\n"
            f"  operating radius : {self.config.max_range_m:.0f} m from home "
            "(dispatch-time: goto_drone, investigate, grid_sweep, line_formation, "
            "and each dog's goto)\n"
            f"  altitude ceiling : {cap} (dispatch-time)\n"
            f"  separation floor : {self.config.min_separation_m:.1f} m between aircraft "
            "(dispatch-time admission; NOT in-flight collision avoidance)\n"
            "  PX4 geofence     : GF_ACTION=hold at GF_MAX_HOR_DIST/GF_MAX_VER_DIST "
            "(in-flight backstop for AIRCRAFT, written by sim_params.py -- "
            "catches set_velocity drift that dispatch checks cannot)\n"
            "  ground fence     : each legged robot halts itself when a walk "
            "carries it past the operating radius (in-motion backstop for GROUND "
            "robots; walking back in is allowed, further escape re-trips it)\n"
            "  ground proximity : two ground robots converging within 0.6 m are "
            "both halted by the simulator (in-motion; re-arms at 1.2 m). Catches "
            "what no dispatch check can: robots ALREADY moving toward each other\n"
            "  GCS heartbeat    : PX4 refuses to arm without DimOS alive, and "
            "returns+lands every aircraft on its own if DimOS dies mid-flight "
            "(NAV_DLL_ACT=2)\n"
            "  airborne gate    : position maneuvers refuse until takeoff_all "
            "(a grounded OFFBOARD trips a failsafe that blocks the next arm)"
        )

    @skill
    def hold_all(self) -> str:
        """Put every drone into AUTO.LOITER (hover in place)."""
        self._send("all", "hold", repeats=3)
        self._start_fleet_verify("hold", self._flying_keys())
        keys = self._flying_keys()
        return (
            f"hold_all dispatched to {len(keys)} aircraft: {', '.join(keys)}"
            + self._ground_note()
        )

    @skill
    def emergency_land_all(self) -> str:
        """Land every drone immediately, regardless of state.

        Safer than ``kill_all`` — each drone descends under its own control
        instead of falling. Use when something is wrong but the autopilots are
        still responsive. ``kill_all`` is the next escalation step.
        """
        ab = self.__dict__.get("_mission_abort")
        if ab is not None:
            ab.set()
        # Broadcast on purpose -- an emergency must reach anything listening,
        # including a drone whose class or telemetry is momentarily missing.
        self._send("all", "emergency_land", repeats=3)
        self._start_fleet_verify("emergency_land", self._flying_keys())
        return f"EMERGENCY LAND broadcast; {len(self._flying_keys())} aircraft known"

    @skill
    def kill_all(self) -> str:
        """Force-disarm every drone — the big red button. They fall.

        Use ONLY when crashing is preferable to what the fleet is about to do.
        Try ``emergency_land_all`` first if the autopilots still respond.
        """
        ab = self.__dict__.get("_mission_abort")
        if ab is not None:
            ab.set()
        # Broadcast on purpose: see emergency_land_all. Never narrow the big
        # red button to a filtered list.
        self._send("all", "kill", repeats=3)
        self._start_fleet_verify("kill", self._flying_keys())
        logger.warning("KILL_ALL broadcast on swarm_cmd")
        return f"KILL_ALL broadcast; {len(self._flying_keys())} aircraft known. Motors stopped."

    # ------------------------------------------------------------------
    # Demo maneuvers
    # ------------------------------------------------------------------

    @skill
    def line_formation(
        self,
        center_north: float = 0.0,
        center_east: float = 0.0,
        altitude: float = 5.0,
        spacing_m: float = 4.0,
        heading_deg: float = 90.0,
    ) -> str:
        """Position all drones on a line at fixed altitude and spacing.

        The PDF's "Return to line formation." The line is centered at
        ``(center_north, center_east)`` and oriented along ``heading_deg`` (NED
        bearing, 0 = north, 90 = east). Spacing is forced to at least
        ``min_separation_m + 0.5`` so the formation cannot violate the guardrail.
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        blocked = self._needs_airborne("line_formation")
        if blocked:
            return blocked
        keys = self._airborne_keys()
        spacing = max(spacing_m, self.config.min_separation_m + 0.5)
        bearing = math.radians(heading_deg)
        n = len(keys)
        half = (n - 1) / 2.0 * spacing
        for sign in (1.0, -1.0):
            end_n = center_north + sign * half * math.cos(bearing)
            end_e = center_east + sign * half * math.sin(bearing)
            range_err = self._check_range(end_n, end_e, "line_formation (line end)")
            if range_err:
                return range_err
        results: list[str] = []
        for i, key in enumerate(keys):
            offset = (i - (n - 1) / 2.0) * spacing
            tn = center_north + offset * math.cos(bearing)
            te = center_east + offset * math.sin(bearing)
            self._send(key, "goto_ned", north=tn, east=te, down=-altitude, yaw_rad=bearing)
            results.append(f"{key}: → ({tn:.1f}, {te:.1f}, alt={altitude:.1f}m)")
        return (
            f"Line formation (spacing {spacing:.1f}m):\n  "
            + "\n  ".join(results)
            + self._ground_note()
        )

    @skill
    def grid_sweep(
        self,
        corner_a_north: float = 0.0,
        corner_a_east: float = 0.0,
        corner_b_north: float = 30.0,
        corner_b_east: float = 30.0,
        altitude: float = 8.0,
        sub_lane_spacing_m: float = 5.0,
    ) -> str:
        """Run a boustrophedon ("lawnmower") sweep with the fleet in parallel strips.

        The PDF's "Take your team and sweep this grid." The rectangle A→B is
        split into one equal-width east-axis strip per drone. Each drone walks a
        zigzag inside its own strip, so no two drones ever share a lane.

        Args:
            corner_a_north, corner_a_east: one corner of the sweep area, in each
                drone's local NED frame (meters relative to its own home).
            corner_b_north, corner_b_east: the opposite corner.
            altitude: sweep altitude (m, positive up).
            sub_lane_spacing_m: distance between zigzag passes inside a strip.
                Smaller = denser coverage, longer flight.

        Guardrails: strips are separated by a ``min_separation_m / 2`` buffer at
        each edge, so neighboring drones stay at least ``min_separation_m`` apart
        even at their closest passes. Rejected outright if the strips would be
        too narrow for that buffer.

        The sweep does NOT auto-return; drones hover at their lane ends. Call
        ``rtl_all`` afterwards if the intent was "search then come back".
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        blocked = self._needs_airborne("grid_sweep")
        if blocked:
            return blocked
        keys = self._airborne_keys()

        north_min = min(corner_a_north, corner_b_north)
        north_max = max(corner_a_north, corner_b_north)
        east_min = min(corner_a_east, corner_b_east)
        east_max = max(corner_a_east, corner_b_east)
        # The farthest point of the rectangle must stay inside the fence.
        far_n = max(abs(north_min), abs(north_max))
        far_e = max(abs(east_min), abs(east_max))
        range_err = self._check_range(far_n, far_e, "grid_sweep (far corner)")
        if range_err:
            return range_err
        n = len(keys)
        strip_width = (east_max - east_min) / n

        edge_buffer = self.config.min_separation_m / 2.0
        usable_strip = strip_width - 2 * edge_buffer
        if usable_strip < 0:
            return (
                f"REJECTED: strip width {strip_width:.1f}m too narrow for a "
                f"{self.config.min_separation_m}m separation buffer across {n} drones. "
                f"Widen the area or use fewer drones."
            )

        results: list[str] = []
        for i, key in enumerate(keys):
            strip_e_lo = east_min + i * strip_width + edge_buffer
            strip_e_hi = east_min + (i + 1) * strip_width - edge_buffer

            sub_e: list[float] = []
            if usable_strip < 0.5 or sub_lane_spacing_m <= 0:
                sub_e = [(strip_e_lo + strip_e_hi) / 2.0]
            else:
                e = strip_e_lo
                while e <= strip_e_hi + 1e-3:
                    sub_e.append(e)
                    e += sub_lane_spacing_m
                if sub_e[-1] < strip_e_hi - 1e-3:
                    sub_e.append(strip_e_hi)

            # Even sub-lanes walk south→north, odd walk north→south: a
            # continuous zigzag with no teleports between passes.
            d = -altitude
            path: list[list[float]] = []
            for j, e in enumerate(sub_e):
                if j % 2 == 0:
                    path.append([north_min, e, d, 0.0])
                    path.append([north_max, e, d, 0.0])
                else:
                    path.append([north_max, e, d, 0.0])
                    path.append([north_min, e, d, 0.0])

            self._send(key, "path", path=path, arrival_radius_m=2.0)
            results.append(
                f"{key}: strip east=[{strip_e_lo:.1f}, {strip_e_hi:.1f}]m, "
                f"{len(sub_e)} pass(es), {len(path)} waypoints, alt={altitude:.1f}m"
            )
        return (
            "Grid sweep dispatched (boustrophedon):\n  "
            + "\n  ".join(results)
            + self._ground_note()
        )

    @skill
    def investigate(
        self,
        north: float = 0.0,
        east: float = 0.0,
        altitude: float = 5.0,
        num_drones: int = 2,
        drones: str = "",
    ) -> str:
        """Task N drones to converge on one coordinate (local NED).

        The PDF's "Task two units to investigate this coordinate." The chosen
        drones are staggered north-south around the target at
        1.5x ``min_separation_m`` intervals so they do not collide, and each
        assignment is still checked against the spacing guardrail.

        Args:
            north: Target north position (m, local NED relative to home).
            east: Target east position (m).
            altitude: Target altitude (m, positive up).
            num_drones: How many drones to send. Ignored when ``drones`` is set.
            drones: CSV of drone keys (e.g. ``"drone1,drone3"``) to send instead.
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        blocked = self._needs_airborne("investigate")
        if blocked:
            return blocked
        known = self._airborne_keys()

        if drones.strip():
            picked = [d.strip() for d in drones.split(",") if d.strip()]
            unknown = [d for d in picked if d not in known]
            if unknown:
                return f"Unknown drone(s): {', '.join(unknown)}. Known: {', '.join(known)}"
        else:
            picked = known[: max(1, min(num_drones, len(known)))]

        spacing = self.config.min_separation_m * 1.5
        k = len(picked)
        # The stagger pushes the outermost drone (k-1)/2 slots past the target;
        # that farthest assignment is the one the fence has to admit.
        far_n = abs(north) + (k - 1) / 2.0 * spacing
        range_err = self._check_range(math.copysign(far_n, north or 1.0), east, "investigate")
        if range_err:
            return range_err
        results: list[str] = []
        for i, key in enumerate(picked):
            offset = (i - (k - 1) / 2.0) * spacing
            tn = north + offset
            te = east
            d = -altitude
            violation = self._violates_separation(key, tn, te, d)
            if violation is not None:
                results.append(
                    f"{key}: REJECTED ({tn:.1f},{te:.1f},alt={altitude:.1f}) — "
                    f"within {self.config.min_separation_m}m of {violation}"
                )
                continue
            self._send(key, "goto_ned", north=tn, east=te, down=d, yaw_rad=0.0)
            results.append(f"{key}: → ({tn:.1f}, {te:.1f}, alt={altitude:.1f}m)")
        return "Investigate dispatched:\n  " + "\n  ".join(results) + self._ground_note()

    @skill
    def goto_drone(
        self,
        drone: str,
        north: float = 0.0,
        east: float = 0.0,
        altitude: float = 3.0,
        yaw_deg: float = 0.0,
    ) -> str:
        """Send one drone to a local-NED waypoint, with the spacing guardrail applied.

        Prefer this over the drone's own ``goto`` when other drones are
        airborne: only the coordinator can see the whole fleet, so only it can
        reject a target that would land too close to someone else.
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        range_err = self._check_range(north, east, "goto_drone")
        if range_err:
            return range_err
        blocked = self._needs_airborne("goto_drone")
        if blocked:
            return blocked
        known = self._airborne_keys()
        if drone not in known:
            if drone in self._known_keys():
                return (
                    f"{drone} is a ground robot; goto_drone assigns an altitude. "
                    f"Use its own namespaced skills instead."
                )
            return f"Unknown drone: {drone!r}. Known aircraft: {', '.join(known) or 'none'}"
        d = -altitude
        violation = self._violates_separation(drone, north, east, d)
        if violation is not None:
            return (
                f"{drone}: REJECTED goto({north:.1f}, {east:.1f}, alt={altitude:.1f}) — "
                f"target within {self.config.min_separation_m}m of {violation}"
            )
        self._send(drone, "goto_ned", north=north, east=east, down=d, yaw_rad=math.radians(yaw_deg))
        return f"{drone}: goto NED ({north:.1f}, {east:.1f}, alt={altitude:.1f}m), yaw={yaw_deg:.0f}°"

    # ------------------------------------------------------------------
    # Map-aware skills
    # ------------------------------------------------------------------

    def _get_gmaps_client(self) -> Any:
        """Lazily build and cache a Google Maps client, or None if unavailable."""
        if self._gmaps_client is False:
            return None
        if self._gmaps_client is not None:
            return self._gmaps_client
        api_key = os.getenv("GOOGLE_MAPS_API_KEY")
        if not api_key:
            logger.info("GOOGLE_MAPS_API_KEY not set — map skills disabled")
            self._gmaps_client = False
            return None
        try:
            import googlemaps  # type: ignore[import-untyped]

            self._gmaps_client = googlemaps.Client(key=api_key)
        except Exception as e:
            logger.info(f"googlemaps unavailable — map skills disabled: {e}")
            self._gmaps_client = False
            return None
        return self._gmaps_client

    @skill
    def find_place_near(self, drone: str = "", query: str = "") -> str:
        """Find the nearest place matching ``query`` to a drone's current position.

        The PDF's "go to the nearest lake" step 1. This only *resolves* a place
        to coordinates — it never dispatches. Present the result to the user and
        get explicit confirmation before calling ``goto_drone_global``.

        Args:
            drone: Reference drone key. Defaults to any connected drone.
            query: Free-text place query, e.g. ``"lake"`` or ``"coastline"``.
        """
        client = self._get_gmaps_client()
        if client is None:
            return (
                "Google Maps unavailable — the daemon needs GOOGLE_MAPS_API_KEY "
                "in its environment and the `googlemaps` package installed."
            )
        if not query.strip():
            return "Provide a query, e.g. find_place_near(drone='drone1', query='lake')."
        globals_ = self._globals()
        if not globals_:
            return "No drone has a usable global position yet."
        key = drone if drone in globals_ else next(iter(sorted(globals_)))
        lat, lon, _alt = globals_[key]
        try:
            res = client.places_nearby(location=(lat, lon), keyword=query, rank_by="distance")
        except Exception as e:
            return f"Google Maps lookup failed: {e}"
        results = res.get("results") or []
        if not results:
            return f"No place matching {query!r} found near {key}."
        top = results[0]
        loc = top["geometry"]["location"]
        dist = distance_3d_m((lat, lon, 0.0), (loc["lat"], loc["lng"], 0.0))
        return json.dumps(
            {
                "reference_drone": key,
                "name": top.get("name"),
                "address": top.get("vicinity"),
                "lat": loc["lat"],
                "lon": loc["lng"],
                "distance_m": round(dist, 1),
                "next_step": (
                    "Ask the user to confirm before calling goto_drone_global with "
                    "these coordinates."
                ),
            },
            indent=2,
        )

    @skill
    def goto_drone_global(
        self,
        drone: str,
        lat: float,
        lon: float,
        altitude: float = 20.0,
    ) -> str:
        """Fly one drone to a global lat/lon at ``altitude`` (m above its home).

        Only call this AFTER the user has explicitly confirmed the destination —
        map-derived goals must never dispatch unconfirmed.
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        globals_ = self._globals()
        if drone not in globals_:
            return f"{drone}: no usable global position yet — cannot compute the NED offset."
        cur_lat, cur_lon, _ = globals_[drone]
        local = self._local_ned(drone)
        if local is None:
            return f"{drone}: no local NED yet."
        # Convert the global target into this drone's local NED frame.
        from dimos.robot.drone.px4_geo import WGS84_EQUATORIAL_M

        dn = math.radians(lat - cur_lat) * WGS84_EQUATORIAL_M
        de = math.radians(lon - cur_lon) * WGS84_EQUATORIAL_M * math.cos(math.radians(cur_lat))
        target_n = local[0] + dn
        target_e = local[1] + de
        violation = self._violates_separation(drone, target_n, target_e, -altitude)
        if violation is not None:
            return (
                f"{drone}: REJECTED global goto — target within "
                f"{self.config.min_separation_m}m of {violation}"
            )
        self._send(drone, "goto_ned", north=target_n, east=target_e, down=-altitude, yaw_rad=0.0)
        return (
            f"{drone}: goto global ({lat:.6f}, {lon:.6f}) at {altitude:.1f}m "
            f"→ local NED ({target_n:.1f}, {target_e:.1f})"
        )


__all__ = ["STATE_STALE_AFTER_SEC", "SwarmCoordinator", "SwarmCoordinatorConfig"]
