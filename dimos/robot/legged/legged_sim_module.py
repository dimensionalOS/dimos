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

"""One simulated legged robot, one module instance -- the ground-robot twin of
[Px4DroneModule][dimos.robot.drone.px4_drone_module.Px4DroneModule].

Deliberately the same shape as the drone module, because the whole point of the
namespace architecture is that a mixed fleet addresses uniformly:

    drone1/px4dronemodule/takeoff
    dog1/leggedsimmodule/stand

Each instance owns one UDP link to the MuJoCo fleet bridge, exactly as the drone
module owns one MAVLink link to a PX4 instance. Under
``.namespace("dog1", expose={"robot_state", "swarm_cmd"})`` it gets its own RPC
surface, its own topics and its own config keys, and it publishes onto the
*same* fleet bus the drones use.

That shared bus is what makes ``SwarmCoordinator`` fleet-aware rather than
drone-aware: every robot publishes a state message carrying ``key``,
``robot_class`` and a position, so ``fleet_state`` and ``count_within`` cover
aircraft and ground robots without special-casing either.

Positions are reported in the simulator's NWU world frame and converted to the
NED convention the rest of the fleet stack uses, so distances between a drone
and a dog are computed in one frame.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.drone.px4_geo import offset_latlon
from dimos.simulation.px4_hil.hil_bridge import (
    DEFAULT_ORIGIN_ALT,
    DEFAULT_ORIGIN_LAT,
    DEFAULT_ORIGIN_LON,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Republish cadence onto the shared fleet bus. Matches Px4DroneModule so the
# fleet-wide picture ages uniformly across robot classes.
STATE_PUBLISH_HZ = 4.0
# The bridge only answers a peer it has heard from, so keep poking it until it
# does. Cheap, and it means module restarts reconnect on their own.
HELLO_INTERVAL_S = 1.0

# Closed-loop `goto` tuning.
#
# This gait turns roughly 5x better while walking than on the spot (~0.22 rad/s
# versus ~0.047), because yaw comes only from a differential stride. So the
# controller never slows down to correct heading -- it holds cruise speed and
# turns through a wide arc. Creeping while misaligned, which is the obvious
# design, actually walks *away* from the target faster than the weakened turn
# can correct, and the distance grows.
GOTO_ARRIVE_M = 0.8
GOTO_YAW_GAIN = 0.9           # rad/s of turn per rad of heading error
# Speeds are sized for the trained Go1 policy, which has a hard deadband: a
# commanded 0.18 m/s produces NO motion at all (measured -- the robot stood
# still through a whole goto), 0.30 walks, 0.45 walks well, and 1.0 tracks at
# 0.94. Anything below ~0.25 m/s is therefore not a slow walk, it is a stop.
# The primitive-quadruped fallback clamps these down to its own lower limits,
# so one set of constants serves both backends.
GOTO_CRUISE_MS = 0.70
# Only slow down on the final approach, where overshoot costs more than time.
GOTO_SLOWDOWN_M = 4.0
GOTO_MIN_SPEED_MS = 0.35
# Deadband on heading error, so the controller stops chattering the turn
# direction back and forth once it is roughly on course.
GOTO_YAW_DEADBAND_RAD = 0.12
GOTO_TICK_S = 0.2
GOTO_TIMEOUT_S = 300.0


class LeggedSimConfig(ModuleConfig):
    """Per-robot identity. Addressable as `-o dog2/leggedsimmodule.endpoint=...`."""

    # UDP endpoint of this robot on the MuJoCo fleet bridge (LEGGED_PORT_BASE + i).
    endpoint: str = "udp:127.0.0.1:15000"
    # Nominal standing height, metres. Used to normalise `set_height`.
    nominal_height_m: float = 0.40
    # World origin, shared with the drones. SwarmCoordinator does all its
    # geometry in world frame off the `global` field, so a robot without one is
    # invisible to fleet_state distances and count_within. Defaults match the
    # PX4 SITL origin the HIL bridge uses.
    origin_lat: float = DEFAULT_ORIGIN_LAT
    origin_lon: float = DEFAULT_ORIGIN_LON
    origin_alt: float = DEFAULT_ORIGIN_ALT
    # Operating radius for `goto`, metres from the world origin. Matches the
    # aircraft coordinator's max_range_m so one boundary governs the whole
    # fleet; the sim ground plane ends at 300 m.
    max_range_m: float = 250.0


class LeggedSimModule(Module):
    """A single simulated quadruped: stance control plus fleet-bus telemetry."""

    config: LeggedSimConfig

    # Exposed (global): the shared fleet bus, same streams the drones use.
    drone_state: Out[String]
    swarm_cmd: In[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sock: socket.socket | None = None
        self._addr: tuple[str, int] | None = None
        self._running = False
        self._rx_thread: threading.Thread | None = None
        self._tx_thread: threading.Thread | None = None
        self._latest: dict[str, Any] = {}
        self._lock = threading.RLock()
        self._goto_thread: threading.Thread | None = None
        self._goto_stop = threading.Event()

    # -- identity ----------------------------------------------------------

    @property
    def robot_key(self) -> str:
        """Fleet-facing name, taken from the namespace prefix (`dog1/...` -> `dog1`)."""
        name = self.config.instance_name
        if name and "/" in name:
            return name.rsplit("/", 1)[0]
        return name or "dog1"

    # -- lifecycle ---------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        host, port = self._parse_endpoint(self.config.endpoint)
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)
        self._running = True

        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name=f"legged-rx-{self.robot_key}"
        )
        self._rx_thread.start()
        self._tx_thread = threading.Thread(
            target=self._state_loop, daemon=True, name=f"legged-tx-{self.robot_key}"
        )
        self._tx_thread.start()

        if getattr(self.swarm_cmd, "transport", None):
            self.swarm_cmd.subscribe(self._on_swarm_cmd)
            logger.info(f"[{self.robot_key}] subscribed to swarm_cmd")
        logger.info(f"[{self.robot_key}] legged sim link -> {host}:{port}")

    def stop(self) -> None:
        """Framework teardown. NOT the agent-facing stop -- that is ``halt``.

        Overrides ``Module.stop()``. Do not add a ``@skill`` named ``stop`` to
        this class: it would shadow this method and the module would never close
        its RPC, tools or event loop.
        """
        self._running = False
        for t in (self._rx_thread, self._tx_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()
        super().stop()

    @staticmethod
    def _parse_endpoint(endpoint: str) -> tuple[str, int]:
        raw = endpoint.split("://", 1)[-1] if "://" in endpoint else endpoint
        raw = raw[4:] if raw.startswith("udp:") else raw
        host, _, port = raw.rpartition(":")
        return (host or "127.0.0.1", int(port))

    # -- transport ---------------------------------------------------------

    def _send(self, payload: dict[str, Any]) -> None:
        if self._sock is None or self._addr is None:
            return
        try:
            self._sock.sendto(json.dumps(payload).encode(), self._addr)
        except OSError as e:
            logger.debug(f"[{self.robot_key}] send failed: {e}")

    def _rx_loop(self) -> None:
        """Receive ground truth from the bridge; re-announce until it replies."""
        last_hello = 0.0
        while self._running:
            now = time.monotonic()
            if not self._latest and now - last_hello > HELLO_INTERVAL_S:
                # The bridge learns our address from any datagram; a no-op
                # stance command is the cheapest way to introduce ourselves.
                self._send({"action": "noop"})
                last_hello = now
            if self._sock is None:
                return
            try:
                payload, _ = self._sock.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError:
                return
            try:
                state = json.loads(payload)
            except ValueError:
                continue
            with self._lock:
                self._latest = state
            self._enforce_ground_fence(state)

    def _enforce_ground_fence(self, state: dict[str, Any]) -> None:
        """Halt a robot that walks out of the operating radius.

        The aircraft get this backstop from PX4's own geofence; a ground robot
        on an open-loop ``walk`` had NOTHING -- ``goto`` refuses far targets at
        dispatch, but a plain "walk forward" continues until the 300 m edge of
        the world. This is the layer that catches what dispatch checks cannot,
        applied at the same radius the whole fleet uses.

        Hysteresis: halts once at the fence, re-arms only after the robot is
        back inside 95% of the radius, so a robot sitting on the line does not
        get halt-spammed.
        """
        nwu = state.get("nwu")
        if not nwu:
            return
        # Bridge frame is NWU (y = West); NED east = -y.
        r = math.hypot(float(nwu[0]), float(nwu[1]))
        limit = self.config.max_range_m
        breached = getattr(self, "_fence_breached", False)
        if not breached:
            if r > limit and state.get("walking"):
                self._fence_breached = True
                self._fence_halt_r = r
                self._cancel_goto()
                self._send({"action": "stop"})
                logger.warning(
                    f"[{self.robot_key}] GROUND FENCE: {r:.0f} m from origin exceeds "
                    f"{limit:.0f} m -- halted. Walk it back with a goto or walk command."
                )
        elif r < limit * 0.95:
            self._fence_breached = False
            logger.info(f"[{self.robot_key}] back inside the operating radius ({r:.0f} m)")
        elif state.get("walking") and r > getattr(self, "_fence_halt_r", limit) + 2.0:
            # Still outside and getting FARTHER: halt again. Walking inbound
            # (or sideways) is allowed -- that is how the robot comes home --
            # but each additional 2 m of escape re-trips the fence.
            self._fence_halt_r = r
            self._cancel_goto()
            self._send({"action": "stop"})
            logger.warning(
                f"[{self.robot_key}] GROUND FENCE: still receding ({r:.0f} m) -- halted again"
            )

    def _state_loop(self) -> None:
        """Republish onto the shared fleet bus so the coordinator sees this robot."""
        period = 1.0 / STATE_PUBLISH_HZ
        while self._running:
            try:
                self.drone_state.publish(String(json.dumps(self._state_dict())))
            except Exception as e:
                logger.debug(f"[{self.robot_key}] state publish failed: {e}")
            time.sleep(period)

    # -- state -------------------------------------------------------------

    def _state_dict(self) -> dict[str, Any]:
        with self._lock:
            raw = dict(self._latest)
        if not raw:
            return {
                "key": self.robot_key,
                "robot_class": "legged",
                "connected": False,
                "ts": time.time(),
            }
        # Simulator world is NWU; the fleet stack reasons in NED.
        x, y, z = raw.get("nwu", [0.0, 0.0, 0.0])
        vx, vy, vz = raw.get("velocity", [0.0, 0.0, 0.0])
        north, east = x, -y
        lat, lon = offset_latlon(self.config.origin_lat, self.config.origin_lon, north, east)
        return {
            "key": self.robot_key,
            "robot_class": "legged",
            "connected": True,
            "capabilities": ["ground", "camera"],
            "ned": [north, east, -z],
            "velocity_ned": [vx, -vy, -vz],
            # Same shape the drones publish, so SwarmCoordinator's Haversine
            # geometry covers ground robots without special-casing them.
            "global": {"lat": lat, "lon": lon, "rel_alt_m": z},
            # Simulator yaw is about +z in NWU (North toward West); NED yaw runs
            # North toward East, so it is the negation.
            "yaw_ned_rad": -float(raw.get("yaw_rad", 0.0)),
            "walking": raw.get("walking", False),
            "fallen": raw.get("fallen", False),
            "altitude_m": z,
            "height_m": raw.get("height_m", z),
            "nominal_height_m": raw.get("nominal_height_m", self.config.nominal_height_m),
            # No battery model in sim yet; report the placeholder the PDF asks
            # for rather than omitting the field and breaking fleet_state.
            "battery_pct": 100.0,
            "ts": time.time(),
        }

    @skill
    def state(self) -> str:
        """Report this robot's position, height and connection state."""
        return json.dumps(self._state_dict(), indent=2)

    # -- skills ------------------------------------------------------------

    @skill
    def stand(self) -> str:
        """Return to the nominal standing stance.

        If the robot is down this also attempts a recovery: the fall latch is
        cleared and the legs driven to full stance, which rights the body when it
        has toppled onto its side. A fully inverted robot cannot get up -- these
        legs have no abduction and there is no getup routine -- and the latch
        simply re-arms.
        """
        self._cancel_goto()
        if self._latest.get("fallen"):
            self._send({"action": "recover"})
            return f"{self.robot_key}: down — attempting recovery to stance"
        self._send({"action": "stance", "height": 1.0})
        return f"{self.robot_key}: standing"

    def _is_go1(self) -> bool:
        """True when the bridge is running the real Go1 + trained policy."""
        return self._latest.get("model") == "unitree_go1"

    @skill
    def crouch(self) -> str:
        """Lower the body to a stable crouch.

        On the real Go1 this is refused honestly: the trained policy is a
        velocity controller with no height input, so the bridge would accept
        the command and do nothing -- a false success worse than a refusal.
        """
        if self._is_go1():
            return (
                f"{self.robot_key}: crouch not supported by the Go1 policy "
                "(no height control); the robot stays standing"
            )
        self._send({"action": "stance", "height": 0.5})
        return f"{self.robot_key}: crouching"

    @skill
    def walk(
        self, speed_mps: float = 0.25, turn_rate_rads: float = 0.0, lateral_mps: float = 0.0
    ) -> str:
        """Walk at a body velocity until told otherwise.

        Args:
            speed_mps: forward speed. Positive is forward; reverse is capped at
                roughly half of forward, because this gait topples if pushed
                backwards at full stride.
            turn_rate_rads: yaw rate in the NED convention shared with the
                aircraft -- positive is nose-right. Turning while walking
                is far more effective than turning on the spot -- 2 DOF per leg
                and no hip abduction means yaw comes only from a differential
                stride.
        """
        self._cancel_goto()
        self._send(
            {
                "action": "walk",
                "vx": float(speed_mps),
                "vy": float(lateral_mps),
                "wz": float(turn_rate_rads),
            }
        )
        lat = f", strafe {lateral_mps:+.2f} m/s" if abs(lateral_mps) > 1e-6 else ""
        note = (
            " (strafe needs the Go1 policy; the primitive gait ignores it)"
            if abs(lateral_mps) > 1e-6 and not self._is_go1()
            else ""
        )
        return (
            f"{self.robot_key}: walking at {speed_mps:.2f} m/s, "
            f"turn {turn_rate_rads:+.2f} rad/s (+ = right){lat}{note}"
        )

    @skill
    def halt(self) -> str:
        """Stop walking and settle into a standing stance.

        Named ``halt``, not ``stop``: ``Module.stop()`` is the framework's
        teardown RPC (it closes the module's RPC, tools and event loop). A skill
        called ``stop`` shadows it, so the module never tears down -- and worse,
        anything that called ``self.stop()`` internally would silently kill the
        module instead of halting the robot.
        """
        self._cancel_goto()
        self._send({"action": "stop"})
        return f"{self.robot_key}: stopped"

    def _move_target(self, forward_m: float, right_m: float) -> tuple[float, float, float] | None:
        """(target_n, target_e, yaw_now) from the LIVE pose, or None if not connected."""
        state = self._state_dict()
        if not state.get("connected"):
            return None
        n, e, _ = state["ned"]
        yaw = float(state.get("yaw_ned_rad", 0.0))
        # NED, yaw from north toward east: heading = (cos y, sin y),
        # right-hand vector = (-sin y, cos y).
        tn = n + forward_m * math.cos(yaw) - right_m * math.sin(yaw)
        te = e + forward_m * math.sin(yaw) + right_m * math.cos(yaw)
        return tn, te, yaw

    @skill
    def move(self, forward_m: float = 0.0, right_m: float = 0.0) -> str:
        """Walk a body-relative offset from where the robot is RIGHT NOW,
        WITHOUT changing which way it faces.

        THE tool for "move left 5 meters", "go forward 2", "back up a bit":
        the target comes from the robot's live pose at call time, so
        consecutive relative commands chain from wherever the previous one
        ended -- and the heading is held (the Go1 policy strafes), so
        "forward" after a "left" still means the direction the robot was
        facing. ``goto`` is different on purpose: it turns toward its target
        (right for long distances, but it changes what "forward" means).

        The control loop runs INSIDE the simulator at physics rate. A first
        version ran here on wall-clock ticks and at ~40x realtime every
        command acted on 6-12 sim-seconds of stale state -- the robot
        wandered 140 m off a 5 m strafe. Control lives with the plant.

        Args:
            forward_m: metres along the current heading (negative = back).
            right_m: metres to the robot's right (negative = LEFT).
        """
        tgt = self._move_target(forward_m, right_m)
        if tgt is None:
            return f"{self.robot_key}: NOT CONNECTED (no state from the bridge yet)"
        tn, te, yaw0 = tgt
        r = math.hypot(tn, te)
        if r > self.config.max_range_m:
            return (
                f"{self.robot_key}: REJECTED move -- target ({tn:.0f}, {te:.0f}) is "
                f"{r:.0f} m from origin, beyond the {self.config.max_range_m:.0f} m radius"
            )
        self._cancel_goto()
        self._send({"action": "move_to", "north": tn, "east": te, "yaw": yaw0})
        return (
            f"{self.robot_key}: moving {forward_m:+.1f} m forward / {right_m:+.1f} m right "
            f"(heading held) -> NED ({tn:.1f}, {te:.1f}); poll state until walking=false"
        )

    @skill
    def goto(self, north: float = 0.0, east: float = 0.0) -> str:
        """Walk to a point in the shared world frame, steering as it goes.

        With the real Go1 and its trained policy this is dependable: measured
        6/6 spread targets reached, worst arrival error 0.62 m, including
        targets directly behind the robot. On the primitive fallback gait it
        reaches about 4 of 6 and can topple on large sustained turns -- there
        ``walk``/``halt`` remain the reliable primitives.

        Targets outside the fleet operating radius (``max_range_m``, shared
        with the aircraft coordinator) are refused at dispatch.

        Runs closed-loop in the background and returns immediately; poll
        ``state`` or ``fleet_state`` to watch progress. Calling ``goto``,
        ``walk`` or ``stop`` again cancels an in-flight one.
        """

        r = math.hypot(north, east)
        if r > self.config.max_range_m:
            return (
                f"{self.robot_key}: REJECTED goto ({north:.0f}, {east:.0f}) -- "
                f"{r:.0f} m from origin exceeds the {self.config.max_range_m:.0f} m "
                "operating radius"
            )
        if not self._latest:
            return f"{self.robot_key}: no position yet from the simulator"
        self._cancel_goto()
        self._goto_stop.clear()
        self._goto_thread = threading.Thread(
            target=self._goto_loop,
            args=(float(north), float(east)),
            daemon=True,
            name=f"legged-goto-{self.robot_key}",
        )
        self._goto_thread.start()
        return f"{self.robot_key}: walking to NED ({north:.1f}, {east:.1f})"

    def _cancel_goto(self) -> None:
        if self._goto_thread is not None and self._goto_thread.is_alive():
            self._goto_stop.set()
            self._goto_thread.join(timeout=1.0)
        self._goto_thread = None

    def _goto_loop(self, north: float, east: float) -> None:
        deadline = time.monotonic() + GOTO_TIMEOUT_S
        while not self._goto_stop.is_set() and time.monotonic() < deadline:
            state = self._state_dict()
            if not state.get("connected"):
                time.sleep(GOTO_TICK_S)
                continue
            if state.get("fallen"):
                logger.warning(f"[{self.robot_key}] goto aborted: robot is down")
                self._send({"action": "stop"})
                return
            cur_n, cur_e, _ = state["ned"]
            dn, de = north - cur_n, east - cur_e
            distance = math.hypot(dn, de)
            if distance <= GOTO_ARRIVE_M:
                self._send({"action": "stop"})
                logger.info(f"[{self.robot_key}] goto arrived ({distance:.2f} m)")
                return
            # Heading error, wrapped to [-pi, pi] so it turns the short way.
            bearing = math.atan2(de, dn)
            yaw_err = math.atan2(
                math.sin(bearing - state["yaw_ned_rad"]),
                math.cos(bearing - state["yaw_ned_rad"]),
            )
            # Hold cruise speed even when badly misaligned; the turn is only
            # effective while moving. Ease off on the final approach.
            if abs(yaw_err) < GOTO_YAW_DEADBAND_RAD:
                yaw_err = 0.0
            speed = GOTO_CRUISE_MS
            if distance < GOTO_SLOWDOWN_M:
                speed = max(GOTO_MIN_SPEED_MS, GOTO_CRUISE_MS * distance / GOTO_SLOWDOWN_M)
            self._send({"action": "walk", "vx": speed, "wz": GOTO_YAW_GAIN * yaw_err})
            time.sleep(GOTO_TICK_S)
        self._send({"action": "stop"})
        if not self._goto_stop.is_set():
            logger.warning(f"[{self.robot_key}] goto timed out after {GOTO_TIMEOUT_S:.0f}s")

    @skill
    def set_height(self, height_fraction: float = 1.0) -> str:
        """Set stance height as a fraction of nominal (0.35 = lowest, 1.0 = standing)."""
        if self._is_go1():
            return (
                f"{self.robot_key}: set_height not supported by the Go1 policy "
                "(no height control); the robot stays at its trained stance"
            )
        h = max(0.35, min(1.0, float(height_fraction)))
        self._send({"action": "stance", "height": h})
        return f"{self.robot_key}: stance height -> {h:.2f} of nominal"

    # -- fleet bus ---------------------------------------------------------

    def _on_swarm_cmd(self, msg: String) -> None:
        """Act on coordinator broadcasts addressed to this robot (or to `all`).

        Shared verbs only. `land`/`rtl` mean "get low and stay put" for a ground
        robot, which is the closest honest equivalent of what they mean for an
        aircraft; motion verbs that have no ground analogue are ignored rather
        than faked.
        """
        try:
            cmd = json.loads(msg.data if hasattr(msg, "data") else str(msg))
        except (ValueError, AttributeError):
            return
        target = cmd.get("target", "all")
        if target not in ("all", self.robot_key):
            return
        action = cmd.get("action", "")
        if action in ("land", "emergency_land", "hold"):
            logger.info(f"[{self.robot_key}] swarm_cmd {action}: stopping and crouching")
            self.halt()
            self.crouch()
        elif action == "ground_goto":
            # The coordinator's staged missions route ground robots with this.
            # It reuses the public goto, so the operating radius and fall
            # handling apply exactly as if the operator had called it.
            # Coordinates live under "args" -- _send() nests its kwargs there.
            # Reading them from the top level silently yielded the (0, 0)
            # defaults, so the dog dutifully walked home instead of to its
            # waypoint and the mission hung waiting for an arrival that could
            # never happen.
            args = cmd.get("args") or {}
            self.goto(north=float(args.get("north", 0.0)), east=float(args.get("east", 0.0)))
        elif action in ("takeoff", "rtl"):
            logger.info(f"[{self.robot_key}] swarm_cmd {action}: standing")
            self.stand()
        elif action == "kill":
            logger.warning(f"[{self.robot_key}] swarm_cmd kill: collapsing to the ground")
            self._send({"action": "stance", "height": 0.35})


__all__ = ["STATE_PUBLISH_HZ", "LeggedSimConfig", "LeggedSimModule"]
