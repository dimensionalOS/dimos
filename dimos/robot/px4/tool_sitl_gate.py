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

"""SITL gate: the whole simulator twin (``px4-sitl``) checked end to end. PASS or FAIL with numbers.

Start ``make px4_sitl gz_x500`` in the PX4 tree, then::

    python dimos/robot/px4/tool_sitl_gate.py          # telemetry, camera
    python dimos/robot/px4/tool_sitl_gate.py --fly    # + the operator flight

Asserts:

- odometry at 25 Hz or better
- video frames arriving
- with ``--fly``, the operator flight: a takeoff cancelled by the enable switch before arming,
  takeoff to 2 m, go 2 m south at 3 m, a go-to past the fence refused, a held teleop key
  moving the vehicle at a locked altitude and holding on release, land to IDLE
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.px4.blueprints import px4_sitl
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.supervisor_core import GOTO_ARRIVED
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# The operator flight: how close the simulated vehicle must end up, and the teleop key.
_POSITION_TOL_M = 0.3
_SETTLE_S = 4.0
_KEY_SPEED_MPS = 0.5
_KEY_HELD_S = 4.0
# PX4 SITL's fixed ground-station port; close QGroundControl first.
_GCS_URL = "udpin:0.0.0.0:14550"


def fake_gcs(stop: threading.Event) -> None:
    """Stand in for QGroundControl on SITL's normal MAVLink instance (port 14550).

    PX4's arming check refuses without a ground station; in the field QGC is always
    connected. SITL only: system 255 is exactly what Px4DroneConnection must never be.
    """
    from pymavlink import mavutil

    gcs = mavutil.mavlink_connection(_GCS_URL, source_system=255)
    while not stop.is_set():
        gcs.recv_match(blocking=True, timeout=0.2)
        gcs.mav.heartbeat_send(6, 8, 0, 0, 4)  # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, ACTIVE
        stop.wait(0.8)
    gcs.close()


def wait_state(drone: Any, wanted: set[str], timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    state: str = ""
    while time.time() < deadline:
        state = drone.status()["state"]
        if state in wanted:
            return state
        time.sleep(0.25)
    return state


def place(drone: Any) -> tuple[float, float, float]:
    """(north, east, altitude) from the takeoff point, metres."""
    st = drone.status()
    return st["north_m"], st["east_m"], st["alt_m"]


def _near(at: tuple[float, float, float], wanted: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= _POSITION_TOL_M for a, b in zip(at, wanted, strict=True))


def _fly(coordinator: ModuleCoordinator, drone: Any) -> bool:
    """The operator flight. Every step prints what it saw; returns whether all of it held."""
    print("sitl_enable:", drone.sitl_enable(True))
    # The enable switch drops while setpoints pre-stream: the takeoff is cancelled, never armed.
    print("takeoff(2.0), then the enable switch off:", drone.takeoff(2.0))
    wait_state(drone, {"STREAMING", "IDLE"}, timeout_s=15.0)
    drone.sitl_enable(False)
    state = wait_state(drone, {"IDLE", "ABORT", "HOVER"}, timeout_s=15.0)
    status = drone.status()
    print(f"cancelled takeoff: state={state} reason={status['reason']!r} armed={status['armed']}")
    ok: bool = state == "IDLE" and status["reason"].startswith("takeoff cancelled")
    ok &= not status["armed"]
    drone.sitl_enable(True)

    print("takeoff(2.0):", drone.takeoff(2.0))
    state = wait_state(drone, {"HOVER", "IDLE", "ABORT"}, timeout_s=60.0)
    time.sleep(_SETTLE_S)
    at = place(drone)
    print(f"after takeoff: state={state} reason={drone.status()['reason']!r} place={at}")
    ok &= state == "HOVER" and _near(at, (0.0, 0.0, 2.0))

    print("go_to 2 m south at 3 m:", drone.go_to(north_m=-2.0, altitude_m=3.0))
    state = wait_state(drone, {"HOVER", "IDLE", "ABORT"}, timeout_s=60.0)
    reason = drone.status()["reason"]
    time.sleep(_SETTLE_S)
    at = place(drone)
    print(f"after go_to: state={state} reason={reason!r} place={at}")
    ok &= state == "HOVER" and reason == GOTO_ARRIVED and _near(at, (-2.0, 0.0, 3.0))

    refused = drone.go_to(north_m=100.0)
    print("go_to past the fence:", refused)
    ok &= refused == {"accepted": False, "rejection": "fence", "state": "HOVER"}

    # A held key, the way the viewer sends it: a fresh Twist every frame, then nothing.
    print("TELEOP:", drone.set_guidance_mode("TELEOP"))
    cmd_vel = coordinator.transports[("cmd_vel", Twist)]
    key = Twist(Vector3(_KEY_SPEED_MPS, 0.0, 0.0), Vector3())
    deadline = time.time() + _KEY_HELD_S
    while time.time() < deadline:
        cmd_vel.publish(key)
        time.sleep(0.05)
    time.sleep(_SETTLE_S)
    north, east, alt = place(drone)
    moved = math.hypot(north - at[0], east - at[1])
    print(f"after the key: moved {moved:.2f} m, altitude {alt:.2f} m")
    ok &= 0.5 * _KEY_SPEED_MPS * _KEY_HELD_S <= moved <= 1.5 * _KEY_SPEED_MPS * _KEY_HELD_S
    ok &= abs(alt - 3.0) <= _POSITION_TOL_M
    time.sleep(2.0)
    held = place(drone)
    print(f"key released, holding: drift {math.hypot(held[0] - north, held[1] - east):.2f} m")
    ok &= _near(held, (north, east, alt))

    print("land:", drone.land())
    state = wait_state(drone, {"IDLE", "ABORT"}, timeout_s=60.0)
    print(f"after land: state={state} reason={drone.status()['reason']!r}")
    print(f"tick jitter ms (flight): {drone.sensor_stats()['tick_jitter_ms']}")
    ok &= state == "IDLE"
    return ok


class _Taps:
    """What the gate listens to while the twin runs."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.odom_stamps: set[float] = set()  # distinct vehicle samples, not republishes
        self.frames = 0

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.odom_stamps.add(msg.ts)

    def on_video(self, msg: CompressedVideo) -> None:
        with self.lock:
            self.frames += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0, help="passive listening window")
    ap.add_argument("--fly", action="store_true", help="fly the operator commands as well")
    args = ap.parse_args()

    gcs_stop = threading.Event()
    gcs = threading.Thread(target=fake_gcs, args=(gcs_stop,), name="fake-gcs", daemon=True)
    gcs.start()
    # The blueprint is composed at import, viewer included whatever ``g.viewer`` says later;
    # the rerun bridge would open a viewer window on every run and leave it behind.
    blueprint = px4_sitl.disabled_modules(RerunBridgeModule, RerunWebSocketServer)
    # Nothing of the shell's dimos settings but a private zenoh bus, if one is set.
    environ = {k: v for k, v in os.environ.items() if k == "ZENOH_SCOUT_ADDR"}
    parsed = BlueprintConfigParser(blueprint).parse(
        environ=environ, overrides={"g": {"viewer": "none"}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    taps = _Taps()
    ok = True
    try:
        unsubs = [
            coordinator.transports[("odometry", Odometry)].subscribe(taps.on_odom),
            coordinator.transports[("video", CompressedVideo)].subscribe(taps.on_video),
        ]
        drone = coordinator.get_instance(Px4DroneConnection)

        time.sleep(args.seconds)
        for u in unsubs:
            u()

        with taps.lock:
            samples = set(taps.odom_stamps)
            frames = taps.frames

        # 1. Telemetry.
        hz = len(samples) / args.seconds
        print(f"odometry: {len(samples)} distinct samples in {args.seconds:.0f}s = {hz:.1f} Hz")
        print(f"tick jitter ms: {drone.sensor_stats()['tick_jitter_ms']}")
        ok &= hz >= 25.0

        # 2. Camera.
        print(f"video: {frames} frames in {args.seconds:.0f}s")
        ok &= frames > 0

        # 3. The operator flight.
        if args.fly:
            ok &= _fly(coordinator, drone)
        print("camera stats:", coordinator.get_instance("rtspcamera").sensor_stats())
    finally:
        coordinator.stop()
        gcs_stop.set()
        gcs.join(timeout=2.0)
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
