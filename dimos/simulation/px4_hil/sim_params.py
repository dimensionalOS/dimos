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

"""Make PX4 SITL instances survive a full demo run.

PX4's simulated battery drains from 100% to empty in ``SIM_BAT_DRAIN`` seconds
— and the default is **60**. Any demo longer than about a minute (a grid sweep
easily is) trips the low-battery failsafe mid-flight: the drone abandons its
mission, tries to descend while OFFBOARD setpoints are still streaming, fights
itself, and usually ends up inverted on the ground hundreds of metres away.

That failure looks exactly like a swarm bug. It is not. This script stretches
the simulated battery so a demo can actually finish.

These are SIMULATION-ONLY parameters (``SIM_*``). They do not exist on real
hardware, so nothing here can mask a real low-battery condition in the field.

Run after the instances are up, before pointing DimOS at them:

    .venv/bin/python dimos/simulation/px4_hil/sim_params.py --count 3
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

# Simulation-only battery behaviour. Held long enough that a full demo
# (takeoff -> sweep -> formation -> RTL) never trips a low-battery failsafe.
PARAMS: dict[str, float] = {
    # SIM-seconds, like every PX4 time param (the sim clock runs 10-18x wall).
    # 45000 sim-s ~= 40-75 wall-minutes full->empty: outlasts any demo without
    # pretending batteries are infinite.
    "SIM_BAT_DRAIN": 45000.0,
    "SIM_BAT_MIN_PCT": 60.0,  # never report below this
    # --- X500 airframe geometry -------------------------------------------
    # We boot `none_iris` because it is the only HIL-capable quadrotor airframe
    # (every x500 airframe is `gz_`-prefixed and starts gz_bridge, which this
    # simulator does not use). So the plumbing is iris and the geometry is
    # overridden here to a real X500: a symmetric 0.174 m square, matching the
    # x500_base SDF the visual mesh comes from.
    #
    # These MUST stay in lockstep with ROTORS in dimos/simulation/px4_hil/scene.py.
    # PX4 builds its control-allocation matrix from these positions; if MuJoCo
    # puts the thrust somewhere else, the controller mis-allocates torque and
    # the vehicle flips on arming.
    #
    # Note PX4's own 4001_gz_x500 uses (0.13, 0.22), which does NOT match its
    # own x500 mesh (0.174, 0.174). We use the true airframe geometry so PX4,
    # MuJoCo and the mesh all agree.
    "CA_ROTOR0_PX": 0.174,
    "CA_ROTOR0_PY": 0.174,
    "CA_ROTOR1_PX": -0.174,
    "CA_ROTOR1_PY": -0.174,
    "CA_ROTOR2_PX": 0.174,
    "CA_ROTOR2_PY": -0.174,
    "CA_ROTOR3_PX": -0.174,
    "CA_ROTOR3_PY": 0.174,
    # --- geofence: the in-flight backstop --------------------------------
    # The coordinator's dispatch-time operating radius (max_range_m=250) can
    # only judge *commands*. A drone drifting under set_velocity, or a
    # maneuver interrupted mid-flight, is invisible to it. PX4's own geofence
    # is the layer that sees actual position: at 260 m horizontal / 45 m
    # vertical it switches to hold. 260 not 250 so the dispatch gate always
    # trips first and the operator sees the polite refusal, not the fence.
    "GF_ACTION": 2.0,          # 2 = hold position at the fence
    "GF_MAX_HOR_DIST": 260.0,
    "GF_MAX_VER_DIST": 45.0,
    # --- datalink-loss failsafe ------------------------------------------
    # The "laptop dies mid-flight" case. With the default (0 = disabled) the
    # fleet just keeps hovering forever when DimOS goes away. 2 = return to
    # launch and land, on the autopilot's own authority, no ground station
    # involved. This is exactly the behaviour wanted on real hardware, so the
    # sim should exercise it too.
    "NAV_DLL_ACT": 2.0,
    # COM_DL_LOSS_T is measured in PX4's OWN clock -- which this simulator
    # drives at 10-18x wall speed. Our GCS heartbeats tick at 1 Hz of WALL
    # time, i.e. one every 10-18 sim-seconds, so against the 10 s default the
    # connection expired between every pair of beats: "GCS connection
    # regained" once a second, arming a lottery, and mid-flight failsafes with
    # a live link. (PX4's rcS scales this by PX4_SIM_SPEED_FACTOR for exactly
    # this reason.) 300 sim-seconds ~= 15-30 wall-seconds at our speeds: the
    # laptop dying still brings the fleet home in under half a minute of wall
    # time, and a 1 Hz heartbeat is never mistaken for a dead link.
    "COM_DL_LOSS_T": 300.0,
    # Same sim-clock arithmetic as COM_DL_LOSS_T, for OFFBOARD: the default
    # 1.0 (sim-)second staleness window meets our wall-clock setpoint stream
    # (20 Hz wall = ~0.4-0.9 sim-s gaps at 10-18x) on a knife edge. 30 sim-s
    # keeps the protection while making it immune to realtime-factor swings.
    "COM_OF_LOSS_T": 30.0,
}

# Parameters PX4 declares as INT32. MAVLink transports these BIT-CAST into the
# float field, not converted: sending a REAL32 2.0 makes PX4 store 1073741824
# (the bits of 2.0f), which for a bounded param like GF_ACTION is out of range
# and silently rejected -- no geofence. Conversely, sending INT32 for a FLOAT
# param (SIM_BAT_DRAIN is FLOAT) is ignored just as silently, so this set must
# list exactly PX4's INT32 params and nothing else. Verified live both ways.
INT32_PARAMS = {"GF_ACTION", "NAV_DLL_ACT", "COM_DL_LOSS_T"}

DEFAULT_PORT_BASE = 14540
CONNECT_TIMEOUT_S = 20.0
VERIFY_TIMEOUT_S = 5.0


def tune_instance(instance: int, host: str, port_base: int) -> bool:
    """Set the sim params on one PX4 instance. Returns True if all verified."""
    from pymavlink import mavutil  # type: ignore[import-untyped]

    endpoint = f"udp:{host}:{port_base + instance}"
    print(f"[sim-params] instance {instance}: connecting to {endpoint}")
    conn = mavutil.mavlink_connection(endpoint, source_system=245, source_component=190)

    if conn.wait_heartbeat(timeout=CONNECT_TIMEOUT_S) is None:
        print(f"[sim-params] instance {instance}: NO HEARTBEAT — skipped", file=sys.stderr)
        return False

    ok = True
    for name, value in PARAMS.items():
        if name in INT32_PARAMS:
            # Pack the integer's bytes into the float field (see INT32_PARAMS).
            wire_value = struct.unpack("<f", struct.pack("<i", int(value)))[0]
            wire_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        else:
            wire_value = float(value)
            wire_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            name.encode(),
            wire_value,
            wire_type,
        )
        # Read back: a silently-dropped param set is the whole failure mode here.
        deadline = time.time() + VERIFY_TIMEOUT_S
        confirmed = None
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.param_id.strip("\x00") == name:
                confirmed = msg.param_value
                if name in INT32_PARAMS:
                    # Undo the bit-cast for the comparison below.
                    confirmed = float(
                        struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
                    )
                break
        if confirmed is None:
            print(f"[sim-params] instance {instance}: {name} NOT CONFIRMED", file=sys.stderr)
            ok = False
        elif abs(confirmed - value) > 1e-3:
            print(
                f"[sim-params] instance {instance}: {name} readback {confirmed} != {value}",
                file=sys.stderr,
            )
            ok = False
        else:
            print(f"[sim-params] instance {instance}: {name} = {confirmed:g}")

    conn.close()
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--count", type=int, default=3, help="number of SITL instances")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port-base", type=int, default=DEFAULT_PORT_BASE)
    args = ap.parse_args()

    failures = [i for i in range(args.count) if not tune_instance(i, args.host, args.port_base)]
    if failures:
        print(f"[sim-params] FAILED on instance(s): {failures}", file=sys.stderr)
        return 1
    print(f"[sim-params] all {args.count} instance(s) tuned.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
