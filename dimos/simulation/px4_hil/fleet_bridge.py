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

"""One MuJoCo world driving N PX4 instances and M legged robots in lockstep.

This is the runnable simulator. The drones and the legged robots share a single
physics world -- the entire reason for moving off Gazebo, where drones lived in
gz and legged robots in a separate MuJoCo process with only positions passed
between them.

Lockstep with N autopilots
--------------------------
Each PX4 advances its own clock from the ``HIL_SENSOR`` stream we send it, so
all N must be driven together or they desynchronise:

    1. send HIL_SENSOR (and periodically HIL_GPS) to every vehicle
    2. wait for every vehicle's HIL_ACTUATOR_CONTROLS reply, all at once
    3. apply the actuator commands
    4. step the shared physics exactly once

Step 2 waits on every socket concurrently with ``select``. Reading each vehicle
in turn with a blocking recv costs one scheduler round-trip *per vehicle per
step*, which dominated the loop and held the fleet to a fraction of the speed
the physics can sustain.

Ports, per drone i (loopback only, no external networking):
    4560 + i   HIL/TCP, we listen and PX4 dials in
    14540 + i  PX4's MAVLink offboard channel, where DimOS connects
    14550 + i  PX4's MAVLink GCS channel, for QGroundControl

Legged robots get a UDP endpoint each at ``LEGGED_PORT_BASE + i``, deliberately
mirroring the drone pattern: the bridge is the simulator, and DimOS modules are
clients that reach it over a socket. That keeps the DimOS side identical in
shape for both robot classes, so both namespace the same way.

Usage::

    python -m dimos.simulation.px4_hil.fleet_bridge --drones 3 --dogs 1
    # then once per drone:
    cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SIM_MODEL=none_iris ./bin/px4 -i 0 -d
"""

from __future__ import annotations

import argparse
import json
import math
import select
import socket
import time
from typing import Any

import numpy as np

from dimos.simulation.px4_hil.hil_bridge import (
    ACTUATOR_WAIT_S,
    DEFAULT_ORIGIN_ALT,
    DEFAULT_ORIGIN_LAT,
    DEFAULT_ORIGIN_LON,
    GPS_RATE_HZ,
    DroneLink,
)
from dimos.simulation.px4_hil.scene import (
    DOG_TORSO_Z,
    GO1_HOME_ANGLES,
    GO1_SPAWN_Z,
    build_model,
    go1_policy_path,
    go1_unavailable_reason,
    x500_meshes_available,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# UDP base port for legged robots. Chosen clear of PX4's 4560/145xx/185xx ranges.
LEGGED_PORT_BASE = 15000
# Legged state publish rate. Matches Px4DroneModule's telemetry cadence so the
# fleet-wide picture updates uniformly across robot classes.
LEGGED_STATE_HZ = 10.0


# --- Legged gait ------------------------------------------------------------
# Link lengths from scene.py. Both segments are 0.20 m, so the leg is
# symmetric and the IK below has a clean closed form.
THIGH_LEN_M = 0.20
CALF_LEN_M = 0.20
# Nominal foot height below the hip at the standing pose, i.e. fk(0.3, -0.6).
STANCE_Z_M = -0.382
# Torso height below which the robot is considered down. An open-loop trot on
# 2-DOF legs can topple; without detection it then drags along the ground
# flailing, and every later command silently does nothing. Detecting it lets the
# gait stop and the robot settle, and lets DimOS report `fallen` honestly rather
# than claiming to be walking.
FALLEN_HEIGHT_M = 0.25
# Stride at full commanded speed, and how high a swinging foot clears ground.
MAX_STRIDE_M = 0.12
SWING_LIFT_M = 0.05
GAIT_FREQ_HZ = 2.0
# Measured travel per gait cycle, in strides. A cycle advances further than one
# stride because both diagonal pairs contribute; calibrated by walking the model
# and dividing distance by (stride * frequency * time). Without this a commanded
# speed and the achieved speed differ by ~50%, which makes closed-loop `goto`
# overshoot.
STRIDES_PER_CYCLE = 1.54
MAX_BODY_SPEED_MS = MAX_STRIDE_M * GAIT_FREQ_HZ * STRIDES_PER_CYCLE  # ~0.37
# Walking backwards at full stride topples this open-loop gait: the swing foot
# catches and the body pitches over. Half stride is stable, so backwards is
# capped rather than left as a way to fall over.
MAX_BACKWARD_SPEED_MS = MAX_BODY_SPEED_MS * 0.5
# Reversing covers about twice the ground per stride that walking forward does
# -- the gait is not symmetric, because the swing and stance arcs are not mirror
# images about the standing pose. Measured, not derived.
BACKWARD_SPEED_RATIO = 2.0
# Turn authority is the honest limit of this airframe: 2 DOF per leg and no hip
# abduction, so yaw comes only from a differential stride. Above 0.45 the robot
# topples turning in place. Turning while walking is meaningfully better
# (~12 deg/s) than turning on the spot (~4 deg/s).
TURN_STRIDE_FRACTION = 0.45
# Kept deliberately inside the stable envelope rather than at its edge. Sweeping
# speed against turn rate shows the failures are non-monotonic -- (0.30, 0.20)
# topples while (0.37, 0.20) survives -- which means the boundary is a resonance,
# not a clean limit. Margin is the right answer, not a tighter fit.
MAX_TURN_RATE_RADS = 0.12
# How much top speed is given up at full turn rate. 0.4 leaves ~0.22 m/s, which
# is the fastest combination measured stable over a sustained turn.
TURN_SPEED_DERATE = 0.4
# Commands ramp rather than snap. A step change from full forward to full
# reverse topples any legged robot, real or simulated; this is the controller
# being physically honest, not a workaround.
CMD_SLEW_MS2 = 0.6
# Yaw slews far more gently than speed. Reversing the differential stride is
# what actually topples this gait: a closed-loop controller crossing zero
# heading error flips the command in a fraction of a second, which catches a
# foot mid-swing. At 0.3 rad/s^2 a full reversal takes about a second.
CMD_SLEW_RADS2 = 0.3
# Legs, with their trot phase offset and which side they are on. Diagonal pairs
# move together, which is what keeps a quadruped statically balanced mid-gait.
LEGS = (
    ("fl", 0.0, +1),
    ("rr", 0.0, -1),
    ("fr", 0.5, -1),
    ("rl", 0.5, +1),
)


def _leg_ik(x: float, z: float) -> tuple[float, float]:
    """Foot position in the leg's sagittal plane -> (hip, knee) angles.

    x is forward, z is up (negative below the hip). The knee is taken as the
    negative solution because scene.py ranges it to [-2.4, 0], which keeps the
    joint bending the same way a real quadruped's does.
    """
    r2 = x * x + z * z
    cos_knee = (r2 - THIGH_LEN_M**2 - CALF_LEN_M**2) / (2 * THIGH_LEN_M * CALF_LEN_M)
    knee = -math.acos(max(-1.0, min(1.0, cos_knee)))
    hip = math.atan2(-x, -z) - math.atan2(
        CALF_LEN_M * math.sin(knee), THIGH_LEN_M + CALF_LEN_M * math.cos(knee)
    )
    return hip, knee


class LeggedLink:
    """One legged robot: UDP control endpoint plus a trot gait controller.

    The gait is deliberately open-loop and simple -- a diagonal trot with a
    differential stride for turning. It is enough to make the robot a real fleet
    member that can be sent somewhere, which is the point; a learned or
    model-predictive controller belongs in a DimOS module, not in the physics
    bridge.
    """

    def __init__(self, mj: Any, model: Any, data: Any, index: int, host: str, port: int) -> None:
        self.index = index
        self.name = f"dog{index}"
        self._mj = mj
        self.model = model
        self.data = data
        self.host = host
        self.port = port

        n2i = mj.mj_name2id
        self.body = n2i(model, mj.mjtObj.mjOBJ_BODY, f"{self.name}_torso")
        if self.body < 0:
            raise ValueError(f"scene has no torso body for {self.name}")
        jid = n2i(model, mj.mjtObj.mjOBJ_JOINT, f"{self.name}_root")
        self.dof_adr = int(model.jnt_dofadr[jid])
        self.qpos_adr = int(model.jnt_qposadr[jid])

        # Resolve actuators per leg, so gait phases map to the right joints.
        self.legs: list[tuple[str, float, int, int, int]] = []
        for leg, phase, side in LEGS:
            hip = n2i(model, mj.mjtObj.mjOBJ_ACTUATOR, f"{self.name}_{leg}_hip")
            knee = n2i(model, mj.mjtObj.mjOBJ_ACTUATOR, f"{self.name}_{leg}_knee")
            if hip < 0 or knee < 0:
                raise ValueError(f"scene has no {leg} actuators for {self.name}")
            self.legs.append((leg, phase, side, hip, knee))

        self.stance_scale = 1.0   # 1.0 = standing, lower = crouched
        # Commanded (target) and applied (slewed) body velocities.
        self.cmd_vx = 0.0         # m/s forward
        self.cmd_wz = 0.0         # rad/s yaw, positive = turn RIGHT (NED)
        self._vx = 0.0
        self._wz = 0.0
        self._phase = 0.0
        self.fallen = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.peer: tuple[str, int] | None = None

    # -- commands ----------------------------------------------------------

    def poll_commands(self) -> None:
        """Apply any waiting command datagrams. Never blocks."""
        while True:
            try:
                payload, addr = self.sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError, OSError):
                return
            self.peer = addr
            try:
                cmd = json.loads(payload)
            except ValueError:
                logger.warning(f"[{self.name}] malformed command datagram")
                continue
            action = cmd.get("action", "")
            if action == "recover":
                # Clear the latch and drive the legs to full stance. If the
                # robot is on its side the servos can often push it back over;
                # if it is fully inverted they cannot, and the latch simply
                # re-arms next step. Either way the operator is not stuck with a
                # robot that ignores every command for the rest of the session.
                self.fallen = False
                self.stance_scale = 1.0
                self.cmd_vx = self.cmd_wz = 0.0
                logger.info(f"[{self.name}] recovery attempt")
            elif action == "stance":
                h = float(cmd.get("height", 1.0))
                self.stance_scale = max(0.35, min(1.0, h))
            elif action == "move_to":
                # Target arrives in NED; the bridge world is NWU (y = -east,
                # yaw sign flips).
                tn = float(cmd.get("north", 0.0))
                te = float(cmd.get("east", 0.0))
                yaw_ned = float(cmd.get("yaw", 0.0))
                self._move_target = (tn, -te, -yaw_ned)
                logger.info(f"[{self.name}] move_to NED ({tn:.1f}, {te:.1f}), heading held")
            elif action == "walk":
                self.cmd_vx = max(
                    -MAX_BACKWARD_SPEED_MS, min(MAX_BODY_SPEED_MS, float(cmd.get("vx", 0.0)))
                )
                self.cmd_wz = max(
                    -MAX_TURN_RATE_RADS, min(MAX_TURN_RATE_RADS, float(cmd.get("wz", 0.0)))
                )
            elif action == "stop":
                self.cmd_vx = 0.0
                self.cmd_wz = 0.0
            elif action == "move_to":
                logger.warning(
                    f"[{self.name}] move_to needs the Go1 policy (strafe); "
                    "the primitive gait cannot hold heading -- use goto"
                )
            elif action == "noop":
                pass

    # -- gait --------------------------------------------------------------

    def apply(self, dt: float) -> None:
        """Advance the gait and write joint targets."""
        # Fall detection. Once down, stop trying to walk: continuing to cycle
        # the legs just thrashes and never recovers. Holding the stance pose at
        # least lets the body settle, and `stand` re-arms the gait.
        height = float(self.data.xpos[self.body][2])
        if height < FALLEN_HEIGHT_M:
            if not self.fallen:
                logger.warning(f"[{self.name}] fallen (h={height:.2f} m); gait stopped")
            self.fallen = True
            self.cmd_vx = self.cmd_wz = 0.0
            self._vx = self._wz = 0.0
        elif self.fallen and height > FALLEN_HEIGHT_M + 0.08:
            logger.info(f"[{self.name}] upright again (h={height:.2f} m)")
            self.fallen = False

        # Slew towards the command so reversals ramp instead of snapping.
        self._vx += max(-CMD_SLEW_MS2 * dt, min(CMD_SLEW_MS2 * dt, self.cmd_vx - self._vx))
        self._wz += max(-CMD_SLEW_RADS2 * dt, min(CMD_SLEW_RADS2 * dt, self.cmd_wz - self._wz))

        # Back off speed while turning. Top speed is stable in a straight line
        # and stable turning at cruise, but sustained full-speed turning topples
        # the robot -- the same trade any vehicle makes in a corner.
        turn_frac = abs(self._wz) / MAX_TURN_RATE_RADS if MAX_TURN_RATE_RADS else 0.0
        speed_limit = MAX_BODY_SPEED_MS * (1.0 - TURN_SPEED_DERATE * min(1.0, turn_frac))
        self._vx = max(-speed_limit, min(speed_limit, self._vx))

        stance_z = STANCE_Z_M * self.stance_scale
        moving = abs(self._vx) > 1e-3 or abs(self._wz) > 1e-3

        if not moving:
            # Park in the standing pose rather than freezing mid-swing, which
            # would leave a foot in the air and topple the robot.
            self._phase = 0.0
            hip, knee = _leg_ik(0.0, stance_z)
            for _leg, _ph, _side, hip_id, knee_id in self.legs:
                self.data.ctrl[hip_id] = hip
                self.data.ctrl[knee_id] = knee
            return

        self._phase = (self._phase + GAIT_FREQ_HZ * dt) % 1.0
        stride = self._vx / (GAIT_FREQ_HZ * STRIDES_PER_CYCLE)
        if stride < 0.0:
            stride /= BACKWARD_SPEED_RATIO
        stride = max(-MAX_STRIDE_M, min(MAX_STRIDE_M, stride))
        # Turning is a differential stride: the outside legs take longer steps.
        turn = MAX_STRIDE_M * TURN_STRIDE_FRACTION * (self._wz / MAX_TURN_RATE_RADS)

        for _leg, phase_off, side, hip_id, knee_id in self.legs:
            # Yaw follows the NED convention the rest of the fleet uses:
            # positive is nose-RIGHT. Turning right means the left (outside)
            # legs take the longer stride, so side (+1 for left) adds.
            leg_stride = stride + side * turn
            p = (self._phase + phase_off) % 1.0
            if p < 0.5:  # stance: foot travels backward, driving the body forward
                u = p / 0.5
                x = leg_stride * (0.5 - u)
                z = stance_z
            else:  # swing: foot returns forward, lifted clear of the ground
                u = (p - 0.5) / 0.5
                x = leg_stride * (u - 0.5)
                z = stance_z + SWING_LIFT_M * math.sin(math.pi * u)
            hip, knee = _leg_ik(x, z)
            self.data.ctrl[hip_id] = hip
            self.data.ctrl[knee_id] = knee

    # -- state -------------------------------------------------------------

    def _yaw(self) -> float:
        """Body yaw in the world NWU frame, radians."""
        qw, qx, qy, qz = self.data.qpos[self.qpos_adr + 3 : self.qpos_adr + 7]
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def state(self, sim_time_us: int) -> dict[str, Any]:
        pos = self.data.xpos[self.body]
        vel = self.data.qvel[self.dof_adr : self.dof_adr + 3]
        return {
            "key": self.name,
            "robot_class": "legged",
            "connected": True,
            "nwu": [float(pos[0]), float(pos[1]), float(pos[2])],
            "velocity": [float(vel[0]), float(vel[1]), float(vel[2])],
            "yaw_rad": self._yaw(),
            "height_m": float(pos[2]),
            "nominal_height_m": DOG_TORSO_Z,
            "walking": abs(self._vx) > 1e-3 or abs(self._wz) > 1e-3,
            "fallen": self.fallen,
            "speed_mps": self._vx,
            "turn_rate_rads": self._wz,
            "sim_time_s": sim_time_us / 1e6,
        }

    def publish(self, sim_time_us: int) -> None:
        if self.peer is None:
            return
        try:
            self.sock.sendto(json.dumps(self.state(sim_time_us)).encode(), self.peer)
        except OSError:
            pass

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Real Unitree Go1 driven by DimOS's trained locomotion policy
# ---------------------------------------------------------------------------
# Measured envelope of the shipped policy under this world's solver
# (implicitfast, 100 iterations -- NOT the Euler/1-iteration setup it was
# trained under; it transfers fine, verified by probe):
#     forward   1.0 commanded -> 0.94 m/s
#     reverse  -0.4           -> -0.31 m/s
#     yaw      +-0.8          -> +-0.55 rad/s, and it holds an arc while moving
#     never fell in any tested command, including reversals
# Commands beyond that DEGRADE rather than saturate: 1.5 rad/s produced
# -0.11 rad/s, i.e. the wrong direction, so the clamps below are real limits and
# not politeness.
GO1_MAX_FWD_MS = 1.0
GO1_MAX_REV_MS = 0.4
GO1_MAX_TURN_RADS = 0.8
GO1_CTRL_DT = 0.02          # policy rate; 5 physics steps at our 4 ms timestep
GO1_ACTION_SCALE = 0.5
GO1_FALLEN_HEIGHT_M = 0.16
GO1_NOMINAL_HEIGHT_M = 0.27
GO1_LEGS = ("FR", "FL", "RR", "RL")
GO1_SEGMENTS = ("hip", "thigh", "calf")


MOVE_ARRIVE_M = 0.5


def _move_cmds(
    dx_nwu: float, dy_nwu: float, yaw_nwu: float, yaw_hold_nwu: float
) -> tuple[float, float, float]:
    """Body commands (vx, vy_ned_right, wz_ned_right) for a heading-held move.

    Pure math so it is unit-testable. Runs INSIDE the bridge at physics rate:
    a first version lived in the DimOS module ticking on wall time, and at
    ~40x realtime every command acted on 6-12 sim-seconds of stale state --
    the loop wandered a robot 140 m off a 5 m strafe. Control must live in
    the same clock domain as the plant.
    """
    fwd_err = dx_nwu * math.cos(yaw_nwu) + dy_nwu * math.sin(yaw_nwu)
    left_err = -dx_nwu * math.sin(yaw_nwu) + dy_nwu * math.cos(yaw_nwu)
    right_err = -left_err
    vx = max(-0.4, min(0.7, 0.9 * fwd_err))
    vy = max(-0.3, min(0.3, 0.9 * right_err))
    # The policy's low-speed deadband makes tiny commands a stop, not a creep.
    if 0.0 < abs(vx) < 0.3 and abs(fwd_err) > MOVE_ARRIVE_M / 2:
        vx = math.copysign(0.3, vx)
    if 0.0 < abs(vy) < 0.15 and abs(right_err) > MOVE_ARRIVE_M / 2:
        vy = math.copysign(0.15, vy)
    yaw_err = math.atan2(math.sin(yaw_hold_nwu - yaw_nwu), math.cos(yaw_hold_nwu - yaw_nwu))
    # NWU positive yaw is a LEFT turn; the NED wz convention is +right.
    wz_ned = -max(-0.5, min(0.5, 1.5 * yaw_err))
    return vx, vy, wz_ned


class Go1Link:
    """One real Unitree Go1, driven by the trained ONNX policy.

    Public surface is identical to :class:`LeggedLink` -- same UDP command
    protocol, same state dict -- so ``LeggedSimModule`` and the swarm
    coordinator do not know or care which one is underneath.

    The policy that ships with DimOS assumes it owns the whole model: it reads
    ``qpos[7:]`` and writes ``ctrl[:]``. In this world that would read a drone's
    joints and overwrite four drones' motor commands, so every lookup here is
    resolved by id under the robot's own ``dogN-`` prefix and only that robot's
    12 actuators are ever written.
    """

    def __init__(
        self, mj: Any, model: Any, data: Any, index: int, host: str, port: int, policy_path: str
    ) -> None:
        import numpy as np
        import onnxruntime as ort

        self._np = np
        self.index = index
        self.name = f"dog{index}"
        self._mj = mj
        self.model = model
        self.data = data
        self.host = host
        self.port = port

        prefix = f"dog{index}-"
        n2i = mj.mj_name2id
        self.body = n2i(model, mj.mjtObj.mjOBJ_BODY, prefix + "trunk")
        if self.body < 0:
            raise ValueError(f"scene has no {prefix}trunk body")
        # The Go1's floating base joint is unnamed in the MJCF, so it has to be
        # reached through the body rather than looked up by name.
        free_j = int(model.body_jntadr[self.body])
        self.qpos_adr = int(model.jnt_qposadr[free_j])
        self.dof_adr = int(model.jnt_dofadr[free_j])

        self._jq: list[int] = []
        self._jv: list[int] = []
        self._act: list[int] = []
        for leg in GO1_LEGS:
            for seg in GO1_SEGMENTS:
                j = n2i(model, mj.mjtObj.mjOBJ_JOINT, f"{prefix}{leg}_{seg}_joint")
                a = n2i(model, mj.mjtObj.mjOBJ_ACTUATOR, f"{prefix}{leg}_{seg}")
                if j < 0 or a < 0:
                    raise ValueError(f"scene missing {prefix}{leg}_{seg}")
                self._jq.append(int(model.jnt_qposadr[j]))
                self._jv.append(int(model.jnt_dofadr[j]))
                self._act.append(a)

        self._sensor: dict[str, tuple[int, int]] = {}
        for sname in ("local_linvel", "gyro"):
            sid = n2i(model, mj.mjtObj.mjOBJ_SENSOR, prefix + sname)
            if sid < 0:
                raise ValueError(f"scene missing sensor {prefix}{sname}")
            self._sensor[sname] = (int(model.sensor_adr[sid]), int(model.sensor_dim[sid]))
        self._imu_site = n2i(model, mj.mjtObj.mjOBJ_SITE, prefix + "imu")

        self._default = np.array(GO1_HOME_ANGLES, dtype=np.float32)
        self._session = ort.InferenceSession(
            policy_path, providers=["CPUExecutionProvider"]
        )
        self._last_action = np.zeros(12, dtype=np.float32)
        self._since_ctrl = GO1_CTRL_DT      # run the policy on the first step

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.fallen = False
        self._diverged = False
        self._policy_failed = False
        # Heading-held relative move: (x_nwu, y_nwu, yaw_hold_nwu) or None.
        self._move_target: tuple[float, float, float] | None = None

        # Stand at the home pose so the first observation is sane.
        self.data.qpos[self.qpos_adr + 2] = GO1_SPAWN_Z
        for k, adr in enumerate(self._jq):
            self.data.qpos[adr] = float(self._default[k])
        for k, a in enumerate(self._act):
            self.data.ctrl[a] = float(self._default[k])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.peer: tuple[str, int] | None = None

    # -- commands ----------------------------------------------------------

    def poll_commands(self) -> None:
        while True:
            try:
                payload, addr = self.sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError, OSError):
                return
            self.peer = addr
            try:
                cmd = json.loads(payload)
            except ValueError:
                logger.warning(f"[{self.name}] malformed command datagram")
                continue
            action = cmd.get("action", "")
            if action == "recover":
                self.fallen = False
                self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
                logger.info(f"[{self.name}] recovery attempt")
            elif action == "stance":
                # The shipped policy is a flat-ground velocity controller with no
                # height input, so crouch/set_height have nothing to drive. Say
                # so rather than silently accepting and doing nothing.
                logger.info(f"[{self.name}] stance height not supported by the Go1 policy")
            elif action == "move_to":
                # Target arrives in NED; the bridge world is NWU (y = -east,
                # yaw sign flips).
                tn = float(cmd.get("north", 0.0))
                te = float(cmd.get("east", 0.0))
                yaw_ned = float(cmd.get("yaw", 0.0))
                self._move_target = (tn, -te, -yaw_ned)
                logger.info(f"[{self.name}] move_to NED ({tn:.1f}, {te:.1f}), heading held")
            elif action == "walk":
                self.cmd_vx = max(-GO1_MAX_REV_MS, min(GO1_MAX_FWD_MS, float(cmd.get("vx", 0.0))))
                self._move_target = None  # manual walk overrides an active move
                # NED convention at the DimOS boundary: positive vy = strafe
                # RIGHT. Clamped to the measured usable strafe (~0.3 m/s).
                self.cmd_vy = max(-0.3, min(0.3, float(cmd.get("vy", 0.0))))
                self.cmd_wz = max(
                    -GO1_MAX_TURN_RADS, min(GO1_MAX_TURN_RADS, float(cmd.get("wz", 0.0)))
                )
            elif action == "stop":
                self._move_target = None
                self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
            elif action == "noop":
                pass

    # -- policy ------------------------------------------------------------

    def _observe(self) -> Any:
        np = self._np
        la, ld = self._sensor["local_linvel"]
        ga, gd = self._sensor["gyro"]
        linvel = self.data.sensordata[la : la + ld]
        gyro = self.data.sensordata[ga : ga + gd]
        rot = self.data.site_xmat[self._imu_site].reshape(3, 3)
        gravity = rot.T @ np.array([0.0, 0.0, -1.0])
        angles = np.array([self.data.qpos[a] for a in self._jq]) - self._default
        vels = np.array([self.data.qvel[a] for a in self._jv])
        # NED sign convention at the DimOS boundary: a positive commanded yaw
        # rate means "nose right", which is negative about MuJoCo's z.
        # cmd_vy and cmd_wz are NED-signed (positive = right); the policy was
        # trained with +y = left and +yaw = counter-clockwise, so both negate.
        command = np.array([self.cmd_vx, -self.cmd_vy, -self.cmd_wz], dtype=np.float32)
        return np.hstack(
            [linvel, gyro, gravity, angles, vels, self._last_action, command]
        ).astype(np.float32)

    def apply(self, dt: float) -> None:
        height = float(self.data.xpos[self.body][2])
        if height < GO1_FALLEN_HEIGHT_M:
            if not self.fallen:
                logger.warning(f"[{self.name}] fallen (h={height:.2f} m)")
            self.fallen = True
            self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        elif self.fallen and height > GO1_FALLEN_HEIGHT_M + 0.06:
            logger.info(f"[{self.name}] upright again (h={height:.2f} m)")
            self.fallen = False

        if self._move_target is not None and not self.fallen:
            xt, yt, yaw_hold = self._move_target
            pos = self.data.xpos[self.body]
            dx, dy = xt - float(pos[0]), yt - float(pos[1])
            if math.hypot(dx, dy) <= MOVE_ARRIVE_M:
                self._move_target = None
                self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
                logger.info(f"[{self.name}] move arrived ({math.hypot(dx, dy):.2f} m)")
            else:
                self.cmd_vx, self.cmd_vy, self.cmd_wz = _move_cmds(
                    dx, dy, self._yaw(), yaw_hold
                )
        elif self._move_target is not None and self.fallen:
            self._move_target = None

        self._since_ctrl += dt
        if self._since_ctrl < GO1_CTRL_DT:
            return
        self._since_ctrl = 0.0

        np = self._np
        obs = self._observe()
        # A non-finite observation means the physics for this robot has already
        # diverged. Feeding it to the policy produces garbage torques that make
        # the blow-up worse and can NaN the whole shared world -- including the
        # drones. Hold the default stance instead and say so once.
        if not np.all(np.isfinite(obs)):
            if not self._diverged:
                logger.error(f"[{self.name}] non-finite state; holding stance")
                self._diverged = True
            for k, a in enumerate(self._act):
                self.data.ctrl[a] = float(self._default[k])
            return
        self._diverged = False

        try:
            self._last_action = self._session.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        except Exception as exc:
            # One robot's policy failing must not take the world with it, for
            # the same reason a dying autopilot does not (see _collect_replies).
            if not self._policy_failed:
                logger.error(f"[{self.name}] policy inference failed: {exc}; holding stance")
                self._policy_failed = True
            for k, a in enumerate(self._act):
                self.data.ctrl[a] = float(self._default[k])
            return
        self._policy_failed = False

        targets = self._last_action * GO1_ACTION_SCALE + self._default
        for k, a in enumerate(self._act):
            self.data.ctrl[a] = float(targets[k])

    # -- telemetry ---------------------------------------------------------

    def _yaw(self) -> float:
        qw, qx, qy, qz = self.data.qpos[self.qpos_adr + 3 : self.qpos_adr + 7]
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def state(self, sim_time_us: int) -> dict[str, Any]:
        pos = self.data.xpos[self.body]
        vel = self.data.qvel[self.dof_adr : self.dof_adr + 3]
        return {
            "key": self.name,
            "robot_class": "legged",
            "connected": True,
            "nwu": [float(pos[0]), float(pos[1]), float(pos[2])],
            "velocity": [float(vel[0]), float(vel[1]), float(vel[2])],
            "yaw_rad": self._yaw(),
            "height_m": float(pos[2]),
            "nominal_height_m": GO1_NOMINAL_HEIGHT_M,
            "walking": self._move_target is not None
            or abs(self.cmd_vx) > 1e-3
            or abs(self.cmd_wz) > 1e-3
            or abs(self.cmd_vy) > 1e-3,
            "fallen": self.fallen,
            "speed_mps": self.cmd_vx,
            "turn_rate_rads": self.cmd_wz,
            "model": "unitree_go1",
            "sim_time_s": sim_time_us / 1e6,
        }

    def publish(self, sim_time_us: int) -> None:
        if self.peer is None:
            return
        try:
            self.sock.sendto(json.dumps(self.state(sim_time_us)).encode(), self.peer)
        except OSError:
            pass

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class Px4HilFleet:
    """Drives N PX4 instances and M legged robots inside one MuJoCo world."""

    def __init__(
        self,
        n_drones: int = 3,
        n_dogs: int = 1,
        host: str = "127.0.0.1",
        port_base: int = 4560,
        legged_port_base: int = LEGGED_PORT_BASE,
        origin: tuple[float, float, float] = (
            DEFAULT_ORIGIN_LAT,
            DEFAULT_ORIGIN_LON,
            DEFAULT_ORIGIN_ALT,
        ),
        viewer: bool = False,
    ) -> None:
        import mujoco

        self._mj = mujoco
        self.origin = origin
        self.want_viewer = viewer

        self.model, self._real_go1 = build_model(n_drones, n_dogs)
        self._policy_path = go1_policy_path() if self._real_go1 else None
        if self._real_go1 and self._policy_path is None:
            # Real mesh but no trained weights would silently fall back to a
            # robot that cannot walk at all, so rebuild the primitive instead.
            logger.warning("Go1 mesh found but no trained policy; using primitive quadruped")
            self.model, self._real_go1 = build_model(n_drones, n_dogs, real_go1=False)
        self.data = mujoco.MjData(self.model)
        # Populate sensordata and site frames before anything is transmitted.
        # PX4 aligns its initial attitude from the first sample it receives, and
        # an all-zero one leaves the EKF stably but completely wrong.
        mujoco.mj_forward(self.model, self.data)

        if n_drones and not x500_meshes_available():
            logger.info(
                f"drones: primitive airframe x{n_drones} (flies identically); "
                "run tools/fetch_x500_meshes.py for the X500 model"
            )
        self.links = [
            DroneLink(mujoco, self.model, self.data, i, host, port_base + i)
            for i in range(n_drones)
        ]
        if self._real_go1 and self._policy_path is not None:
            logger.info(f"legged robots: real Unitree Go1 + trained policy x{n_dogs}")
            self.dogs: list[Any] = [
                Go1Link(
                    mujoco, self.model, self.data, i, host,
                    legged_port_base + i, self._policy_path,
                )
                for i in range(n_dogs)
            ]
        else:
            if n_dogs:
                # Say WHY. A silent downgrade leaves a robot that walks badly
                # and no clue that a far better one was one command away.
                reason = go1_unavailable_reason() or "trained policy missing"
                logger.warning(
                    f"legged robots: PRIMITIVE quadruped x{n_dogs} "
                    f"(0.37 m/s, falls on reversals) -- real Go1 unavailable: {reason}"
                )
            self.dogs = [
                LeggedLink(mujoco, self.model, self.data, i, host, legged_port_base + i)
                for i in range(n_dogs)
            ]
        # The Go1 links write their home stance into qpos during construction,
        # so the world has to be re-evaluated before the first sensor read.
        mujoco.mj_forward(self.model, self.data)
        for dog in self.dogs:
            dog.poll_commands()
            dog.apply(self.model.opt.timestep)

        # One wind for the shared world: constant mean (SIM_WIND_N/E, m/s in
        # the NED sense: N = from where it blows TO... no -- the vector the
        # air MOVES with, world frame: +N pushes everything north) plus an
        # Ornstein-Uhlenbeck gust (SIM_GUST_STD, m/s). Zero by default.
        import os as _os

        self._wind_mean = np.array(
            [
                float(_os.getenv("SIM_WIND_N", "0") or 0),
                -float(_os.getenv("SIM_WIND_E", "0") or 0),  # world is NWU: y = -E
                0.0,
            ]
        )
        self._gust_std = float(_os.getenv("SIM_GUST_STD", "0") or 0)
        self._gust = np.zeros(3)
        self._wind_rng = np.random.default_rng(7)
        if np.any(self._wind_mean) or self._gust_std:
            logger.info(
                f"wind enabled: mean N={self._wind_mean[0]:.1f} E={-self._wind_mean[1]:.1f} m/s, "
                f"gust std {self._gust_std:.1f} m/s"
            )

        self.sim_time_us = 1_000_000
        self.step_us = round(self.model.opt.timestep * 1e6)
        self.gps_interval_us = int(1e6 / GPS_RATE_HZ)
        self._last_gps_us = 0
        self._legged_interval_us = int(1e6 / LEGGED_STATE_HZ)
        self._last_legged_us = 0

    # -- lifecycle ---------------------------------------------------------

    def wait_for_px4(self, timeout_s: float = 180.0) -> None:
        for link in self.links:
            link.listen()
        if self.links:
            ports = ", ".join(str(link.port) for link in self.links)
            logger.info(f"listening on {ports}; waiting for {len(self.links)} PX4 instance(s)")
        if self.dogs:
            logger.info(
                "legged endpoints: "
                + ", ".join(f"{d.name}=udp:{d.host}:{d.port}" for d in self.dogs)
            )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(link.try_accept() for link in self.links):
                logger.info(f"all {len(self.links)} PX4 instance(s) connected")
                return
        missing = [link.name for link in self.links if link.conn is None]
        raise TimeoutError(f"PX4 never connected for: {', '.join(missing)}")

    GROUND_PROX_HALT_M = 0.6
    GROUND_PROX_REARM_M = 1.2

    def _ground_proximity_guard(self) -> None:
        """Halt any two ground robots about to walk into each other.

        The aircraft get dispatch-time separation; ground robots deliberately
        have no separation floor (slow, low stakes) -- but "two gotos to the
        same point" should still not end in robots pushing at each other. This
        runs in the bridge at sim rate, so it also catches what no dispatch
        check can: two already-moving robots converging. Hysteresis: halted
        pairs re-arm once they are GROUND_PROX_REARM_M apart, so the operator
        can walk one away without the guard re-firing every step.
        """
        if len(self.dogs) < 2:
            return
        halted: set[tuple[int, int]] = getattr(self, "_prox_halted", set())
        for i in range(len(self.dogs)):
            for j in range(i + 1, len(self.dogs)):
                a, b = self.dogs[i], self.dogs[j]
                pa, pb = self.data.xpos[a.body], self.data.xpos[b.body]
                d = math.hypot(float(pa[0] - pb[0]), float(pa[1] - pb[1]))
                key = (i, j)
                moving = any(
                    abs(getattr(lk, "cmd_vx", 0.0)) > 1e-3
                    or abs(getattr(lk, "cmd_vy", 0.0)) > 1e-3
                    or getattr(lk, "_move_target", None) is not None
                    for lk in (a, b)
                )
                if key not in halted and d < self.GROUND_PROX_HALT_M and moving:
                    for lk in (a, b):
                        lk.cmd_vx = lk.cmd_wz = 0.0
                        if hasattr(lk, "cmd_vy"):
                            lk.cmd_vy = 0.0
                        if getattr(lk, "_move_target", None) is not None:
                            lk._move_target = None
                    halted.add(key)
                    logger.warning(
                        f"GROUND PROXIMITY: {a.name} and {b.name} {d:.2f} m apart "
                        "and converging -- both halted"
                    )
                elif key in halted and d > self.GROUND_PROX_REARM_M:
                    halted.discard(key)
                    logger.info(f"ground proximity cleared: {a.name}/{b.name} ({d:.2f} m)")
        self._prox_halted = halted

    def _collect_replies(self) -> None:
        """Wait for every live vehicle's actuator reply, all sockets at once.

        Polling each vehicle with its own blocking recv costs a scheduler
        round-trip per vehicle per step and was the dominant cost in the loop.
        """
        pending = [lk for lk in self.links if lk.conn is not None]
        for lk in pending:
            lk.got_reply = False
        deadline = time.monotonic() + ACTUATOR_WAIT_S
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                readable, _, _ = select.select([lk.conn for lk in pending], [], [], remaining)
            except (OSError, ValueError):
                break
            if not readable:
                break
            for lk in list(pending):
                if lk.conn in readable:
                    try:
                        if lk.pump():
                            pending.remove(lk)
                    except ConnectionError as e:
                        # One autopilot dying must not take the world with it.
                        logger.warning(f"{e}; continuing without it")
                        lk.close()
                        pending.remove(lk)

    def step(self) -> None:
        send_gps = self.sim_time_us - self._last_gps_us >= self.gps_interval_us
        for link in self.links:
            try:
                link.send_sensors(self.sim_time_us, self.origin)
                if send_gps:
                    link.send_gps(self.sim_time_us, self.origin)
            except ConnectionError as e:
                logger.warning(f"{e}; continuing without it")
                link.close()
        if send_gps:
            self._last_gps_us = self.sim_time_us

        self._collect_replies()
        if self._gust_std:
            from dimos.simulation.px4_hil.hil_bridge import GUST_TAU_S

            dt = float(self.model.opt.timestep)
            self._gust[:2] += (-self._gust[:2] / GUST_TAU_S) * dt + self._gust_std * math.sqrt(
                2.0 * dt / GUST_TAU_S
            ) * self._wind_rng.standard_normal(2)
        wind = self._wind_mean + self._gust
        for link in self.links:
            link.apply(wind)

        # Legged robots: take any waiting command, then advance the gait by one
        # physics step. Both must happen every step -- polling only at startup
        # means the bridge never learns a module's address and never publishes
        # state back to it, and skipping apply() freezes the gait mid-swing.
        for dog in self.dogs:
            dog.poll_commands()
            dog.apply(self.model.opt.timestep)

        self._ground_proximity_guard()

        if self.sim_time_us - self._last_legged_us >= self._legged_interval_us:
            for dog in self.dogs:
                dog.publish(self.sim_time_us)
            self._last_legged_us = self.sim_time_us

        self._mj.mj_step(self.model, self.data)
        self.sim_time_us += self.step_us

    def run(self) -> None:
        self.wait_for_px4()
        viewer_ctx = None
        try:
            if self.want_viewer:
                import mujoco.viewer

                viewer_ctx = mujoco.viewer.launch_passive(self.model, self.data)
            last = time.monotonic()
            steps = 0
            while True:
                self.step()
                steps += 1
                if viewer_ctx is not None and steps % 25 == 0:
                    if not viewer_ctx.is_running():
                        # Closing the window must not take the fleet with it.
                        # Before this check, an accidental click on the X killed
                        # the physics while PX4 and the daemon kept running,
                        # leaving a fleet of ghosts that answered but never
                        # moved. Drop to headless instead.
                        viewer_ctx.close()
                        viewer_ctx = None
                        logger.info("viewer closed; continuing headless (sim.sh stop to shut down)")
                    else:
                        viewer_ctx.sync()
                now = time.monotonic()
                if now - last >= 5.0:
                    rtf = (steps * self.model.opt.timestep) / (now - last)
                    logger.info(
                        f"sim={self.sim_time_us / 1e6:7.1f}s rtf={rtf:5.2f}x "
                        + " ".join(
                            f"{lk.name}(alt={self.data.xpos[lk.body][2]:5.2f},"
                            f"armed={int(lk.armed)})"
                            for lk in self.links
                        )
                        + "".join(
                            f" {d.name}(z={self.data.xpos[d.body][2]:4.2f})" for d in self.dogs
                        )
                    )
                    last, steps = now, 0
        except (KeyboardInterrupt, ConnectionError, TimeoutError) as e:
            logger.info(f"fleet bridge stopping: {e or 'interrupted'}")
        finally:
            if viewer_ctx is not None:
                viewer_ctx.close()
            for link in self.links:
                link.close()
            for dog in self.dogs:
                dog.close()


def main() -> None:
    p = argparse.ArgumentParser(description="PX4 SITL fleet + legged robots <-> MuJoCo")
    p.add_argument("--drones", type=int, default=3)
    p.add_argument("--dogs", type=int, default=1)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port-base", type=int, default=4560)
    p.add_argument("--legged-port-base", type=int, default=LEGGED_PORT_BASE)
    p.add_argument("--lat", type=float, default=DEFAULT_ORIGIN_LAT)
    p.add_argument("--lon", type=float, default=DEFAULT_ORIGIN_LON)
    p.add_argument("--alt", type=float, default=DEFAULT_ORIGIN_ALT)
    p.add_argument("--viewer", action="store_true")
    a = p.parse_args()
    Px4HilFleet(
        n_drones=a.drones, n_dogs=a.dogs, host=a.host, port_base=a.port_base,
        legged_port_base=a.legged_port_base, origin=(a.lat, a.lon, a.alt), viewer=a.viewer,
    ).run()


if __name__ == "__main__":
    main()
