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

"""MAVLink HIL protocol: constants, frame conversions, and one vehicle's link.

Gazebo talks to PX4 through ``gz_bridge``, a module compiled into the firmware.
MuJoCo has no equivalent, so this implements the other supported path: PX4's
generic HIL interface (``simulator_mavlink``), the same one jMAVSim, JSBSim and
FlightGear use. The firmware is unmodified, which is the property the hardware
deliverable depends on -- what flies here is what flies on a Pixhawk.

Protocol, from PX4's SimulatorMavlink.cpp:

* Transport is **TCP on 4560 + instance**, and *PX4 is the client*. It sits in a
  connect() retry loop at startup, so we must be listening first.
* Sim -> PX4: ``HIL_SENSOR`` every step, ``HIL_GPS`` at a lower rate.
* PX4 -> Sim: ``HIL_ACTUATOR_CONTROLS``, normalised motor outputs, zero while
  disarmed.

Lockstep is the part worth understanding. On every ``HIL_SENSOR`` with
``id == 0`` PX4 calls ``px4_clock_settime()`` with our ``time_usec`` and then
``px4_lockstep_progress()``. **The simulator owns PX4's clock.** PX4 never runs
on wall-clock time; it advances exactly as fast as we feed it. A slow host makes
the simulation run slow rather than starving the sensor stream -- the failure
that caps the Gazebo path at one flyable drone on a loaded machine.

Frames, which is where this kind of bridge usually goes wrong:

* MuJoCo body frame is **FLU** (x-Forward, y-Left, z-Up); PX4 body is **FRD**.
* MuJoCo world frame is **NWU** (x-North, y-West, z-Up); PX4 world is **NED**.

Both conversions are ``(x, -y, -z)``. NWU rather than the more usual ENU is
deliberate: PX4 assumes a body at zero yaw points North, and a body at identity
in MuJoCo points along world +x. An ENU world puts "forward" at East, which
yaws the magnetometer 90 degrees and diverges the attitude estimate.

The runnable entry point is :mod:`dimos.simulation.px4_hil.fleet_bridge`, which
drives any number of these links (including exactly one) against one shared
MuJoCo world.
"""

from __future__ import annotations

import math
import socket
from typing import Any

import numpy as np

from dimos.simulation.px4_hil.scene import MAX_THRUST_PER_ROTOR_N

# --- HIL_SENSOR.fields_updated bitmask -------------------------------------
# Verbatim from SimulatorMavlink.hpp `enum class SensorSource`. PX4 ignores any
# sensor group whose bits are not set, so getting these wrong shows up as "no
# valid data from Baro 0" rather than as an obvious protocol error.
FIELD_ACCEL = 0b111                # xacc, yacc, zacc
FIELD_GYRO = 0b111000              # xgyro, ygyro, zgyro
FIELD_MAG = 0b111000000            # xmag, ymag, zmag
FIELD_BARO = 0b1101000000000       # abs_pressure, pressure_alt, temperature
FIELDS_MULTIROTOR = FIELD_ACCEL | FIELD_GYRO | FIELD_MAG | FIELD_BARO

# PX4's default SITL home (Zurich Irchel), so QGroundControl and the Gazebo-era
# blueprints agree on where "home" is.
DEFAULT_ORIGIN_LAT = 47.397742
DEFAULT_ORIGIN_LON = 8.545594
DEFAULT_ORIGIN_ALT = 488.0

# Earth magnetic field at that origin, in NED, Gauss. A wrong magnitude or sign
# shows up as the EKF refusing to converge on yaw, which reads like a control
# bug rather than a sensor one.
MAG_FIELD_NED_GAUSS = (0.21523, 0.0, 0.42980)

EARTH_RADIUS_M = 6371000.0
SEA_LEVEL_PRESSURE_HPA = 1013.25
SEA_LEVEL_TEMP_C = 15.0
TEMP_LAPSE_RATE_C_PER_M = 0.0065

# --- Sensor noise ----------------------------------------------------------
# EKF2 is tuned for real, noisy sensors and *needs* that noise. Fed a perfectly
# deterministic stream its covariance collapses and any small model mismatch
# lands in the bias states instead: an invented accelerometer bias, a yaw that
# settles far from truth, a position that drifts kilometres at zero velocity.
# gz_bridge and jMAVSim inject noise for exactly this reason. Values are
# 1-sigma per sample at 250 Hz (sigma = density * sqrt(rate)).
NOISE_ACCEL_MS2 = 0.03      # 1.86e-3 m/s^2/sqrt(Hz)
NOISE_GYRO_RADS = 0.003     # 1.87e-4 rad/s/sqrt(Hz)
NOISE_MAG_GAUSS = 0.0004
NOISE_BARO_HPA = 0.01
NOISE_GPS_POS_M = 0.03      # well inside the eph/epv we advertise
NOISE_GPS_VEL_MS = 0.02

# HIL_GPS is expensive relative to HIL_SENSOR, and real receivers are slow.
GPS_RATE_HZ = 10.0

# Deadline for collecting every vehicle's actuator reply in one lockstep cycle.
# PX4 publishes nothing until its output modules are up, so early cycles time
# out legitimately; this only has to be long enough to cover a scheduler
# round-trip once things are running.
ACTUATOR_WAIT_S = 0.05


def _flu_to_frd(v: Any) -> tuple[float, float, float]:
    """MuJoCo body frame (Forward-Left-Up) -> PX4 body frame (Forward-Right-Down)."""
    return (float(v[0]), float(-v[1]), float(-v[2]))


def _nwu_to_ned(v: Any) -> tuple[float, float, float]:
    """MuJoCo world frame (North-West-Up) -> PX4 world frame (North-East-Down)."""
    return (float(v[0]), float(-v[1]), float(-v[2]))


def _pressure_hpa(altitude_m: float) -> float:
    """Barometric pressure at an altitude above mean sea level, hPa."""
    base_k = SEA_LEVEL_TEMP_C + 273.15
    temp_k = base_k - TEMP_LAPSE_RATE_C_PER_M * altitude_m
    return SEA_LEVEL_PRESSURE_HPA * (temp_k / base_k) ** 5.25588


# ---------------------------------------------------------------------------
# Rotor-craft aerodynamics -- the "necessary physics" layer
# ---------------------------------------------------------------------------
# MuJoCo's built-in density-based drag covers the airframe as a bluff body, but
# none of what makes a multirotor feel like one. These are the standard
# first-order additions every serious PX4 sim carries, with the standard
# approximations, all computed in SIM time inside the bridge:
#
#   * motor lag      -- ESC+prop spool is a first-order response, not a step.
#   * rotor H-drag   -- a translating rotor disc produces an in-plane force
#                       opposing airspeed, proportional to thrust. THE dominant
#                       damping on a multirotor; without it, braking and wind
#                       response are wrong.
#   * ground effect  -- thrust rises near the ground (classic image model),
#                       felt as float in the last half-metre of a landing.
#   * wind + gusts   -- constant wind (SIM_WIND_N/SIM_WIND_E, m/s) plus an
#                       Ornstein-Uhlenbeck gust process (SIM_GUST_STD, m/s).
#                       Enters through the airspeed the H-drag sees.
MOTOR_TAU_S = 0.06          # ESC+prop spool time constant (typ. 0.02-0.1)
ROTOR_HDRAG_S_PER_M = 0.06  # H-force coefficient (typ. 0.03-0.10)
ROTOR_RADIUS_M = 0.127      # 10-inch prop
GROUND_EFFECT_MAX = 0.12    # cap the boost at 12% of current thrust
GUST_TAU_S = 2.0            # gust correlation time


def _motor_lag_step(state: np.ndarray, command: np.ndarray, dt: float) -> np.ndarray:
    """First-order spool toward `command`; returns the new state."""
    alpha = 1.0 - math.exp(-dt / MOTOR_TAU_S)
    return state + alpha * (command - state)


def _ground_effect_boost(height_m: float) -> float:
    """Extra thrust fraction from the classic image-rotor model, capped.

    boost = 1/(1 - (R/4h)^2) - 1, clamped below h = R/2 where the model blows
    up (a landed vehicle is not hovering in its own wake).
    """
    h = max(height_m, ROTOR_RADIUS_M / 2.0)
    ratio = ROTOR_RADIUS_M / (4.0 * h)
    return min(GROUND_EFFECT_MAX, 1.0 / (1.0 - ratio * ratio) - 1.0)


def _rotor_hdrag_force(v_air_world: np.ndarray, thrust_n: float) -> np.ndarray:
    """In-plane rotor drag, world frame: -c * T * v_air (xy only).

    Proportional to thrust because the H-force scales with blade lift; zero
    thrust (motors off, falling) produces none, which is also correct.
    """
    f = -ROTOR_HDRAG_S_PER_M * thrust_n * v_air_world
    f[2] = 0.0
    return f


class DroneLink:
    """One PX4 instance's HIL endpoint, bound to one body in a shared world.

    Owns the socket and the MuJoCo handles for a single vehicle; the caller owns
    the world and the stepping. Keeping the loop out of here is what lets one
    process drive an arbitrary number of vehicles in lockstep together.
    """

    def __init__(self, mj: Any, model: Any, data: Any, index: int, host: str, port: int) -> None:
        self.index = index
        self.name = f"drone{index}"
        self.host = host
        self.port = port
        self._mj = mj
        self.model = model
        self.data = data

        n2i = mj.mj_name2id
        self.site = n2i(model, mj.mjtObj.mjOBJ_SITE, f"{self.name}_imu")
        self.body = n2i(model, mj.mjtObj.mjOBJ_BODY, f"{self.name}_base")
        if self.site < 0 or self.body < 0:
            raise ValueError(f"scene has no body/site for {self.name}")
        self.acc_adr = int(model.sensor_adr[n2i(model, mj.mjtObj.mjOBJ_SENSOR, f"{self.name}_acc")])
        self.gyro_adr = int(
            model.sensor_adr[n2i(model, mj.mjtObj.mjOBJ_SENSOR, f"{self.name}_gyro")]
        )
        jid = n2i(model, mj.mjtObj.mjOBJ_JOINT, f"{self.name}_root")
        self.dof_adr = int(model.jnt_dofadr[jid])
        self.motor_ids = [
            n2i(model, mj.mjtObj.mjOBJ_ACTUATOR, f"{self.name}_motor{k}") for k in range(4)
        ]

        self.sock: socket.socket | None = None
        self.conn: socket.socket | None = None
        self.mav: Any = None
        self.controls = np.zeros(4)
        # Filtered motor state (what the "motors" actually produce after lag).
        self.motor_state = np.zeros(4)
        # Visual handles are optional -- absent on the primitive airframe.
        gid = lambda n: n2i(model, mj.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
        self.prop_gids = [gid(f"{self.name}_prop{k}") for k in range(4)]
        self.disc_gids = [gid(f"{self.name}_disc{k}") for k in range(4)]
        self.led_gid = gid(f"{self.name}_led")
        self._have_fx = all(g >= 0 for g in self.prop_gids + self.disc_gids)
        self._prop_angle = 0.0
        self.armed = False
        self.rx_actuator = 0
        self.got_reply = False
        self.rng = np.random.default_rng(1000 + index)
        # Scratch buffers, reused every step. At 250 Hz x N vehicles the
        # allocation churn from fresh noise arrays is measurable.
        self._noise3 = np.empty(3)

    # -- connection --------------------------------------------------------

    def listen(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        self.sock.settimeout(0.5)

    def try_accept(self) -> bool:
        if self.conn is not None:
            return True
        assert self.sock is not None
        try:
            conn, _addr = self.sock.accept()
        except (TimeoutError, BlockingIOError):
            return False
        from pymavlink.dialects.v20 import common as mavlink

        # PX4 sets TCP_NODELAY on its side; match it or lockstep pays a Nagle
        # delay on every single exchange.
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setblocking(False)
        self.conn = conn
        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        return True

    def fileno(self) -> int:
        return self.conn.fileno() if self.conn is not None else -1

    def close(self) -> None:
        for s in (self.conn, self.sock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self.conn = None

    # -- sim -> PX4 --------------------------------------------------------

    def send_sensors(self, sim_time_us: int, origin: tuple[float, float, float]) -> None:
        from pymavlink.dialects.v20 import common as mavlink

        if self.conn is None:
            return
        d = self.data
        rng = self.rng
        acc = d.sensordata[self.acc_adr : self.acc_adr + 3]
        gyro = d.sensordata[self.gyro_adr : self.gyro_adr + 3]
        xacc, yacc, zacc = _flu_to_frd(acc + rng.normal(0, NOISE_ACCEL_MS2, 3))
        xgyro, ygyro, zgyro = _flu_to_frd(gyro + rng.normal(0, NOISE_GYRO_RADS, 3))

        # site_xmat is body->world (NWU); its transpose takes world->body.
        r_body_to_world = d.site_xmat[self.site].reshape(3, 3)
        n, e, dn = MAG_FIELD_NED_GAUSS
        mag = r_body_to_world.T @ np.array([n, -e, -dn]) + rng.normal(0, NOISE_MAG_GAUSS, 3)
        xmag, ymag, zmag = _flu_to_frd(mag)

        altitude = float(d.xpos[self.body][2]) + origin[2]
        self._send(
            mavlink.MAVLink_hil_sensor_message(
                time_usec=sim_time_us,
                xacc=xacc, yacc=yacc, zacc=zacc,
                xgyro=xgyro, ygyro=ygyro, zgyro=zgyro,
                xmag=xmag, ymag=ymag, zmag=zmag,
                abs_pressure=_pressure_hpa(altitude) + rng.normal(0, NOISE_BARO_HPA),
                diff_pressure=0.0,
                pressure_alt=altitude,
                temperature=SEA_LEVEL_TEMP_C - TEMP_LAPSE_RATE_C_PER_M * altitude,
                fields_updated=FIELDS_MULTIROTOR,
                id=0,  # primary IMU: this is the message that drives lockstep
            )
        )

    def send_gps(self, sim_time_us: int, origin: tuple[float, float, float]) -> None:
        from pymavlink.dialects.v20 import common as mavlink

        if self.conn is None:
            return
        olat, olon, oalt = origin
        d = self.data
        pos = d.xpos[self.body] + self.rng.normal(0, NOISE_GPS_POS_M, 3)
        vel = d.qvel[self.dof_adr : self.dof_adr + 3] + self.rng.normal(0, NOISE_GPS_VEL_MS, 3)
        n, e, down = _nwu_to_ned(pos)
        vn, ve, vd = _nwu_to_ned(vel)
        lat = olat + math.degrees(n / EARTH_RADIUS_M)
        lon = olon + math.degrees(e / (EARTH_RADIUS_M * math.cos(math.radians(olat))))
        self._send(
            mavlink.MAVLink_hil_gps_message(
                time_usec=sim_time_us,
                fix_type=3,
                lat=int(lat * 1e7), lon=int(lon * 1e7), alt=int((oalt - down) * 1e3),
                eph=30, epv=40,
                vel=int(math.hypot(vn, ve) * 100),
                vn=int(vn * 100), ve=int(ve * 100), vd=int(vd * 100),
                cog=int((math.degrees(math.atan2(ve, vn)) % 360.0) * 100),
                satellites_visible=14, id=0,
            )
        )

    def _send(self, msg: Any) -> None:
        assert self.conn is not None
        try:
            self.conn.sendall(msg.pack(self.mav))
        except (BlockingIOError, InterruptedError):
            # Kernel send buffer full: PX4 is behind. Dropping this sample is
            # better than stalling every other vehicle in the world.
            pass
        except OSError as e:
            raise ConnectionError(f"[{self.name}] send failed: {e}") from e

    # -- PX4 -> sim --------------------------------------------------------

    def pump(self) -> bool:
        """Drain whatever is readable. Returns True if an actuator command arrived."""
        if self.conn is None or self.mav is None:
            return False
        got = False
        while True:
            try:
                chunk = self.conn.recv(8192)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:
                raise ConnectionError(f"[{self.name}] recv failed: {e}") from e
            if not chunk:
                raise ConnectionError(f"[{self.name}] PX4 closed the connection")
            for msg in self.mav.parse_buffer(chunk) or []:
                if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                    self.controls[:] = msg.controls[:4]
                    # bit 7 of `mode` is MAV_MODE_FLAG_SAFETY_ARMED.
                    self.armed = bool(msg.mode & 0b10000000)
                    self.rx_actuator += 1
                    got = True
            if len(chunk) < 8192:
                break
        self.got_reply = self.got_reply or got
        return got

    def apply(self, wind_world: np.ndarray | None = None) -> None:
        np.clip(self.controls, 0.0, 1.0, out=self.controls)
        dt = float(self.model.opt.timestep)
        # Motor spool: PX4 commands a step; the airframe answers with a lag.
        self.motor_state = _motor_lag_step(self.motor_state, self.controls, dt)
        for k, aid in enumerate(self.motor_ids):
            self.data.ctrl[aid] = self.motor_state[k]

        thrust_n = float(self.motor_state.sum()) * MAX_THRUST_PER_ROTOR_N
        v_world = np.array(self.data.qvel[self.dof_adr : self.dof_adr + 3])
        v_air = v_world if wind_world is None else v_world - wind_world
        force = _rotor_hdrag_force(v_air, thrust_n)
        height = float(self.data.xpos[self.body][2])
        force[2] += _ground_effect_boost(height) * thrust_n
        self.data.xfrc_applied[self.body, :3] = force

        self._update_fx(dt)

    def _update_fx(self, dt: float) -> None:
        """Drive the cosmetic layer: prop/disc cross-fade, spin, status LED.

        Pure rendering -- every geom touched is contype 0 / mass 0. Writes go
        to the model's rgba/quat arrays, which the viewer re-reads each frame.
        """
        if not self._have_fx:
            return
        throttle = float(self.motor_state.mean())
        # Cross-fade: still blades at rest, translucent disc at speed.
        prop_a = max(0.15, 1.0 - 3.0 * throttle)
        disc_a = min(0.38, 1.6 * throttle)
        # Slow visible churn during spool-up sells the transition; the quat
        # write is on the mesh geom only.
        self._prop_angle = (self._prop_angle + 60.0 * throttle * dt) % (2.0 * math.pi)
        half = self._prop_angle / 2.0
        cw, sw = math.cos(half), math.sin(half)
        for k in range(4):
            self.model.geom_rgba[self.prop_gids[k], 3] = prop_a
            self.model.geom_rgba[self.disc_gids[k], 3] = disc_a
            q = self.model.geom_quat[self.prop_gids[k]]
            q[0], q[1], q[2], q[3] = cw, 0.0, 0.0, (sw if k < 2 else -sw)
        if self.led_gid >= 0:
            self.model.geom_rgba[self.led_gid, :3] = (
                (0.95, 0.15, 0.1) if self.armed else (0.1, 0.9, 0.2)
            )


__all__ = [
    "ACTUATOR_WAIT_S",
    "DEFAULT_ORIGIN_ALT",
    "DEFAULT_ORIGIN_LAT",
    "DEFAULT_ORIGIN_LON",
    "FIELDS_MULTIROTOR",
    "GPS_RATE_HZ",
    "MAG_FIELD_NED_GAUSS",
    "DroneLink",
]
