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

"""One PX4 vehicle, one module instance.

This is the namespaced replacement for the per-drone half of
``Px4SitlFleetModule``. Instead of one module multiplexing N connections behind
``drone="drone-1"`` string arguments, each vehicle gets its own instance under
its own namespace::

    autoconnect(
        *[
            Px4DroneModule.blueprint(connection_string=c.connection_string)
            .namespace(f"drone{i + 1}", expose={"drone_state", "swarm_cmd"})
            for i, c in enumerate(get_px4_sitl_fleet_configs())
        ],
    )

which gives each drone its own RPC surface (``drone1/px4dronemodule/takeoff``),
its own topics (``/drone1/odom``), its own TF frames, and its own config keys
(``-o drone1/px4dronemodule.connection_string=...``).

Two streams are deliberately *exposed* (left unprefixed, so they stay global and
cross the namespace boundary):

``drone_state``
    Every drone publishes its telemetry snapshot here. ``SwarmCoordinator``
    subscribes once and sees the whole fleet.
``swarm_cmd``
    The coordinator broadcasts fleet commands here. Each drone acts only on
    messages addressed to its own key (or to ``all``).

Everything else — notably ``cmd_vel`` from a vision tracker — stays namespace
local, so drone 2's tracker can never drive drone 1.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.drone.px4_sitl_connection import Px4SitlConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How often each drone republishes its state snapshot on the shared bus.
STATE_PUBLISH_HZ = 4.0


class Px4DroneConfig(ModuleConfig):
    """Per-drone identity and safety envelope.

    Every field is addressable per instance, e.g.
    ``-o drone2/px4dronemodule.connection_string=udp:127.0.0.1:14541`` or
    ``DRONE2_PX4DRONEMODULE__MAX_ALTITUDE_M=30``.
    """

    # pymavlink connection string. SITL instance N is udp:127.0.0.1:1454N;
    # hardware is serial:/dev/ttyACM0:57600 or udp:192.168.1.50:14550.
    connection_string: str = "udp:127.0.0.1:14540"
    # PX4 SITL instance index (selects the OFFBOARD port PX4 binds locally).
    instance: int = 0
    # MAV_SYS_ID this vehicle heartbeats with. Informational for reporting.
    sys_id: int = 1
    # Optional ceiling in meters above home (positive up). None = no software
    # cap; PX4's own geofence params take over. Set on hardware blueprints.
    max_altitude_m: float | None = None


class Px4DroneModule(Module):
    """A single PX4 vehicle: MAVLink connection, telemetry, and flight skills.

    The skills carry no ``drone`` argument — the instance *is* the drone. When
    three of these run under ``drone1``/``drone2``/``drone3`` namespaces the
    agent sees three separate tool sets and addresses them by name natively.
    """

    # One vehicle per worker process. Namespacing isolates *names*, not faults:
    # without this, modules share workers, so a wedged MAVLink socket or a
    # GIL-holding loop on one drone can stall another. Cheap insurance for a
    # handful of vehicles, and the same thing GO2Connection does upstream.
    dedicated_worker = True

    config: Px4DroneConfig

    # Namespace-local: body-frame velocity from this drone's vision tracker.
    # Gated — dropped entirely until start_follow() arms forwarding.
    cmd_vel: In[Twist]

    # Namespace-local: this drone's pose, for viewers and TF.
    odom: Out[PoseStamped]

    # Exposed (global): telemetry out to the coordinator, commands back in.
    drone_state: Out[String]
    swarm_cmd: In[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.connection: Px4SitlConnection | None = None
        self._running = False
        self._telemetry_thread: threading.Thread | None = None
        self._state_thread: threading.Thread | None = None
        # Vision-follow gating: cmd_vel is dropped unless follow is explicitly on.
        self._follow_enabled = False
        self._follow_lock_altitude = True

    # ------------------------------------------------------------------
    # Identity
    # ------------------------------------------------------------------

    @property
    def drone_key(self) -> str:
        """Short fleet-facing name for this drone, derived from its namespace.

        Under ``.namespace("drone2")`` the coordinator sets ``instance_name`` to
        ``drone2/px4dronemodule``, so the key is ``drone2``. Un-namespaced (a
        single-drone blueprint) it falls back to ``drone1``.
        """
        name = self.config.instance_name
        if name and "/" in name:
            return name.rsplit("/", 1)[0]
        return name or "drone1"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True

        conn = Px4SitlConnection(
            connection_string=self.config.connection_string,
            instance=self.config.instance,
        )
        if not conn.connect():
            logger.warning(
                f"[{self.drone_key}] failed to connect at {self.config.connection_string}"
            )
            return
        self.connection = conn
        logger.info(
            f"[{self.drone_key}] connected "
            f"(sys_id={self.config.sys_id}, endpoint={self.config.connection_string})"
        )

        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            daemon=True,
            name=f"px4-tel-{self.drone_key}",
        )
        self._telemetry_thread.start()

        self._state_thread = threading.Thread(
            target=self._state_publish_loop,
            daemon=True,
            name=f"px4-state-{self.drone_key}",
        )
        self._state_thread.start()

        if getattr(self.cmd_vel, "transport", None):
            self.cmd_vel.subscribe(self._on_cmd_vel)
            logger.info(f"[{self.drone_key}] subscribed to cmd_vel (vision follow input)")
        if getattr(self.swarm_cmd, "transport", None):
            self.swarm_cmd.subscribe(self._on_swarm_cmd)
            logger.info(f"[{self.drone_key}] subscribed to swarm_cmd")

    def stop(self) -> None:
        self._running = False
        if self.connection is not None:
            try:
                self.connection.disconnect()
            except Exception as e:
                logger.debug(f"[{self.drone_key}] disconnect error: {e}")
        for t in (self._telemetry_thread, self._state_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)
        super().stop()

    def _telemetry_loop(self) -> None:
        while self._running and self.connection is not None:
            try:
                self.connection.update_telemetry(timeout=0.05)
            except Exception as e:
                logger.debug(f"[{self.drone_key}] telemetry error: {e}")
                time.sleep(0.1)

    def _state_publish_loop(self) -> None:
        """Republish this drone's snapshot on the shared bus at a fixed rate."""
        period = 1.0 / STATE_PUBLISH_HZ
        while self._running:
            try:
                self.drone_state.publish(String(json.dumps(self._state_dict())))
                pose = self._pose_stamped()
                if pose is not None:
                    self.odom.publish(pose)
            except Exception as e:
                logger.debug(f"[{self.drone_key}] state publish error: {e}")
            time.sleep(period)

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def _state_dict(self) -> dict[str, Any]:
        conn = self.connection
        if conn is None:
            return {"key": self.drone_key, "connected": False, "ts": time.time()}
        ned = conn.get_local_ned()
        g = conn.get_global_position()
        return {
            "key": self.drone_key,
            # Declared so SwarmCoordinator can pick robots by capability rather
            # than assume every reporting robot can fly. Without this a ground
            # robot gets handed an air lane in grid_sweep.
            "robot_class": "multirotor",
            "capabilities": ["air", "camera"],
            "connected": True,
            "sys_id": self.config.sys_id,
            "instance": self.config.instance,
            "armed": conn.get_armed(),
            "mode": conn.get_mode(),
            "battery_pct": conn.get_battery_pct(),
            "ned": list(ned) if ned is not None else None,
            "altitude_m": (-ned[2]) if ned is not None else None,
            "global": ({"lat": g[0], "lon": g[1], "rel_alt_m": g[2]} if g is not None else None),
            "ts": time.time(),
        }

    def _pose_stamped(self) -> PoseStamped | None:
        """This drone's local NED position as a ROS-convention pose (X fwd, Y left, Z up)."""
        conn = self.connection
        if conn is None:
            return None
        ned = conn.get_local_ned()
        if ned is None:
            return None
        n, e, d = ned
        # MAVLink NED (X=north, Y=east, Z=down) -> ROS/DimOS (X=fwd, Y=left, Z=up).
        pose = PoseStamped(ts=time.time(), frame_id=self.frame_id)
        pose.position.x = n
        pose.position.y = -e
        pose.position.z = -d
        return pose

    @skill
    def state(self) -> str:
        """Report this drone's position, armed state, battery, and altitude."""
        return json.dumps(self._state_dict(), indent=2)

    # ------------------------------------------------------------------
    # Guardrails
    # ------------------------------------------------------------------

    def _check_altitude_cap(self, altitude: float) -> str | None:
        """Return an error string if ``altitude`` exceeds the configured cap."""
        cap = self.config.max_altitude_m
        if cap is None:
            return None
        if altitude > cap:
            return (
                f"{self.drone_key}: REJECTED altitude {altitude:.1f}m exceeds cap "
                f"{cap:.1f}m. Stay below the cap or raise max_altitude_m."
            )
        return None

    def _require_conn(self) -> Px4SitlConnection | None:
        return self.connection

    # ------------------------------------------------------------------
    # Flight skills — no `drone` argument; the instance is the drone
    # ------------------------------------------------------------------

    @skill
    def arm(self) -> str:
        """Arm this drone's motors."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        return f"{self.drone_key}: arm {'OK' if conn.arm() else 'FAILED'}"

    @skill
    def takeoff(self, altitude: float = 3.0) -> str:
        """Arm and take off to ``altitude`` (meters above home, positive up)."""
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        ok = conn.takeoff(altitude)
        return f"{self.drone_key}: takeoff to {altitude}m {'OK' if ok else 'FAILED'}"

    @skill
    def land(self) -> str:
        """Land this drone where it currently is."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        return f"{self.drone_key}: land {'OK' if conn.land() else 'FAILED'}"

    @skill
    def rtl(self) -> str:
        """Return to launch and land."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        return f"{self.drone_key}: RTL {'OK' if conn.rtl() else 'FAILED'}"

    @skill
    def hold(self) -> str:
        """Engage AUTO.LOITER (hover in place)."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        return f"{self.drone_key}: hold {'OK' if conn.hold() else 'FAILED'}"

    @skill
    def goto(
        self,
        north: float = 0.0,
        east: float = 0.0,
        altitude: float = 3.0,
        yaw_deg: float = 0.0,
    ) -> str:
        """Fly to a local-NED waypoint via OFFBOARD setpoints.

        Args:
            north: North position (m) in this drone's home-relative NED frame.
            east: East position (m).
            altitude: Altitude above home in meters (positive up).
            yaw_deg: Heading in degrees (NED, 0 = north).

        Fleet-wide spacing is enforced by ``SwarmCoordinator``; this skill only
        applies the per-drone altitude cap. Prefer the coordinator's
        ``investigate``/``grid_sweep`` for multi-drone moves.
        """
        cap_err = self._check_altitude_cap(altitude)
        if cap_err:
            return cap_err
        return self._goto_ned(north, east, -altitude, math.radians(yaw_deg))

    def _goto_ned(self, north: float, east: float, down: float, yaw_rad: float = 0.0) -> str:
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        if not conn._offboard_running and not conn.start_offboard():
            return f"{self.drone_key}: failed to enter OFFBOARD"
        conn.set_position_ned(north, east, down, yaw_rad)
        return (
            f"{self.drone_key}: goto NED ({north:.1f}, {east:.1f}, "
            f"alt={-down:.1f}m), yaw={math.degrees(yaw_rad):.0f}°"
        )

    @skill
    def set_velocity(
        self,
        vn: float = 0.0,
        ve: float = 0.0,
        vd: float = 0.0,
        yaw_rate_deg: float = 0.0,
    ) -> str:
        """Stream a velocity setpoint (NED, m/s) in OFFBOARD.

        ``vd`` is positive-down. To climb at 1 m/s, pass ``vd=-1.0``.
        """
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        yaw_rate = math.radians(yaw_rate_deg)
        if not conn._offboard_running and not conn.start_offboard((vn, ve, vd, yaw_rate)):
            return f"{self.drone_key}: failed to enter OFFBOARD"
        conn.set_velocity_ned(vn, ve, vd, yaw_rate)
        return f"{self.drone_key}: velocity NED ({vn:.2f}, {ve:.2f}, {vd:.2f}) m/s"

    def _follow_path(self, path: list[list[float]], arrival_radius_m: float = 2.0) -> str:
        """Execute a waypoint list; the OFFBOARD streamer advances on arrival."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        if not conn._offboard_running and not conn.start_offboard():
            return f"{self.drone_key}: failed to enter OFFBOARD"
        conn.set_position_path(
            [(p[0], p[1], p[2], p[3] if len(p) > 3 else 0.0) for p in path],
            arrival_radius_m=arrival_radius_m,
        )
        return f"{self.drone_key}: {len(path)} waypoints dispatched"

    # ------------------------------------------------------------------
    # Emergency
    # ------------------------------------------------------------------

    @skill
    def kill(self) -> str:
        """Immediately force-disarm this drone — emergency stop.

        Sends MAV_CMD_COMPONENT_ARM_DISARM with the force magic value (21196),
        which PX4 honors **even mid-flight**. Motors cut. The drone falls.
        **Use ONLY when crashing is preferable to whatever it is about to do** —
        e.g. heading toward a person, lost link, autopilot runaway. For normal
        shutdown use ``land`` or ``rtl``.

        Also tears down the OFFBOARD streamer so the disarm sticks.
        """
        from pymavlink import mavutil

        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        conn.stop_offboard()
        conn.mavlink.mav.command_long_send(
            conn.mavlink.target_system,
            conn.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # param1: 0 = disarm
            21196.0,  # param2: force flag (PX4 specific)
            0, 0, 0, 0, 0,
        )
        logger.warning(f"[{self.drone_key}] KILL — force-disarm sent")
        return f"{self.drone_key}: KILL sent (force-disarm). Motors stopped."

    @skill
    def emergency_land(self) -> str:
        """Abort OFFBOARD and land now."""
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        conn.stop_offboard()
        return f"{self.drone_key}: emergency land {'OK' if conn.land() else 'FAILED'}"

    # ------------------------------------------------------------------
    # Vision follow (namespace-local cmd_vel)
    # ------------------------------------------------------------------

    @skill
    def start_follow(self, lock_altitude: bool = True) -> str:
        """Allow this drone's vision tracker to drive it (OFFBOARD body-frame).

        Safety gate: until this is called, cmd_vel from the tracker is ignored.
        The drone should already be armed and airborne.
        """
        conn = self._require_conn()
        if conn is None:
            return f"{self.drone_key}: NOT CONNECTED"
        if not conn._offboard_running and not conn.start_offboard():
            return f"{self.drone_key}: failed to enter OFFBOARD"
        self._follow_lock_altitude = lock_altitude
        self._follow_enabled = True
        return f"{self.drone_key}: follow ENABLED (lock_altitude={lock_altitude})"

    @skill
    def stop_follow(self) -> str:
        """Stop honoring tracker velocities and hold position."""
        self._follow_enabled = False
        conn = self._require_conn()
        if conn is not None:
            try:
                conn.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            except Exception as e:
                logger.debug(f"[{self.drone_key}] zero-velocity on stop_follow failed: {e}")
        return f"{self.drone_key}: follow DISABLED"

    def _on_cmd_vel(self, twist: Twist) -> None:
        """Forward a tracking Twist — only when follow is explicitly armed."""
        if not self._follow_enabled:
            return
        conn = self.connection
        if conn is None:
            return
        try:
            conn.move_twist(twist, duration=0.0, lock_altitude=self._follow_lock_altitude)
        except Exception as e:
            logger.warning(f"[{self.drone_key}] cmd_vel forward failed: {e}")

    # ------------------------------------------------------------------
    # Swarm command bus
    # ------------------------------------------------------------------

    def _on_swarm_cmd(self, msg: String) -> None:
        """Act on a coordinator broadcast addressed to this drone (or to ``all``)."""
        try:
            cmd = json.loads(msg.data if hasattr(msg, "data") else str(msg))
        except (ValueError, AttributeError) as e:
            logger.warning(f"[{self.drone_key}] malformed swarm_cmd: {e}")
            return

        target = cmd.get("target", "all")
        if target != "all" and target != self.drone_key:
            return

        action = cmd.get("action", "")
        args = cmd.get("args", {}) or {}
        try:
            result = self._dispatch(action, args)
        except Exception as e:
            logger.warning(f"[{self.drone_key}] swarm_cmd {action} failed: {e}")
            return
        logger.info(f"[{self.drone_key}] swarm_cmd {action}: {result}")

    def _dispatch(self, action: str, args: dict[str, Any]) -> str:
        """Map a swarm-bus action name onto this drone's implementation."""
        if action == "takeoff":
            return self.takeoff(float(args.get("altitude", 3.0)))
        if action == "land":
            return self.land()
        if action == "rtl":
            return self.rtl()
        if action == "hold":
            return self.hold()
        if action == "arm":
            return self.arm()
        if action == "kill":
            return self.kill()
        if action == "emergency_land":
            return self.emergency_land()
        if action == "goto_ned":
            return self._goto_ned(
                float(args["north"]),
                float(args["east"]),
                float(args["down"]),
                float(args.get("yaw_rad", 0.0)),
            )
        if action == "path":
            return self._follow_path(
                args.get("path", []),
                float(args.get("arrival_radius_m", 2.0)),
            )
        return f"unknown action {action!r}"


__all__ = ["STATE_PUBLISH_HZ", "Px4DroneConfig", "Px4DroneModule"]
