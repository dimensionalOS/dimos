#!/usr/bin/env python3
"""Lightweight skill proxy for MuJoCo drone simulation.

Exposes the same @skill interface as DroneConnectionModule but sends
commands to the MuJoCo viewer bridge via a UDP socket (no LCM dependency).
"""

import json
import math
import socket
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# UDP port for command channel (proxy → bridge)
CMD_PORT = 19876


class MuJocoSkillProxy(Module):
    """Skill proxy that forwards agent commands to the MuJoCo viewer via UDP."""

    # Stream interface (compatible with DroneConnectionModule)
    movecmd: In[Vector3]
    movecmd_twist: In[Twist]
    gps_goal: In[LatLon]
    tracking_status: In[Any]
    odom: Out[PoseStamped]
    gps_location: Out[LatLon]
    status: Out[Any]
    telemetry: Out[Any]
    video: Out[Image]
    follow_object_cmd: Out[Any]

    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        super().__init__()

    @rpc
    def start(self) -> None:
        super().start()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info("MuJocoSkillProxy started — sending commands via UDP")

    def _send_cmd(self, cmd: dict) -> None:
        if self._sock:
            data = json.dumps(cmd).encode()
            self._sock.sendto(data, ("127.0.0.1", CMD_PORT))

    @skill
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, duration: float = 0.0) -> str:
        """Send velocity movement command to drone.

        Args:
            x: Forward velocity m/s (positive = forward)
            y: Left/right velocity m/s (positive = left)
            z: Up/down velocity m/s (positive = up)
            duration: How long to move in seconds (0 = single command)
        """
        self._send_cmd({"type": "velocity", "vx": x, "vy": y, "vz": z, "yaw_rate": 0, "duration": duration})
        return f"Moving: vx={x}, vy={y}, vz={z} for {duration}s"

    @skill
    def move_with_yaw(
        self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
        yaw_rate: float = 0.0, duration: float = 2.0,
    ) -> str:
        """Move the drone with velocity and yaw control.

        Positive yaw_rate = turn right, negative = turn left.

        Args:
            vx: Forward velocity m/s
            vy: Right velocity m/s
            vz: Up velocity m/s
            yaw_rate: Yaw rate rad/s (positive=right). 1.57 = ~90 deg/s.
            duration: Seconds to apply
        """
        self._send_cmd({"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate, "duration": duration})
        return f"Moving: vx={vx}, vy={vy}, vz={vz}, yaw={yaw_rate} for {duration}s"

    @skill
    def takeoff(self, altitude: float = 3.0) -> str:
        """Takeoff to altitude in meters."""
        self._send_cmd({"type": "takeoff", "altitude": altitude})
        return f"Taking off to {altitude}m"

    @skill
    def land(self) -> str:
        """Land the drone."""
        self._send_cmd({"type": "land"})
        return "Landing"

    @skill
    def arm(self) -> str:
        """Arm the drone motors."""
        self._send_cmd({"type": "arm"})
        return "Armed"

    @skill
    def disarm(self) -> str:
        """Disarm the drone motors."""
        self._send_cmd({"type": "disarm"})
        return "Disarmed"

    @skill
    def set_mode(self, mode: str) -> str:
        """Set flight mode (GUIDED, STABILIZE, LAND, RTL)."""
        self._send_cmd({"type": "set_mode", "mode": mode})
        return f"Mode set to {mode}"

    @skill
    def fly_to(self, lat: float, lon: float, alt: float) -> str:
        """Fly to GPS coordinates."""
        self._send_cmd({"type": "fly_to", "lat": lat, "lon": lon, "alt": alt})
        return f"Flying to {lat:.6f}, {lon:.6f} at {alt}m"

    def _exec_segment(self, vx: float, vy: float, vz: float, duration: float) -> None:
        """Execute a velocity segment and wait."""
        self._send_cmd({"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": 0, "duration": duration})
        time.sleep(duration)

    def _hover(self, duration: float) -> None:
        """Hover in place."""
        self._exec_segment(0, 0, 0, duration)

    def _stroke(self, segments: list[tuple[float, float, float]], speed: float = 0.2) -> None:
        """Trace a path defined by (dx, dy, dz) displacement segments at given speed."""
        for dx, dy, dz in segments:
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist < 0.001:
                continue
            dur = dist / speed
            self._exec_segment(dx / dur, dy / dur, dz / dur, dur)
            time.sleep(0.15)  # tiny pause between segments for stability

    @skill
    def draw_dimos(self) -> str:
        """Draw the word 'DimOS' in the air using the drone. The drone will take off,
        stabilize, then trace each letter with smooth movements. This is a choreographed
        flight sequence that takes about 2 minutes.

        Letters are drawn in the vertical plane (XZ): forward along the road for
        letter width, altitude for letter height. No tilting — the drone stays level.
        """
        SP = 0.3  # speed m/s
        H = 1.0   # letter height (altitude change)
        W = 0.6   # letter width (forward distance)
        GAP = 0.3 # gap between letters (forward)

        logger.info("Starting DimOS sky-writing sequence...")

        # ── Phase 1: Takeoff + stabilize ────────────────────────────
        self._send_cmd({"type": "takeoff", "altitude": 3.0})
        time.sleep(5.0)
        self._hover(2.0)
        logger.info("Takeoff complete, beginning sky-writing")

        # Drawing in XZ plane: X = forward (along road), Z = up/down
        # Letters progress forward (+X). No Y movement needed.

        # ── Letter D ────────────────────────────────────────────────
        logger.info("Drawing: D")
        # Vertical stroke up
        self._stroke([(0, 0, H)], SP)
        # Curved forward side (arc going forward and down)
        n = 10
        for i in range(n):
            a0 = math.pi * i / n
            a1 = math.pi * (i + 1) / n
            dx = (W * 0.5) * (math.sin(a1) - math.sin(a0))
            dz = -(H / 2) * (1 - math.cos(a1)) + (H / 2) * (1 - math.cos(a0))
            self._stroke([(dx, 0, dz)], SP * 0.7)
        self._hover(0.3)
        # Move forward to next letter start
        self._stroke([(GAP, 0, 0)], SP)
        self._hover(0.3)

        # ── Letter i ────────────────────────────────────────────────
        logger.info("Drawing: i")
        # Short vertical stroke
        self._stroke([(0, 0, H * 0.6)], SP)
        # Back to baseline
        self._stroke([(0, 0, -H * 0.6)], SP)
        # Dot: move up above the stroke, pause, come back
        self._stroke([(0, 0, H * 0.75)], SP * 0.8)
        self._hover(0.4)
        self._stroke([(0, 0, -H * 0.75)], SP * 0.8)
        self._hover(0.3)
        # Move forward to next letter
        self._stroke([(GAP, 0, 0)], SP)
        self._hover(0.3)

        # ── Letter m ────────────────────────────────────────────────
        logger.info("Drawing: m")
        mH = H * 0.6  # m is shorter
        # Up
        self._stroke([(0, 0, mH)], SP)
        # First hump: arc forward and down
        n = 8
        for i in range(n):
            a0 = math.pi * i / n
            a1 = math.pi * (i + 1) / n
            dx = (W * 0.35) * (a1 - a0) / math.pi
            dz = -mH * (math.cos(a0) - math.cos(a1)) / 2
            self._stroke([(dx, 0, dz)], SP * 0.7)
        # Back up for second hump
        self._stroke([(0, 0, mH)], SP)
        # Second hump
        for i in range(n):
            a0 = math.pi * i / n
            a1 = math.pi * (i + 1) / n
            dx = (W * 0.35) * (a1 - a0) / math.pi
            dz = -mH * (math.cos(a0) - math.cos(a1)) / 2
            self._stroke([(dx, 0, dz)], SP * 0.7)
        self._hover(0.3)
        # Move forward to next letter
        self._stroke([(GAP, 0, 0)], SP)
        self._hover(0.3)

        # ── Letter O ────────────────────────────────────────────────
        logger.info("Drawing: O")
        # Ellipse in XZ plane (forward + up/down)
        n = 16
        for i in range(n):
            a0 = 2 * math.pi * i / n
            a1 = 2 * math.pi * (i + 1) / n
            dx = (W * 0.4) * (math.cos(a1) - math.cos(a0))
            dz = (H * 0.5) * (math.sin(a1) - math.sin(a0))
            self._stroke([(dx, 0, dz)], SP * 0.7)
        self._hover(0.3)
        # Move forward past the O width + gap
        self._stroke([(W * 0.4 + GAP, 0, 0)], SP)
        self._hover(0.3)

        # ── Letter S ────────────────────────────────────────────────
        logger.info("Drawing: S")
        # Go to top of S
        self._stroke([(0, 0, H)], SP)
        # Top half-circle: forward and down to midpoint
        n = 8
        for i in range(n):
            a0 = math.pi / 2 + math.pi * i / n
            a1 = math.pi / 2 + math.pi * (i + 1) / n
            dx = (W * 0.35) * (math.cos(a0) - math.cos(a1))
            dz = -(H * 0.25) * (math.sin(a0) - math.sin(a1))
            self._stroke([(dx, 0, dz)], SP * 0.7)
        # Bottom half-circle: backward and down (reverse curl)
        for i in range(n):
            a0 = -math.pi / 2 + math.pi * i / n
            a1 = -math.pi / 2 + math.pi * (i + 1) / n
            dx = (W * 0.35) * (math.cos(a0) - math.cos(a1))
            dz = -(H * 0.25) * (math.sin(a0) - math.sin(a1))
            self._stroke([(dx, 0, dz)], SP * 0.7)
        self._hover(0.5)

        # ── Phase 5: Return + land ──────────────────────────────────
        logger.info("Drawing complete! Hovering before landing...")
        self._hover(2.0)
        self._send_cmd({"type": "land"})

        return "DimOS sky-writing complete! The drone traced all 5 letters and is landing."

    @skill
    def execute_path(self, moves: str) -> str:
        """Execute a sequence of velocity moves. Each move is 'vx,vy,vz,duration' separated by semicolons.

        Example: '0.2,0,0,1.0;0,0.2,0,1.0;-0.2,0,0,1.0'

        Args:
            moves: Semicolon-separated move commands, each as 'vx,vy,vz,duration'
        """
        segments = moves.strip().split(";")
        for seg in segments:
            parts = seg.strip().split(",")
            if len(parts) != 4:
                continue
            vx, vy, vz, dur = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
            self._exec_segment(vx, vy, vz, dur)
            time.sleep(0.1)
        return f"Executed {len(segments)} path segments"

    @rpc
    def stop(self) -> None:
        if self._sock:
            self._sock.close()
        super().stop()
