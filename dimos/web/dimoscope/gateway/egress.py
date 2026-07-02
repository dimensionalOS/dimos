#!/usr/bin/env python3
# The teleop/goal/rpc trust boundary, shared by every transport that carries write-path control.
# Velocity clamp + per-connection TTL deadman + stop-on-disconnect + a whitelisted @rpc bridge. One
# instance per service, used by both the /ws DataPlane and the WebTransport server, keyed by a
# per-connection object (WebSocket or WT session) so each connection gets its own deadman timer.
#
# Egress goes to both backends: teleop Twist / nav goal PointStamped publish over Zenoh *and* LCM (each
# via dimos make_transport), so whichever the robot listens on receives them; the other is harmless.
# RPC uses the service's default backend.
from __future__ import annotations

import asyncio
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAX_LIN = 1.0  # m/s clamp
MAX_ANG = 1.5  # rad/s clamp
DEFAULT_TTL = 400.0  # deadman timeout (ms)

# RPC bridge: which dimos @rpc commands the browser may invoke (server-side authoritative whitelist).
RPC_COMMANDS = [
    {"target": "GO2Connection", "method": "standup", "label": "Stand up"},
    {"target": "GO2Connection", "method": "liedown", "label": "Lie down"},
    {"target": "GO2Load", "method": "start_all", "label": "Start streams"},
    {"target": "GO2Load", "method": "stop_all", "label": "Stop streams"},
    {"target": "GO2Load", "method": "start_bench", "label": "Start bench"},
    {"target": "GO2Load", "method": "stop_bench", "label": "Stop bench"},
]
RPC_WHITELIST = {(c["target"], c["method"]) for c in RPC_COMMANDS}


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _jsonable(v: object) -> object:
    return v if isinstance(v, (bool, int, float, str)) or v is None else str(v)


class SafetyEgress:
    """Teleop/goal/rpc egress with the safety boundary, keyed by a per-connection object."""

    def __init__(self) -> None:
        self._cmd: list = []  # teleop Twist publishers (one per backend)
        self._goal: list = []  # nav goal PointStamped publishers (one per backend)
        self._rpc = None
        self.has_rpc = False
        self._deadman: dict[object, asyncio.TimerHandle] = {}  # per-connection

    @property
    def commands(self) -> list:
        return RPC_COMMANDS if self.has_rpc else []

    # setup: publishers to both backends + the rpc bridge
    def start(self) -> None:
        try:
            from dimos.core.transport_factory import make_transport, rpc_backend
        except ImportError:  # dimos build without the transport factory → read-only gateway
            logger.warning("transport_factory unavailable — teleop/goal/rpc egress disabled")
            return
        from dimos.core.global_config import global_config
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        self._Twist, self._Point, self._Vector3 = Twist, PointStamped, Vector3
        for backend in ("zenoh", "lcm"):
            try:
                g = global_config.model_copy(update={"transport": backend})
                cmd = make_transport("/cmd_vel", Twist, g=g)
                cmd.start()
                goal = make_transport("/clicked_point", PointStamped, g=g)
                goal.start()
                self._cmd.append(cmd)
                self._goal.append(goal)
            except Exception as e:
                logger.warning("egress backend unavailable", backend=backend, error=str(e))
        # RPC bridge on the service's default backend (matches the robot's transport).
        try:
            self._rpc = rpc_backend()()
            self._rpc.start()
            self.has_rpc = True
        except Exception as e:
            logger.warning("rpc bridge unavailable", error=str(e))
        logger.info("egress ready", backends=len(self._cmd), rpc="on" if self.has_rpc else "off")

    def _publish_twist(self, lin: float, ang: float) -> None:
        if not self._cmd:
            return  # no egress backend (e.g. dimos transport unavailable)
        Vector3 = self._Vector3
        msg = self._Twist(
            linear=Vector3(x=_clamp(lin, -MAX_LIN, MAX_LIN), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=_clamp(ang, -MAX_ANG, MAX_ANG)),
        )
        for t in self._cmd:
            try:
                t.publish(msg)
            except Exception:
                pass

    def teleop(self, key: object, lin: float, ang: float, ttl_ms: float, loop) -> None:
        self._publish_twist(lin, ang)
        self._arm_deadman(key, ttl_ms, loop)

    def goal(self, x: float, y: float, z: float) -> None:
        if not self._goal:
            return
        msg = self._Point(x=float(x), y=float(y), z=float(z), ts=time.time(), frame_id="world")
        for t in self._goal:
            try:
                t.publish(msg)
            except Exception:
                pass

    def stop(self, key: object) -> None:
        self._cancel_deadman(key)
        self._publish_twist(0.0, 0.0)

    def disconnect(self, key: object) -> None:
        """Operator connection gone → cancel its deadman and stop the robot."""
        self._cancel_deadman(key)
        self._publish_twist(0.0, 0.0)

    def _arm_deadman(self, key: object, ttl_ms: float, loop) -> None:
        h = self._deadman.get(key)
        if h:
            h.cancel()
        self._deadman[key] = loop.call_later(
            max(0.05, ttl_ms / 1000.0), lambda: self._publish_twist(0.0, 0.0)
        )

    def _cancel_deadman(self, key: object) -> None:
        h = self._deadman.pop(key, None)
        if h:
            h.cancel()

    async def rpc(self, target: str, method: str, args: list, loop) -> dict:
        """Whitelisted dimos @rpc bridge → {"res": ...} or {"error": ...}."""
        if (target, method) not in RPC_WHITELIST or self._rpc is None:
            reason = "rpc unavailable" if self._rpc is None else f"not allowed: {target}/{method}"
            return {"error": reason}
        try:
            res = await loop.run_in_executor(
                None, lambda: self._rpc.call_sync(f"{target}/{method}", (list(args), {}))[0]
            )
            return {"res": _jsonable(res)}
        except Exception as e:
            return {"error": str(e)}
