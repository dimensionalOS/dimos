"""HTTP bridge for driving the robot from an external process.

  - POST /cmd_vel  -> publish a single Twist for `duration` seconds (then 0)
  - POST /path     -> execute a sequence of relative moves in order
  - POST /stop     -> emergency zero-Twist
  - GET  /pose     -> latest base_link pose in the world frame

Compose with any blueprint that has GO2Connection (sim or real robot):

    dimos --simulation run unitree-go2-basic cmd-bridge-module

The "path" is a list of relative steps that are sent as raw cmd_vel
Twists for a per-step duration, then a stop. This is open-loop — no
SLAM, no obstacle avoidance — but works in sim and on the bare-metal
robot without requiring the nav stack. Good fit for a VLM that decides
the next short step from a camera frame and re-plans every iteration.
"""

import asyncio
import math
import queue
import threading
import time
from typing import Any

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
import uvicorn

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CmdBridgeConfig(ModuleConfig):
    port: int = 7782


class TwistRequest(BaseModel):
    linear: list[float] = Field(default=[0.0, 0.0, 0.0], min_length=3, max_length=3)
    angular: list[float] = Field(default=[0.0, 0.0, 0.0], min_length=3, max_length=3)
    duration: float = 0.5  # seconds to hold the command before stopping


class PathStep(BaseModel):
    """One step of an open-loop path.

    `linear`/`angular` are the same Twist components as `/cmd_vel`. The step
    runs for `duration` seconds, then the next step starts. If the model
    prefers semantic steps, use `forward`/`left`/`degrees` and let the bridge
    convert.
    """

    linear: list[float] | None = None
    angular: list[float] | None = None
    # Semantic alternative: distances in meters, rotation in degrees,
    # executed over `duration` seconds. Converted to Twist by dividing.
    forward: float | None = None
    left: float | None = None
    degrees: float | None = None
    duration: float = 1.0


class PathRequest(BaseModel):
    steps: list[PathStep]


class CmdBridgeModule(Module):
    """Open-loop HTTP -> cmd_vel bridge."""

    config: CmdBridgeConfig
    cmd_vel: Out[Twist]
    odom: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._app = FastAPI()
        self._app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["GET", "POST"],
            allow_headers=["*"],
        )
        self._latest_pose: PoseStamped | None = None
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        # Serialize POSTs so /path doesn't interleave with /cmd_vel.
        self._drive_lock = threading.Lock()
        # Cooperative cancel for an in-flight /path when /stop arrives.
        self._cancel_event = threading.Event()
        self._setup_routes()

    def _setup_routes(self) -> None:
        @self._app.get("/pose")
        def pose() -> dict[str, Any]:
            if self._latest_pose is None:
                return {"status": "no pose yet"}
            p = self._latest_pose
            q = p.orientation
            # Yaw from quaternion (assumes near-zero roll/pitch, fine for a quadruped).
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny, cosy)
            return {
                "status": "ok",
                "x": p.position.x,
                "y": p.position.y,
                "z": p.position.z,
                "theta": theta,
                "ts": p.ts,
            }

        @self._app.post("/cmd_vel")
        def cmd_vel(req: TwistRequest) -> dict[str, Any]:
            with self._drive_lock:
                self._cancel_event.clear()
                self._publish_twist(req.linear, req.angular)
                self._sleep_or_cancel(req.duration)
                self._publish_twist([0, 0, 0], [0, 0, 0])
            return {"status": "ok"}

        @self._app.post("/path")
        def path(req: PathRequest) -> dict[str, Any]:
            with self._drive_lock:
                self._cancel_event.clear()
                executed = 0
                for step in req.steps:
                    if self._cancel_event.is_set():
                        break
                    linear, angular = _step_to_twist(step)
                    self._publish_twist(linear, angular)
                    if self._sleep_or_cancel(step.duration):
                        break
                    executed += 1
                self._publish_twist([0, 0, 0], [0, 0, 0])
            return {
                "status": "cancelled" if self._cancel_event.is_set() else "ok",
                "executed": executed,
                "total": len(req.steps),
            }

        @self._app.post("/stop")
        def stop() -> dict[str, str]:
            self._cancel_event.set()
            self._publish_twist([0, 0, 0], [0, 0, 0])
            return {"status": "stopped"}

    def _publish_twist(self, linear: list[float], angular: list[float]) -> None:
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(linear[0], linear[1], linear[2]),
                angular=Vector3(angular[0], angular[1], angular[2]),
                ts=time.time(),
            )
        )

    def _sleep_or_cancel(self, duration: float) -> bool:
        """Sleep up to `duration`, returns True if cancelled mid-sleep."""
        return self._cancel_event.wait(timeout=max(0.0, duration))

    def _on_odom(self, pose: PoseStamped) -> None:
        self._latest_pose = pose

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.odom.subscribe(self._on_odom))

        def run() -> None:
            config = uvicorn.Config(
                self._app,
                host="127.0.0.1",
                port=self.config.port,
                log_level="warning",
                lifespan="on",
            )
            self._server = uvicorn.Server(config)
            try:
                self._server.run()
            except OSError as e:
                logger.error(
                    f"cmd-bridge failed to bind :{self.config.port} ({e}); "
                    f"another instance? `lsof -ti :{self.config.port} | xargs kill -9`"
                )
            except Exception:
                logger.exception("cmd-bridge server crashed")

        self._thread = threading.Thread(target=run, daemon=True, name="cmd-bridge-uvicorn")
        self._thread.start()
        logger.info(
            f"cmd-bridge-module: POST http://127.0.0.1:{self.config.port}/cmd_vel  "
            f"| POST /path | POST /stop | GET /pose"
        )

    @rpc
    def stop(self) -> None:
        self._cancel_event.set()
        self._publish_twist([0, 0, 0], [0, 0, 0])
        if self._server is not None:
            self._server.should_exit = True
        super().stop()


def _step_to_twist(step: PathStep) -> tuple[list[float], list[float]]:
    """Convert a PathStep to (linear, angular) Twist components."""
    if step.linear is not None or step.angular is not None:
        return (
            step.linear or [0.0, 0.0, 0.0],
            step.angular or [0.0, 0.0, 0.0],
        )
    dur = max(step.duration, 1e-3)
    fwd = (step.forward or 0.0) / dur
    lat = (step.left or 0.0) / dur
    yaw = math.radians(step.degrees or 0.0) / dur
    return [fwd, lat, 0.0], [0.0, 0.0, yaw]
