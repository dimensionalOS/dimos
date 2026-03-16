"""Simulated drone connection module.

Minimal interface module that bridges the MuJoCo drone simulation to the
DimOS stream/skill system. Follows the same pattern as G1SimConnection:
the controller lives in dimos/simulation/mujoco/policy.py, and this module
only handles the interface (streams in/out, skills).

The scene XML and mesh assets live under data/mujoco_sim/.
"""

import math
import os
import threading
import time
from typing import Any

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.simulation.mujoco.policy import DroneController
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GPS_ORIGIN_LAT = 37.780967
GPS_ORIGIN_LON = -122.406883
EARTH_R = 6_371_000.0


def _local_to_gps(x: float, y: float) -> tuple[float, float]:
    lat = GPS_ORIGIN_LAT + math.degrees(x / EARTH_R)
    lon = GPS_ORIGIN_LON + math.degrees(
        -y / (EARTH_R * math.cos(math.radians(GPS_ORIGIN_LAT)))
    )
    return lat, lon


class SimulatedDroneConnection(Module):
    """MuJoCo drone simulation with the same stream/skill interface
    as DroneConnectionModule.

    The flight controller (DroneController) lives in
    dimos.simulation.mujoco.policy. This module only handles:
    - Loading the scene from data/mujoco_sim/
    - Publishing sensor streams (odom, gps, video, status, telemetry)
    - Exposing skills (move, takeoff, land, etc.)
    """

    # Inputs (same as DroneConnectionModule)
    movecmd: In[Vector3]
    movecmd_twist: In[Twist]
    gps_goal: In[LatLon]
    tracking_status: In[Any]

    # Outputs (same as DroneConnectionModule)
    odom: Out[PoseStamped]
    gps_location: Out[LatLon]
    status: Out[Any]
    telemetry: Out[Any]
    video: Out[Image]
    follow_object_cmd: Out[Any]

    # Parameters
    render_width: int
    render_height: int

    def __init__(
        self,
        scene: str = "city",
        render_width: int = 640,
        render_height: int = 480,
        camera_name: str = "track",
        headless: bool = False,
    ) -> None:
        self.render_width = render_width
        self.render_height = render_height
        self._scene = scene
        self._camera_name = camera_name
        self._headless = headless

        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._controller: DroneController | None = None
        self._renderer: mujoco.Renderer | None = None

        self._running = False
        self._armed = False
        self._mode = "STABILIZE"
        self._sim_thread: threading.Thread | None = None

        super().__init__()

    @rpc
    def start(self) -> None:
        super().start()

        scene_path = str(get_data("mujoco_sim") / f"{self._scene}_scene.xml")
        logger.info(f"Loading drone scene: {scene_path}")
        self._model = mujoco.MjModel.from_xml_path(scene_path)
        self._data = mujoco.MjData(self._model)
        self._model.opt.timestep = 0.01

        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
        mujoco.mj_forward(self._model, self._data)

        self._controller = DroneController(self._model, self._data)

        if self.movecmd.transport:
            self._disposables.add(self.movecmd.subscribe(self._on_move))
        if self.movecmd_twist.transport:
            self._disposables.add(self.movecmd_twist.subscribe(self._on_move_twist))
        if self.gps_goal.transport:
            self._disposables.add(self.gps_goal.subscribe(self._on_gps_goal))

        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()
        logger.info("Simulated drone connection started")

    def _sim_loop(self) -> None:
        assert self._model and self._data and self._controller

        if not self._headless:
            self._renderer = mujoco.Renderer(
                self._model, self.render_height, self.render_width
            )

        step_count = 0
        steps_per_publish = max(1, int(1.0 / self._model.opt.timestep / 30))
        steps_per_render = max(1, int(1.0 / self._model.opt.timestep / 15))

        while self._running:
            if self._armed and self._mode == "GUIDED":
                self._data.ctrl[:] = self._controller.compute_control()
            elif self._armed:
                self._data.ctrl[:] = self._controller.hover_thrust
            else:
                self._data.ctrl[:] = 0.0

            mujoco.mj_step(self._model, self._data)
            step_count += 1

            if step_count % steps_per_publish == 0:
                self._publish_state()
            if not self._headless and step_count % steps_per_render == 0:
                self._publish_camera()

            # Real-time pacing
            sim_t = self._data.time
            wall_t = time.monotonic()
            if not hasattr(self, "_wall0"):
                self._wall0 = wall_t
                self._sim0 = sim_t
            dt = (self._wall0 + (sim_t - self._sim0)) - wall_t
            if dt > 0.001:
                time.sleep(dt)

    # -- state publishing --

    def _publish_state(self) -> None:
        m, d = self._model, self._data
        bid = self._controller.body_id
        pos = d.xpos[bid].copy()
        q = d.xquat[bid].copy()
        vel = d.cvel[bid]
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        now = time.time()

        self.odom.publish(
            PoseStamped(
                position=Vector3(x, y, z),
                orientation=Quaternion(float(q[1]), float(q[2]), float(q[3]), float(q[0])),
                frame_id="world",
                ts=now,
            )
        )

        lat, lon = _local_to_gps(x, y)
        self.gps_location.publish(LatLon(lat=lat, lon=lon))

        siny = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        heading = math.degrees(math.atan2(siny, cosy)) % 360

        self.status.publish({
            "armed": self._armed, "mode": self._mode,
            "altitude": z, "heading": heading,
            "vx": float(vel[3]), "vy": float(vel[4]), "vz": float(vel[5]),
            "lat": lat, "lon": lon, "ts": now, "simulator": "mujoco",
        })

    def _publish_camera(self) -> None:
        if not self._renderer or not self._model or not self._data:
            return
        self._renderer.update_scene(self._data, camera=self._camera_name)
        pixels = self._renderer.render()
        self.video.publish(Image(
            data=pixels.tobytes(),
            width=self.render_width,
            height=self.render_height,
            encoding="rgb8",
            step=self.render_width * 3,
        ))

    # -- input handlers --

    def _on_move(self, v: Vector3) -> None:
        if self._controller:
            self._controller.set_velocity(float(v.x), float(v.y), float(v.z), 0.0)

    def _on_move_twist(self, msg: Twist) -> None:
        if self._controller:
            self._controller.set_velocity(
                float(msg.linear.x), float(msg.linear.y),
                float(msg.linear.z), float(msg.angular.z),
            )

    def _on_gps_goal(self, cmd: LatLon) -> None:
        if not self._data or not self._controller:
            return
        bid = self._controller.body_id
        pos = self._data.xpos[bid]
        dx = math.radians(cmd.lat - GPS_ORIGIN_LAT) * EARTH_R
        dy = -(math.radians(cmd.lon - GPS_ORIGIN_LON) * EARTH_R * math.cos(math.radians(GPS_ORIGIN_LAT)))
        ex, ey = dx - pos[0], dy - pos[1]
        dist = math.sqrt(ex**2 + ey**2)
        if dist > 0.5:
            s = min(2.0, dist)
            self._controller.set_velocity(s * ex / dist, s * ey / dist, 0.0, 0.0)
        else:
            self._controller.set_velocity(0.0, 0.0, 0.0, 0.0)

    # -- skills --

    @skill
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, duration: float = 0.0) -> str:
        """Send velocity command. x=forward, y=right, z=up (m/s)."""
        if self._controller:
            self._controller.set_velocity(x, y, z, 0.0)
            if duration > 0:
                threading.Timer(duration, lambda: self._controller.set_velocity(0, 0, 0, 0)).start()
            return f"Moving: vx={x}, vy={y}, vz={z} for {duration}s"
        return "Simulation not running"

    @skill
    def move_with_yaw(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                      yaw_rate: float = 0.0, duration: float = 2.0) -> str:
        """Move with velocity and yaw. Positive yaw_rate = turn right."""
        if self._controller:
            self._controller.set_velocity(vx, vy, vz, yaw_rate)
            if duration > 0:
                threading.Timer(duration, lambda: self._controller.set_velocity(0, 0, 0, 0)).start()
            return f"Moving: vx={vx}, vy={vy}, vz={vz}, yaw={yaw_rate} for {duration}s"
        return "Simulation not running"

    @skill
    def takeoff(self, altitude: float = 3.0) -> str:
        """Arm and takeoff to altitude."""
        self._armed = True
        self._mode = "GUIDED"
        if self._controller:
            self._controller.set_velocity(0, 0, 1.0, 0)
            def _check() -> None:
                if self._data is not None:
                    bid = self._controller.body_id
                    if self._data.xpos[bid][2] >= altitude * 0.9:
                        self._controller.set_velocity(0, 0, 0, 0)
                        return
                if self._running:
                    threading.Timer(0.2, _check).start()
            threading.Timer(0.5, _check).start()
        return f"Taking off to {altitude}m"

    @skill
    def land(self) -> str:
        """Land the drone."""
        self._mode = "LAND"
        if self._controller:
            self._controller.set_velocity(0, 0, -0.5, 0)
            def _check() -> None:
                if self._data is not None:
                    bid = self._controller.body_id
                    if self._data.xpos[bid][2] < 0.15:
                        self._controller.set_velocity(0, 0, 0, 0)
                        self._armed = False
                        return
                if self._running:
                    threading.Timer(0.2, _check).start()
            threading.Timer(0.5, _check).start()
        return "Landing"

    @skill
    def arm(self) -> str:
        self._armed = True
        return "Armed"

    @skill
    def disarm(self) -> str:
        self._armed = False
        if self._controller:
            self._controller.set_velocity(0, 0, 0, 0)
        return "Disarmed"

    @skill
    def set_mode(self, mode: str) -> str:
        self._mode = mode.upper()
        return f"Mode set to {self._mode}"

    @skill
    def fly_to(self, lat: float, lon: float, alt: float) -> str:
        self._on_gps_goal(LatLon(lat=lat, lon=lon))
        return f"Flying to {lat:.6f}, {lon:.6f} at {alt}m"

    @skill
    def observe(self) -> Image | None:
        if not self._renderer or not self._model or not self._data:
            return None
        self._renderer.update_scene(self._data, camera=self._camera_name)
        pixels = self._renderer.render()
        return Image(
            data=pixels.tobytes(),
            width=self.render_width,
            height=self.render_height,
            encoding="rgb8",
            step=self.render_width * 3,
        )

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=3.0)
        if self._renderer:
            self._renderer.close()
        logger.info("Simulated drone connection stopped")
        super().stop()
