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

"""Go2 safe-patrol: scan for nearby person, then step back/forward/left/right.

On start, after the robot stands up and detections stabilize, checks whether
any person (COCO class 0) is detected with at least one LIDAR return within
`safety_radius` of the base link. If clear, walks `step_distance` in each of
four directions using odometry as the stop criterion.

Run:
    python -m dimos.robot.unitree.go2.blueprints.smart.unitree_go2_patrol --ip <ROBOT_IP>
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PERSON_CLASS_ID = 0  # COCO


class PatrolConfig(ModuleConfig):
    safety_radius: float = Field(default=1.0)  # meters
    step_distance: float = Field(default=0.5)  # meters per move
    move_speed: float = Field(default=0.3)  # m/s
    startup_delay: float = Field(default=4.0)  # let standup settle
    detection_warmup: float = Field(default=2.0)  # collect detections before deciding
    move_timeout_factor: float = Field(default=4.0)  # safety vs. expected duration
    z_floor: float = Field(default=-0.1)  # ignore points below (ground)
    z_ceiling: float = Field(default=1.8)  # ignore points above (ceiling)
    publish_hz: float = Field(default=20.0)


class PatrolModule(Module):
    """Single-shot safety-checked patrol sequence."""

    config: PatrolConfig

    detections: In[Detection2DArray]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    cmd_vel: Out[Twist]

    _latest_detections: Detection2DArray | None = None
    _latest_lidar: PointCloud2 | None = None
    _latest_pose: PoseStamped | None = None
    _patrol_thread: threading.Thread | None = None
    _stop_event: threading.Event

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            self.detections.observable().subscribe(self._on_detections)
        )
        self.register_disposable(self.lidar.observable().subscribe(self._on_lidar))
        self.register_disposable(self.odom.observable().subscribe(self._on_odom))

        self._patrol_thread = threading.Thread(
            target=self._run_patrol, name="go2-patrol", daemon=True
        )
        self._patrol_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._send_twist(0.0, 0.0)
        if self._patrol_thread and self._patrol_thread.is_alive():
            self._patrol_thread.join(timeout=2.0)
        super().stop()

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._latest_detections = msg

    def _on_lidar(self, msg: PointCloud2) -> None:
        self._latest_lidar = msg

    def _on_odom(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _person_detected(self) -> bool:
        msg = self._latest_detections
        if msg is None or not msg.detections:
            return False
        for det in msg.detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            # LCM declares class_id as string, but YOLO converter may pass int.
            if cls == PERSON_CLASS_ID or str(cls) == str(PERSON_CLASS_ID):
                return True
        return False

    def _lidar_point_within(self, radius: float) -> bool:
        pc = self._latest_lidar
        if pc is None or len(pc) == 0:
            return False
        try:
            pts = np.asarray(pc.pointcloud.points, dtype=np.float32)
        except Exception as e:
            logger.warning("Could not read pointcloud: %s", e)
            return False
        if pts.size == 0:
            return False
        z = pts[:, 2]
        mask = (z > self.config.z_floor) & (z < self.config.z_ceiling)
        xy = pts[mask, :2]
        if xy.size == 0:
            return False
        d2 = xy[:, 0] ** 2 + xy[:, 1] ** 2
        return bool(np.any(d2 < radius * radius))

    def _person_within_radius(self) -> bool:
        return self._person_detected() and self._lidar_point_within(
            self.config.safety_radius
        )

    def _send_twist(self, vx: float, vy: float) -> None:
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(vx, vy, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            )
        )

    def _move_by(self, vx: float, vy: float, distance: float, label: str) -> None:
        if self._latest_pose is None:
            logger.warning("Patrol: no odometry yet, skipping %s", label)
            return

        start_x = self._latest_pose.x
        start_y = self._latest_pose.y
        nominal_speed = max(abs(vx), abs(vy)) or self.config.move_speed
        timeout = (distance / nominal_speed) * self.config.move_timeout_factor
        period = 1.0 / max(self.config.publish_hz, 1.0)

        logger.info("Patrol: moving %s (~%.2fm)", label, distance)
        t0 = time.time()
        while not self._stop_event.is_set():
            if time.time() - t0 > timeout:
                logger.warning("Patrol: %s timed out after %.1fs", label, timeout)
                break
            pose = self._latest_pose
            if pose is None:
                break
            traveled = math.hypot(pose.x - start_x, pose.y - start_y)
            if traveled >= distance:
                logger.info("Patrol: %s reached %.2fm", label, traveled)
                break
            # Safety: abort if a person walks into range mid-motion.
            if self._person_within_radius():
                logger.warning("Patrol: person entered safety zone, aborting %s", label)
                break
            self._send_twist(vx, vy)
            time.sleep(period)

        self._send_twist(0.0, 0.0)
        time.sleep(0.3)  # let the robot settle before the next leg

    def _run_patrol(self) -> None:
        try:
            logger.info("Patrol: waiting %.1fs for standup", self.config.startup_delay)
            if self._stop_event.wait(self.config.startup_delay):
                return

            logger.info(
                "Patrol: warming up detections for %.1fs", self.config.detection_warmup
            )
            if self._stop_event.wait(self.config.detection_warmup):
                return

            if self._person_within_radius():
                logger.warning(
                    "Patrol: person within %.1fm — aborting", self.config.safety_radius
                )
                return

            logger.info("Patrol: clear, starting 4-direction sequence")
            speed = self.config.move_speed
            dist = self.config.step_distance
            # ROS REP-103: +x forward, +y left
            sequence = [
                ("backward", -speed, 0.0),
                ("forward", speed, 0.0),
                ("left", 0.0, speed),
                ("right", 0.0, -speed),
            ]
            for label, vx, vy in sequence:
                if self._stop_event.is_set():
                    break
                self._move_by(vx, vy, dist, label)

            logger.info("Patrol: complete")
        except Exception:
            logger.exception("Patrol: unexpected error, stopping robot")
            self._send_twist(0.0, 0.0)


# Compose: basic (GO2Connection + vis) + Detection3DModule + PatrolModule.
# Detection3DModule's `pointcloud` input is remapped to the raw `lidar` stream
# (basic blueprint has no global_map / VoxelGridMapper).
unitree_go2_patrol = (
    autoconnect(
        unitree_go2_basic,
        Detection3DModule.blueprint(camera_info=GO2Connection.camera_info_static),
        PatrolModule.blueprint(),
    )
    .remappings(
        [
            (Detection3DModule, "pointcloud", "lidar"),
        ]
    )
    .global_config(n_workers=4, robot_model="unitree_go2")
)


__all__ = ["PatrolModule", "PatrolConfig", "unitree_go2_patrol"]


if __name__ == "__main__":
    import argparse
    import os
    import sys
    from pathlib import Path

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    parser = argparse.ArgumentParser(description="Go2 safe-patrol blueprint runner")
    parser.add_argument("--ip", required=True, help="Robot IP (or 'mujoco' / 'replay')")
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--step-distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.3)
    args = parser.parse_args()

    # MuJoCo on macOS spawns a subprocess that needs `mjpython` on PATH.
    if args.ip in ("mujoco", "replay", "fake", "mock"):
        venv_bin = str(Path(sys.executable).parent)
        if venv_bin not in os.environ.get("PATH", "").split(os.pathsep):
            os.environ["PATH"] = venv_bin + os.pathsep + os.environ.get("PATH", "")

    kwargs: dict[str, Any] = {
        "g": {"robot_ip": args.ip},
        "patrolmodule": {
            "safety_radius": args.safety_radius,
            "step_distance": args.step_distance,
            "move_speed": args.speed,
        },
    }

    coordinator = ModuleCoordinator.build(unitree_go2_patrol, kwargs)
    coordinator.start_rpc_service()
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.stop()
