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

from __future__ import annotations  # noqa: I001

import threading
import time
from typing import TYPE_CHECKING, Any, Literal
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT

import cv2
import numpy as np
from dimos_lcm.std_msgs import String, Bool
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.experimental.security_demo.depth_estimator import DepthEstimator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.type.detection2d.person import Detection2DPerson
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.routers.patrol_router import PatrolRouter
from dimos.agents.skills.speak_skill_spec import SpeakSkillSpec
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D
from dimos.perception.common.utils import draw_bounding_box
from dimos.utils.logging_config import setup_logger
from dimos.navigation.patrolling.constants import EXTRA_CLEARANCE

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

logger = setup_logger()

# COCO skeleton connections for drawing
_SKELETON_CONNECTIONS = [
    (0, 1),
    (0, 2),
    (1, 3),
    (2, 4),  # face
    (5, 6),  # shoulders
    (5, 7),
    (7, 9),  # left arm
    (6, 8),
    (8, 10),  # right arm
    (5, 11),
    (6, 12),
    (11, 12),  # torso
    (11, 13),
    (13, 15),  # left leg
    (12, 14),
    (14, 16),  # right leg
]

_KP_CONF_THRESHOLD = 0.3
_ANTI_BUSY_LOOP_TIMEOUT = 0.01


def _draw_skeleton(
    image: np.ndarray,
    person: Detection2DPerson,
    joint_color: tuple[int, int, int] = (0, 255, 0),
    bone_color: tuple[int, int, int] = (255, 255, 0),
) -> None:
    """Draw pose skeleton directly on *image* (in-place, BGR assumed)."""
    kps = person.keypoints  # (17, 2)
    scores = person.keypoint_scores  # (17,)

    # Draw bones first so joints are drawn on top
    for i, j in _SKELETON_CONNECTIONS:
        if scores[i] > _KP_CONF_THRESHOLD and scores[j] > _KP_CONF_THRESHOLD:
            pt1 = (int(kps[i][0]), int(kps[i][1]))
            pt2 = (int(kps[j][0]), int(kps[j][1]))
            cv2.line(image, pt1, pt2, bone_color, 2, cv2.LINE_AA)

    # Draw joints
    for idx in range(len(kps)):
        if scores[idx] > _KP_CONF_THRESHOLD:
            cx, cy = int(kps[idx][0]), int(kps[idx][1])
            cv2.circle(image, (cx, cy), 4, joint_color, -1, cv2.LINE_AA)


State = Literal["IDLE", "PATROLLING", "FOLLOWING"]


class SecurityModuleConfig(ModuleConfig):
    camera_info: CameraInfo
    follow_frequency: float = 20.0


def _create_router(global_config: GlobalConfig) -> PatrolRouter:
    clearance_multiplier = 0.5
    clearance_radius_m: float = global_config.robot_width * clearance_multiplier
    return create_patrol_router("coverage", clearance_radius_m)


def _create_visual_servo(
    config: SecurityModuleConfig, global_config: GlobalConfig
) -> VisualServoing2D:
    camera_info = config.camera_info
    if global_config.simulation:
        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        camera_info = MujocoConnection.camera_info_static

    return VisualServoing2D(camera_info, bool(global_config.simulation))


class SecurityModule(Module):
    """Integrated security patrol module.

    Manages the full patrol-detect-follow state machine internally,
    eliminating agent round-trips between separate modules.
    """

    config: SecurityModuleConfig

    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    goal_reached: In[Bool]
    color_image: In[Image]
    depth_image: Out[Image]
    detection: Out[Image]
    tracking_image: Out[Image]
    security_state: Out[String]

    goal_request: Out[PoseStamped]
    cmd_vel: Out[Twist]

    _planner_spec: ReplanningAStarPlannerSpec
    _speak_skill: SpeakSkillSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._router: PatrolRouter = _create_router(self.config.g)
        self._visual_servo = _create_visual_servo(self.config, self.config.g)
        self._detector = YoloPersonDetector()
        self._tracker = EdgeTAMProcessor()

        self._depth_estimator = DepthEstimator(self.depth_image.publish)

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._goal_reached_event = threading.Event()
        self._main_thread: threading.Thread | None = None
        self._state: State = "IDLE"
        self._latest_pose: PoseStamped | None = None
        self._latest_image: Image | None = None
        self._has_active_goal = False
        self._depth_started = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(
            Disposable(self.global_costmap.subscribe(self._router.handle_occupancy_grid))
        )
        self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))

    @rpc
    def stop(self) -> None:
        self._stop_security_patrol_internal()
        self._depth_estimator.stop()
        self._detector.stop()
        self._tracker.stop()
        super().stop()

    @skill
    def start_security_patrol(self) -> str:
        """
        Start the security patrol behavior. The robot will patrol, detect
        persons visually and then follow them automatically.
        """
        with self._lock:
            if self._main_thread is not None and self._main_thread.is_alive():
                return "Security patrol is already running. Use `stop_security_patrol` to stop."

        if not self._depth_started:
            self._depth_estimator.start()
            self._depth_started = True

        self._router.reset()

        self._planner_spec.set_replanning_enabled(False)
        self._planner_spec.set_safe_goal_clearance(
            self.config.g.robot_rotation_diameter / 2 + EXTRA_CLEARANCE
        )

        self._stop_event.clear()
        self._has_active_goal = False

        self._main_thread = threading.Thread(
            target=self._main_loop, daemon=True, name=f"{self.__class__.__name__}-main"
        )
        self._main_thread.start()

        return (
            "Security patrol started. The robot will patrol, detect, and follow "
            "persons automatically. Use `stop_security_patrol` to stop."
        )

    @skill
    def stop_security_patrol(self) -> str:
        """Stop the security patrol behavior entirely."""
        self._stop_security_patrol_internal()
        return "Security patrol stopped."

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
        self._router.handle_odom(msg)

    def _on_goal_reached(self, _msg: Bool) -> None:
        self._goal_reached_event.set()

    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image
        self._depth_estimator.submit(image)

    def _main_loop(self) -> None:
        self._transition_to("PATROLLING")

        while not self._stop_event.is_set():
            with self._lock:
                state: State = self._state
            match state:
                case "PATROLLING":
                    self._patrol_step()
                case "FOLLOWING":
                    self._follow_step()

        self.cmd_vel.publish(Twist.zero())
        self._transition_to("IDLE")

    def _patrol_step(self) -> None:
        """Send patrol goals and run detection in a single non-blocking step."""
        if not self._has_active_goal:
            goal = self._router.next_goal()
            if goal is None:
                # TODO: Fix this
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                # #########################################################################################
                logger.info("no patrol goal available, retrying in 2s")
                self._stop_event.wait(timeout=2.0)
                return
            self._goal_reached_event.clear()
            self.goal_request.publish(goal)
            self._has_active_goal = True

        if self._goal_reached_event.is_set():
            self._goal_reached_event.clear()
            self._has_active_goal = False

        with self._lock:
            image = self._latest_image
        if image is None:
            self._stop_event.wait(timeout=_ANTI_BUSY_LOOP_TIMEOUT)
            return

        best = self._find_best_person(image)
        if best is None:
            return

        logger.info(
            "Detection",
            best_bbox=best.bbox,
            confidence=f"{best.confidence:.2f}",
            area=f"{best.bbox_2d_volume():.0f}px",
        )

        annotated = draw_bounding_box(
            image.data.copy(),
            list(best.bbox),
            label=best.name,
            confidence=best.confidence,
        )
        if isinstance(best, Detection2DPerson):
            _draw_skeleton(annotated, best)
        self.detection.publish(Image.from_numpy(annotated, format=image.format))

        # Init EdgeTAM with YOLO bbox for continuous tracking
        box = np.array(list(best.bbox), dtype=np.float32)
        self._tracker.init_track(image=image, box=box, obj_id=1)

        self._cancel_current_goal()
        self._has_active_goal = False
        self._speak_skill.speak("Intruder detected", blocking=False)
        self._transition_to("FOLLOWING")

    def _follow_step(self) -> None:
        """One iteration of the follow loop (EdgeTAM track + servo + publish)."""
        with self._lock:
            latest_image = self._latest_image

        if latest_image is None:
            self._stop_event.wait(timeout=_ANTI_BUSY_LOOP_TIMEOUT)
            return

        detections = self._tracker.process_image(latest_image)

        if len(detections) == 0:
            self.cmd_vel.publish(Twist.zero())
            self._speak_skill.speak("Lost sight of intruder, resuming patrol", blocking=False)
            self._router.reset()
            self._has_active_goal = False
            self._transition_to("PATROLLING")
            return

        best = max(detections.detections, key=lambda d: d.bbox_2d_volume())
        twist = self._visual_servo.compute_twist(best.bbox, latest_image.width)
        self.cmd_vel.publish(twist)

        overlay = latest_image.data.copy()
        if hasattr(best, "mask") and best.mask is not None:
            mask_bool = best.mask > 0
            green = np.zeros_like(overlay)
            green[:, :] = (0, 255, 0)
            overlay[mask_bool] = cv2.addWeighted(overlay[mask_bool], 0.6, green[mask_bool], 0.4, 0)

        # Run pose estimation on the tracked frame and draw skeleton
        pose_detections = self._detector.process_image(latest_image)
        persons = [
            d
            for d in pose_detections.detections
            if isinstance(d, Detection2DPerson) and d.is_valid()
        ]
        for person in persons:
            _draw_skeleton(overlay, person)

        self.tracking_image.publish(
            Image.from_numpy(overlay, format=latest_image.format, ts=latest_image.ts)
        )

        time.sleep(1.0 / self.config.follow_frequency)

    def _find_best_person(self, image: Image) -> Detection2DBBox | None:
        """Run YOLO and return the largest person detection, or None."""
        all_detections = self._detector.process_image(image)
        persons = [d for d in all_detections.detections if d.name == "person"]
        if not persons:
            return None
        return max(persons, key=lambda d: d.bbox_2d_volume())

    def _cancel_current_goal(self) -> None:
        """Publish current pose as goal to cancel in-progress navigation."""
        with self._lock:
            pose = self._latest_pose
        if pose is not None:
            self.goal_request.publish(pose)

    def _transition_to(self, new_state: State) -> None:
        with self._lock:
            old = self._state
            self._state = new_state

        logger.info("state transition", old=old, new=new_state)
        self.security_state.publish(String(new_state))

    def _stop_security_patrol_internal(self) -> None:
        self._stop_event.set()

        self._planner_spec.set_replanning_enabled(True)
        self._planner_spec.reset_safe_goal_clearance()

        self._cancel_current_goal()

        with self._lock:
            thread = self._main_thread
        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            with self._lock:
                self._main_thread = None
