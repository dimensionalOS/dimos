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

import asyncio
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.memory2.location_spec import LocationMemorySpec
from dimos.memory2.locations import (
    FRAME_WORLD,
    LocationError,
    MapMismatchError,
    NotRelocalizedError,
    RunLocalError,
    StaleAnchorError,
    normalize_name,
)
from dimos.models.qwen.bbox import BBox
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.perception.object_tracking_spec import ObjectTrackingSpec
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _normalized(name: str) -> str:
    try:
        return normalize_name(name)
    except ValueError:
        return ""


def _planar_distance(a: PoseStamped, b: PoseStamped) -> float:
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


def _explain(query: str, error: LocationError) -> str:
    """Turn a typed resolve failure into something the user can act on.

    Each of these is a distinct situation with a distinct remedy, so they get
    distinct wording — collapsing them into "couldn't find it" would hide the
    difference between "wait a moment" and "that will never work again".
    """
    if isinstance(error, NotRelocalizedError):
        return (
            f"I know where '{query}' is, but I haven't recognised where I am on the map "
            f"yet. Give me a moment to look around, or drive me somewhere more open."
        )
    if isinstance(error, StaleAnchorError):
        return (
            f"I'm no longer confident where I am, so I can't reliably navigate to "
            f"'{query}' right now."
        )
    if isinstance(error, RunLocalError):
        return (
            f"I only saved '{query}' for the session I tagged it in, and I've been "
            f"restarted since. Take me there and I'll tag it properly this time."
        )
    if isinstance(error, MapMismatchError):
        return (
            f"'{query}' was saved against a different map of this place, so its "
            f"position doesn't apply to the map I'm using now."
        )
    return f"Cannot navigate to '{query}': {error}"


class NavigationSkillContainer(Module):
    _latest_image: Image | None = None
    _latest_odom: PoseStamped | None = None
    _skill_started: bool = False
    _similarity_threshold: float = 0.23

    _reissue_threshold_m: float = 0.35
    _reissue_poll_s: float = 1.0

    _spatial_memory: SpatialMemorySpec
    _navigation: NavigationInterfaceSpec
    _object_tracking: ObjectTrackingSpec | None = None
    _locations: LocationMemorySpec | None = None

    color_image: In[Image]
    odom: In[PoseStamped]
    goal_reached: In[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._skill_started = False
        self._goal_reached_event = asyncio.Event()
        self._nav_cancelled = threading.Event()
        self._nav_future: Any = None

        # Here to prevent unwanted imports in the file.
        from dimos.models.vl.qwen import QwenVlModel

        self._vl_model = QwenVlModel()

    async def handle_goal_reached(self, _msg: Bool) -> None:
        self._goal_reached_event.set()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self._skill_started = True

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    @skill
    def tag_location(self, location_name: str) -> str:
        """Tag this location in the spatial memory with a name.

        This associates the current location with the given name in the spatial memory, allowing you to navigate back to it.

        Args:
            location_name (str): the name for the location

        Returns:
            str: the outcome
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        if self._locations is not None:
            return self._tag_durable(location_name)

        if not self._latest_odom:
            return "No odometry data received yet, cannot tag location."

        position = self._latest_odom.position
        rotation = self._latest_odom.orientation.to_euler()

        location = RobotLocation(
            name=location_name,
            position=(position.x, position.y, position.z),
            rotation=(rotation.x, rotation.y, rotation.z),
        )

        if not self._spatial_memory.tag_location(location):
            return f"Error: Failed to store '{location_name}' in the spatial memory"

        logger.info(f"Tagged {location}")
        return f"Tagged '{location_name}': ({position.x},{position.y})."

    # TODO(capabilities): this skill is `instant`, so the `movement` hold is
    # released the moment the call returns even though the tagged-location and
    # semantic-map paths only fire set_goal() and keep navigating. Make it
    # `background` and close the hold when the robot actually stops -- the
    # planner already emits a goal-reached signal (see PatrollingModule) -- so
    # patrol/follow/explore can't start over an active navigation goal.
    @skill(uses=[CAP_MOVEMENT])
    def navigate_with_text(self, query: str) -> str:
        """Navigate to a location by querying the existing semantic map using natural language.

        First attempts to locate an object in the robot's camera view using vision.
        If the object is found, navigates to it. If not, falls back to querying the
        semantic map for a location matching the description.
        CALL THIS SKILL FOR ONE SUBJECT AT A TIME. For example: "Go to the person wearing a blue shirt in the living room",
        you should call this skill twice, once for the person wearing a blue shirt and once for the living room.
        Args:
            query: Text query to search for in the semantic map
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")
        success_msg = self._navigate_by_tagged_location(query)
        if success_msg:
            return success_msg

        logger.info(f"No tagged location found for {query}")

        success_msg = self._navigate_to_object(query)
        if success_msg:
            return success_msg

        logger.info(f"No object in view found for {query}")

        success_msg = self._navigate_using_semantic_map(query)
        if success_msg:
            return success_msg

        return f"No tagged location called '{query}'. No object in view matching '{query}'. No matching location found in semantic map for '{query}'."

    def _tag_durable(self, location_name: str) -> str:
        """Save through LocationMemory, and say plainly whether it will survive a restart."""
        assert self._locations is not None
        try:
            loc = self._locations.save_location(location_name)
        except LookupError as e:
            return f"Cannot tag '{location_name}': {e}"

        x, y = loc.position[0], loc.position[1]
        if loc.anchored:
            return (
                f"Tagged '{location_name}' at ({x:.1f}, {y:.1f}). "
                f"I'll still know where it is after a restart."
            )
        return (
            f"Tagged '{location_name}' at ({x:.1f}, {y:.1f}) — for this session only, "
            f"because I haven't recognised where I am on the map yet. "
            f"I'll save it permanently as soon as I do."
        )

    def _navigate_by_tagged_location(self, query: str) -> str | None:
        if self._locations is not None:
            return self._navigate_by_durable_location(query)

        robot_location = self._spatial_memory.query_tagged_location(query)

        if not robot_location:
            return None

        logger.info("Found tagged location", location=robot_location)
        goal_pose = PoseStamped(
            position=make_vector3(*robot_location.position),
            orientation=Quaternion.from_euler(Vector3(*robot_location.rotation)),
            frame_id="map",
        )

        return self._navigate_to(goal_pose, f"Found a tagged location called '{query}'.")

    def _navigate_by_durable_location(self, query: str) -> str | None:
        """Resolve a saved location into the costmap frame and start navigating.

        Returns ``None`` only when there is genuinely no such location, so
        ``navigate_with_text`` falls through to its object/semantic-map attempts.
        Every other failure is reported as itself — driving to a pose we could not
        actually resolve is worse than admitting we cannot.
        """
        assert self._locations is not None
        try:
            goal_pose = self._locations.resolve_location(query, FRAME_WORLD)
        except LocationError as e:
            known = {loc.name for loc in self._locations.list_locations()}
            if _normalized(query) not in known:
                return None
            return _explain(query, e)

        self._start_location_navigation(query, goal_pose)
        return (
            f"Found a tagged location called '{query}'. Started navigating there. "
            f"To cancel movement call the 'stop_navigation' tool."
        )

    def _navigate_to(self, pose: PoseStamped, message: str) -> str:
        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        self._navigation.set_goal(pose)

        return (
            f"{message}. Started navigating to that position. "
            f"To cancel movement call the 'stop_navigation' tool."
        )

    def _navigate_to_object(self, query: str) -> str | None:
        if self._object_tracking is None:
            return None

        try:
            bbox = self._get_bbox_for_current_frame(query)
        except Exception:
            logger.error(f"Failed to get bbox for {query}", exc_info=True)
            return None

        if bbox is None:
            return None

        logger.info(f"Found {query} at {bbox}")

        # Start tracking - BBoxNavigationModule automatically generates goals
        self._object_tracking.track(bbox)  # type: ignore[arg-type]

        start_time = time.time()
        timeout = 30.0
        goal_set = False

        while time.time() - start_time < timeout:
            # Check if navigator finished
            if self._navigation.get_state() == NavigationState.IDLE and goal_set:
                logger.info("Waiting for goal result")
                time.sleep(1.0)
                if not self._navigation.is_goal_reached():
                    logger.info(f"Goal cancelled, tracking '{query}' failed")
                    self._object_tracking.stop_track()
                    return None
                else:
                    logger.info(f"Reached '{query}'")
                    self._object_tracking.stop_track()
                    return f"Successfully arrived at '{query}'"

            # If goal set and tracking lost, just continue (tracker will resume or timeout)
            if goal_set and not self._object_tracking.is_tracking():
                continue

            # BBoxNavigationModule automatically sends goals when tracker publishes
            # Just check if we have any detections to mark goal_set
            if self._object_tracking.is_tracking():
                goal_set = True

            time.sleep(0.25)

        logger.warning(f"Navigation to '{query}' timed out after {timeout}s")
        self._object_tracking.stop_track()
        return None

    def _get_bbox_for_current_frame(self, query: str) -> BBox | None:
        if self._latest_image is None:
            return None

        return get_object_bbox_from_image(self._vl_model, self._latest_image, query)

    def _navigate_using_semantic_map(self, query: str) -> str:
        results = self._spatial_memory.query_by_text(query)

        if not results:
            return f"No matching location found in semantic map for '{query}'"

        best_match = results[0]

        goal_pose = self._get_goal_pose_from_result(best_match)

        logger.info("Goal pose for semantic nav", pose=goal_pose)
        if not goal_pose:
            return f"Found a result for '{query}' but it didn't have a valid position."

        message = f"Found a location in the semantic map matching '{query}'."
        return self._navigate_to(goal_pose, message)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def navigate_to_location(self, location_name: str) -> str:
        """Navigate to a place previously remembered with `tag_location`.

        Unlike `navigate_with_text` this only considers named locations, and it keeps
        the goal correct if the robot revises its estimate of where it is partway there.

        Args:
            location_name (str): the name the place was tagged with, e.g. "kitchen"
        """
        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")
        if self._locations is None:
            return "Durable saved locations are not available in this configuration."

        self.start_tool("navigate_to_location")
        background_launched = False
        try:
            try:
                goal_pose = self._locations.resolve_location(location_name, FRAME_WORLD)
            except LocationError as e:
                known = {loc.name for loc in self._locations.list_locations()}
                if _normalized(location_name) not in known:
                    return f"I don't know anywhere called '{location_name}'."
                return _explain(location_name, e)

            self._start_location_navigation(location_name, goal_pose, hold_tool=True)
            background_launched = True
            return (
                f"Navigating to '{location_name}'. "
                f"To cancel movement call the 'stop_navigation' tool."
            )
        finally:
            if not background_launched:
                self.stop_tool("navigate_to_location")

    def _start_location_navigation(
        self, name: str, goal_pose: PoseStamped, *, hold_tool: bool = False
    ) -> None:
        self._cancel_reissue()
        self._nav_cancelled.clear()
        self._goal_reached_event.clear()
        self._navigation.set_goal(goal_pose)
        self._nav_future = self.spawn(self._hold_location_goal(name, goal_pose, hold_tool))

    async def _hold_location_goal(self, name: str, issued: PoseStamped, hold_tool: bool) -> None:
        """Keep a saved-location goal correct while relocalization is still moving.

        The planner ignores ``goal.frame_id`` and consumes coordinates raw against the
        costmap, so a goal resolved into ``world`` silently becomes wrong the instant
        ``world -> map`` is corrected. Nothing downstream can notice that, so we
        re-resolve and re-issue here.
        """
        assert self._locations is not None
        try:
            while not self._nav_cancelled.is_set():
                try:
                    await asyncio.wait_for(
                        self._goal_reached_event.wait(), timeout=self._reissue_poll_s
                    )
                    logger.info("Arrived at tagged location", name=name)
                    return
                except (TimeoutError, asyncio.TimeoutError):
                    pass

                if self._nav_cancelled.is_set():
                    return

                try:
                    latest = self._locations.resolve_location(name, FRAME_WORLD)
                except LocationError as e:
                    logger.warning("Anchor unavailable while navigating", name=name, reason=str(e))
                    continue

                drift = _planar_distance(latest, issued)
                if drift < self._reissue_threshold_m:
                    continue

                logger.info(
                    "Relocalization moved the goal, re-issuing",
                    name=name,
                    drift_m=round(drift, 2),
                )
                issued = latest
                self._goal_reached_event.clear()
                self._navigation.set_goal(latest)
        finally:
            if hold_tool:
                self.stop_tool("navigate_to_location")

    def _cancel_reissue(self) -> None:
        """Stop the re-issue loop. A cancelled goal must stay cancelled — without
        this, the next relocalization jump would resurrect it."""
        self._nav_cancelled.set()
        future = self._nav_future
        if future is not None:
            future.cancel()
        self._nav_future = None

    @skill
    def stop_navigation(self) -> str:
        """Immediatly stop moving."""

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        self._cancel_goal_and_stop()

        return "Stopped"

    def _cancel_goal_and_stop(self) -> None:
        self._cancel_reissue()
        self._navigation.cancel_goal()

    def _get_goal_pose_from_result(self, result: dict[str, Any]) -> PoseStamped | None:
        similarity = 1.0 - (result.get("distance") or 1)
        if similarity < self._similarity_threshold:
            logger.warning(
                f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
            )
            return None

        metadata = result.get("metadata")
        if not metadata:
            return None
        first = metadata[0]
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            position=make_vector3(pos_x, pos_y, 0),
            orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
            frame_id="map",
        )
