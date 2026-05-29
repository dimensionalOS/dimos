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

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from threading import RLock
import time
from typing import Protocol

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.models.vl.base import VlModel
from dimos.models.vl.types import VlModelName
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class SeatObservation:
    """A candidate chair pose in the room map frame."""

    seat_id: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class PersonObservation:
    """A detected person position in the room map frame."""

    x: float
    y: float


@dataclass(frozen=True)
class SeatGuideResult:
    seat: SeatObservation
    goal_x: float
    goal_y: float
    goal_yaw: float
    spoken_summary: str


@dataclass(frozen=True)
class SeatSceneObservation:
    seats: list[SeatObservation]
    people: list[PersonObservation]
    robot_x: float = 0.0
    robot_y: float = 0.0
    source: str = "unknown"


@dataclass(frozen=True)
class SeatGuideIntent:
    should_find_seat: bool
    normalized_text: str


class SeatObservationProviderSpec(Spec, Protocol):
    def get_seat_scene(self) -> SeatSceneObservation: ...


class ExplorationSpec(Spec, Protocol):
    def begin_exploration(self) -> str: ...
    def end_exploration(self) -> str: ...


class RelativeMoveSpec(Spec, Protocol):
    def relative_move(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        degrees: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        duration: float = 0.0,
    ) -> str: ...


class DirectMoveSpec(Spec, Protocol):
    def direct_move(
        self,
        x: float,
        y: float = 0.0,
        yaw: float = 0.0,
        duration: float = 1.0,
    ) -> str: ...


class SeatGuideRequestSpec(Spec, Protocol):
    def handle_seat_request(self, text: str) -> str: ...
    def preview_seat_request(self, text: str) -> str: ...


class SeatGuidePlanner:
    """Selects an empty conference room seat and computes the robot guide pose."""

    def __init__(
        self,
        *,
        occupied_radius_m: float = 0.75,
        aisle_offset_m: float = 0.65,
    ) -> None:
        if occupied_radius_m <= 0:
            raise ValueError("occupied_radius_m must be positive.")
        if aisle_offset_m < 0:
            raise ValueError("aisle_offset_m cannot be negative.")
        self.occupied_radius_m = occupied_radius_m
        self.aisle_offset_m = aisle_offset_m

    def find_empty_seat(
        self,
        seats: list[SeatObservation],
        people: list[PersonObservation],
        robot_x: float = 0.0,
        robot_y: float = 0.0,
    ) -> SeatGuideResult | None:
        empty_seats = [seat for seat in seats if not self._is_occupied(seat, people)]
        if not empty_seats:
            return None

        selected = min(
            empty_seats,
            key=lambda seat: math.hypot(seat.x - robot_x, seat.y - robot_y),
        )
        goal_x, goal_y = self._guide_pose_for(selected)
        return SeatGuideResult(
            seat=selected,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=selected.yaw,
            spoken_summary=(
                f"I found an empty seat {selected.seat_id}. "
                "Please follow me to the chair beside the table."
            ),
        )

    def _is_occupied(self, seat: SeatObservation, people: list[PersonObservation]) -> bool:
        return any(
            math.hypot(person.x - seat.x, person.y - seat.y) <= self.occupied_radius_m
            for person in people
        )

    def occupancy_counts(
        self, seats: list[SeatObservation], people: list[PersonObservation]
    ) -> tuple[int, int]:
        occupied = sum(1 for seat in seats if self._is_occupied(seat, people))
        return len(seats) - occupied, occupied

    def _guide_pose_for(self, seat: SeatObservation) -> tuple[float, float]:
        offset_x = math.cos(seat.yaw) * self.aisle_offset_m
        offset_y = math.sin(seat.yaw) * self.aisle_offset_m
        return seat.x + offset_x, seat.y + offset_y


class SeatGuideSkillContainer(Module):
    """Skill container for finding and guiding to an empty conference room seat."""

    _navigation: NavigationInterfaceSpec
    _seat_observation_provider: SeatObservationProviderSpec | None = None
    _explorer: ExplorationSpec | None = None
    _direct_mover: DirectMoveSpec | None = None
    _relative_mover: RelativeMoveSpec | None = None
    _seat_guide_goal_sequence: int = 0
    _seat_guide_goal_reached_reset_required: bool = False

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def find_empty_seat(
        self,
        seats: list[float],
        people: list[float],
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        wait_for_arrival: bool = False,
        arrival_timeout_s: float = 60.0,
        arrival_poll_s: float = 0.5,
        arrival_message: str = "我已经到了, 空椅子在我右边, 请坐。",
    ) -> str:
        """Find an empty chair in a conference room and navigate next to it.

        This is the demo-critical SeatGuide skill for a controlled conference room.
        Provide chair detections as a flat list of [x, y, yaw] triples in the map frame.
        Provide person detections as a flat list of [x, y] pairs in the map frame.
        A chair is considered occupied when a person is within 0.75 meters.

        Args:
            seats: Flat chair pose list [x, y, yaw, x, y, yaw, ...].
            people: Flat person position list [x, y, x, y, ...].
            robot_x: Robot x position used to choose the nearest empty seat.
            robot_y: Robot y position used to choose the nearest empty seat.
            wait_for_arrival: When true, wait for navigation to finish before returning.
            arrival_timeout_s: Maximum seconds to wait for arrival.
            arrival_poll_s: Delay between navigation status checks.
            arrival_message: Text response to return after arrival.
        """
        seat_observations = _parse_seats(seats)
        person_observations = _parse_people(people)
        if not seat_observations:
            message = (
                "I cannot see any seats yet. Please face the conference table or calibrate "
                "the room layout."
            )
            return message

        planner = SeatGuidePlanner()
        result = planner.find_empty_seat(
            seat_observations,
            person_observations,
            robot_x=robot_x,
            robot_y=robot_y,
        )
        if result is None:
            message = "I could not find an empty seat in the conference room."
            return message

        goal = PoseStamped(
            frame_id="map",
            position=Vector3(result.goal_x, result.goal_y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, result.goal_yaw)),
        )
        navigation_text, navigation_ok = self._navigation_readiness_text()
        if not navigation_ok:
            message = (
                f"Found empty seat {result.seat.seat_id}, but navigation is not ready "
                f"for a new goal: {navigation_text}."
            )
            return message

        previous_goal_reached = self._navigation_goal_reached_or_false()
        try:
            goal_started = self._navigation.set_goal(goal)
        except Exception as exc:
            message = (
                f"Found empty seat {result.seat.seat_id}, but navigation raised an error: "
                f"{exc}."
            )
            return message

        if not goal_started:
            message = f"Found empty seat {result.seat.seat_id}, but failed to start navigation."
            return message

        self._seat_guide_goal_sequence = getattr(self, "_seat_guide_goal_sequence", 0) + 1
        self._seat_guide_goal_reached_reset_required = previous_goal_reached
        navigating_message = (
            f"{result.spoken_summary} Navigating to ({result.goal_x:.2f}, {result.goal_y:.2f})."
        )
        if not wait_for_arrival:
            return navigating_message

        arrival_result = self._wait_for_arrival(
            timeout_s=arrival_timeout_s,
            poll_s=arrival_poll_s,
        )
        if arrival_result == "arrived":
            return f"{navigating_message} {arrival_message}"
        if arrival_result == "failed":
            message = (
                f"{navigating_message} I found the empty seat, but navigation stopped "
                "before reaching it."
            )
            return message

        message = (
            f"{navigating_message} I found the empty seat, but navigation did not "
            f"finish within {arrival_timeout_s:.0f} seconds."
        )
        return message

    @skill
    def find_empty_seat_from_scene(
        self,
        require_live_perception: bool = True,
        wait_for_arrival: bool = False,
        arrival_timeout_s: float = 60.0,
    ) -> str:
        """Find an empty chair using the configured conference room observation provider.

        Use this for the SeatGuide demo when perception or a synthetic room provider
        is already running as a module. The provider returns chair poses, person
        positions, and robot position in the map frame.

        Args:
            require_live_perception: When true, only a camera-backed scene can
                trigger navigation. Set false only for explicit fallback calibration.
            wait_for_arrival: When true, wait for navigation to finish before returning.
            arrival_timeout_s: Maximum seconds to wait for arrival.
        """
        if self._seat_observation_provider is None:
            message = "No seat observation provider is connected."
            return message

        scene = self._seat_observation_provider.get_seat_scene()
        if require_live_perception and not _is_live_camera_source(scene.source):
            message = _describe_live_perception_required(scene)
            return message
        seats = _flatten_seats(scene.seats)
        people = _flatten_people(scene.people)
        return self.find_empty_seat(
            seats=seats,
            people=people,
            robot_x=scene.robot_x,
            robot_y=scene.robot_y,
            wait_for_arrival=wait_for_arrival,
            arrival_timeout_s=arrival_timeout_s,
        )

    @skill
    def search_for_empty_seat_from_scene(
        self,
        search_timeout_s: float = 30.0,
        poll_interval_s: float = 0.5,
        require_live_perception: bool = True,
        wait_for_arrival: bool = False,
        arrival_timeout_s: float = 60.0,
    ) -> str:
        """Move around while scanning for a visible empty chair, then navigate to it.

        Use this when the current camera view has no chair but the Go2 should
        actively search the nearby area. The robot starts exploration, polls the
        SeatGuide camera scene, stops exploration when a live camera seat is
        visible, then sends the normal empty-seat navigation goal.

        Args:
            search_timeout_s: Maximum time to search before stopping exploration.
            poll_interval_s: Delay between camera scene checks during search.
            require_live_perception: When true, only camera-backed detections can
                trigger final navigation.
            wait_for_arrival: When true, wait for navigation to finish before returning.
            arrival_timeout_s: Maximum seconds to wait for arrival.
        """
        if self._seat_observation_provider is None:
            message = "No seat observation provider is connected."
            return message

        if self._direct_mover is not None or self._relative_mover is not None:
            return self.scan_for_empty_seat_from_scene(
                max_turn_degrees=min(360.0, max(30.0, search_timeout_s * 12.0)),
                step_degrees=30.0,
                settle_s=max(0.5, poll_interval_s),
                require_live_perception=require_live_perception,
                wait_for_arrival=wait_for_arrival,
                arrival_timeout_s=arrival_timeout_s,
            )

        if self._explorer is None:
            message = (
                "I cannot see any seats yet, and no exploration module is connected "
                "to search for one."
            )
            return message

        search_timeout_s = max(1.0, min(search_timeout_s, 120.0))
        poll_interval_s = max(0.2, min(poll_interval_s, 5.0))
        initial_scene = self._seat_observation_provider.get_seat_scene()
        if _scene_has_empty_seat(initial_scene) and (
            not require_live_perception or _is_live_camera_source(initial_scene.source)
        ):
            return self.find_empty_seat_from_scene(
                require_live_perception=require_live_perception
            )
        if (
            require_live_perception
            and initial_scene.source != "camera_no_seats_detected"
            and not _is_live_camera_source(initial_scene.source)
        ):
            message = _describe_live_perception_required(initial_scene)
            return message

        self._explorer.begin_exploration()
        exploration_active = True
        deadline = time.time() + search_timeout_s
        try:
            while time.time() < deadline:
                scene = self._seat_observation_provider.get_seat_scene()
                if _scene_has_empty_seat(scene) and (
                    not require_live_perception or _is_live_camera_source(scene.source)
                ):
                    self._explorer.end_exploration()
                    exploration_active = False
                    return self.find_empty_seat_from_scene(
                        require_live_perception=require_live_perception,
                        wait_for_arrival=wait_for_arrival,
                        arrival_timeout_s=arrival_timeout_s,
                    )
                time.sleep(poll_interval_s)
        finally:
            if exploration_active:
                self._explorer.end_exploration()

        message = (
            "I searched but still cannot see an empty seat. Please reposition me "
            "or point the camera toward the conference table."
        )
        return message

    @skill
    def scan_for_empty_seat_from_scene(
        self,
        max_turn_degrees: float = 360.0,
        step_degrees: float = 30.0,
        settle_s: float = 0.75,
        turn_yaw_rate_rad_s: float = 0.5,
        require_live_perception: bool = True,
        wait_for_arrival: bool = False,
        arrival_timeout_s: float = 60.0,
    ) -> str:
        """Rotate in place while scanning for a visible empty chair, then navigate.

        Use this for the SeatGuide demo when the current camera view does not
        contain an empty chair. The robot turns in place in small increments,
        checks camera-backed SeatGuide perception after each turn, and navigates
        once an empty seat is visible.

        Args:
            max_turn_degrees: Maximum total in-place scan angle.
            step_degrees: Degrees to rotate per scan step.
            settle_s: Delay after each turn before reading camera perception.
            turn_yaw_rate_rad_s: Yaw velocity for direct in-place turns.
            require_live_perception: When true, only camera-backed detections can
                trigger final navigation.
            wait_for_arrival: When true, wait for navigation to finish before returning.
            arrival_timeout_s: Maximum seconds to wait for arrival.
        """
        if self._seat_observation_provider is None:
            message = "No seat observation provider is connected."
            return message
        if self._direct_mover is None and self._relative_mover is None:
            message = (
                "I cannot rotate-scan for a seat because no relative movement "
                "module is connected."
            )
            return message

        max_turn_degrees = max(30.0, min(float(max_turn_degrees), 720.0))
        step_degrees = max(10.0, min(abs(float(step_degrees)), 90.0))
        settle_s = max(0.1, min(float(settle_s), 5.0))
        turn_yaw_rate_rad_s = max(0.1, min(abs(float(turn_yaw_rate_rad_s)), 1.5))
        steps = max(1, math.ceil(max_turn_degrees / step_degrees))

        scene = self._seat_observation_provider.get_seat_scene()
        if _scene_has_empty_seat(scene) and (
            not require_live_perception or _is_live_camera_source(scene.source)
        ):
            return self.find_empty_seat_from_scene(
                require_live_perception=require_live_perception,
                wait_for_arrival=wait_for_arrival,
                arrival_timeout_s=arrival_timeout_s,
            )
        if (
            require_live_perception
            and not _is_live_camera_source(scene.source)
            and scene.source != "camera_no_seats_detected"
        ):
            message = _describe_live_perception_required(scene)
            return message

        for _ in range(steps):
            move_result = self._turn_in_place(
                degrees=step_degrees,
                yaw_rate_rad_s=turn_yaw_rate_rad_s,
            )
            if "failed" in move_result.lower() or "cancelled" in move_result.lower():
                message = f"SeatGuide scan stopped because rotation failed: {move_result}."
                return message
            time.sleep(settle_s)

            scene = self._seat_observation_provider.get_seat_scene()
            if _scene_has_empty_seat(scene) and (
                not require_live_perception or _is_live_camera_source(scene.source)
            ):
                return self.find_empty_seat_from_scene(
                    require_live_perception=require_live_perception,
                    wait_for_arrival=wait_for_arrival,
                    arrival_timeout_s=arrival_timeout_s,
                )

        message = (
            "I rotated in place but still cannot see an empty seat. Please reposition me "
            "or point the camera toward the conference table."
        )
        return message

    def _turn_in_place(self, *, degrees: float, yaw_rate_rad_s: float) -> str:
        if self._direct_mover is not None:
            yaw_direction = 1.0 if degrees >= 0 else -1.0
            yaw = yaw_direction * yaw_rate_rad_s
            duration = max(0.2, abs(math.radians(degrees)) / yaw_rate_rad_s)
            return self._direct_mover.direct_move(
                x=0.0,
                y=0.0,
                yaw=yaw,
                duration=duration,
            )
        if self._relative_mover is not None:
            return self._relative_mover.relative_move(
                forward=0.0,
                left=0.0,
                degrees=degrees,
            )
        return "Rotation failed: no movement module is connected."

    @skill
    def preview_empty_seat_goal(self) -> str:
        """Preview the selected empty seat and navigation goal without moving.

        Use this during real Go2 bring-up after `seat_guide_status` and before
        `handle_seat_request` to verify the selected chair and map-frame goal.
        This never calls navigation.
        """
        if self._seat_observation_provider is None:
            return "No seat observation provider is connected."

        scene = self._seat_observation_provider.get_seat_scene()
        return _describe_goal_preview(scene)

    @skill
    def handle_seat_request(
        self,
        text: str,
        require_live_perception: bool = True,
        wait_for_arrival: bool = True,
        arrival_timeout_s: float = 60.0,
    ) -> str:
        """Handle a spoken or typed request to find an empty conference room seat.

        This is the Go2-free voice intake boundary for the SeatGuide demo. Pass
        speech-to-text output or typed text here. If the text asks for an empty
        seat, this delegates to the configured scene provider and navigation.

        Args:
            text: Transcribed or typed user request.
            require_live_perception: When true, only a camera-backed scene can
                trigger navigation. Set false only for explicit fallback calibration.
            wait_for_arrival: When true, wait for navigation to finish before returning.
            arrival_timeout_s: Maximum seconds to wait for arrival.
        """
        intent = parse_seat_guide_intent(text)
        if not intent.should_find_seat:
            message = "I did not hear a request to find an empty seat."
            return message

        scene = (
            self._seat_observation_provider.get_seat_scene()
            if self._seat_observation_provider is not None
            else None
        )
        if (
            scene is not None
            and (
                (scene.source == "camera_no_seats_detected" and not scene.seats)
                or (_is_live_camera_source(scene.source) and not _scene_has_empty_seat(scene))
            )
            and self._explorer is not None
        ):
            return self.search_for_empty_seat_from_scene(
                require_live_perception=require_live_perception,
                wait_for_arrival=wait_for_arrival,
                arrival_timeout_s=arrival_timeout_s,
            )

        return self.find_empty_seat_from_scene(
            require_live_perception=require_live_perception,
            wait_for_arrival=wait_for_arrival,
            arrival_timeout_s=arrival_timeout_s,
        )

    @skill
    def preview_seat_request(self, text: str) -> str:
        """Preview a spoken or typed SeatGuide request without moving.

        Use this to validate the real microphone or typed WebInput path during
        bring-up. If the text asks for an empty seat, this runs the same
        no-motion preflight used before live hardware navigation.

        Args:
            text: Transcribed or typed user request.
        """
        intent = parse_seat_guide_intent(text)
        if not intent.should_find_seat:
            message = "I did not hear a request to find an empty seat."
            return message

        message = self.seat_guide_preflight()
        return message

    @skill
    def seat_guide_preflight(self, require_live_perception: bool = True) -> str:
        """Run a no-motion SeatGuide hardware preflight before sending a goal.

        Use this on the real Go2 before asking a person to follow the robot. It
        checks navigation reachability at the interface level, the current seat
        scene source, whether an empty seat can be selected, and whether speech
        feedback is connected. This never calls navigation.

        Args:
            require_live_perception: When true, only a camera-backed scene can
                pass preflight. Set false only for explicit fallback calibration.
        """
        if self._seat_observation_provider is None:
            return (
                "SeatGuide preflight no-go: "
                f"{self._navigation_readiness_text()[0]}; perception=missing; "
                "feedback=phone_or_web."
            )

        scene = self._seat_observation_provider.get_seat_scene()
        return self._describe_preflight(scene, require_live_perception=require_live_perception)

    @skill
    def seat_guide_status(self) -> str:
        """Describe the current SeatGuide scene provider state without navigating.

        Use this during bring-up to confirm whether SeatGuide can see chairs and
        people before asking the robot to guide a user to an empty seat.
        """
        if self._seat_observation_provider is None:
            return "SeatGuide status: no seat observation provider is connected."

        scene = self._seat_observation_provider.get_seat_scene()
        return _describe_scene(scene)

    @skill
    def seat_guide_readiness_report(self, require_live_perception: bool = True) -> str:
        """Run all no-motion SeatGuide readiness checks in one report.

        Use this as the first hardware bring-up command. It combines scene
        status, live-perception preflight, and selected-goal preview without
        calling navigation.

        Args:
            require_live_perception: When true, preflight only passes a
                camera-backed scene. Set false only for explicit fallback calibration.
        """
        if self._seat_observation_provider is None:
            return "SeatGuide readiness report: no seat observation provider is connected."

        scene = self._seat_observation_provider.get_seat_scene()
        status = _describe_scene(scene)
        preflight = self._describe_preflight(
            scene, require_live_perception=require_live_perception
        )
        preview = _describe_goal_preview(scene)
        return f"SeatGuide readiness report: {status} | {preflight} | {preview}"

    @skill
    def seat_guide_navigation_status(self) -> str:
        """Report whether the current SeatGuide navigation goal has completed.

        Use this after a live SeatGuide request to verify the robot did more
        than accept a goal. It reads the navigation interface state and
        `is_goal_reached()` without sending or canceling any goal.
        """
        goal_sequence = getattr(self, "_seat_guide_goal_sequence", 0)
        if not hasattr(self, "_navigation") or self._navigation is None:
            return (
                "SeatGuide navigation status: navigation=missing; "
                f"goal_reached=unknown; goal_sequence={goal_sequence}."
            )
        try:
            state = self._navigation.get_state()
            raw_goal_reached = self._navigation.is_goal_reached()
        except Exception as exc:
            return (
                f"SeatGuide navigation status: navigation=error({exc}); "
                f"goal_reached=unknown; goal_sequence={goal_sequence}."
            )

        reset_suffix = ""
        goal_reached = raw_goal_reached
        if (
            goal_sequence > 0
            and getattr(self, "_seat_guide_goal_reached_reset_required", False)
        ):
            if raw_goal_reached:
                goal_reached = False
                reset_suffix = "; completion_reset=waiting_for_false"
            else:
                self._seat_guide_goal_reached_reset_required = False

        return (
            f"SeatGuide navigation status: navigation={state.name}; "
            f"goal_reached={'true' if goal_reached else 'false'}; "
            f"goal_sequence={goal_sequence}{reset_suffix}."
        )

    def _navigation_goal_reached_or_false(self) -> bool:
        try:
            return self._navigation.is_goal_reached()
        except Exception:
            return False

    def _wait_for_arrival(self, *, timeout_s: float, poll_s: float) -> str:
        timeout_s = max(1.0, min(float(timeout_s), 300.0))
        poll_s = max(0.1, min(float(poll_s), 5.0))
        start = time.time()
        deadline = start + timeout_s
        saw_non_idle = False

        while time.time() < deadline:
            try:
                state = self._navigation.get_state()
                raw_goal_reached = self._navigation.is_goal_reached()
            except Exception:
                logger.warning("SeatGuide navigation arrival check failed", exc_info=True)
                return "failed"

            goal_reached = raw_goal_reached
            if getattr(self, "_seat_guide_goal_reached_reset_required", False):
                if raw_goal_reached:
                    goal_reached = False
                else:
                    self._seat_guide_goal_reached_reset_required = False

            if goal_reached:
                return "arrived"
            if state.name != "IDLE":
                saw_non_idle = True
            elif saw_non_idle or time.time() - start >= min(1.0, timeout_s):
                return "failed"

            time.sleep(poll_s)

        return "timeout"

    def _navigation_readiness_text(self) -> tuple[str, bool]:
        if not hasattr(self, "_navigation") or self._navigation is None:
            return "navigation=missing", False
        try:
            state = self._navigation.get_state()
            return f"navigation={state.name}", state.name == "IDLE"
        except Exception as exc:
            return f"navigation=error({exc})", False

    def _describe_preflight(
        self,
        scene: SeatSceneObservation,
        *,
        require_live_perception: bool,
    ) -> str:
        navigation_text, navigation_ok = self._navigation_readiness_text()

        if not scene.seats:
            return (
                "SeatGuide preflight no-go: "
                f"{navigation_text}; perception={scene.source} no seats; "
                "feedback=phone_or_web."
            )
        if require_live_perception and not _is_live_camera_source(scene.source):
            return (
                "SeatGuide preflight no-go: "
                f"{navigation_text}; perception={scene.source} is not live camera; "
                f"seats={len(scene.seats)} people={len(scene.people)}; "
                "feedback=phone_or_web."
            )

        planner = SeatGuidePlanner()
        empty_count, occupied_count = planner.occupancy_counts(scene.seats, scene.people)
        result = planner.find_empty_seat(
            scene.seats,
            scene.people,
            robot_x=scene.robot_x,
            robot_y=scene.robot_y,
        )
        if result is None:
            return (
                "SeatGuide preflight no-go: "
                f"{navigation_text}; perception={scene.source}; no empty seat; "
                f"empty={empty_count} occupied={occupied_count}; "
                "feedback=phone_or_web."
            )

        verdict = "ready" if navigation_ok else "no-go"
        return (
            f"SeatGuide preflight {verdict}: {navigation_text}; "
            f"perception={scene.source} seats={len(scene.seats)} people={len(scene.people)}; "
            f"empty={empty_count} occupied={occupied_count}; "
            f"selected={result.seat.seat_id}; "
            f"goal=({result.goal_x:.2f}, {result.goal_y:.2f}, yaw={result.goal_yaw:.2f}); "
            "feedback=phone_or_web."
        )


class SyntheticSeatSceneConfig(ModuleConfig):
    seats: list[float] = Field(
        default_factory=lambda: [0.0, -1.0, 0.0, 1.5, -1.0, 0.0, 3.0, -1.0, 0.0]
    )
    people: list[float] = Field(default_factory=lambda: [0.1, -1.0, 1.6, -1.0])
    robot_x: float = -1.0
    robot_y: float = -1.0


class SyntheticSeatObservationProvider(Module):
    """Go2-free conference room observation provider for tests and demos."""

    config: SyntheticSeatSceneConfig
    _scene_override: SeatSceneObservation | None = None
    _scene_lock: RLock = RLock()

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def get_seat_scene(self) -> SeatSceneObservation:
        with self._scene_lock:
            if self._scene_override is not None:
                return self._scene_override

        return _scene_from_flat_config(self.config)

    @skill
    def set_seat_scene(
        self,
        seats: list[float],
        people: list[float],
        robot_x: float = 0.0,
        robot_y: float = 0.0,
    ) -> str:
        """Configure the synthetic conference room scene at runtime.

        Use this during Go2 bring-up to align the fallback scene with the real
        chair layout before calling `handle_seat_request`. Chair poses are flat
        [x, y, yaw] triples in the map frame. Person positions are flat [x, y]
        pairs in the map frame.

        Args:
            seats: Flat chair pose list [x, y, yaw, x, y, yaw, ...].
            people: Flat person position list [x, y, x, y, ...].
            robot_x: Robot x position in the map frame.
            robot_y: Robot y position in the map frame.
        """
        scene = SeatSceneObservation(
            seats=_parse_seats(seats),
            people=_parse_people(people),
            robot_x=robot_x,
            robot_y=robot_y,
            source="runtime_override",
        )
        with self._scene_lock:
            self._scene_override = scene

        people_word = "person" if len(scene.people) == 1 else "people"
        return f"Configured {len(scene.seats)} seats and {len(scene.people)} {people_word}."

    @skill
    def clear_seat_scene_override(self) -> str:
        """Clear the runtime synthetic scene and return to configured defaults."""
        with self._scene_lock:
            self._scene_override = None
        return "Cleared synthetic seat scene override."


class CameraSeatSceneConfig(SyntheticSeatSceneConfig):
    seats: list[float] = Field(default_factory=list)
    people: list[float] = Field(default_factory=list)
    detection_model: VlModelName | None = None
    fast_detector_enabled: bool = True
    fast_detector_model_name: str = "yolo11n.pt"
    vlm_fallback_enabled: bool = False
    chair_distance_m: float = 2.0
    lateral_span_m: float = 3.0
    max_input_age_s: float = 5.0


class CameraSeatObservationProvider(Module):
    """Camera-backed conference room observation provider for SeatGuide."""

    config: CameraSeatSceneConfig
    color_image: In[Image]
    camera_info: In[CameraInfo]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]

    _latest_image: Image | None = None
    _latest_camera_info: CameraInfo | None = None
    _latest_lidar: PointCloud2 | None = None
    _latest_odom: PoseStamped | None = None
    _vl_model: VlModel | None = None
    _fast_detector: Detector | None = None
    _scene_override: SeatSceneObservation | None = None
    _scene_lock: RLock = RLock()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        with self._scene_lock:
            self._latest_image = image

    def _on_camera_info(self, camera_info: CameraInfo) -> None:
        with self._scene_lock:
            self._latest_camera_info = camera_info

    def _on_lidar(self, lidar: PointCloud2) -> None:
        with self._scene_lock:
            self._latest_lidar = lidar

    def _on_odom(self, odom: PoseStamped) -> None:
        with self._scene_lock:
            self._latest_odom = odom

    @rpc
    def get_seat_scene(self) -> SeatSceneObservation:
        with self._scene_lock:
            if self._scene_override is not None:
                return self._scene_override
            latest_image = self._latest_image
            latest_odom = self._latest_odom
            latest_camera_info = getattr(self, "_latest_camera_info", None)
            latest_lidar = getattr(self, "_latest_lidar", None)

        if latest_image is None:
            return _scene_from_flat_config(self.config, source="no_camera_image")
        if latest_odom is None:
            return _scene_from_flat_config(self.config, source="camera_no_odom")
        if _message_age_s(latest_image.ts) > self.config.max_input_age_s:
            return _scene_from_flat_config(self.config, source="stale_camera_image")
        if _message_age_s(latest_odom.ts) > self.config.max_input_age_s:
            return _scene_from_flat_config(self.config, source="stale_camera_odom")

        try:
            detected_scene = self._detect_scene_from_image(
                latest_image,
                latest_odom,
                camera_info=latest_camera_info,
                lidar=latest_lidar,
            )
        except Exception:
            logger.warning(
                "Failed to detect conference room seats from camera image", exc_info=True
            )
            return _scene_from_flat_config(self.config, source="camera_detection_error")

        if not detected_scene.seats:
            return _scene_from_flat_config(self.config, source="camera_no_seats_detected")

        return detected_scene

    @skill
    def set_seat_scene(
        self,
        seats: list[float],
        people: list[float],
        robot_x: float = 0.0,
        robot_y: float = 0.0,
    ) -> str:
        """Configure the fallback conference room scene at runtime.

        Use this during Go2 bring-up when visual detection is unavailable or
        unreliable. Chair poses are flat [x, y, yaw] triples in the map frame.
        Person positions are flat [x, y] pairs in the map frame.

        Args:
            seats: Flat chair pose list [x, y, yaw, x, y, yaw, ...].
            people: Flat person position list [x, y, x, y, ...].
            robot_x: Robot x position in the map frame.
            robot_y: Robot y position in the map frame.
        """
        scene = SeatSceneObservation(
            seats=_parse_seats(seats),
            people=_parse_people(people),
            robot_x=robot_x,
            robot_y=robot_y,
            source="runtime_override",
        )
        with self._scene_lock:
            self._scene_override = scene

        people_word = "person" if len(scene.people) == 1 else "people"
        return f"Configured {len(scene.seats)} seats and {len(scene.people)} {people_word}."

    @skill
    def clear_seat_scene_override(self) -> str:
        """Clear the runtime fallback scene and return to camera detection/defaults."""
        with self._scene_lock:
            self._scene_override = None
        return "Cleared camera seat scene override."

    @skill
    def camera_seat_provider_status(self) -> str:
        """Report camera-backed SeatGuide perception readiness without running detection.

        Use this during Go2 bring-up before `seat_guide_status` to check whether
        camera frames and odometry are arriving, whether the VLM credential path
        is configured, and whether a runtime fallback override is active.
        """
        with self._scene_lock:
            override_active = self._scene_override is not None
            latest_image = self._latest_image
            latest_camera_info = getattr(self, "_latest_camera_info", None)
            latest_lidar = getattr(self, "_latest_lidar", None)
            latest_odom = self._latest_odom
        image_text = (
            f"image={latest_image.width}x{latest_image.height}"
            if latest_image is not None
            else "image=missing"
        )
        odom_text = (
            f"odom=({latest_odom.x:.2f}, {latest_odom.y:.2f}, "
            f"yaw={latest_odom.yaw:.2f})"
            if latest_odom is not None
            else "odom=missing"
        )
        camera_info_text = (
            f"camera_info={latest_camera_info.width}x{latest_camera_info.height}"
            if latest_camera_info is not None
            else "camera_info=missing"
        )
        lidar_text = (
            f"lidar={len(latest_lidar)} points" if latest_lidar is not None else "lidar=missing"
        )
        image_fresh_text = _freshness_text(
            latest_image.ts if latest_image is not None else None,
            self.config.max_input_age_s,
        )
        camera_info_fresh_text = _freshness_text(
            latest_camera_info.ts if latest_camera_info is not None else None,
            self.config.max_input_age_s,
        )
        lidar_fresh_text = _freshness_text(
            latest_lidar.ts if latest_lidar is not None else None,
            self.config.max_input_age_s,
        )
        odom_fresh_text = _freshness_text(
            latest_odom.ts if latest_odom is not None else None,
            self.config.max_input_age_s,
        )
        detection_model = self._detection_model()
        credential_text = self._credential_status_for(detection_model)
        detector_text = "fast_detector=yolo" if self.config.fast_detector_enabled else "fast_detector=off"
        return (
            "CameraSeatObservationProvider status: "
            f"{image_text}; image_fresh={image_fresh_text}; "
            f"{camera_info_text}; camera_info_fresh={camera_info_fresh_text}; "
            f"{lidar_text}; lidar_fresh={lidar_fresh_text}; "
            f"{odom_text}; odom_fresh={odom_fresh_text}; "
            f"{detector_text}; "
            f"detection_model={detection_model}; "
            f"{credential_text}; override={'active' if override_active else 'inactive'}; "
            f"configured_fallback_seats={len(_parse_seats(self.config.seats))}; "
            f"configured_fallback_people={len(_parse_people(self.config.people))}."
        )

    def _detect_scene_from_image(
        self,
        image: Image,
        odom: PoseStamped | None = None,
        *,
        camera_info: CameraInfo | None = None,
        lidar: PointCloud2 | None = None,
    ) -> SeatSceneObservation:
        chair_detections, person_detections = self._detect_chairs_and_people(image)
        robot_x, robot_y, robot_yaw = self._robot_pose_for_detection(odom)
        transform = None
        if camera_info is not None and lidar is not None:
            transform = self.tf.get("camera_optical", lidar.frame_id, image.ts, 1.0)
        seats: list[SeatObservation] = []
        people: list[PersonObservation] = []
        used_3d = False

        for detection in chair_detections:
            seat_id = f"seat_{len(seats) + 1}"
            seat = None
            if camera_info is not None and lidar is not None and transform is not None:
                seat = _bbox_to_seat_observation_3d(
                    seat_id=seat_id,
                    detection=detection,
                    camera_info=camera_info,
                    lidar=lidar,
                    world_to_optical_transform=transform,
                    robot_x=robot_x,
                    robot_y=robot_y,
                )
            if seat is not None:
                used_3d = True
                seats.append(seat)
                continue

            seats.append(
                _bbox_to_seat_observation(
                    seat_id=seat_id,
                    bbox=detection.bbox,
                    image_width=image.width,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    distance_m=self.config.chair_distance_m,
                    lateral_span_m=self.config.lateral_span_m,
                )
            )
        for detection in person_detections:
            person = None
            if camera_info is not None and lidar is not None and transform is not None:
                person = _bbox_to_person_observation_3d(
                    detection=detection,
                    camera_info=camera_info,
                    lidar=lidar,
                    world_to_optical_transform=transform,
                )
            if person is not None:
                used_3d = True
                people.append(person)
                continue

            people.append(
                _bbox_to_person_observation(
                    bbox=detection.bbox,
                    image_width=image.width,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    distance_m=self.config.chair_distance_m,
                    lateral_span_m=self.config.lateral_span_m,
                )
            )

        return SeatSceneObservation(
            seats=seats,
            people=people,
            robot_x=robot_x,
            robot_y=robot_y,
            source="camera_3d" if used_3d else "camera",
        )

    def _robot_pose_for_detection(
        self, odom: PoseStamped | None = None
    ) -> tuple[float, float, float]:
        if odom is None:
            return self.config.robot_x, self.config.robot_y, 0.0
        return odom.x, odom.y, odom.yaw

    def _get_vl_model(self) -> VlModel:
        if self._vl_model is not None:
            return self._vl_model
        detection_model = self._detection_model()
        if detection_model == "qwen" and not os.getenv("ALIBABA_API_KEY"):
            raise ValueError(
                "CameraSeatObservationProvider detection_model=qwen requires ALIBABA_API_KEY"
            )

        from dimos.models.vl.create import create

        self._vl_model = create(detection_model)
        return self._vl_model

    def _detect_chairs_and_people(
        self, image: Image
    ) -> tuple[list[Detection2DBBox], list[Detection2DBBox]]:
        if getattr(self, "_vl_model", None) is not None:
            return self._detect_chairs_and_people_with_vlm(image)

        if self.config.fast_detector_enabled:
            try:
                detections = self._get_fast_detector().process_image(image)
                chairs = _detections_named(detections, {"chair"})
                people = _detections_named(detections, {"person"})
                if chairs or people or not self.config.vlm_fallback_enabled:
                    return chairs, people
            except Exception:
                logger.warning("SeatGuide fast detector failed; falling back to VLM", exc_info=True)
                if not self.config.vlm_fallback_enabled:
                    return [], []

        return self._detect_chairs_and_people_with_vlm(image)

    def _detect_chairs_and_people_with_vlm(
        self, image: Image
    ) -> tuple[list[Detection2DBBox], list[Detection2DBBox]]:
        vl_model = self._get_vl_model()
        chair_detections = vl_model.query_detections(image, "chair").detections
        person_detections = vl_model.query_detections(image, "person").detections
        return list(chair_detections), list(person_detections)

    def _get_fast_detector(self) -> Detector:
        if getattr(self, "_fast_detector", None) is not None:
            return self._fast_detector

        from dimos.perception.detection.detectors.yolo import Yolo2DDetector

        self._fast_detector = Yolo2DDetector(model_name=self.config.fast_detector_model_name)
        return self._fast_detector

    def _detection_model(self) -> VlModelName:
        return self.config.detection_model or self.config.g.detection_model

    def _credential_status_for(self, detection_model: VlModelName) -> str:
        if detection_model == "qwen" and not os.getenv("ALIBABA_API_KEY"):
            return "credential=missing"
        return "credential=present"


def _parse_seats(values: list[float]) -> list[SeatObservation]:
    if len(values) % 3 != 0:
        raise ValueError("seats must be a flat list of [x, y, yaw] triples.")
    return [
        SeatObservation(
            seat_id=f"seat_{index + 1}",
            x=float(values[offset]),
            y=float(values[offset + 1]),
            yaw=float(values[offset + 2]),
        )
        for index, offset in enumerate(range(0, len(values), 3))
    ]


def _parse_people(values: list[float]) -> list[PersonObservation]:
    if len(values) % 2 != 0:
        raise ValueError("people must be a flat list of [x, y] pairs.")
    return [
        PersonObservation(x=float(values[offset]), y=float(values[offset + 1]))
        for offset in range(0, len(values), 2)
    ]


def _flatten_seats(seats: list[SeatObservation]) -> list[float]:
    values: list[float] = []
    for seat in seats:
        values.extend([seat.x, seat.y, seat.yaw])
    return values


def _flatten_people(people: list[PersonObservation]) -> list[float]:
    values: list[float] = []
    for person in people:
        values.extend([person.x, person.y])
    return values


def _detections_named(
    detections: ImageDetections2D[Detection2DBBox],
    names: set[str],
) -> list[Detection2DBBox]:
    return [
        detection
        for detection in detections.detections
        if detection.name.strip().lower() in names
    ]


def _scene_from_flat_config(
    config: SyntheticSeatSceneConfig,
    *,
    source: str = "configured_fallback",
) -> SeatSceneObservation:
    return SeatSceneObservation(
        seats=_parse_seats(config.seats),
        people=_parse_people(config.people),
        robot_x=config.robot_x,
        robot_y=config.robot_y,
        source=source,
    )


def _describe_scene(scene: SeatSceneObservation) -> str:
    if not scene.seats:
        return (
            f"SeatGuide scene source={scene.source}: no seats visible or configured; "
            f"{len(scene.people)} people detected."
        )
    seats = ", ".join(
        f"{seat.seat_id}=({seat.x:.2f}, {seat.y:.2f}, yaw={seat.yaw:.2f})"
        for seat in scene.seats
    )
    people = ", ".join(f"({person.x:.2f}, {person.y:.2f})" for person in scene.people)
    people_text = people if people else "none"
    return (
        f"SeatGuide scene source={scene.source}: {len(scene.seats)} seats [{seats}], "
        f"{len(scene.people)} people [{people_text}], "
        f"robot=({scene.robot_x:.2f}, {scene.robot_y:.2f})."
    )


def _describe_goal_preview(scene: SeatSceneObservation) -> str:
    if not scene.seats:
        return f"SeatGuide preview source={scene.source}: no seats visible or configured."

    planner = SeatGuidePlanner()
    empty_count, occupied_count = planner.occupancy_counts(scene.seats, scene.people)
    result = planner.find_empty_seat(
        scene.seats,
        scene.people,
        robot_x=scene.robot_x,
        robot_y=scene.robot_y,
    )
    if result is None:
        return (
            f"SeatGuide preview source={scene.source}: no empty seat available; "
            f"empty={empty_count} occupied={occupied_count}."
        )

    return (
        f"SeatGuide preview source={scene.source}: selected {result.seat.seat_id} "
        f"empty={empty_count} occupied={occupied_count} "
        f"seat=({result.seat.x:.2f}, {result.seat.y:.2f}, yaw={result.seat.yaw:.2f}) "
        f"goal=({result.goal_x:.2f}, {result.goal_y:.2f}, yaw={result.goal_yaw:.2f})."
    )


def _scene_has_empty_seat(scene: SeatSceneObservation) -> bool:
    if not scene.seats:
        return False
    planner = SeatGuidePlanner()
    return (
        planner.find_empty_seat(
            scene.seats,
            scene.people,
            robot_x=scene.robot_x,
            robot_y=scene.robot_y,
        )
        is not None
    )


def _describe_live_perception_required(scene: SeatSceneObservation) -> str:
    advice_by_source = {
        "no_camera_image": "check camera stream wiring and face the conference table",
        "camera_no_odom": "check localization/odometry before sending a map-frame goal",
        "stale_camera_image": "camera frames are stale; restore the live camera stream",
        "stale_camera_odom": "odometry is stale; restore localization before sending a goal",
        "camera_no_seats_detected": "turn the robot toward the chairs or adjust the detector",
        "camera_detection_error": "check VLM/API key setup and logs",
        "configured_fallback": "use require_live_perception=false only for explicit fallback calibration",
        "runtime_override": "use require_live_perception=false only for explicit fallback calibration",
    }
    advice = advice_by_source.get(scene.source, "run seat_guide_status and inspect perception")
    return (
        "SeatGuide requires live camera perception before navigation; "
        f"source={scene.source}; seats={len(scene.seats)}; people={len(scene.people)}; "
        f"robot=({scene.robot_x:.2f}, {scene.robot_y:.2f}); next={advice}."
    )


def _is_live_camera_source(source: str) -> bool:
    return source in {"camera", "camera_3d"}


def _message_age_s(ts: float) -> float:
    return max(0.0, time.time() - ts)


def _freshness_text(ts: float | None, max_age_s: float) -> str:
    if ts is None:
        return "missing"
    return "true" if _message_age_s(ts) <= max_age_s else "false"


def _bbox_center_x(bbox: tuple[float, float, float, float], image_width: int) -> float:
    left = max(0.0, min(float(bbox[0]), float(bbox[2]), float(image_width)))
    right = max(0.0, min(max(float(bbox[0]), float(bbox[2])), float(image_width)))
    return (left + right) / 2.0


def _bbox_to_lateral_offset(
    bbox: tuple[float, float, float, float], image_width: int, lateral_span_m: float
) -> float:
    if image_width <= 0:
        return 0.0
    normalized_x = (_bbox_center_x(bbox, image_width) / image_width) - 0.5
    return normalized_x * lateral_span_m


def _bbox_to_seat_observation(
    *,
    seat_id: str,
    bbox: tuple[float, float, float, float],
    image_width: int,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    distance_m: float,
    lateral_span_m: float,
) -> SeatObservation:
    x, y = _camera_relative_to_map(
        forward_m=distance_m,
        lateral_m=_bbox_to_lateral_offset(bbox, image_width, lateral_span_m),
        robot_x=robot_x,
        robot_y=robot_y,
        robot_yaw=robot_yaw,
    )
    return SeatObservation(
        seat_id=seat_id,
        x=x,
        y=y,
        yaw=robot_yaw,
    )


def _bbox_to_person_observation(
    *,
    bbox: tuple[float, float, float, float],
    image_width: int,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    distance_m: float,
    lateral_span_m: float,
) -> PersonObservation:
    x, y = _camera_relative_to_map(
        forward_m=distance_m,
        lateral_m=_bbox_to_lateral_offset(bbox, image_width, lateral_span_m),
        robot_x=robot_x,
        robot_y=robot_y,
        robot_yaw=robot_yaw,
    )
    return PersonObservation(x=x, y=y)


def _bbox_to_seat_observation_3d(
    *,
    seat_id: str,
    detection: Detection2DBBox,
    camera_info: CameraInfo,
    lidar: PointCloud2,
    world_to_optical_transform: Transform,
    robot_x: float,
    robot_y: float,
) -> SeatObservation | None:
    x, y = _bbox_to_map_xy_3d(
        detection=detection,
        camera_info=camera_info,
        lidar=lidar,
        world_to_optical_transform=world_to_optical_transform,
    )
    if x is None or y is None:
        return None
    return SeatObservation(
        seat_id=seat_id,
        x=x,
        y=y,
        yaw=math.atan2(robot_y - y, robot_x - x),
    )


def _bbox_to_person_observation_3d(
    *,
    detection: Detection2DBBox,
    camera_info: CameraInfo,
    lidar: PointCloud2,
    world_to_optical_transform: Transform,
) -> PersonObservation | None:
    x, y = _bbox_to_map_xy_3d(
        detection=detection,
        camera_info=camera_info,
        lidar=lidar,
        world_to_optical_transform=world_to_optical_transform,
    )
    if x is None or y is None:
        return None
    return PersonObservation(x=x, y=y)


def _bbox_to_map_xy_3d(
    *,
    detection: Detection2DBBox,
    camera_info: CameraInfo,
    lidar: PointCloud2,
    world_to_optical_transform: Transform,
) -> tuple[float | None, float | None]:
    detection_3d = Detection3DPC.from_2d(
        det=detection,
        world_pointcloud=lidar,
        camera_info=camera_info,
        world_to_optical_transform=world_to_optical_transform,
        filters=[],
    )
    if detection_3d is None:
        return None, None
    points, _ = detection_3d.pointcloud.as_numpy()
    if len(points) == 0:
        return None, None
    xy = _robust_detection_xy(points)
    return float(xy[0]), float(xy[1])


def _robust_detection_xy(points: object) -> tuple[float, float]:
    import numpy as np

    point_array = np.asarray(points, dtype=float)
    finite = point_array[np.isfinite(point_array).all(axis=1)]
    if len(finite) == 0:
        raise ValueError("detection pointcloud has no finite points")
    if finite.shape[1] >= 3:
        height_threshold = np.percentile(finite[:, 2], 20)
        above_floor = finite[finite[:, 2] >= height_threshold]
        if len(above_floor) > 0:
            finite = above_floor
    xy = np.median(finite[:, :2], axis=0)
    return float(xy[0]), float(xy[1])


def _camera_relative_to_map(
    *,
    forward_m: float,
    lateral_m: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> tuple[float, float]:
    return (
        robot_x + math.cos(robot_yaw) * forward_m - math.sin(robot_yaw) * lateral_m,
        robot_y + math.sin(robot_yaw) * forward_m + math.cos(robot_yaw) * lateral_m,
    )


def parse_seat_guide_intent(text: str) -> SeatGuideIntent:
    normalized = " ".join(text.strip().lower().split())
    if not normalized:
        return SeatGuideIntent(should_find_seat=False, normalized_text="")

    english_seat_words = ("seat", "chair", "place to sit", "empty place")
    english_find_words = ("find", "look for", "take me", "guide me", "show me")
    chinese_seat_words = ("座位", "椅子", "空位", "位置")
    chinese_find_words = ("找", "带我", "帮我", "引导", "去")

    should_find_seat = (
        any(word in normalized for word in english_seat_words)
        and any(word in normalized for word in english_find_words)
    ) or (
        any(word in normalized for word in chinese_seat_words)
        and any(word in normalized for word in chinese_find_words)
    )
    return SeatGuideIntent(should_find_seat=should_find_seat, normalized_text=normalized)


def is_seat_guide_preview_request(text: str) -> bool:
    normalized = text.casefold()
    preview_words = (
        "preview",
        "preflight",
        "dry run",
        "test",
        "check",
        "预检",
        "测试",
        "先看",
        "检查",
        "不要动",
        "别动",
    )
    return parse_seat_guide_intent(text).should_find_seat and any(
        word in normalized for word in preview_words
    )
