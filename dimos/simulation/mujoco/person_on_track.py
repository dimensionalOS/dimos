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

from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Pose


class PersonPositionController:
    """Controls the person position in MuJoCo by subscribing to LCM pose updates."""

    def __init__(self, model: mujoco.MjModel) -> None:
        person_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "person")
        self._person_mocap_id = model.body_mocapid[person_body_id]
        self._latest_pose: Pose | None = None
        self._transport: LCMTransport[Pose] = LCMTransport("/person_pose", Pose)
        self._transport.subscribe(self._on_pose)

    def _on_pose(self, pose: Pose) -> None:
        self._latest_pose = pose

    def tick(self, data: mujoco.MjData) -> None:
        if self._latest_pose is None:
            return

        pose = self._latest_pose
        data.mocap_pos[self._person_mocap_id][0] = pose.position.x
        data.mocap_pos[self._person_mocap_id][1] = pose.position.y
        data.mocap_pos[self._person_mocap_id][2] = pose.position.z
        data.mocap_quat[self._person_mocap_id] = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]

    def stop(self) -> None:
        self._transport.stop()


class PersonTrackPublisher:
    """Publishes person poses along a track via LCM."""

    def __init__(self, track: list[tuple[float, float]]) -> None:
        self._speed = 0.004
        self._waypoint_threshold = 0.1
        self._rotation_radius = 1.0
        self._track = track
        self._current_waypoint_idx = 0
        self._initialized = False
        self._current_pos = np.array([0.0, 0.0])
        self._transport: LCMTransport[Pose] = LCMTransport("/person_pose", Pose)

    def _get_segment_heading(self, from_idx: int, to_idx: int) -> float:
        """Get heading angle for traveling from one waypoint to another."""
        from_wp = np.array(self._track[from_idx])
        to_wp = np.array(self._track[to_idx])
        direction = to_wp - from_wp
        return float(np.arctan2(direction[1], direction[0]))

    def _lerp_angle(self, a1: float, a2: float, t: float) -> float:
        """Interpolate between two angles, handling wrapping."""
        diff = a2 - a1
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return a1 + diff * t

    def tick(self) -> None:
        if not self._initialized:
            first_point = self._track[0]
            self._current_pos = np.array([first_point[0], first_point[1]])
            self._current_waypoint_idx = 1
            heading = self._get_segment_heading(0, 1)
            self._publish_pose(self._current_pos, heading)
            self._initialized = True
            return

        n = len(self._track)

        prev_idx = (self._current_waypoint_idx - 1) % n
        curr_idx = self._current_waypoint_idx
        next_idx = (self._current_waypoint_idx + 1) % n
        prev_prev_idx = (prev_idx - 1) % n

        prev_wp = np.array(self._track[prev_idx])
        curr_wp = np.array(self._track[curr_idx])

        to_target = curr_wp - self._current_pos
        distance_to_curr = float(np.linalg.norm(to_target))
        distance_from_prev = float(np.linalg.norm(self._current_pos - prev_wp))

        # Headings for current turn (at curr_wp)
        incoming_heading = self._get_segment_heading(prev_idx, curr_idx)
        outgoing_heading = self._get_segment_heading(curr_idx, next_idx)

        # Headings for previous turn (at prev_wp)
        prev_incoming_heading = self._get_segment_heading(prev_prev_idx, prev_idx)
        prev_outgoing_heading = incoming_heading

        # Determine heading based on position in rotation zones
        in_leaving_zone = distance_from_prev < self._rotation_radius
        in_approaching_zone = distance_to_curr < self._rotation_radius

        if in_leaving_zone and in_approaching_zone:
            # Overlap - prioritize approaching zone
            t = 0.5 * (1.0 - distance_to_curr / self._rotation_radius)
            heading = self._lerp_angle(incoming_heading, outgoing_heading, t)
        elif in_leaving_zone:
            # Finishing turn after passing prev_wp (t goes from 0.5 to 1.0)
            t = 0.5 + 0.5 * (distance_from_prev / self._rotation_radius)
            heading = self._lerp_angle(prev_incoming_heading, prev_outgoing_heading, t)
        elif in_approaching_zone:
            # Starting turn before reaching curr_wp (t goes from 0.0 to 0.5)
            t = 0.5 * (1.0 - distance_to_curr / self._rotation_radius)
            heading = self._lerp_angle(incoming_heading, outgoing_heading, t)
        else:
            # Between zones, use segment heading
            heading = incoming_heading

        # Move toward target
        if distance_to_curr > 0:
            dir_norm = to_target / distance_to_curr
            self._current_pos[0] += dir_norm[0] * self._speed
            self._current_pos[1] += dir_norm[1] * self._speed

        # Check if reached waypoint
        if distance_to_curr < self._waypoint_threshold:
            self._current_waypoint_idx = next_idx

        # Publish pose
        self._publish_pose(self._current_pos, heading + np.pi)

    def _publish_pose(self, pos: NDArray[np.floating[Any]], heading: float) -> None:
        c, s = np.cos(heading / 2), np.sin(heading / 2)
        pose = Pose(
            position=[pos[0], pos[1], 0.0],
            orientation=[0.0, 0.0, s, c],  # x, y, z, w
        )
        self._transport.broadcast(None, pose)

    def stop(self) -> None:
        self._transport.stop()
