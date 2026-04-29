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

"""G1-aware ManipulationModule that does world↔pelvis frame translation.

Drake's IK is built around a stationary base — the URDF's ``base_link``
gets welded at ``base_pose`` (identity for us) and the planner solves
in that local frame.  G1's pelvis lives at whatever world position the
floating base settles at, not at the origin, so target poses the agent
expresses in MuJoCo-world coordinates need to be transformed into
pelvis-local coordinates before they reach the IK.

This subclass keeps the parent's @skill surface but pre-rotates the
incoming target by the inverse of the live ``/odom`` pose, and rotates
the EE pose Drake returns back into world frame for the agent.

Caveats still standing:
- Waist joints (yaw/roll/pitch) are owned by GR00T WBC and not
  reflected in Drake's model — Drake assumes waist=0.  Small error
  while standing still, larger if the WBC actively tilts the torso.
- Orientation overrides on ``move_to_pose`` are still passed through
  in pelvis-yaw frame; the user/agent should reason in
  pelvis-yaw-aligned coordinates if they specify roll/pitch/yaw.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1ManipulationModule(ManipulationModule):
    """ManipulationModule that uses /odom to put IK targets in world frame.

    Inherits every ``@skill`` method from ManipulationModule.  Only
    ``move_to_pose`` and ``get_robot_state`` are overridden because
    they're the ones that touch Cartesian space; ``move_to_joints``,
    ``go_home`` etc. work on raw joint values and don't care about
    the base frame.
    """

    odom: In[PoseStamped]

    _latest_odom: PoseStamped | None
    _odom_lock: threading.Lock

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_odom = None
        self._odom_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"G1ManipulationModule: odom subscribe failed: {e}")

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._odom_lock:
            self._latest_odom = msg

    # ------------------------------------------------------------------
    # Frame conversions
    # ------------------------------------------------------------------
    def _world_to_pelvis(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Translate a world-frame point into pelvis-local coordinates."""
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return (x, y, z)
        R = _quat_to_rotation(
            odom.orientation.w,
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
        )
        t = np.array([odom.position.x, odom.position.y, odom.position.z])
        p = np.array([x, y, z])
        p_local = R.T @ (p - t)
        return float(p_local[0]), float(p_local[1]), float(p_local[2])

    def _pelvis_to_world(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Translate a pelvis-local point into world coordinates."""
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return (x, y, z)
        R = _quat_to_rotation(
            odom.orientation.w,
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
        )
        t = np.array([odom.position.x, odom.position.y, odom.position.z])
        p = np.array([x, y, z])
        p_world = R @ p + t
        return float(p_world[0]), float(p_world[1]), float(p_world[2])

    # ------------------------------------------------------------------
    # Skill overrides
    # ------------------------------------------------------------------
    @skill
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> str:
        """Move the robot end-effector to a target pose **in world frame**.

        Coordinates are in the MuJoCo world frame (meters), not robot-local.
        For G1: the robot's pelvis is the floating base; this skill takes
        the live /odom and transforms (x, y, z) into pelvis-local before
        invoking IK.

        roll/pitch/yaw, if given, are interpreted in the pelvis's
        yaw-aligned frame for now (the planner doesn't know about the
        floating-base rotation otherwise).

        Args:
            x: Target X position in world frame (meters).
            y: Target Y position in world frame (meters).
            z: Target Z position in world frame (meters).
            roll: Optional target roll in pelvis yaw-frame (radians).
            pitch: Optional target pitch in pelvis yaw-frame (radians).
            yaw: Optional target yaw in pelvis yaw-frame (radians).
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        x_l, y_l, z_l = self._world_to_pelvis(x, y, z)
        logger.info(
            f"G1Manipulation move_to_pose: world=({x:.3f}, {y:.3f}, {z:.3f}) -> "
            f"pelvis=({x_l:.3f}, {y_l:.3f}, {z_l:.3f})"
        )
        return super().move_to_pose(  # type: ignore[no-any-return]
            x=x_l, y=y_l, z=z_l, roll=roll, pitch=pitch, yaw=yaw, robot_name=robot_name
        )

    @skill
    def get_robot_state(self, robot_name: str | None = None) -> str:
        """Get current robot state with EE pose **in world frame**.

        Joint positions are absolute (no transform).  EE pose is
        translated from pelvis-local (Drake's frame) to world via the
        live /odom.
        """
        lines: list[str] = []
        joints = self.get_current_joints(robot_name)
        if joints is not None:
            lines.append(f"Joints: [{', '.join(f'{j:.3f}' for j in joints)}]")
        else:
            lines.append("Joints: unavailable (no state received)")

        ee_pose = self.get_ee_pose(robot_name)
        if ee_pose is not None:
            wx, wy, wz = self._pelvis_to_world(
                ee_pose.position.x, ee_pose.position.y, ee_pose.position.z
            )
            lines.append(f"EE pose (world): ({wx:.4f}, {wy:.4f}, {wz:.4f})")
            lines.append(
                f"  pelvis-local: ({ee_pose.position.x:.4f}, "
                f"{ee_pose.position.y:.4f}, {ee_pose.position.z:.4f})"
            )
        else:
            lines.append("EE pose: unavailable")

        gripper_pos = self.get_gripper(robot_name)
        if gripper_pos is not None:
            lines.append(f"Gripper: {gripper_pos:.3f}m")
        else:
            lines.append("Gripper: not configured")

        state = self.get_state(robot_name)
        if state is not None:
            lines.append(f"State: {state.name}")

        return "\n".join(lines)


def _quat_to_rotation(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Quaternion (w, x, y, z) → 3x3 rotation matrix."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
