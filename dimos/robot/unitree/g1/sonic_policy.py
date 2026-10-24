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

"""SONIC policy module: walking intent in, motor targets out."""

from __future__ import annotations

import asyncio
from typing import Any, ClassVar

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.g1.sonic_controller import SonicController, SonicControllerConfig
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons


class G1SonicPolicyConfig(ModuleConfig, SonicControllerConfig):
    pass


class G1SonicPolicyModule(Module):
    """Walk through Twist commands and condition SONIC with arm references.

    SonicController owns the policy loop and activation lifecycle. Hardware
    motor delivery remains in G1WholeBodyConnection. MovementManager selects
    navigation or teleoperation input before it reaches base_command.
    """

    config: G1SonicPolicyConfig
    dedicated_worker: ClassVar[bool] = True

    base_command: In[Twist]
    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]

    motor_states: In[JointState]
    low_level_imu: In[Imu]
    motor_command: Out[MotorCommandArray]
    controller_fault: Out[String]
    g1_fault: In[String]
    body_tracking: In[BodyTrackingSnapshot]
    teleop_buttons: In[Buttons]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._controller = SonicController(
            self.config,
            publish_joint_state=lambda msg: self.joint_state.publish(msg),
            publish_imu=lambda msg: self.imu.publish(msg),
            publish_motor_command=lambda msg: self.motor_command.publish(msg),
            publish_fault=lambda msg: self.controller_fault.publish(msg),
        )

    @rpc
    def start(self) -> None:
        if not self._controller.start():
            return
        try:
            super().start()
        except Exception:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        try:
            self._controller.stop()
        finally:
            super().stop()

    async def handle_base_command(self, msg: Twist) -> None:
        self.set_velocity_command(float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    async def handle_position_command(self, msg: JointState) -> None:
        await asyncio.to_thread(self._controller.set_joint_reference, msg)

    async def handle_motor_states(self, msg: JointState) -> None:
        self._controller.update_motor_states(msg)

    async def handle_low_level_imu(self, msg: Imu) -> None:
        self._controller.update_imu(msg)

    async def handle_g1_fault(self, msg: String) -> None:
        self._controller.latch_fault(msg.data)

    async def handle_body_tracking(self, msg: BodyTrackingSnapshot) -> None:
        await asyncio.to_thread(self._controller.update_body_tracking, msg)

    async def handle_teleop_buttons(self, msg: Buttons) -> None:
        await asyncio.to_thread(self._controller.update_teleop_buttons, msg)

    @rpc
    def activate(self, ramp_seconds: float | None = None) -> bool:
        """Activate live balancing through the pose ramp, with fresh feedback required."""
        return self._controller.activate(ramp_seconds)

    @rpc
    def stop_motion(self) -> None:
        """Stop clips and operator references, keeping balance. Cancel navigation separately."""
        self._controller.stop_motion()

    @rpc
    def arm(self, ramp_seconds: float | None = None) -> bool:
        """Ramp from the measured pose to the default pose, then start balancing."""
        return self._controller.arm(ramp_seconds)

    @rpc
    def disarm(self) -> bool:
        """Return to measured-pose hold; the balancing policy stops."""
        return self._controller.disarm()

    @rpc
    def set_dry_run(self, enabled: bool) -> None:
        """Run inference while holding the prepared pose when enabled."""
        self._controller.set_dry_run(enabled)

    @rpc
    def set_estop(self, estopped: bool) -> None:
        """Latch a damping stop; button release and runtime resets cannot clear it."""
        self._controller.set_estop(estopped)

    @rpc
    def halt_robot(self) -> bool:
        """Latch the emergency damping stop; recovery requires restarting the stack."""
        self._controller.set_estop(True)
        return True

    @rpc
    def status(self) -> dict[str, Any]:
        """Policy lifecycle, fault and timing status."""
        return self._controller.status()

    @rpc
    def set_velocity_command(
        self, vx: float, vy: float, yaw_rate: float, t_now: float | None = None
    ) -> None:
        """Set forward/lateral velocity and yaw rate; expires after the command timeout."""
        self._controller.set_velocity_command(vx, vy, yaw_rate, t_now)

    @rpc
    def set_locomotion_mode(self, mode: int | str | None) -> dict[str, Any]:
        """Select a supported GEAR locomotion mode; None restores SLOW_WALK."""
        return self._controller.set_locomotion_mode(mode)

    @rpc
    def list_locomotion_modes(self) -> dict[str, int]:
        """List the supported gait names and their numeric IDs."""
        return self._controller.list_locomotion_modes()

    @rpc
    def set_base_height(self, height: float) -> None:
        """Set the desired pelvis height in meters."""
        self._controller.set_base_height(height)

    @rpc
    def set_upper_body(self, positions: list[float]) -> bool:
        """14 arm-joint encoder hints, DDS order (indices 15-28)."""
        return self._controller.set_upper_body(positions)

    @rpc
    def clear_upper_body(self) -> None:
        """Restore the default arm reference."""
        self._controller.clear_upper_body()

    @rpc
    def play_motion_clip(self, name: str) -> dict[str, Any]:
        """Play a reference motion clip from the sonic data dir by name.

        Clips are 50 Hz CSVs in SONIC's reference layout (joint_pos.csv,
        joint_vel.csv, body_quat.csv - IsaacLab joint order, header row).
        """
        return self._controller.play_motion_clip(name)

    @rpc
    def stop_motion_clip(self) -> bool:
        """Return to the locomotion planner."""
        return self._controller.stop_motion_clip()

    @rpc
    def list_motion_clips(self) -> list[str]:
        """List installed reference motion clips."""
        return self._controller.list_motion_clips()

    @rpc
    def set_vr_3point(
        self,
        positions: list[float],
        orientations: list[float],
        t_now: float | None = None,
    ) -> dict[str, Any]:
        """VR 3-point teleop targets (SONIC encoder mode 1).

        positions: 9 floats - [left wrist, right wrist, head] xyz, root-relative
        (world minus pelvis, rotated into the pelvis frame). orientations: 12
        floats - the same three points as quat wxyz, root-relative
        (quat_inv(root) * q_world). The C++ deploy stack's wrist offsets
        [0.18, -/+0.025, 0] and head offset [0, 0, 0.35] must already be
        applied by the caller. Targets are encoder HINTS through the policy
        latent - expect coordinated whole-body following, not servo-accurate
        end-effector tracking. Stale data (> 0.5 s) reverts to planner obs;
        re-send at teleop rate.
        """
        return self._controller.set_vr_3point(positions, orientations, t_now)

    @rpc
    def clear_vr_3point(self) -> bool:
        """Clear wrist/head hints and return to the planner observations."""
        return self._controller.clear_vr_3point()

    @rpc
    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        """Clear commands and policy history; optionally repeat the pose ramp."""
        return self._controller.reset_runtime_state(reactivate)

    @rpc
    def state_snapshot(self) -> dict[str, Any]:
        """Alias for status()."""
        return self._controller.status()
