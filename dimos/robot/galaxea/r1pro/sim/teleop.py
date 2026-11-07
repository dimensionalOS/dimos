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

"""R1 simulation binding for the existing WebXR, coordinator and Pink IK path."""

from pydantic import Field

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.stream import In, Out
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.sim.config import MODEL
from dimos.robot.galaxea.r1pro.sim.model import FINGER_JOINTS, STEER_JOINTS, WHEEL_JOINTS
from dimos.teleop.webxr.controller_types import Hand
from dimos.teleop.webxr.extensions import VideoArmTeleopConfig, VideoArmTeleopModule

# The coordinator owns the upper body; per-arm IK below uses shoulder subtrees
# so a nonzero torso pose cannot be silently replaced with a neutral torso.
TELEOP_MODEL = MODEL.with_fixed_joints(
    *STEER_JOINTS, *WHEEL_JOINTS, *FINGER_JOINTS
).with_default_joint_acceleration_limit(2.0)


def arm_tasks() -> list[TaskConfig]:
    """Independent grip/clutch sessions, using the same tasks as dual OpenYAM."""
    return [
        TaskConfig(
            name=f"teleop_{side}",
            type="teleop_ik",
            joint_names=[f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)],
            priority=20,
            params={
                "robot_model": RobotModelConfig(
                    model=MODEL.with_subtree_rooted_at(f"r1pro/{side}_arm_base_link")
                    .with_fixed_joints(*(f"r1pro/{side}_gripper_finger_joint{i}" for i in (1, 2)))
                    .with_default_joint_acceleration_limit(2.0),
                    joint_names=[f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)],
                    base_link=f"r1pro/{side}_arm_base_link",
                ),
                "bindings": [{"hand": side, "target_frame": f"r1pro/{side}_gripper_link"}],
                "pink": PinkKinematicsConfig(
                    # ProxQP falsely reports infeasibility for feasible R1
                    # differential steps, including with preconditioning off.
                    solver="quadprog",
                    dt=0.02,
                    position_cost=8.0,
                    orientation_cost=2.0,
                    posture_cost=0.01,
                    joint_limit_posture_margin=0.3,
                    lm_damping=0.01,
                    gain=1.0,
                ),
                "timeout": 0.3,
                "max_joint_velocity_rad_s": 0.5,
                "max_command_tracking_error_deg": 10.0,
                # MuJoCo soft stops can overshoot by a few milliradians. Permit
                # recovery, while commanding 0.03 rad inside physical limits.
                "feedback_limit_tolerance": 0.01,
                "command_limit_margin": 0.03,
                "joint_command_filter_cutoff_hz": 10.0,
            },
        )
        for side in ("left", "right")
    ]


class R1ProTeleopCoordinator(TeleopControlCoordinator):
    """Use the coordinator's standard connection ports instead of hardware adapters."""

    dedicated_worker = True
    r1pro_joint_state: In[JointState]
    r1pro_position_command: Out[JointState]


class R1ProWebXRConfig(VideoArmTeleopConfig):
    linear_speed: float = Field(default=0.2, gt=0, le=0.4)
    angular_speed: float = Field(default=0.3, gt=0, le=0.5)
    deadzone: float = Field(default=0.15, ge=0, lt=1)
    input_timeout_s: float = Field(default=0.3, gt=0)


class R1ProWebXR(VideoArmTeleopModule):
    """OpenYAM arm input and camera interface, plus holonomic thumbstick driving.

    Left stick: forward/back and strafe. Right stick X: yaw. Releasing sticks,
    stale input or disconnect sends zero velocity. Grip independently clutches
    each arm in the coordinator. Finger actuation is not modeled in this demo.
    """

    config: R1ProWebXRConfig
    dedicated_worker = True
    cmd_vel: Out[Twist]

    def _publish_safe_command(self) -> None:
        self.cmd_vel.publish(Twist.zero())

    def _on_joy_bytes(self, data: bytes) -> bool:
        try:
            valid = super()._on_joy_bytes(data)
        except ValueError:
            self._publish_safe_command()
            raise
        if not valid:
            self._publish_safe_command()
            return False
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
            twist = Twist.zero()
            if left is not None:
                twist.linear.x = -self._axis(left.thumbstick.y) * self.config.linear_speed
                twist.linear.y = -self._axis(left.thumbstick.x) * self.config.linear_speed
            if right is not None:
                twist.angular.z = -self._axis(right.thumbstick.x) * self.config.angular_speed
            self.cmd_vel.publish(twist)
        return True

    def _axis(self, value: float) -> float:
        return 0.0 if abs(value) < self.config.deadzone else max(-1.0, min(1.0, value))
