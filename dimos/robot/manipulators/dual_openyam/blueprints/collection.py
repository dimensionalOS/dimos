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

"""What a dual OpenYAM collection runs and records: teleop tuning, cameras, recorder."""

import threading
from typing import Any

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    build_dual_openyam_webxr,
    dual_openyam_webxr_task,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)
from dimos.teleop.webxr.controller_types import BUTTON_ALIASES, Buttons, Hand
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DUAL_OPENYAM_SIDES = ("left", "right")

DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S = 1.2
DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S = 0.6
DUAL_OPENYAM_COLLECTION_MOTION_SCALE = 0.75


def scale_controller_pose(pose: PoseStamped, scale: float) -> PoseStamped:
    """The same pose with its position scaled about the headset origin."""
    return PoseStamped(
        position=pose.position * scale,
        orientation=pose.orientation,
        ts=pose.ts,
        frame_id=pose.frame_id,
    )


class DualOpenYamCollectionTeleopModule(ArmTeleopModule):
    """ArmTeleopModule whose controller positions are scaled for a slower arm."""

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        pose = super()._get_output_pose(hand)
        if pose is None:
            return None
        return scale_controller_pose(pose, DUAL_OPENYAM_COLLECTION_MOTION_SCALE)


def dual_openyam_collection_teleop_task() -> TaskConfig:
    """The stock dual-arm teleop task at the collection speed ceilings."""
    return dual_openyam_webxr_task(
        max_joint_velocity_rad_s=DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
        joint_velocity_limits_rad_s={
            DUAL_OPENYAM_LEFT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
            DUAL_OPENYAM_RIGHT_ARM_JOINTS[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
        },
    )


teleop_webxr_dual_openyam_collection = autoconnect(
    build_dual_openyam_webxr(
        teleop_task=dual_openyam_collection_teleop_task(),
        teleop_module=DualOpenYamCollectionTeleopModule,
    )
)

# Override with --left-wrist-camera.serial-number / --right-wrist-camera.serial-number.
DUAL_OPENYAM_WRIST_CAMERA_SERIALS = {"left": "260322272983", "right": "260322276650"}

DUAL_OPENYAM_COLLECTION_STREAMS = [
    "color_image",
    "coordinator_joint_state",
    "coordinator_joint_command",
    "status",
    "left_wrist_color_image",
    "right_wrist_color_image",
    "left_cartesian_command",
    "right_cartesian_command",
    "left_gripper_command",
    "right_gripper_command",
    "teleop_buttons",
]


class DualOpenYamCollectionRecorder(CollectionRecorder):
    """The collection streams plus commands, wrist views and buttons."""

    coordinator_joint_command: In[JointState]

    left_wrist_color_image: In[Image]
    right_wrist_color_image: In[Image]
    left_cartesian_command: In[PoseStamped]
    right_cartesian_command: In[PoseStamped]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]
    teleop_buttons: In[Buttons]


DUAL_OPENYAM_PLANNING_GROUPS = ("left_manipulator", "right_manipulator")

# left_joint1..6, right_joint1..6; X redefines it at runtime.
DUAL_OPENYAM_COLLECTION_HOME_JOINTS = [
    -0.127, 0.560, 0.489, -0.574, -0.010, -0.038,
    -0.347, 0.695, 0.335, 0.017, -0.776, 0.144,
]  # fmt: skip


def dual_openyam_home_targets(home_joints: list[float]) -> dict[str, JointState]:
    """One joint target per planning group from a 12-value home pose."""
    if len(home_joints) != len(DUAL_OPENYAM_ARM_JOINTS):
        raise ValueError(f"home pose needs {len(DUAL_OPENYAM_ARM_JOINTS)} joints")
    per_arm = len(DUAL_OPENYAM_LEFT_ARM_JOINTS)
    return {
        group: JointState(
            name=list(DUAL_OPENYAM_ARM_JOINTS[i * per_arm : (i + 1) * per_arm]),
            position=list(home_joints[i * per_arm : (i + 1) * per_arm]),
        )
        for i, group in enumerate(DUAL_OPENYAM_PLANNING_GROUPS)
    }


class DualOpenYamHomeButtonConfig(ModuleConfig):
    home_button: str = "A"
    set_home_button: str = "X"
    speed_scale: float = Field(default=0.2, gt=0.0, le=1.0)
    # A zero-length path fails trajectory parametrization, so skip when already home.
    at_home_tolerance_rad: float = Field(default=0.02, gt=0.0)
    home_joints: list[float] = Field(
        default_factory=lambda: list(DUAL_OPENYAM_COLLECTION_HOME_JOINTS)
    )

    @field_validator("home_button", "set_home_button")
    @classmethod
    def _known_button(cls, name: str) -> str:
        if BUTTON_ALIASES.get(name, name) not in Buttons.BITS:
            raise ValueError(f"unknown Quest button {name!r}")
        return name

    @field_validator("home_joints")
    @classmethod
    def _twelve_joints(cls, value: list[float]) -> list[float]:
        if len(value) != len(DUAL_OPENYAM_ARM_JOINTS):
            raise ValueError(f"home_joints needs {len(DUAL_OPENYAM_ARM_JOINTS)} values")
        return value


class DualOpenYamHomeButtonModule(Module):
    """Quest face buttons that park the arms during collection."""

    config: DualOpenYamHomeButtonConfig

    button_pressed: In[Buttons]
    teleop_buttons: In[Buttons]
    coordinator_joint_state: In[JointState]

    _manipulation: ManipulationModule

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._grip_held = False
        self._latest_joints: JointState | None = None
        self._home_joints = list(self.config.home_joints)
        self._worker: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_pressed)))
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def home_joints(self) -> list[float]:
        with self._lock:
            return list(self._home_joints)

    def _on_buttons(self, msg: Buttons) -> None:
        with self._lock:
            self._grip_held = bool(msg.left_grip or msg.right_grip)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joints = msg

    def _on_pressed(self, msg: Buttons) -> None:
        if getattr(
            msg, BUTTON_ALIASES.get(self.config.set_home_button, self.config.set_home_button)
        ):
            self._set_home_to_current()
        if getattr(msg, BUTTON_ALIASES.get(self.config.home_button, self.config.home_button)):
            self._go_home()

    def _set_home_to_current(self) -> None:
        with self._lock:
            latest = self._latest_joints
            if latest is None:
                logger.warning("home: no joint state yet, home pose unchanged")
                return
            by_name = dict(zip(latest.name, latest.position, strict=False))
            missing = [n for n in DUAL_OPENYAM_ARM_JOINTS if n not in by_name]
            if missing:
                logger.warning("home: joint state lacks arm joints", missing=missing)
                return
            self._home_joints = [float(by_name[n]) for n in DUAL_OPENYAM_ARM_JOINTS]
            home = list(self._home_joints)
        logger.info("home: pose set to current", joints=[round(v, 3) for v in home])

    def _go_home(self) -> None:
        with self._lock:
            if self._grip_held:
                logger.warning("home: release both grips first")
                return
            if self._worker is not None and self._worker.is_alive():
                logger.warning("home: a parking move is already running")
                return
            distance = self._distance_to_home_locked()
            if distance is not None and distance < self.config.at_home_tolerance_rad:
                logger.info("home: already there", distance_rad=round(distance, 3))
                return
            targets = dual_openyam_home_targets(self._home_joints)
            self._worker = threading.Thread(
                target=self._plan_and_execute, args=(targets,), daemon=True, name="openyam-home"
            )
            self._worker.start()

    def _distance_to_home_locked(self) -> float | None:
        """Largest joint error to the home pose, or None without a joint state."""
        if self._latest_joints is None:
            return None
        by_name = dict(zip(self._latest_joints.name, self._latest_joints.position, strict=False))
        if any(n not in by_name for n in DUAL_OPENYAM_ARM_JOINTS):
            return None
        return max(
            abs(float(by_name[n]) - h)
            for n, h in zip(DUAL_OPENYAM_ARM_JOINTS, self._home_joints, strict=True)
        )

    def _plan_and_execute(self, targets: dict[str, JointState]) -> None:
        # RPCs block, so they run off the subscriber thread.
        result = self._manipulation.plan_to_joints(targets, speed_scale=self.config.speed_scale)
        if not result.succeeded:
            logger.warning("home: planning failed", message=result.message)
            return
        execution = self._manipulation.execute(blocking=False)
        logger.info(
            "home: moving both arms",
            speed_scale=self.config.speed_scale,
            status=str(execution.status),
            message=execution.message,
        )


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """A wrist D405 with ``{side}_wrist_`` stream names; colour only, the D405s share USB 2 hubs."""
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    name = f"{side}_wrist_camera"
    camera = RealSenseCamera.blueprint(
        instance_name=name,
        frame_id_prefix=f"{side}_wrist",
        width=640,
        height=480,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
        enable_pointcloud=False,
        serial_number=DUAL_OPENYAM_WRIST_CAMERA_SERIALS[side],
    )
    streams = [stream.name for stream in camera.blueprints[0].streams]
    return camera.remappings([(name, stream, f"{side}_wrist_{stream}") for stream in streams])
