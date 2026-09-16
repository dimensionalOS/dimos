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

"""OmniGibson adapter. All methods run on the subprocess main thread."""

from collections.abc import Iterator
import math
from pathlib import Path
import random
import re
from typing import Any, cast

import numpy as np
import omnigibson as og
from omnigibson.action_primitives.starter_semantic_action_primitives import (
    StarterSemanticActionPrimitives,
    StarterSemanticActionPrimitiveSet,
)
from omnigibson.action_primitives.symbolic_semantic_action_primitives import (
    SymbolicSemanticActionPrimitives,
    SymbolicSemanticActionPrimitiveSet,
)
from omnigibson.controllers import ControllerView
from omnigibson.macros import gm
from omnigibson.object_states.object_state_base import AbsoluteObjectState, BooleanStateMixin
from omnigibson.sensors.vision_sensor import VisionSensor
from omnigibson.tasks.behavior_task import BehaviorTask
from scipy.spatial.transform import Rotation
import torch
import yaml

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.behavior.connection import BehaviorConfig
from dimos.simulation.behavior.control import RobotControl
from dimos.simulation.behavior.types import ControlMode, TaskSelection

PHYSICAL = ("GRASP", "PLACE_ON_TOP", "PLACE_INSIDE", "NAVIGATE_TO", "RELEASE")
SYMBOLIC = tuple(item.name for item in SymbolicSemanticActionPrimitiveSet)


def cpu(value: Any) -> Any:
    """Detach tensors before values cross the process boundary."""
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy().copy()
    if isinstance(value, dict):
        return {str(k): cpu(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [cpu(v) for v in value]
    return value


def plain(value: Any) -> Any:
    value = cpu(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


class OmniEngine:
    def __init__(self, config: BehaviorConfig) -> None:
        self.config = config
        self._goal_status: dict[str, Any] = {}

    def initialize(self) -> None:
        config = self.config
        gm.HEADLESS = config.headless
        random.seed(config.seed)
        np.random.seed(config.seed)
        torch.manual_seed(config.seed)
        self.env = og.Environment(configs=self._configuration(config.task))
        self.task = config.task
        self._bind_robot()

    def _configuration(self, task: TaskSelection | None) -> dict[str, Any]:
        with (Path(og.example_config_path) / "r1_primitives.yaml").open() as stream:
            config: dict[str, Any] = yaml.safe_load(stream)
        config["env"].update(
            action_frequency=self.config.action_hz,
            physics_frequency=self.config.physics_hz,
            rendering_frequency=self.config.action_hz,
            automatic_reset=False,
        )
        config["scene"]["scene_model"] = task.scene if task else self.config.scene
        robot = config["robots"][0]
        robot["name"] = "r1"
        robot["obs_modalities"] = ["rgb", "depth_linear", "proprio"]
        if self.config.publish_semantic:
            robot["obs_modalities"].append("seg_semantic")
        robot["sensor_config"]["VisionSensor"]["sensor_kwargs"].update(
            image_width=self.config.image_width,
            image_height=self.config.image_height,
        )
        if task is not None:
            config["scene"].update(
                load_task_relevant_only=True, not_load_object_categories=["ceilings"]
            )
            config["task"] = {
                "type": "BehaviorTask",
                "activity_name": task.activity,
                "activity_definition_id": task.definition,
                "activity_instance_id": task.instance,
                "online_object_sampling": False,
                "termination_config": {"max_steps": self.config.max_episode_steps},
            }
        return config

    def _bind_robot(self) -> None:
        self.robot = self.env.robots[0]
        self.names = list(self.robot.joints)
        self.controllers = dict(self.robot.controllers)
        self.indices = {
            name: cpu(ControllerView.get_dof_idx(key)).astype(int)
            for name, (key, _) in self.controllers.items()
        }
        lower, upper = cpu(self.robot.joint_lower_limits), cpu(self.robot.joint_upper_limits)
        controllable = set(
            int(i) for name, idx in self.indices.items() if name != "base" for i in idx
        )
        self.limits = {
            name: (float(lower[i]), float(upper[i]))
            for i, name in enumerate(self.names)
            if i in controllable
        }
        self.bounds = list(
            zip(
                plain(self.robot.action_space.low), plain(self.robot.action_space.high), strict=True
            )
        )
        self.cameras: dict[str, Any] = {}
        for name, sensor in self.robot.sensors.items():
            if not isinstance(sensor, VisionSensor):
                continue
            slot = (
                "left_wrist"
                if "left" in name.lower()
                else "right_wrist"
                if "right" in name.lower()
                else "head"
            )
            if slot in self.cameras:
                raise ValueError(f"Ambiguous R1 {slot} camera: {name}")
            self.cameras[slot] = sensor
        if set(self.cameras) != {"head", "left_wrist", "right_wrist"}:
            raise ValueError(f"Expected R1 head and wrist cameras, got {list(self.cameras)}")
        # Attach all calibration annotators before rendering. Lazy attachment while
        # publishing the first frame can return an uninitialized projection matrix.
        for sensor in self.cameras.values():
            sensor.initialize_sensors(names="camera_params")
        # Task reset changes the robot pose. A simulation step propagates its
        # cameras into Fabric; render-only updates can leave calibration empty.
        og.sim.step()
        for _ in range(4):  # OmniGibson's documented annotator activation interval.
            og.sim.render()
        self.intrinsics = {
            slot: plain(sensor.intrinsic_matrix) for slot, sensor in self.cameras.items()
        }
        self.physical: Any = None
        self.symbolic: Any = None

    def describe(self) -> dict[str, Any]:
        return {
            "robot": "R1",
            "action_hz": self.config.action_hz,
            "max_episode_steps": self.config.max_episode_steps,
            "joint_names": self.names,
            "joint_limits": self.limits,
            "cameras": {slot: sensor.name for slot, sensor in self.cameras.items()},
            "joint_groups": {
                name: [self.names[i] for i in indices] for name, indices in self.indices.items()
            },
            "action_layout": plain(self.robot.controller_action_idx),
            "action_bounds": self.bounds,
            "physical_primitives": list(PHYSICAL),
            "symbolic_primitives": list(SYMBOLIC),
            "unsupported_physical": ["OPEN", "CLOSE", "TOGGLE_ON", "TOGGLE_OFF"],
            "primitive_arm": self.robot.default_arm,
            "grasping_mode": self.robot.grasping_mode,
        }

    def list_tasks(self) -> list[TaskSelection]:
        result = []
        for file in Path(gm.DATA_PATH).glob("*/scenes/*/json/*_task_*_template.json"):
            match = re.search(r"_task_(.+)_(\d+)_(\d+)_template$", file.stem)
            if match:
                result.append(
                    TaskSelection(
                        scene=file.parent.parent.name,
                        activity=match[1],
                        definition=int(match[2]),
                        instance=int(match[3]),
                    )
                )
        return sorted(result, key=lambda t: (t.scene, t.activity, t.definition, t.instance))

    def list_scenes(self) -> list[str]:
        return sorted({p.name for p in Path(gm.DATA_PATH).glob("*/scenes/*") if p.is_dir()})

    def measured(self) -> dict[str, float]:
        return dict(
            zip(self.names, (float(v) for v in cpu(self.robot.get_joint_positions())), strict=True)
        )

    def reset(self, task: TaskSelection | None = None) -> None:
        self._goal_status = {}
        if task is not None:
            # Clear the previous scene before reload so no objects or robot controllers survive.
            og.clear()
            self.env = og.Environment(configs=self._configuration(task))
            self.task = task
        else:
            self.env.reset()
        self._bind_robot()

    def action(self, control: RobotControl, now: float) -> Any:
        if control.mode == ControlMode.NATIVE:
            native = control.get_action(now)
            if native is not None:
                return torch.tensor(native, dtype=torch.float32)
        action = torch.zeros(self.robot.action_dim, dtype=torch.float32)
        for name, (key, index) in self.controllers.items():
            values = ControllerView.compute_no_op_action(key, index)
            if name != "base":
                values = torch.tensor(
                    [control.targets[self.names[i]] for i in self.indices[name]],
                    dtype=torch.float32,
                )
            elif name == "base" and control.mode == ControlMode.DIMOS:
                # The upstream position-mode holonomic controller expects body-frame deltas.
                values = (
                    torch.tensor(control.get_velocity(now), dtype=torch.float32)
                    / self.config.action_hz
                )
            action[self.robot.controller_action_idx[name]] = values
        return action

    def primitive(self, kind: str, name: str, target: str) -> Iterator[Any]:
        supported = PHYSICAL if kind == "physical" else SYMBOLIC
        if kind not in ("physical", "symbolic") or name not in supported:
            raise ValueError(f"Unsupported {kind} primitive: {name}")
        obj = None
        if name != "RELEASE":
            scope = self.env.task.object_scope if isinstance(self.env.task, BehaviorTask) else {}
            obj = scope.get(target)
            if obj is None:
                obj = self.env.scene.object_registry("name", target)
            if obj is None:
                raise ValueError(f"Unknown object in this episode: {target}")
        if kind == "physical":
            if self.physical is None:
                self.physical = StarterSemanticActionPrimitives(
                    self.env, self.robot, enable_head_tracking=False
                )
            controller, primitive = self.physical, StarterSemanticActionPrimitiveSet[name]
        else:
            if self.symbolic is None:
                self.symbolic = SymbolicSemanticActionPrimitives(self.env, self.robot)
            controller, primitive = self.symbolic, SymbolicSemanticActionPrimitiveSet[name]
        return cast(
            "Iterator[Any]", controller.apply_ref(primitive, *([] if name == "RELEASE" else [obj]))
        )

    def step(self, action: Any) -> tuple[float, bool, bool, dict[str, Any]]:
        _, reward, terminated, truncated, info = self.env.step(action)
        self._goal_status = plain(info.get("done", {}).get("goal_status", {}))
        return float(reward), bool(terminated), bool(truncated), plain(info)

    def ground_truth(self) -> dict[str, Any]:
        objects = {}
        scope = (
            self.env.task.object_scope
            if isinstance(self.env.task, BehaviorTask)
            else {o.name: o for o in self.env.scene.objects}
        )
        for key, obj in scope.items():
            if obj is None or not hasattr(obj, "get_position_orientation"):
                continue  # Task scopes also contain particle systems.
            pos, quat = obj.get_position_orientation()
            objects[key] = {
                "name": obj.name,
                "category": obj.category,
                "position": plain(pos),
                "orientation": plain(quat),
                "states": {
                    type(state).__name__: bool(state.get_value())
                    for state in obj.states.values()
                    if isinstance(state, AbsoluteObjectState)
                    and isinstance(state, BooleanStateMixin)
                },
            }
        return {
            "objects": objects,
            # The evaluator reports goals through step(); task.info rejects pre-step reads.
            "goal_status": self._goal_status,
            "goal_description": self.env.task.activity_natural_language_goal_conditions
            if isinstance(self.env.task, BehaviorTask)
            else [],
        }

    def observation(self) -> dict[str, Any]:
        obs, info = self.env.get_obs()
        return {"obs": cpu(obs), "info": cpu(info)}

    def messages(self, ts: float) -> dict[str, Any]:
        pos, quat = self.robot.get_position_orientation()
        p, q = plain(pos), plain(quat)
        base_rotation = Rotation.from_quat(q)
        linear = base_rotation.inv().apply(plain(self.robot.get_linear_velocity()))
        angular = base_rotation.inv().apply(plain(self.robot.get_angular_velocity()))
        messages: dict[str, Any] = {
            "joint_state": JointState(
                ts=ts,
                frame_id="base_link",
                name=self.names,
                position=plain(self.robot.get_joint_positions()),
                velocity=plain(self.robot.get_joint_velocities()),
            ),
            "odometry": Odometry(
                ts=ts,
                frame_id="world",
                child_frame_id="base_link",
                pose=Pose(position=Vector3(*p), orientation=Quaternion(*q)),
                twist=Twist(linear=linear.tolist(), angular=angular.tolist()),
            ),
        }
        transforms = [
            Transform(
                translation=Vector3(*p),
                rotation=Quaternion(*q),
                frame_id="world",
                child_frame_id="base_link",
                ts=ts,
            )
        ]
        base_rotation = Rotation.from_quat(q)
        for slot, sensor in self.cameras.items():
            data, _ = sensor.get_obs()
            frame = "camera_optical" if slot == "head" else f"{slot}_optical"
            prefix = "" if slot == "head" else f"{slot}_"
            rgb = Image(cpu(data["rgb"])[..., :3].astype(np.uint8), ImageFormat.RGB, frame, ts)
            depth = Image(
                cpu(data["depth_linear"]).astype(np.float32), ImageFormat.DEPTH, frame, ts
            )
            k = self.intrinsics[slot]
            calibration = CameraInfo(
                width=rgb.data.shape[1],
                height=rgb.data.shape[0],
                K=np.asarray(k).reshape(-1).tolist(),
                frame_id=frame,
                ts=ts,
            )
            messages["color_image" if slot == "head" else f"{prefix}image"] = rgb
            messages["depth_image" if slot == "head" else f"{prefix}depth"] = depth
            messages[f"{prefix}camera_info"] = calibration
            camera_pos, camera_quat = sensor.get_position_orientation()
            optical_rotation = Rotation.from_quat(plain(camera_quat)) * Rotation.from_euler(
                "x", math.pi
            )
            translation = base_rotation.inv().apply(np.asarray(plain(camera_pos)) - np.asarray(p))
            rotation = (base_rotation.inv() * optical_rotation).as_quat()
            transforms.append(
                Transform(
                    translation=Vector3(*translation),
                    rotation=Quaternion(*rotation),
                    frame_id="base_link",
                    child_frame_id=frame,
                    ts=ts,
                )
            )
            if slot == "head":
                if self.config.publish_scan:
                    messages["registered_scan"] = PointCloud2.from_rgbd(
                        rgb, depth, calibration, depth_trunc=self.config.max_depth
                    )
                if self.config.publish_semantic:
                    # Semantic IDs are metadata, not an 8-bit color image.
                    messages["semantic_image"] = Image(
                        cpu(data["seg_semantic"]).astype(np.uint16), ImageFormat.GRAY16, frame, ts
                    )
        messages["tf"] = TFMessage(*transforms)
        return messages

    def close(self) -> None:
        og.shutdown(due_to_signal=True)
