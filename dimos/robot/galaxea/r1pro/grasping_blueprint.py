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

"""DimOS ACT rollout for the R1Pro tabletop manipulation task."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any, ClassVar
from uuid import uuid4

import mujoco

from dimos.constants import RECORDINGS_DIR
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.hardware.spec import JointLimits
from dimos.imitation.policy.lerobot.module import R1ProPickPlacePolicy
from dimos.imitation.policy.module import (
    POLICY_ROLLOUT_INSTANCE_NAME,
    POLICY_ROLLOUT_TASK_NAME,
    _PolicyModule,
)
from dimos.imitation.policy.skills import PolicySkills
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS, prepare_grasping_scene
from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask, score_task
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS, R1PRO_PICK_PLACE_TASK
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion
from dimos.robot.galaxea.r1pro.tray_sim import configure_tray_holding
from dimos.robot.galaxea.r1pro.tray_task import laptop_destination, tray_state
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, SimCameraSpec
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec


class R1ProGraspingSim(MujocoSimModule):
    """Native simulation with wrist RGB and read-only task evidence for evaluation."""

    right_wrist: Out[Image]
    cargo_bodies: ClassVar[tuple[str, ...]] = ("task_bottle",)

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._initial_bottle_z: float | None = None
        self._peak_lift = 0.0
        self._bilateral_grasp = False
        self._transport_planner: PlanarTransport | None = None

    @rpc
    def is_simulation_running(self) -> bool:
        """Return false after the native viewer closes and its physics loop exits."""
        engine = self._engine
        return bool(engine and engine._sim_thread and engine._sim_thread.is_alive())

    @rpc
    def task_state(self) -> dict[str, Any]:
        """Score physics without feeding object coordinates into the ACT policy.

        Poll throughout a rollout to retain lift and fingertip-contact evidence.
        """
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            model, data = engine.model, engine.data
            free_tray = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "task_tray_free") >= 0
            if free_tray and self._transport_planner is None:
                self._transport_planner = PlanarTransport(
                    model, data, cargo_bodies=self.cargo_bodies
                )
            body = data.body("task_bottle")
            pos = body.xpos.copy()
            if self._initial_bottle_z is None:
                self._initial_bottle_z = float(pos[2])
            lift = float(pos[2]) - self._initial_bottle_z
            self._peak_lift = max(self._peak_lift, lift)
            pads = {model.geom(f"right_finger_pad{i}").id for i in (1, 2)}
            bottle = {model.geom(name).id for name in ("bottle_body", "bottle_cap")}
            touching = set()
            for contact in data.contact:
                geoms = set(map(int, contact.geom))
                if contact.dist <= 0 and geoms & bottle:
                    touching.update(geoms & pads)
            self._bilateral_grasp |= lift > 0.04 and touching == pads
            result = score_task(
                data,
                peak_lift=self._peak_lift,
                bilateral_grasp=self._bilateral_grasp,
                touching_pads=touching,
            )
            base = (
                {
                    "base_pose": [float(data.joint(n).qpos[0]) for n in VIRTUAL_BASE_JOINTS],
                    "base_velocity": [float(data.joint(n).qvel[0]) for n in VIRTUAL_BASE_JOINTS],
                }
                if self.config.dof > 20
                else {}
            )
            return {
                **result.to_dict(),
                **base,
                **(
                    {"tray": tray_state(model, data, cargo_bodies=self.cargo_bodies)}
                    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "task_tray_free") >= 0
                    else {}
                ),
                "robot_obstacles": self._transport_planner.collisions(data, ignore_cargo=True)
                if self._transport_planner
                else [],
                "gripper": float(data.joint("r1pro/right_gripper").qpos[0]),
                "sim_time": float(data.time),
                "obstacles": self._transport_planner.collisions(data)
                if self._transport_planner
                else [],
            }

    @rpc
    def plan_transport(self, x: float, y: float, yaw: float | None = None) -> list[list[float]]:
        """Plan a local collision-free planar path after ACT has loaded the tray."""
        engine = self._engine
        if engine is None or self.config.dof <= 20:
            raise RuntimeError("This scene has no movable planar base")
        with engine._lock:
            planner = PlanarTransport(engine.model, engine.data, cargo_bodies=self.cargo_bodies)
        path = planner.plan_delivery([x, y, yaw]) if yaw is not None else planner.plan((x, y))
        self._transport_planner = planner
        return path

    @rpc
    def simulation_snapshot(self) -> dict[str, Any]:
        """Read complete simulation state for replaying evaluation evidence."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            return {
                "qpos": engine.data.qpos.tolist(),
                "qvel": engine.data.qvel.tolist(),
                "ctrl": engine.data.ctrl.tolist(),
                "sim_time": float(engine.data.time),
                "actuator_biasprm": engine.model.actuator_biasprm.tolist(),
            }

    @rpc
    def prepare_tray_holding(self) -> None:
        """Apply loaded-torso damping and pause policy RGB after rollout stops."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        engine.set_camera_streaming_enabled(False)
        with engine._lock:
            configure_tray_holding(engine.model)

    @rpc
    def set_tray_delivery_view(self) -> None:
        """Use an elevated carrying view to clear cabinets beside the passage."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        self._engine.set_viewer_camera(azimuth=225.0, elevation=-75.0, distance=2.3)

    @rpc
    def tray_destination(self) -> dict[str, Any]:
        """Resolve the actual tabletop beside the laptop from house geometry."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            return laptop_destination(engine.model, engine.data).to_dict()

    @rpc
    def plan_tray_motion(
        self, phase: str, target: list[float] | None = None
    ) -> list[dict[str, Any]]:
        """Compute bimanual waypoints in a snapshot; execution belongs to the coordinator."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            snapshot = mujoco.MjData(engine.model)
            snapshot.qpos[:] = engine.data.qpos
            snapshot.ctrl[:] = engine.data.ctrl
            mujoco.mj_forward(engine.model, snapshot)
        motion = TrayMotion(engine.model, snapshot, cargo_bodies=self.cargo_bodies)
        if phase == "pickup":
            points = motion.pickup(snapshot)
        elif phase == "place" and target is not None and len(target) == 3:
            points = motion.placement(snapshot, target)
        else:
            raise ValueError("Expected pickup or place with a three-dimensional target")
        return [asdict(point) for point in points]


def build_r1pro_pick_place(
    *,
    scene_path: Path,
    artifact: str,
    device: str = "cuda",
    headless: bool = False,
) -> Blueprint:
    """Connect two cameras, twenty joint servos, and learned ACT in DimOS.

    The base stays parked during this manipulation task. The checkpoint must
    match R1PRO_PICK_PLACE_IO. Policy activation is explicit through PolicySkills.
    """
    return build_r1pro_manipulation(
        scene_path=scene_path,
        artifact=artifact,
        device=device,
        headless=headless,
        simulator=R1ProGraspingSim,
        policy_module=R1ProPickPlacePolicy,
        task_description=R1PRO_PICK_PLACE_TASK,
    )


def build_r1pro_manipulation(
    *,
    scene_path: Path,
    artifact: str,
    device: str,
    headless: bool,
    simulator: type[R1ProGraspingSim],
    policy_module: type[_PolicyModule],
    task_description: str,
    background_camera_rendering: bool = False,
    viewer_lookat: tuple[float, float, float] = (0.0, -0.4, 0.85),
    viewer_distance: float = 3.0,
    viewer_azimuth: float | None = None,
    viewer_elevation: float | None = None,
    velocity_base: HardwareComponent | None = None,
    navigation_task: TaskConfig | None = None,
    coordinator_type: type[ControlCoordinator] = ControlCoordinator,
) -> Blueprint:
    """Shared physical robot/camera/coordinator wiring for manipulation profiles."""
    scene_path = scene_path.expanduser().resolve()
    with GraspingTask(scene_path, images=False) as task:
        mobile = (
            mujoco.mj_name2id(task.model, mujoco.mjtObj.mjOBJ_JOINT, VIRTUAL_BASE_JOINTS[0]) >= 0
        )
        free_tray = mujoco.mj_name2id(task.model, mujoco.mjtObj.mjOBJ_JOINT, "task_tray_free") >= 0
        if velocity_base is not None and not (mobile and free_tray):
            raise ValueError("Velocity navigation requires the mobile base and free tray scene")
        position_base = mobile and velocity_base is None
        joints = (*R1PRO_PICK_PLACE_JOINTS, *(VIRTUAL_BASE_JOINTS if position_base else ()))
        home = task.home.tolist() + ([0.0] * 3 if position_base else [])
        ranges = [task.model.joint(name).range.tolist() for name in joints]
    camera = SimCameraSpec(name="right_wrist", stream="right_wrist", width=160, height=160, fps=40)
    hardware = HardwareComponent(
        hardware_id="r1pro",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(joints),
        adapter_type="sim_mujoco_whole_body",
        address=scene_path,
        auto_enable=True,
        adapter_kwargs={"num_motors": len(joints), "command_mode": "position"},
        limits=JointLimits(
            position_lower=[bounds[0] for bounds in ranges],
            position_upper=[bounds[1] for bounds in ranges],
            velocity_max=[2.0] * 18 + [0.25, 0.25] + ([0.4, 0.4, 0.4] if position_base else []),
        ),
    )
    return autoconnect(
        simulator.blueprint(
            address=scene_path,
            dof=len(joints),
            headless=headless,
            viewer_lookat=viewer_lookat,
            viewer_distance=viewer_distance,
            viewer_azimuth=viewer_azimuth,
            viewer_elevation=viewer_elevation,
            background_camera_rendering=background_camera_rendering,
            viewer_track_body="base_link" if free_tray else None,
            position_target_velocity_limits=(
                dict(zip(VIRTUAL_BASE_JOINTS, [0.1, 0.1, 0.15], strict=True))
                if free_tray and position_base
                else {}
            ),
            camera_name="head",
            width=160,
            height=160,
            fps=40,
            extra_cameras=[camera],
            base_frame_id="world",
            enable_depth=False,
            enable_pointcloud=False,
            reset_joint_positions=home,
            robot_sim_spec=RobotSimSpec(
                robot_id="r1pro",
                hardware_joints=joints,
                model_joint_names=joints,
                model_actuator_names=joints,
                root_body_names=("base_link",),
            ),
        ),
        coordinator_type.blueprint(
            hardware=[hardware, *([velocity_base] if velocity_base is not None else [])],
            tasks=[
                *([navigation_task] if navigation_task is not None else []),
                *(
                    [
                        TaskConfig(
                            name="base_transport",
                            type="trajectory",
                            priority=30,
                            joint_names=list(VIRTUAL_BASE_JOINTS),
                        )
                    ]
                    if position_base
                    else []
                ),
                TaskConfig(
                    name="tray_manipulation",
                    type="trajectory",
                    joint_names=list(R1PRO_PICK_PLACE_JOINTS),
                    priority=30,
                    params={"start_position_tolerance": 0.05},
                ),
                TaskConfig(
                    name=POLICY_ROLLOUT_TASK_NAME,
                    type="trajectory",
                    joint_names=list(R1PRO_PICK_PLACE_JOINTS),
                    priority=30,
                    params={"start_position_tolerance": 0.05},
                ),
            ],
        ),
        policy_module.blueprint(
            instance_name=POLICY_ROLLOUT_INSTANCE_NAME,
            artifact=artifact,
            task=task_description,
            device=device,
            startup_timeout=120.0,
            max_execution_horizon_s=1.5,
        ),
        PolicySkills.blueprint(),
    )


def build_r1pro_sim_rollout(
    *,
    artifact: str,
    task: str,
    device: str | None = None,
    cameras: dict[str, int | str] | None = None,
    quest_control: bool = False,
) -> Blueprint:
    """Standard imitation CLI entry point for the learned bottle-to-bin task."""
    if cameras or quest_control:
        raise ValueError("R1Pro simulation supplies its own cameras and policy controls")
    scene = prepare_grasping_scene(
        RECORDINGS_DIR / "r1pro-act-sessions" / uuid4().hex / "scene.xml"
    )
    return build_r1pro_pick_place(scene_path=scene, artifact=artifact, device=device or "cuda")
