#!/usr/bin/env python3
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

"""Run layer-2 Robosuite validation through AgenticManipulationModule.

The script keeps Robosuite out of the DimOS process by launching the Robosuite
sidecar subprocess, then builds a DimOS stack with the benchmark-runtime SHM
adapter, ControlCoordinator, ManipulationModule, and AgenticManipulationModule.
It calls the agent-facing module API directly and writes artifacts describing
the episode, runtime plan, API results, motor trace, score, sidecar log, and
cleanup status.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"

for package_src in (PROTOCOL_SRC, ROBOSUITE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import EpisodeResetRequest, MotorActionFrame, StepRequest

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    ResolvedRuntimePlan,
    resolve_runtime_plan,
)
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.whole_body.spec import MotorState, WholeBodyConfig
from dimos.manipulation.agentic_manipulation_module import AgenticManipulationModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.config import GripperConfig, RobotConfig
from dimos.simulation.runtime_client.http_client import RuntimeSidecarClient
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


@dataclass(frozen=True)
class RuntimeRobotStackConfig:
    """DimOS stack config derived from a Robosuite runtime plan."""

    robot: RobotConfig
    hardware: HardwareComponent
    task: TaskConfig
    model: RobotModelConfig


@dataclass(frozen=True)
class ApiCallRecord:
    """Summary of one direct AgenticManipulationModule API call."""

    name: str
    success: bool
    message: str
    duration_ms: float | None


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _ensure_drake_available() -> None:
    if importlib.util.find_spec("pydrake") is None:
        raise RuntimeError(
            "Agentic manipulation Robosuite demo requires the DimOS manipulation planning "
            "extra because ManipulationModule uses the Drake planning backend in this demo. "
            "Run: uv run --extra manipulation --with robosuite python "
            "scripts/benchmarks/demo_agentic_manipulation_robosuite.py"
        )


def _sidecar_env() -> dict[str, str]:
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    paths = [str(PROTOCOL_SRC), str(ROBOSUITE_SIDECAR_SRC)]
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env


def _start_robosuite_sidecar(config: BenchmarkEpisodeConfig) -> subprocess.Popen[str]:
    command = [
        sys.executable,
        "-m",
        "dimos_robosuite_sidecar.server",
        "--host",
        config.runtime_host,
        "--port",
        str(config.runtime_port),
        "--env-name",
        config.env_name,
        "--robot-id",
        config.robot_id,
        "--robot-model",
        config.robot_model,
        "--controller",
        config.controller,
        "--control-freq",
        str(config.control_step_hz),
        "--horizon",
        str(config.horizon),
        "--camera-name",
        config.camera_name,
        "--seed",
        str(config.seed) if config.seed is not None else "0",
    ]
    if config.visualize:
        command.append("--visualize")
    return subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=_sidecar_env(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _wait_sidecar_healthy(
    sidecar: subprocess.Popen[str],
    client: RuntimeSidecarClient,
    timeout_s: float,
) -> object:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        if sidecar.poll() is not None:
            raise RuntimeError("Robosuite sidecar exited before becoming healthy")
        try:
            return client.health()
        except Exception as exc:
            last_error = str(exc)
            time.sleep(0.1)
    raise RuntimeError(f"Robosuite sidecar did not become healthy: {last_error}")


def _command_frame(owner: MotorShmOwner, robot_id: str) -> tuple[int, MotorActionFrame]:
    sequence, commands = owner.read_commands()
    return sequence, MotorActionFrame(
        robot_id=robot_id,
        names=owner.motor_names,
        q=[command.q for command in commands],
        dq=[command.dq for command in commands],
        kp=[command.kp for command in commands],
        kd=[command.kd for command in commands],
        tau=[command.tau for command in commands],
        sequence=sequence,
    )


def _local_motor_names(plan: ResolvedRuntimePlan) -> list[str]:
    prefix = f"{plan.robot_id}/"
    local_names: list[str] = []
    for motor_name in plan.motor_names:
        if motor_name.startswith(prefix):
            local_names.append(motor_name[len(prefix) :])
        else:
            local_names.append(motor_name)
    return local_names


def _write_minimal_urdf(path: Path, joint_names: list[str]) -> None:
    links = ['  <link name="base_link"/>']
    joints: list[str] = []
    parent = "base_link"
    for index, joint_name in enumerate(joint_names, start=1):
        child = f"link_{index}"
        links.append(f'  <link name="{child}"/>')
        joints.extend(
            [
                f'  <joint name="{joint_name}" type="revolute">',
                f'    <parent link="{parent}"/>',
                f'    <child link="{child}"/>',
                '    <origin xyz="0.05 0 0" rpy="0 0 0"/>',
                '    <axis xyz="0 0 1"/>',
                '    <limit lower="-6.28" upper="6.28" effort="20" velocity="2"/>',
                "  </joint>",
            ]
        )
        parent = child
    xml = [
        '<?xml version="1.0"?>',
        '<robot name="agentic_robosuite_runtime">',
        *links,
        *joints,
        "</robot>",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(xml))


def _derive_runtime_robot_stack(
    plan: ResolvedRuntimePlan,
    artifact_dir: Path,
) -> RuntimeRobotStackConfig:
    local_joint_names = _local_motor_names(plan)
    if len(local_joint_names) != len(plan.motor_names):
        raise ValueError("local/global motor name count mismatch")

    model_path = artifact_dir / "runtime_robot.urdf"
    _write_minimal_urdf(model_path, local_joint_names)
    gripper_joint = local_joint_names[-1]
    robot = RobotConfig(
        name=plan.robot_id,
        model_path=model_path,
        joint_names=local_joint_names,
        base_link="base_link",
        end_effector_link=f"link_{len(local_joint_names)}",
        adapter_type="benchmark_runtime",
        address=plan.shm_key,
        adapter_kwargs={"motor_names": plan.motor_names, "connect_timeout_s": 5.0},
        home_joints=[0.0] * len(local_joint_names),
        gripper=GripperConfig(type="runtime_motor", joints=[gripper_joint]),
    )
    hardware = HardwareComponent(
        hardware_id=plan.robot_id,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=plan.motor_names,
        adapter_type="benchmark_runtime",
        address=plan.shm_key,
        adapter_kwargs={"motor_names": plan.motor_names, "connect_timeout_s": 5.0},
        wb_config=WholeBodyConfig(
            kp=tuple(40.0 for _ in plan.motor_names),
            kd=tuple(3.0 for _ in plan.motor_names),
        ),
    )
    return RuntimeRobotStackConfig(
        robot=robot,
        hardware=hardware,
        task=robot.to_task_config(task_type="trajectory"),
        model=robot.to_robot_model_config(),
    )


class SidecarSteppingLoop:
    """Background SHM-to-sidecar loop for blocking manipulation API calls."""

    def __init__(
        self,
        *,
        client: RuntimeSidecarClient,
        owner: MotorShmOwner,
        plan: ResolvedRuntimePlan,
    ) -> None:
        self._client = client
        self._owner = owner
        self._plan = plan
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="robosuite-sidecar-step", daemon=True
        )
        self._trace: list[dict[str, object]] = []
        self._error: str | None = None

    @property
    def trace(self) -> list[dict[str, object]]:
        return list(self._trace)

    @property
    def error(self) -> str | None:
        return self._error

    @property
    def tick_count(self) -> int:
        return len(self._trace)

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=5.0)

    def _run(self) -> None:
        tick = 0
        period_s = 1.0 / float(self._plan.control_step_hz)
        while not self._stop_event.is_set() and tick < self._plan.ticks:
            try:
                command_sequence, action = _command_frame(self._owner, self._plan.robot_id)
                response = self._client.step(
                    StepRequest(episode_id=self._plan.episode_id, tick_id=tick, action=action)
                )
                self._owner.write_state(
                    [
                        MotorState(
                            q=response.motor_state.q[i],
                            dq=response.motor_state.dq[i],
                            tau=response.motor_state.tau[i],
                        )
                        for i in range(len(self._plan.motor_names))
                    ],
                    sequence=response.motor_state.sequence,
                )
                self._trace.append(
                    {
                        "tick": tick,
                        "command_sequence": command_sequence,
                        "state_sequence": response.motor_state.sequence,
                        "command_q": action.q,
                        "state_q": response.motor_state.q,
                        "reward": response.reward,
                        "done": response.done,
                        "success": response.success,
                    }
                )
                if response.done:
                    break
                tick += 1
                time.sleep(period_s)
            except Exception as exc:
                self._error = str(exc)
                break


def _call_agentic_api(
    name: str,
    call: Callable[[], object],
    records: list[ApiCallRecord],
) -> None:
    result = call()
    success_attr = getattr(result, "success", False)
    success = bool(success_attr)
    message_attr = getattr(result, "message", "")
    duration_attr = getattr(result, "duration_ms", None)
    records.append(
        ApiCallRecord(
            name=name,
            success=success,
            message=str(message_attr),
            duration_ms=float(duration_attr) if isinstance(duration_attr, int | float) else None,
        )
    )
    if not success:
        raise RuntimeError(f"AgenticManipulationModule.{name} failed: {message_attr}")


def _wait_for_sidecar_ticks(
    stepping_loop: SidecarSteppingLoop,
    *,
    seconds: float,
    control_step_hz: float,
    label: str,
) -> None:
    """Let Robosuite advance for a visible amount of simulated time."""

    if seconds <= 0.0:
        return
    start_tick = stepping_loop.tick_count
    target_ticks = max(1, round(seconds * control_step_hz))
    deadline = time.monotonic() + max(seconds * 4.0, seconds + 2.0)
    while stepping_loop.tick_count - start_tick < target_ticks:
        if stepping_loop.error is not None:
            raise RuntimeError(
                f"sidecar stepping loop failed during {label}: {stepping_loop.error}"
            )
        if not stepping_loop.is_alive:
            raise RuntimeError(
                f"sidecar stepping loop ended while waiting after {label}; "
                "increase --ticks or reduce the visual pause durations"
            )
        if time.monotonic() > deadline:
            raise RuntimeError(f"timed out waiting for Robosuite ticks after {label}")
        time.sleep(0.02)


def _small_offset_target(current_positions: list[float], offset: float) -> str:
    target = list(current_positions)
    if target:
        target[0] += offset
    return ", ".join(f"{value:.4f}" for value in target)


def _latest_state_positions(stepping_loop: SidecarSteppingLoop, motor_count: int) -> list[float]:
    trace = stepping_loop.trace
    if not trace:
        return [0.0] * motor_count
    state_q = trace[-1].get("state_q", [])
    if not isinstance(state_q, list):
        return [0.0] * motor_count
    positions = [float(value) for value in state_q]
    if len(positions) != motor_count:
        return [0.0] * motor_count
    return positions


def _hardware_summary(hardware: HardwareComponent) -> dict[str, object]:
    wb_config = hardware.wb_config
    return {
        "hardware_id": hardware.hardware_id,
        "hardware_type": hardware.hardware_type.value,
        "joints": hardware.joints,
        "adapter_type": hardware.adapter_type,
        "address": str(hardware.address) if hardware.address is not None else None,
        "auto_enable": hardware.auto_enable,
        "gripper_joints": hardware.gripper_joints,
        "domain_id": hardware.domain_id,
        "adapter_kwargs": hardware.adapter_kwargs,
        "wb_config": {
            "kp": list(wb_config.kp) if wb_config.kp is not None else None,
            "kd": list(wb_config.kd) if wb_config.kd is not None else None,
        }
        if wb_config is not None
        else None,
        "derivation_note": (
            "Constructed directly as WHOLE_BODY because RobotConfig.to_hardware_component() "
            "derives MANIPULATOR hardware, while the benchmark runtime adapter exposes a "
            "whole-body SHM motor plane. TaskConfig and RobotModelConfig still derive from RobotConfig."
        ),
    }


def _gripper_summary(gripper: GripperConfig | None) -> dict[str, object] | None:
    if gripper is None:
        return None
    return {
        "type": gripper.type,
        "joints": gripper.joints,
        "collision_exclusions": [list(pair) for pair in gripper.collision_exclusions],
        "open_position": gripper.open_position,
        "close_position": gripper.close_position,
    }


def _robot_config_summary(robot: RobotConfig) -> dict[str, object]:
    return {
        "name": robot.name,
        "model_path": str(robot.model_path) if robot.model_path is not None else None,
        "end_effector_link": robot.end_effector_link,
        "height_clearance": robot.height_clearance,
        "width_clearance": robot.width_clearance,
        "adapter_type": robot.adapter_type,
        "address": robot.address,
        "adapter_kwargs": robot.adapter_kwargs,
        "auto_enable": robot.auto_enable,
        "joint_names": robot.joint_names,
        "base_link": robot.base_link,
        "home_joints": robot.home_joints,
        "base_pose": robot.base_pose,
        "strip_model_world_joint": robot.strip_model_world_joint,
        "max_velocity": robot.max_velocity,
        "max_acceleration": robot.max_acceleration,
        "pre_grasp_offset": robot.pre_grasp_offset,
        "gripper": _gripper_summary(robot.gripper),
        "package_paths": {key: str(path) for key, path in robot.package_paths.items()},
        "xacro_args": robot.xacro_args,
        "auto_convert_meshes": robot.auto_convert_meshes,
        "srdf_path": str(robot.srdf_path) if robot.srdf_path is not None else None,
        "tf_extra_links": robot.tf_extra_links,
        "task_type": robot.task_type,
        "task_priority": robot.task_priority,
        "collision_exclusion_pairs": [list(pair) for pair in robot.collision_exclusion_pairs],
    }


def _task_summary(task: TaskConfig) -> dict[str, object]:
    return {
        "name": task.name,
        "type": task.type,
        "joint_names": task.joint_names,
        "priority": task.priority,
        "auto_start": task.auto_start,
        "params": task.params,
    }


def _robot_model_summary(model: RobotModelConfig) -> dict[str, object]:
    return {
        "name": model.name,
        "model_path": str(model.model_path),
        "srdf_path": str(model.srdf_path) if model.srdf_path is not None else None,
        "base_pose": {
            "position": [
                model.base_pose.position.x,
                model.base_pose.position.y,
                model.base_pose.position.z,
            ],
            "orientation": [
                model.base_pose.orientation.x,
                model.base_pose.orientation.y,
                model.base_pose.orientation.z,
                model.base_pose.orientation.w,
            ],
            "frame_id": model.base_pose.frame_id,
            "ts": model.base_pose.ts,
        },
        "strip_model_world_joint": model.strip_model_world_joint,
        "joint_names": model.joint_names,
        "end_effector_link": model.end_effector_link,
        "base_link": model.base_link,
        "planning_groups": [
            {
                "name": group.name,
                "joint_names": list(group.joint_names),
                "base_link": group.base_link,
                "tip_link": group.tip_link,
                "source": group.source,
            }
            for group in model.planning_groups
        ],
        "package_paths": {key: str(path) for key, path in model.package_paths.items()},
        "joint_limits_lower": model.joint_limits_lower,
        "joint_limits_upper": model.joint_limits_upper,
        "velocity_limits": model.velocity_limits,
        "auto_convert_meshes": model.auto_convert_meshes,
        "xacro_args": model.xacro_args,
        "collision_exclusion_pairs": [list(pair) for pair in model.collision_exclusion_pairs],
        "max_velocity": model.max_velocity,
        "max_acceleration": model.max_acceleration,
        "coordinator_task_name": model.coordinator_task_name,
        "gripper_hardware_id": model.gripper_hardware_id,
        "tf_extra_links": model.tf_extra_links,
        "home_joints": model.home_joints,
        "pre_grasp_offset": model.pre_grasp_offset,
        "fallback_model_note": (
            "Script-generated minimal URDF for runtime validation only. Joint limits are "
            "conservative to cover Robosuite reset states until sidecar metadata can provide "
            "authoritative model limits."
        ),
    }


def _api_call_summary(records: list[ApiCallRecord]) -> list[dict[str, object]]:
    return [
        {
            "name": record.name,
            "success": record.success,
            "message": record.message,
            "duration_ms": record.duration_ms,
        }
        for record in records
    ]


def _write_json_artifact(
    path: Path,
    payload: object,
    errors: dict[str, str],
) -> None:
    try:
        write_json(path, payload)
    except Exception as exc:
        errors[path.name] = str(exc)


def _write_available_artifacts(
    *,
    artifact_dir: Path,
    config: BenchmarkEpisodeConfig,
    health: object | None,
    description: object | None,
    plan: ResolvedRuntimePlan | None,
    reset: object | None,
    stack_config: RuntimeRobotStackConfig | None,
    api_calls: list[ApiCallRecord],
    stepping_loop: SidecarSteppingLoop | None,
    score: object | None,
    failure: str | None,
) -> dict[str, str]:
    """Write every validation artifact available on success or failure."""

    errors: dict[str, str] = {}
    _write_json_artifact(artifact_dir / "episode_config.json", config, errors)
    if health is not None:
        _write_json_artifact(artifact_dir / "health.json", health, errors)
    if description is not None:
        _write_json_artifact(artifact_dir / "runtime_description.json", description, errors)
    if plan is not None:
        _write_json_artifact(artifact_dir / "resolved_runtime_plan.json", plan, errors)
    if reset is not None:
        _write_json_artifact(artifact_dir / "reset_response.json", reset, errors)
    if stack_config is not None:
        _write_json_artifact(
            artifact_dir / "runtime_robot_config.json",
            _robot_config_summary(stack_config.robot),
            errors,
        )
        _write_json_artifact(
            artifact_dir / "stack_config_summary.json",
            {
                "hardware": _hardware_summary(stack_config.hardware),
                "task": _task_summary(stack_config.task),
                "robot_model": _robot_model_summary(stack_config.model),
            },
            errors,
        )
    _write_json_artifact(
        artifact_dir / "api_call_summary.json", _api_call_summary(api_calls), errors
    )
    _write_json_artifact(
        artifact_dir / "motor_trace.json",
        stepping_loop.trace if stepping_loop is not None else [],
        errors,
    )
    _write_json_artifact(
        artifact_dir / "score.json",
        score if score is not None else {"available": False, "reason": "not reached"},
        errors,
    )
    if failure is not None:
        _write_json_artifact(artifact_dir / "failure.json", {"reason": failure}, errors)
    return errors


def _build_stack(stack_config: RuntimeRobotStackConfig, tick_rate: float) -> ModuleCoordinator:
    blueprint = autoconnect(
        ControlCoordinator.blueprint(
            tick_rate=tick_rate,
            publish_joint_state=True,
            hardware=[stack_config.hardware],
            tasks=[stack_config.task],
        ),
        ManipulationModule.blueprint(
            robots=[stack_config.model],
            world_backend="drake",
            planner_name="rrt_connect",
            coordinator_rpc_timeout=10.0,
        ),
        AgenticManipulationModule.blueprint(),
    )
    return ModuleCoordinator.build(blueprint)


def run_demo_config(
    config: BenchmarkEpisodeConfig,
    *,
    move_offset: float = 0.25,
    settle_s: float = 0.25,
    primitive_pause_s: float = 1.0,
    post_demo_s: float = 2.0,
) -> Path:
    return _run_demo(
        config,
        move_offset=move_offset,
        settle_s=settle_s,
        primitive_pause_s=primitive_pause_s,
        post_demo_s=post_demo_s,
    )


def run_demo(config_path: Path) -> Path:
    return run_demo_config(_load_config(config_path))


def _run_demo(
    config: BenchmarkEpisodeConfig,
    *,
    move_offset: float,
    settle_s: float,
    primitive_pause_s: float,
    post_demo_s: float,
) -> Path:
    _ensure_drake_available()

    artifact_dir = (REPO_ROOT / config.artifact_dir).resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    client = RuntimeSidecarClient(f"http://{config.runtime_host}:{config.runtime_port}")
    sidecar: subprocess.Popen[str] | None = None
    owner: MotorShmOwner | None = None
    coordinator: ModuleCoordinator | None = None
    stepping_loop: SidecarSteppingLoop | None = None
    health: object | None = None
    description: object | None = None
    plan: ResolvedRuntimePlan | None = None
    reset: object | None = None
    stack_config: RuntimeRobotStackConfig | None = None
    score: object | None = None
    api_calls: list[ApiCallRecord] = []
    failure: str | None = None
    cleanup_status: dict[str, object] = {
        "module_coordinator_stopped": False,
        "stepping_loop_stopped": False,
        "shm_unlinked": False,
        "sidecar_stopped": False,
    }
    sidecar_output = ""
    try:
        sidecar = _start_robosuite_sidecar(config)
        try:
            health = _wait_sidecar_healthy(sidecar, client, timeout_s=20.0)
        except RuntimeError as exc:
            raise RuntimeError(
                "Robosuite sidecar did not become healthy. If this environment does not "
                "have Robosuite installed, run this script in the Robosuite sidecar env."
            ) from exc
        description = client.describe()
        plan = resolve_runtime_plan(config, description)
        reset = client.reset(
            EpisodeResetRequest(episode_id=plan.episode_id, task_id=plan.task_id, seed=config.seed)
        )
        stack_config = _derive_runtime_robot_stack(plan, artifact_dir)

        owner = MotorShmOwner(plan.shm_key, plan.motor_names)
        owner.write_state([MotorState(q=0.0) for _ in plan.motor_names], sequence=0)

        coordinator = _build_stack(stack_config, float(plan.control_step_hz))
        agentic_module = coordinator.get_instance(AgenticManipulationModule)
        stepping_loop = SidecarSteppingLoop(client=client, owner=owner, plan=plan)
        stepping_loop.start()

        # Allow the coordinator to publish initial joint state into ManipulationModule.
        _wait_for_sidecar_ticks(
            stepping_loop,
            seconds=settle_s,
            control_step_hz=float(plan.control_step_hz),
            label="initial state propagation",
        )

        _call_agentic_api("get_robot_state", agentic_module.get_robot_state, api_calls)
        _call_agentic_api("open_gripper", agentic_module.open_gripper, api_calls)
        _wait_for_sidecar_ticks(
            stepping_loop,
            seconds=primitive_pause_s,
            control_step_hz=float(plan.control_step_hz),
            label="open_gripper",
        )
        _call_agentic_api("close_gripper", agentic_module.close_gripper, api_calls)
        _wait_for_sidecar_ticks(
            stepping_loop,
            seconds=primitive_pause_s,
            control_step_hz=float(plan.control_step_hz),
            label="close_gripper",
        )
        target_joints = _small_offset_target(
            _latest_state_positions(stepping_loop, len(plan.motor_names)), move_offset
        )
        _call_agentic_api(
            "move_to_joints",
            lambda: agentic_module.move_to_joints(target_joints),
            api_calls,
        )
        _wait_for_sidecar_ticks(
            stepping_loop,
            seconds=post_demo_s,
            control_step_hz=float(plan.control_step_hz),
            label="move_to_joints hold",
        )
        if stepping_loop.error is not None:
            raise RuntimeError(f"sidecar stepping loop failed: {stepping_loop.error}")

        try:
            score = client.score()
        except Exception as exc:
            score = {"available": False, "error": str(exc)}

        return artifact_dir
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        if stepping_loop is not None:
            try:
                stepping_loop.stop()
                cleanup_status["stepping_loop_stopped"] = True
                cleanup_status["stepping_loop_error"] = stepping_loop.error
            except Exception as exc:
                cleanup_status["stepping_loop_stop_error"] = str(exc)
        if coordinator is not None:
            try:
                coordinator.stop()
                cleanup_status["module_coordinator_stopped"] = True
            except Exception as exc:
                cleanup_status["module_coordinator_error"] = str(exc)
        if owner is not None:
            try:
                owner.close()
                owner.unlink()
                cleanup_status["shm_unlinked"] = True
            except Exception as exc:
                cleanup_status["shm_error"] = str(exc)
        if sidecar is not None:
            sidecar.terminate()
            try:
                sidecar_output, _ = sidecar.communicate(timeout=2.0)
            except subprocess.TimeoutExpired:
                sidecar.kill()
                sidecar_output, _ = sidecar.communicate(timeout=2.0)
            cleanup_status["sidecar_returncode"] = sidecar.returncode
            cleanup_status["sidecar_stopped"] = sidecar.returncode is not None
        else:
            cleanup_status["sidecar_returncode"] = None
        sidecar_log = artifact_dir / "robosuite_sidecar.log"
        sidecar_log.parent.mkdir(parents=True, exist_ok=True)
        sidecar_log.write_text(sidecar_output)
        artifact_errors = _write_available_artifacts(
            artifact_dir=artifact_dir,
            config=config,
            health=health,
            description=description,
            plan=plan,
            reset=reset,
            stack_config=stack_config,
            api_calls=api_calls,
            stepping_loop=stepping_loop,
            score=score,
            failure=failure,
        )
        cleanup_status["artifact_write_errors"] = artifact_errors
        write_json(sidecar_log.parent / "cleanup_status.json", cleanup_status)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT
        / "dimos"
        / "benchmark"
        / "runtime"
        / "configs"
        / "robosuite_panda_lift.json",
    )
    parser.add_argument("--ticks", type=int, default=None, help="Override configured tick count.")
    parser.add_argument("--horizon", type=int, default=None, help="Override episode horizon.")
    parser.add_argument(
        "--move-offset",
        type=float,
        default=0.25,
        help="Safe offset applied to the first motor for the direct move_to_joints call.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.25,
        help="Seconds of sidecar stepping before the first API call so initial state propagates.",
    )
    parser.add_argument(
        "--primitive-pause-s",
        type=float,
        default=1.0,
        help="Seconds to keep stepping after open_gripper and close_gripper for visual inspection.",
    )
    parser.add_argument(
        "--post-demo-s",
        type=float,
        default=2.0,
        help="Seconds to keep the viewer alive after move_to_joints completes.",
    )
    visual_group = parser.add_mutually_exclusive_group()
    visual_group.add_argument(
        "--visual",
        dest="visualize",
        action="store_true",
        default=True,
        help="Open the Robosuite viewer through the sidecar. This is the default.",
    )
    visual_group.add_argument(
        "--headless",
        dest="visualize",
        action="store_false",
        help="Disable the Robosuite viewer for CI or non-GUI environments.",
    )
    args = parser.parse_args()
    try:
        config = _load_config(args.config)
        updates: dict[str, object] = {"visualize": args.visualize}
        if args.ticks is not None:
            updates["ticks"] = args.ticks
        else:
            visible_seconds = (
                args.settle_s + (2.0 * args.primitive_pause_s) + args.post_demo_s + 1.0
            )
            updates["ticks"] = max(config.ticks, int(config.control_step_hz * visible_seconds))
        if args.horizon is not None:
            updates["horizon"] = args.horizon
        else:
            updates["horizon"] = max(config.horizon, int(updates["ticks"]) + 1)
        if updates:
            config = BenchmarkEpisodeConfig.model_validate({**config.model_dump(), **updates})
        artifact_dir = run_demo_config(
            config,
            move_offset=args.move_offset,
            settle_s=args.settle_s,
            primitive_pause_s=args.primitive_pause_s,
            post_demo_s=args.post_demo_s,
        )
    except Exception as exc:
        print(json.dumps({"ok": False, "reason": str(exc)}, indent=2))
        sys.exit(2)
    print(json.dumps({"ok": True, "artifact_dir": str(artifact_dir)}, indent=2))


if __name__ == "__main__":
    main()
