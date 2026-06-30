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

"""Run the Robosuite Panda Lift plumbing demo with a real ControlCoordinator.

By default this demo runs from the host DimOS environment and deploys
``RobosuiteRuntimeModule`` into the package-local Robosuite Python project runtime
via the standard blueprint placement API. Robosuite dependencies stay in the
placed worker environment; the host process owns orchestration, artifacts, and
Rerun stream forwarding.

Use ``--runtime-mode local`` only as a debug fallback when intentionally running
the whole demo inside the Robosuite-capable package environment.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import cast

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"

for package_src in (PROTOCOL_SRC, ROBOSUITE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    MotorActionFrame,
    MotorStateFrame,
    StepRequest,
)

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    resolve_runtime_plan,
)
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.hardware.whole_body.spec import MotorState
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


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


def _free_tcp_port() -> int:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _lcm_url(port: int) -> str:
    return f"udpm://239.255.76.67:{port}?ttl=0"


class RuntimeRerunBridge:
    """Bridge normal DimOS runtime streams into Rerun."""

    def __init__(
        self,
        *,
        grpc_port: int,
        lcm_port: int,
        memory_limit: str,
        max_hz: float,
    ) -> None:
        from dimos.visualization.rerun.bridge import RerunBridgeModule

        self._image_entity = "world/color_image"
        self._camera_info_entity = "world/camera_info"

        def runtime_camera_blueprint() -> object:
            import rerun.blueprint as rrb

            return rrb.Blueprint(
                rrb.Vertical(
                    rrb.Spatial2DView(origin=self._image_entity, name="Robosuite camera"),
                )
            )

        max_hz_by_entity = {self._image_entity: max_hz} if max_hz > 0.0 else {}
        self._bridge = RerunBridgeModule(
            blueprint=runtime_camera_blueprint,
            connect_url=f"rerun+http://127.0.0.1:{grpc_port}/proxy",
            memory_limit=memory_limit,
            max_hz=max_hz_by_entity,
            visual_override={self._camera_info_entity: None},
        )

    def start(self) -> None:
        self._bridge.start()

    def stop(self) -> None:
        self._bridge.stop()


def _dump_stream_observation_image(
    capture: RuntimeStreamCapture,
    *,
    image_dump_dir: Path | None = None,
    image_dump_label: str = "frame",
) -> int:
    if image_dump_dir is None:
        return 0
    published = 0
    image = capture.last_color_image
    if image is not None:
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import Image

        if not isinstance(image, Image):
            raise TypeError(f"expected DimOS Image stream output, got {type(image).__name__}")
        display_image = np.asarray(image.to_rgb().data)
        frame_id = getattr(image, "frame_id", "camera") or "camera"
        if image_dump_dir is not None:
            _write_rgb_jpeg(
                image_dump_dir / f"{image_dump_label}_{frame_id}_display.jpg",
                display_image,
            )
        published += 1
    return published


class RuntimeStreamCapture:
    """Small local subscriber for RobosuiteRuntimeModule output streams."""

    def __init__(self) -> None:
        self.last_motor_state: object | None = None
        self.last_color_image: object | None = None
        self.last_camera_info: object | None = None
        self.runtime_events: list[object] = []
        self.streams_since_mark: list[str] = []

    def mark(self) -> None:
        self.streams_since_mark = []

    def color_image(self, image: object) -> None:
        self.last_color_image = image
        self.streams_since_mark.append("color_image")

    def motor_state(self, motor_state: object) -> None:
        self.last_motor_state = motor_state
        self.streams_since_mark.append("motor_state")

    def camera_info(self, camera_info: object) -> None:
        self.last_camera_info = camera_info
        self.streams_since_mark.append("camera_info")

    def runtime_event(self, event: object) -> None:
        self.runtime_events.append(event)
        stream = getattr(event, "stream", "runtime_event")
        self.streams_since_mark.append(stream if isinstance(stream, str) else "runtime_event")


def _write_rgb_jpeg(path: Path, rgb: object) -> None:
    import cv2
    import numpy as np

    array = np.asarray(rgb)
    if array.ndim != 3 or array.shape[2] < 3:
        raise ValueError(f"expected HxWx3 RGB image, got shape {array.shape}")
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(array[:, :, :3], cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise RuntimeError(f"failed to write JPEG {path}")


def _trace_summary(trace: list[dict[str, object]]) -> dict[str, object]:
    final = trace[-1] if trace else {}
    stream_set: set[str] = set()
    for entry in trace:
        value = entry.get("observation_streams", [])
        if isinstance(value, list):
            stream_set.update(stream for stream in value if isinstance(stream, str))
    streams = sorted(stream_set)
    return {
        "ticks": len(trace),
        "first_command_sequence": trace[0].get("command_sequence") if trace else None,
        "final_command_sequence": final.get("command_sequence"),
        "final_state_sequence": final.get("state_sequence"),
        "final_command_q": final.get("command_q"),
        "final_state_q": final.get("state_q"),
        "observation_streams": streams,
        "final_reward": final.get("reward"),
        "final_done": final.get("done"),
        "final_success": final.get("success"),
    }


def _target_for_tick(plan_target: float, motor_count: int, tick: int, visual: bool) -> list[float]:
    if not visual:
        return [plan_target] * motor_count
    # Deliberately obvious visual command: hold a large normalized target long
    # enough for the on-screen Robosuite viewer to show the arm moving, then
    # alternate direction. Robosuite clips normalized JOINT_POSITION inputs to
    # [-1, 1], so this stays inside the controller action range.
    phase = 1.0 if (tick // 120) % 2 == 0 else -1.0
    amplitude = max(abs(plan_target), 0.9)
    arm_pattern = [1.0, -0.9, 0.8, -0.7, 0.6, -0.5, 0.4]
    targets = [amplitude * phase * scale for scale in arm_pattern[:motor_count]]
    if motor_count > 7:
        gripper_phase = 1.0 if (tick // 80) % 2 == 0 else -1.0
        targets.extend([0.6 * gripper_phase] * (motor_count - 7))
    return targets[:motor_count]


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
    parser.add_argument(
        "--visual",
        action="store_true",
        help="Open the Robosuite viewer and run a longer moving command sequence.",
    )
    parser.add_argument("--ticks", type=int, default=None, help="Override configured tick count.")
    parser.add_argument(
        "--horizon", type=int, default=None, help="Override Robosuite episode horizon."
    )
    parser.add_argument(
        "--target-position",
        type=float,
        default=None,
        help="Override configured command amplitude/target.",
    )
    parser.add_argument(
        "--camera-name",
        default=None,
        help="Override Robosuite camera, e.g. agentview or robot0_eye_in_hand.",
    )
    parser.add_argument(
        "--runtime-mode",
        choices=("placed", "local"),
        default="placed",
        help=(
            "Runtime execution mode. 'placed' builds the Robosuite blueprint and "
            "runs the simulator module in its named Python project runtime; "
            "'local' directly instantiates the runtime module in the current process."
        ),
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Publish Robosuite camera stream outputs to Rerun through DimOS streams.",
    )
    parser.add_argument(
        "--rerun-memory-limit",
        default="128MB",
        help="Memory cap for the Rerun server/viewer used by --rerun.",
    )
    parser.add_argument(
        "--rerun-grpc-port",
        type=int,
        default=0,
        help="Rerun gRPC port for --rerun. 0 selects a free port to avoid mixing with old recordings.",
    )
    parser.add_argument(
        "--rerun-lcm-port",
        type=int,
        default=0,
        help="Private LCM multicast port for --rerun streams. 0 selects a free port to avoid mixing with other LCM traffic.",
    )
    parser.add_argument(
        "--rerun-max-hz",
        type=float,
        default=10.0,
        help="Maximum image publish rate to Rerun. Use <=0 for every simulator tick.",
    )
    parser.add_argument(
        "--camera-jpeg-dump-every",
        type=int,
        default=25,
        help=(
            "When --rerun is enabled, write every Nth Robosuite camera stream output "
            "as display JPEGs under artifacts/.../images. Use 1 for every tick, <=0 to disable."
        ),
    )
    args = parser.parse_args()
    try:
        config = _load_config(args.config)
        updates: dict[str, object] = {}
        if args.visual:
            updates["visualize"] = True
            ticks = args.ticks if args.ticks is not None else max(config.ticks, 600)
            updates["ticks"] = ticks
            updates["horizon"] = (
                args.horizon if args.horizon is not None else max(config.horizon, ticks + 1)
            )
            updates["target_position"] = (
                args.target_position
                if args.target_position is not None
                else max(abs(config.target_position), 0.9)
            )
        else:
            if args.ticks is not None:
                updates["ticks"] = args.ticks
            if args.horizon is not None:
                updates["horizon"] = args.horizon
            if args.target_position is not None:
                updates["target_position"] = args.target_position
        if args.camera_name is not None:
            updates["camera_name"] = args.camera_name
        if updates:
            config = BenchmarkEpisodeConfig.model_validate({**config.model_dump(), **updates})
        artifact_dir = run_demo_config(
            config,
            rerun=args.rerun,
            rerun_memory_limit=args.rerun_memory_limit,
            rerun_grpc_port=args.rerun_grpc_port,
            rerun_lcm_port=args.rerun_lcm_port,
            rerun_max_hz=args.rerun_max_hz,
            camera_jpeg_dump_every=args.camera_jpeg_dump_every,
            runtime_mode=args.runtime_mode,
        )
    except Exception as exc:
        print(json.dumps({"ok": False, "reason": str(exc)}, indent=2))
        sys.exit(2)
    print(json.dumps({"ok": True, "artifact_dir": str(artifact_dir)}, indent=2))


def run_demo_config(
    config: BenchmarkEpisodeConfig,
    *,
    rerun: bool = False,
    rerun_memory_limit: str = "128MB",
    rerun_grpc_port: int = 0,
    rerun_lcm_port: int = 0,
    rerun_max_hz: float = 10.0,
    camera_jpeg_dump_every: int = 25,
    runtime_mode: str = "placed",
) -> Path:
    return _run_demo(
        config,
        rerun=rerun,
        rerun_memory_limit=rerun_memory_limit,
        rerun_grpc_port=rerun_grpc_port,
        rerun_lcm_port=rerun_lcm_port,
        rerun_max_hz=rerun_max_hz,
        camera_jpeg_dump_every=camera_jpeg_dump_every,
        runtime_mode=runtime_mode,
    )


def run_demo(config_path: Path) -> Path:
    return _run_demo(
        _load_config(config_path),
        rerun=False,
        rerun_memory_limit="128MB",
        rerun_grpc_port=0,
        rerun_lcm_port=0,
        rerun_max_hz=10.0,
        camera_jpeg_dump_every=25,
        runtime_mode="placed",
    )


def _run_demo(
    config: BenchmarkEpisodeConfig,
    *,
    rerun: bool,
    rerun_memory_limit: str,
    rerun_grpc_port: int,
    rerun_lcm_port: int,
    rerun_max_hz: float,
    camera_jpeg_dump_every: int,
    runtime_mode: str,
) -> Path:
    from dimos_robosuite_sidecar.module import RobosuiteRuntimeModule

    artifact_dir = (REPO_ROOT / config.artifact_dir).resolve()
    client_image_dump_dir = artifact_dir / "images" / "client"
    runtime_image_dump_dir = artifact_dir / "images" / "runtime"
    owner: MotorShmOwner | None = None
    coordinator: ControlCoordinator | None = None
    rerun_bridge: RuntimeRerunBridge | None = None
    cleanup_status: dict[str, object] = {
        "coordinator_stopped": False,
        "module_coordinator_stopped": False,
        "shm_unlinked": False,
        "runtime_stopped": False,
    }
    selected_rerun_grpc_port: int | None = None
    selected_rerun_lcm_port: int | None = None
    module_coordinator = None
    if rerun:
        # Start Rerun before constructing Robosuite/MuJoCo. Starting the viewer
        # after MuJoCo visual contexts exist can corrupt subsequent offscreen
        # camera buffers in visual mode.
        selected_rerun_grpc_port = rerun_grpc_port if rerun_grpc_port > 0 else _free_tcp_port()
        selected_rerun_lcm_port = rerun_lcm_port if rerun_lcm_port > 0 else _free_tcp_port()
        rerun_bridge = RuntimeRerunBridge(
            grpc_port=selected_rerun_grpc_port,
            lcm_port=selected_rerun_lcm_port,
            memory_limit=rerun_memory_limit,
            max_hz=rerun_max_hz,
        )
        rerun_bridge.start()
    try:
        if runtime_mode == "placed":
            from dimos_robosuite_sidecar.blueprint import robosuite_runtime_blueprint

            from dimos.core.coordination.module_coordinator import ModuleCoordinator

            blueprint = robosuite_runtime_blueprint(
                env_name=config.env_name,
                robot_id=config.robot_id,
                robot_model=config.robot_model,
                controller=config.controller,
                control_freq=config.control_step_hz,
                horizon=config.horizon,
                camera_name=config.camera_name,
                seed=config.seed,
                visualize=config.visualize,
                image_dump_dir=runtime_image_dump_dir if camera_jpeg_dump_every > 0 else None,
                image_dump_every=camera_jpeg_dump_every,
            ).global_config(viewer="none", n_workers=1)
            module_coordinator = ModuleCoordinator.build(blueprint)
            runtime = module_coordinator.get_instance(RobosuiteRuntimeModule)
        else:
            runtime = RobosuiteRuntimeModule(
                env_name=config.env_name,
                robot_id=config.robot_id,
                robot_model=config.robot_model,
                controller=config.controller,
                control_freq=config.control_step_hz,
                horizon=config.horizon,
                camera_name=config.camera_name,
                seed=config.seed,
                visualize=config.visualize,
                image_dump_dir=runtime_image_dump_dir if camera_jpeg_dump_every > 0 else None,
                image_dump_every=camera_jpeg_dump_every,
            )
    except Exception as exc:
        if rerun_bridge is not None:
            rerun_bridge.stop()
        raise RuntimeError(
            "RobosuiteRuntimeModule could not be deployed. For placed mode, prepare "
            "packages/dimos-robosuite-sidecar as the named runtime environment; for "
            "local debug mode, run from a Robosuite-capable package environment."
        ) from exc
    capture = RuntimeStreamCapture()
    runtime.motor_state.subscribe(capture.motor_state)
    runtime.color_image.subscribe(capture.color_image)
    runtime.camera_info.subscribe(capture.camera_info)
    runtime.runtime_event.subscribe(capture.runtime_event)
    health: dict[str, object] = {
        "ok": True,
        "runtime": "RobosuiteRuntimeModule",
        "transport": f"module-rpc-{runtime_mode}",
        "http_runtime_api": False,
    }
    try:
        description = runtime.describe()
        plan = resolve_runtime_plan(config, description)
        capture.mark()
        reset = runtime.reset(
            EpisodeResetRequest(
                episode_id=plan.episode_id,
                task_id=plan.task_id,
                seed=config.seed,
            )
        )

        task_name = f"servo_{plan.robot_id}"
        if not config.visualize:
            owner = MotorShmOwner(plan.shm_key, plan.motor_names)
            owner.write_state([MotorState(q=0.0) for _ in plan.motor_names], sequence=0)

            hardware = HardwareComponent(
                hardware_id=plan.robot_id,
                hardware_type=HardwareType.WHOLE_BODY,
                joints=plan.motor_names,
                adapter_type="benchmark_runtime",
                address=plan.shm_key,
                adapter_kwargs={"motor_names": plan.motor_names, "connect_timeout_s": 5.0},
            )
            coordinator = ControlCoordinator(
                tick_rate=float(plan.control_step_hz),
                publish_joint_state=False,
                hardware=[hardware],
                tasks=[
                    TaskConfig(
                        name=task_name,
                        type="servo",
                        joint_names=plan.motor_names,
                        auto_start=True,
                        params={
                            "timeout": 0.0,
                            "default_positions": [0.0] * len(plan.motor_names),
                        },
                    )
                ],
            )
            coordinator.start()
        if rerun:
            if camera_jpeg_dump_every > 0:
                _dump_stream_observation_image(
                    capture,
                    image_dump_dir=client_image_dump_dir,
                    image_dump_label="reset",
                )

        trace: list[dict[str, object]] = []
        published_rerun_frames = 0
        for tick in range(plan.ticks):
            target = _target_for_tick(
                plan.target_position, len(plan.motor_names), tick, config.visualize
            )
            if config.visualize:
                # The visual smoke path is intended to make simulator motion obvious.
                # The ControlCoordinator/SHM servo path intentionally damps targets,
                # which is useful for normal benchmark plumbing but too subtle for
                # visual debugging. Send the runtime action directly here while the
                # non-visual smoke keeps exercising the coordinator bridge.
                command_sequence = tick + 1
                action = MotorActionFrame(
                    robot_id=plan.robot_id,
                    names=plan.motor_names,
                    q=target,
                    sequence=command_sequence,
                )
                time.sleep(1.0 / plan.control_step_hz)
            else:
                if coordinator is None:
                    raise RuntimeError("non-visual demo requires a ControlCoordinator")
                accepted = coordinator.task_invoke(
                    task_name, "set_target", {"positions": target, "t_now": None}
                )
                if accepted is not True:
                    raise RuntimeError(f"servo task rejected target at tick {tick}")
                time.sleep(1.0 / plan.control_step_hz)
                if owner is None:
                    raise RuntimeError("non-visual demo requires a motor SHM owner")
                command_sequence, action = _command_frame(owner, plan.robot_id)
            capture.mark()
            response = runtime.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=action)
            )
            motor_state = capture.last_motor_state
            if not isinstance(motor_state, MotorStateFrame):
                raise RuntimeError("RobosuiteRuntimeModule did not publish motor_state")
            motor_state = cast("MotorStateFrame", motor_state)
            should_dump_jpeg = (
                rerun and camera_jpeg_dump_every > 0 and tick % camera_jpeg_dump_every == 0
            )
            if rerun and capture.last_color_image is not None:
                published_rerun_frames += 1
            _dump_stream_observation_image(
                capture,
                image_dump_dir=client_image_dump_dir if should_dump_jpeg else None,
                image_dump_label=f"tick_{tick:06d}",
            )
            if owner is not None:
                owner.write_state(
                    [
                        MotorState(
                            q=motor_state.q[i],
                            dq=motor_state.dq[i],
                            tau=motor_state.tau[i],
                        )
                        for i in range(len(plan.motor_names))
                    ],
                    sequence=motor_state.sequence,
                )
            trace.append(
                {
                    "tick": tick,
                    "command_sequence": command_sequence,
                    "state_sequence": motor_state.sequence,
                    "command_q": action.q,
                    "state_q": motor_state.q,
                    "observation_streams": list(capture.streams_since_mark),
                    "rerun_frames_published": published_rerun_frames,
                    "reward": response.reward,
                    "done": response.done,
                    "success": response.success,
                }
            )
            if response.done:
                break

        if config.visualize:
            print("visual demo complete; keeping Robosuite viewer open for 5 seconds")
            time.sleep(5.0)

        score = runtime.score()
        write_json(artifact_dir / "episode_config.json", config)
        write_json(artifact_dir / "runtime_description.json", description)
        write_json(artifact_dir / "resolved_runtime_plan.json", plan)
        write_json(artifact_dir / "reset_response.json", reset)
        write_json(artifact_dir / "motor_trace.json", trace)
        write_json(artifact_dir / "protocol_trace_summary.json", _trace_summary(trace))
        write_json(
            artifact_dir / "rerun_summary.json",
            {
                "enabled": rerun,
                "frames_published": published_rerun_frames,
                "memory_limit": rerun_memory_limit,
                "max_hz": rerun_max_hz,
                "grpc_port": selected_rerun_grpc_port,
                "lcm_port": selected_rerun_lcm_port,
                "client_jpeg_dump_dir": str(client_image_dump_dir)
                if rerun and camera_jpeg_dump_every > 0
                else None,
                "runtime_jpeg_dump_dir": str(runtime_image_dump_dir)
                if camera_jpeg_dump_every > 0
                else None,
                "jpeg_dump_every": camera_jpeg_dump_every,
            },
        )
        write_json(artifact_dir / "score.json", score)
        write_json(artifact_dir / "health.json", health)
        return artifact_dir
    finally:
        if rerun_bridge is not None:
            try:
                rerun_bridge.stop()
                cleanup_status["rerun_stopped"] = True
            except Exception as exc:
                cleanup_status["rerun_error"] = str(exc)
        if coordinator is not None:
            try:
                coordinator.stop()
                cleanup_status["coordinator_stopped"] = True
            except Exception as exc:
                cleanup_status["coordinator_error"] = str(exc)
        if owner is not None:
            try:
                owner.close()
                owner.unlink()
                cleanup_status["shm_unlinked"] = True
            except Exception as exc:
                cleanup_status["shm_error"] = str(exc)
        if module_coordinator is not None:
            try:
                module_coordinator.stop()
                cleanup_status["module_coordinator_stopped"] = True
                cleanup_status["runtime_stopped"] = True
            except Exception as exc:
                cleanup_status["module_coordinator_error"] = str(exc)
        else:
            try:
                runtime.stop()
                cleanup_status["runtime_stopped"] = True
            except Exception as exc:
                cleanup_status["runtime_error"] = str(exc)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        write_json(artifact_dir / "cleanup_status.json", cleanup_status)


if __name__ == "__main__":
    main()
