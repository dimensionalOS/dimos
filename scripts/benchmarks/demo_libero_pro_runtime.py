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

"""Run a LIBERO-PRO registered-task demo through the DimOS runtime module path.

This script runs from the host DimOS environment and deploys the first-class
``LiberoProRuntimeModule`` into the package-local LIBERO-PRO Python project
runtime. The simulator dependencies stay in the placed worker environment while
the host process drives DimOS-native RPCs and streams.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Protocol

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
LIBERO_PRO_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-libero-pro-sidecar" / "src"

for package_src in (PROTOCOL_SRC, LIBERO_PRO_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    MotorActionFrame,
    MotorStateFrame,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)

from dimos.benchmark.runtime.artifacts import write_json
from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    LiberoProBackendOptions,
    resolve_runtime_plan,
    validate_libero_pro_backend_options,
)
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.whole_body.spec import MotorState
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


class _Subscribable(Protocol):
    def subscribe(self, cb: Callable[..., object]) -> object: ...


class _ImageMessage(Protocol):
    data: object
    frame_id: str


class _LiberoProRuntime(Protocol):
    color_image: _Subscribable
    camera_info: _Subscribable
    motor_state: _Subscribable
    runtime_event: _Subscribable

    def describe(self) -> RuntimeDescription: ...

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse: ...

    def step(self, request: StepRequest) -> StepResponse: ...

    def score(self) -> ScoreOutput: ...

    def stop(self) -> None: ...


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _sidecar_env(*, visualize: bool = False) -> dict[str, str]:
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    paths = [str(PROTOCOL_SRC), str(LIBERO_PRO_SIDECAR_SRC)]
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    env.setdefault("MUJOCO_GL", "glfw" if visualize else "egl")
    return env


def _sidecar_python() -> str:
    return os.environ.get("DIMOS_LIBERO_PRO_SIDECAR_PYTHON", sys.executable)


def _repo_path(path: Path) -> Path:
    return path if path.is_absolute() else REPO_ROOT / path


def _prepare_assets(config: BenchmarkEpisodeConfig, options: LiberoProBackendOptions) -> None:
    subprocess.run(
        [
            _sidecar_python(),
            "-m",
            "dimos_libero_pro_sidecar.assets",
            "bootstrap",
            "--benchmark-name",
            options.benchmark_name,
            "--bddl-root",
            str(_repo_path(options.bddl_root)),
            "--init-states-root",
            str(_repo_path(options.init_states_root)),
            "--task-index",
            str(options.task_index),
        ],
        cwd=REPO_ROOT,
        env=_sidecar_env(),
        check=True,
    )


def _create_runtime_module(
    config: BenchmarkEpisodeConfig,
    options: LiberoProBackendOptions,
) -> tuple[_LiberoProRuntime, ModuleCoordinator]:
    from dimos_libero_pro_sidecar.blueprint import libero_pro_runtime_blueprint
    from dimos_libero_pro_sidecar.module import LiberoProRuntimeModule

    blueprint = libero_pro_runtime_blueprint(
        bddl_root=_repo_path(options.bddl_root),
        init_states_root=_repo_path(options.init_states_root),
        benchmark_name=options.benchmark_name,
        robot_id=config.robot_id,
        task_order_index=options.task_order_index,
        task_index=options.task_index,
        init_state_index=options.init_state_index,
        controller=options.controller,
        camera_names=tuple(options.camera_names),
        control_freq=config.control_step_hz,
        horizon=options.horizon,
        seed=config.seed,
        allow_asset_bootstrap=False,
        visualize=config.visualize,
    ).global_config(viewer="none", n_workers=1)
    coordinator = ModuleCoordinator.build(blueprint)
    return coordinator.get_instance(LiberoProRuntimeModule), coordinator


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
                    rrb.Spatial2DView(origin=self._image_entity, name="LIBERO-PRO camera"),
                )
            )

        max_hz_by_entity = {self._image_entity: max_hz} if max_hz > 0.0 else {}
        self._bridge = RerunBridgeModule(
            blueprint=runtime_camera_blueprint,
            connect_url=f"rerun+http://127.0.0.1:{grpc_port}/proxy",
            memory_limit=memory_limit,
            max_hz=max_hz_by_entity,
            visual_override={
                self._camera_info_entity: lambda camera_info: camera_info.to_rerun(
                    image_topic=self._image_entity
                ),
            },
        )

    def start(self) -> None:
        self._bridge.start()

    def stop(self) -> None:
        self._bridge.stop()


def _target_for_tick(plan_target: float, motor_count: int, tick: int) -> list[float]:
    phase = 1.0 if (tick // 50) % 2 == 0 else -1.0
    arm_pattern = [1.0, -0.8, 0.6, -0.4, 0.3, -0.2, 0.1]
    targets = [plan_target * phase * scale for scale in arm_pattern[:motor_count]]
    if motor_count > 7:
        targets.extend([plan_target * phase] * (motor_count - 7))
    return targets[:motor_count]


def _safe_stream_name(stream_name: str) -> str:
    return stream_name.strip("/").replace("/", "_") or "camera"


def _write_rgb_jpeg(path: Path, rgb: object) -> None:
    import cv2

    array = np.asarray(rgb)
    if array.ndim != 3 or array.shape[2] < 3:
        raise ValueError(f"expected HxWx3 RGB image, got shape {array.shape}")
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(array[:, :, :3], cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise RuntimeError(f"failed to write JPEG {path}")


class RuntimeStreamCapture:
    """Collect DimOS-native outputs published by the local runtime module."""

    def __init__(self) -> None:
        self.images: list[_ImageMessage] = []
        self.camera_infos: list[object] = []
        self.motor_states: list[MotorStateFrame] = []
        self.runtime_events: list[object] = []

    def attach(self, module: _LiberoProRuntime) -> None:
        module.color_image.subscribe(self.images.append)
        module.camera_info.subscribe(self.camera_infos.append)
        module.motor_state.subscribe(self.motor_states.append)
        module.runtime_event.subscribe(self.runtime_events.append)

    def take_images(self) -> list[_ImageMessage]:
        images = self.images
        self.images = []
        return images

    def take_camera_infos(self) -> list[object]:
        camera_infos = self.camera_infos
        self.camera_infos = []
        return camera_infos

    def take_motor_states(self) -> list[MotorStateFrame]:
        motor_states = self.motor_states
        self.motor_states = []
        return motor_states

    def take_runtime_events(self) -> list[object]:
        runtime_events = self.runtime_events
        self.runtime_events = []
        return runtime_events


def _camera_image_records(
    images: list[_ImageMessage],
    image_dir: Path,
    label: str,
) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for index, image in enumerate(images):
        array = np.asarray(image.data)
        frame_id = image.frame_id or "camera"
        image_path = image_dir / f"{label}_{_safe_stream_name(str(frame_id))}_{index:02d}.npy"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(image_path, array, allow_pickle=False)
        records.append(
            {
                "stream": frame_id,
                "path": str(image_path),
                "shape": list(array.shape),
                "dtype": str(array.dtype),
                "min": float(array.min()) if array.size else 0.0,
                "max": float(array.max()) if array.size else 0.0,
            }
        )
    return records


def _dump_stream_images(
    images: list[_ImageMessage],
    *,
    image_dump_dir: Path | None = None,
    image_dump_label: str = "frame",
) -> int:
    if image_dump_dir is None:
        return 0
    published = 0
    for index, image_msg in enumerate(images):
        image = np.asarray(image_msg.data)
        display_image = image
        frame_id = image_msg.frame_id or "camera"
        if image_dump_dir is not None:
            _write_rgb_jpeg(
                image_dump_dir / f"{image_dump_label}_{frame_id}_{index:02d}_raw.jpg", image
            )
            _write_rgb_jpeg(
                image_dump_dir / f"{image_dump_label}_{frame_id}_{index:02d}_display.jpg",
                display_image,
            )
        published += 1
    return published


def _trace_summary(trace: list[dict[str, object]]) -> dict[str, object]:
    final = trace[-1] if trace else {}
    stream_set: set[str] = set()
    image_count = 0
    for entry in trace:
        value = entry.get("observation_streams", [])
        if isinstance(value, list):
            stream_set.update(stream for stream in value if isinstance(stream, str))
        images = entry.get("camera_images", [])
        if isinstance(images, list):
            image_count += len(images)
    return {
        "ticks": len(trace),
        "first_command_sequence": trace[0].get("command_sequence") if trace else None,
        "final_command_sequence": final.get("command_sequence"),
        "final_state_sequence": final.get("state_sequence"),
        "final_command_q": final.get("command_q"),
        "final_state_q": final.get("state_q"),
        "observation_streams": sorted(stream_set),
        "camera_image_count": image_count,
        "final_reward": final.get("reward"),
        "final_done": final.get("done"),
        "final_success": final.get("success"),
    }


def run_demo_config(
    config: BenchmarkEpisodeConfig,
    *,
    prepare_assets: bool = False,
    rerun: bool = False,
    rerun_memory_limit: str = "128MB",
    rerun_grpc_port: int = 0,
    rerun_lcm_port: int = 0,
    rerun_max_hz: float = 10.0,
    camera_jpeg_dump_every: int = 25,
) -> Path:
    options = validate_libero_pro_backend_options(config)
    if prepare_assets:
        _prepare_assets(config, options)
    artifact_dir = (REPO_ROOT / config.artifact_dir).resolve()
    image_dir = artifact_dir / "camera_images"
    client_image_dump_dir = artifact_dir / "images" / "client"
    runtime, module_coordinator = _create_runtime_module(config, options)
    stream_capture = RuntimeStreamCapture()
    stream_capture.attach(runtime)
    owner: MotorShmOwner | None = None
    coordinator: ControlCoordinator | None = None
    rerun_bridge: RuntimeRerunBridge | None = None
    selected_rerun_grpc_port: int | None = None
    selected_rerun_lcm_port: int | None = None
    published_rerun_frames = 0
    cleanup_status: dict[str, object] = {
        "coordinator_stopped": False,
        "module_coordinator_stopped": False,
        "shm_unlinked": False,
        "runtime_stopped": False,
    }
    try:
        try:
            description = runtime.describe()
        except RuntimeError as exc:
            raise RuntimeError(
                "LIBERO-PRO runtime module could not start. Prepare the "
                "packages/dimos-libero-pro-sidecar runtime environment and BDDL/init "
                "assets first (or pass --prepare-assets for explicit asset bootstrap)."
            ) from exc
        plan = resolve_runtime_plan(config, description)
        reset = runtime.reset(
            EpisodeResetRequest(
                episode_id=plan.episode_id,
                task_id=plan.task_id,
                seed=config.seed,
                options=config.backend_options,
            )
        )

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
        task_name = f"servo_{plan.robot_id}"
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
                    params={"timeout": 0.0, "default_positions": [0.0] * len(plan.motor_names)},
                )
            ],
        )
        coordinator.start()
        if rerun:
            selected_rerun_grpc_port = rerun_grpc_port if rerun_grpc_port > 0 else _free_tcp_port()
            selected_rerun_lcm_port = rerun_lcm_port if rerun_lcm_port > 0 else _free_tcp_port()
            rerun_bridge = RuntimeRerunBridge(
                grpc_port=selected_rerun_grpc_port,
                lcm_port=selected_rerun_lcm_port,
                memory_limit=rerun_memory_limit,
                max_hz=rerun_max_hz,
            )
            rerun_bridge.start()

        trace: list[dict[str, object]] = []
        reset_images = stream_capture.take_images()
        reset_camera_infos = stream_capture.take_camera_infos()
        reset_events = stream_capture.take_runtime_events()
        reset_image_records = _camera_image_records(reset_images, image_dir, "reset")
        if rerun and camera_jpeg_dump_every > 0:
            _dump_stream_images(
                reset_images,
                image_dump_dir=client_image_dump_dir,
                image_dump_label="reset",
            )
        for tick in range(plan.ticks):
            target = _target_for_tick(plan.target_position, len(plan.motor_names), tick)
            accepted = coordinator.task_invoke(
                task_name, "set_target", {"positions": target, "t_now": None}
            )
            if accepted is not True:
                raise RuntimeError(f"servo task rejected target at tick {tick}")
            time.sleep(1.0 / plan.control_step_hz)
            command_sequence, action = _command_frame(owner, plan.robot_id)
            response = runtime.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=action)
            )
            tick_images = stream_capture.take_images()
            tick_camera_infos = stream_capture.take_camera_infos()
            tick_motor_states = stream_capture.take_motor_states()
            tick_runtime_events = stream_capture.take_runtime_events()
            camera_images = _camera_image_records(tick_images, image_dir, f"tick_{tick:06d}")
            should_dump_jpeg = (
                rerun and camera_jpeg_dump_every > 0 and tick % camera_jpeg_dump_every == 0
            )
            if rerun:
                published_rerun_frames += len(tick_images)
            _dump_stream_images(
                tick_images,
                image_dump_dir=client_image_dump_dir if should_dump_jpeg else None,
                image_dump_label=f"tick_{tick:06d}",
            )
            stream_motor_state = (
                tick_motor_states[-1] if tick_motor_states else response.motor_state
            )
            owner.write_state(
                [
                    MotorState(
                        q=stream_motor_state.q[i],
                        dq=stream_motor_state.dq[i],
                        tau=stream_motor_state.tau[i],
                    )
                    for i in range(len(plan.motor_names))
                ],
                sequence=stream_motor_state.sequence,
            )
            trace.append(
                {
                    "tick": tick,
                    "command_sequence": command_sequence,
                    "state_sequence": stream_motor_state.sequence,
                    "command_q": action.q,
                    "state_q": stream_motor_state.q,
                    "observation_streams": [str(record["stream"]) for record in camera_images],
                    "camera_images": camera_images,
                    "camera_info_count": len(tick_camera_infos),
                    "runtime_event_count": len(tick_runtime_events),
                    "rerun_frames_published": published_rerun_frames,
                    "reward": response.reward,
                    "done": response.done,
                    "success": response.success,
                }
            )
            if response.done:
                break

        if config.visualize:
            print("visual demo complete; keeping LIBERO-PRO viewer open for 5 seconds")
            time.sleep(5.0)

        score = runtime.score()
        write_json(artifact_dir / "episode_config.json", config)
        write_json(artifact_dir / "runtime_description.json", description)
        write_json(artifact_dir / "resolved_runtime_plan.json", plan)
        write_json(artifact_dir / "reset_response.json", reset)
        write_json(
            artifact_dir / "reset_runtime_streams.json",
            {
                "image_count": len(reset_images),
                "camera_info_count": len(reset_camera_infos),
                "runtime_event_count": len(reset_events),
            },
        )
        write_json(artifact_dir / "reset_camera_images.json", reset_image_records)
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
                "jpeg_dump_every": camera_jpeg_dump_every,
            },
        )
        write_json(artifact_dir / "score.json", score)
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
        try:
            module_coordinator.stop()
            cleanup_status["module_coordinator_stopped"] = True
            cleanup_status["runtime_stopped"] = True
        except Exception as exc:
            cleanup_status["runtime_error"] = str(exc)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        write_json(artifact_dir / "cleanup_status.json", cleanup_status)


def run_demo(config_path: Path) -> Path:
    return run_demo_config(_load_config(config_path), prepare_assets=False)


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
        / "libero_pro_goal_task0.json",
    )
    parser.add_argument(
        "--prepare-assets",
        action="store_true",
        help="Run the explicit LIBERO-PRO asset bootstrap before runtime module startup.",
    )
    parser.add_argument(
        "--visual",
        action="store_true",
        help="Open the LIBERO/Robosuite viewer and run a longer moving command sequence.",
    )
    parser.add_argument("--ticks", type=int, default=None, help="Override configured tick count.")
    parser.add_argument(
        "--horizon",
        type=int,
        default=None,
        help="Override LIBERO-PRO episode horizon.",
    )
    parser.add_argument(
        "--target-position",
        type=float,
        default=None,
        help="Override configured joint-position target amplitude.",
    )
    parser.add_argument(
        "--camera-name",
        action="append",
        dest="camera_names",
        help="Override LIBERO-PRO camera name. Repeat for multiple cameras.",
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Publish LIBERO-PRO camera stream outputs to Rerun through DimOS streams.",
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
        help="Rerun gRPC port for --rerun. 0 selects a free port.",
    )
    parser.add_argument(
        "--rerun-lcm-port",
        type=int,
        default=0,
        help="Private LCM multicast port for --rerun streams. 0 selects a free port.",
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
        help="When --rerun is enabled, dump every Nth camera stream output as JPEGs. Use <=0 to disable.",
    )
    args = parser.parse_args()
    try:
        config = _load_config(args.config)
        updates: dict[str, object] = {}
        if args.visual:
            updates["visualize"] = True
            ticks = args.ticks if args.ticks is not None else max(config.ticks, 600)
            updates["ticks"] = ticks
            updates["target_position"] = (
                args.target_position
                if args.target_position is not None
                else max(abs(config.target_position), 0.9)
            )
            backend_options = dict(config.backend_options)
            backend_options["horizon"] = (
                args.horizon
                if args.horizon is not None
                else max(int(backend_options.get("horizon", 1000)), ticks + 1)
            )
            updates["backend_options"] = backend_options
        elif args.rerun and args.target_position is None:
            # The default config target is intentionally tiny for smoke tests.
            # Live verification should visibly move the arm, not only the gripper.
            updates["target_position"] = max(abs(config.target_position), 0.9)
        if args.ticks is not None:
            updates["ticks"] = args.ticks
        if args.target_position is not None:
            updates["target_position"] = args.target_position
        if args.horizon is not None and not args.visual:
            backend_options = dict(config.backend_options)
            backend_options["horizon"] = args.horizon
            updates["backend_options"] = backend_options
        if args.camera_names:
            existing_backend_options = updates.get("backend_options")
            if isinstance(existing_backend_options, dict):
                backend_options = dict(existing_backend_options)
            else:
                backend_options = dict(config.backend_options)
            backend_options["camera_names"] = args.camera_names
            updates["backend_options"] = backend_options
        if updates:
            config = BenchmarkEpisodeConfig.model_validate({**config.model_dump(), **updates})
        artifact_dir = run_demo_config(
            config,
            prepare_assets=args.prepare_assets,
            rerun=args.rerun,
            rerun_memory_limit=args.rerun_memory_limit,
            rerun_grpc_port=args.rerun_grpc_port,
            rerun_lcm_port=args.rerun_lcm_port,
            rerun_max_hz=args.rerun_max_hz,
            camera_jpeg_dump_every=args.camera_jpeg_dump_every,
        )
    except Exception as exc:
        print(json.dumps({"ok": False, "reason": str(exc)}, indent=2))
        sys.exit(2)
    print(json.dumps({"ok": True, "artifact_dir": str(artifact_dir)}, indent=2))


if __name__ == "__main__":
    main()
