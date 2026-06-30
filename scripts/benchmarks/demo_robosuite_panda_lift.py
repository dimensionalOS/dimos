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

This script intentionally does not install or import Robosuite in the DimOS
process. It starts the Robosuite sidecar package in a subprocess and communicates
through the shared runtime protocol plus the local SHM motor bridge.
"""

from __future__ import annotations

import argparse
from io import BytesIO
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"

for package_src in (PROTOCOL_SRC, ROBOSUITE_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    MotorActionFrame,
    ObservationKind,
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
from dimos.simulation.runtime_client.http_client import RuntimeSidecarClient
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


def _load_config(path: Path) -> BenchmarkEpisodeConfig:
    return BenchmarkEpisodeConfig.model_validate_json(path.read_text())


def _sidecar_env() -> dict[str, str]:
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    paths = [str(PROTOCOL_SRC), str(ROBOSUITE_SIDECAR_SRC)]
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env


def _start_robosuite_sidecar(
    config: BenchmarkEpisodeConfig,
    *,
    server_image_dump_dir: Path | None = None,
    image_dump_every: int = 0,
) -> subprocess.Popen[str]:
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
    if server_image_dump_dir is not None and image_dump_every > 0:
        command.extend(
            [
                "--image-dump-dir",
                str(server_image_dump_dir),
                "--image-dump-every",
                str(image_dump_every),
            ]
        )
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


def _free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _lcm_url(port: int) -> str:
    return f"udpm://239.255.76.67:{port}?ttl=0"


class RerunStreamPublisher:
    """Publish runtime camera observations through DimOS streams for Rerun."""

    def __init__(
        self,
        *,
        grpc_port: int,
        lcm_port: int,
        memory_limit: str,
        max_hz: float,
        topic_prefix: str = "/robosuite_runtime",
    ) -> None:
        from dimos.core.transport import LCMTransport
        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
        from dimos.msgs.sensor_msgs.Image import Image
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
        from dimos.visualization.rerun.bridge import RerunBridgeModule

        prefix = topic_prefix.rstrip("/")
        self._image_topic = f"{prefix}/color_image"
        self._camera_info_topic = f"{prefix}/camera_info"
        self._image_entity = "world/robosuite_runtime/color_image"
        self._camera_info_entity = "world/robosuite_runtime/camera_info"
        self._camera_info_published = False

        def runtime_camera_blueprint() -> object:
            import rerun.blueprint as rrb

            return rrb.Blueprint(
                rrb.Vertical(
                    rrb.Spatial2DView(origin=self._image_entity, name="Robosuite camera"),
                )
            )

        def topic_to_entity(topic: object) -> str:
            topic_name = getattr(topic, "topic", None)
            if not isinstance(topic_name, str):
                topic_name = getattr(topic, "name", None)
            if not isinstance(topic_name, str):
                topic_name = str(topic).split("#")[0]
            if topic_name == self._image_topic:
                return self._image_entity
            if topic_name == self._camera_info_topic:
                return self._camera_info_entity
            return f"world{topic_name}"

        max_hz_by_entity = {self._image_entity: max_hz} if max_hz > 0.0 else {}
        lcm_url = _lcm_url(lcm_port)

        self._image_transport = LCMTransport(self._image_topic, Image, url=lcm_url)
        self._camera_info_transport = LCMTransport(self._camera_info_topic, CameraInfo, url=lcm_url)
        self._bridge = RerunBridgeModule(
            pubsubs=[LCM(url=lcm_url)],
            blueprint=runtime_camera_blueprint,
            connect_url=f"rerun+http://127.0.0.1:{grpc_port}/proxy",
            memory_limit=memory_limit,
            max_hz=max_hz_by_entity,
            topic_to_entity=topic_to_entity,
            visual_override={
                self._camera_info_entity: lambda camera_info: camera_info.to_rerun(
                    image_topic=self._image_entity
                ),
            },
        )

    def start(self) -> None:
        self._bridge.start()

    def stop(self) -> None:
        self._image_transport.stop()
        self._camera_info_transport.stop()
        self._bridge.stop()

    def publish_rgb(self, rgb: object, *, fov_y_deg: float, frame_id: str) -> None:
        import numpy as np

        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        array = np.asarray(rgb)
        if array.ndim != 3 or array.shape[2] < 3:
            raise ValueError(f"expected HxWx3 RGB image, got shape {array.shape}")
        # Leave Image.frame_id empty because the Rerun bridge logs the image at
        # the canonical entity path (`world/color_image`) while CameraInfo logs
        # the pinhole for that same entity. Setting a frame_id on the image would
        # attach a second transform parent and Rerun rejects that.
        image = Image.from_numpy(array[:, :, :3], format=ImageFormat.RGB, frame_id="")
        camera_info = CameraInfo.from_fov(
            fov_y_deg,
            width=image.width,
            height=image.height,
            axis="vertical",
            frame_id=frame_id,
        ).with_ts(image.ts)
        self._image_transport.broadcast(None, image)
        if not self._camera_info_published:
            self._camera_info_transport.broadcast(None, camera_info)
            self._camera_info_published = True


def _publish_rerun_observations(
    client: RuntimeSidecarClient,
    response_observations: object,
    publisher: RerunStreamPublisher | None,
    *,
    image_dump_dir: Path | None = None,
    image_dump_label: str = "frame",
) -> int:
    if publisher is None and image_dump_dir is None:
        return 0
    if not isinstance(response_observations, list):
        return 0
    published = 0
    for frame in response_observations:
        kind = getattr(frame, "kind", None)
        data_ref = getattr(frame, "data_ref", None)
        if kind != ObservationKind.IMAGE:
            continue
        if not isinstance(data_ref, str):
            continue
        payload = client.payload(data_ref)
        import numpy as np

        image = np.load(BytesIO(payload), allow_pickle=False)
        metadata = getattr(frame, "metadata", {})
        fov_y_deg = 45.0
        if isinstance(metadata, dict):
            maybe_fov = metadata.get("fov_y_deg")
            if isinstance(maybe_fov, int | float):
                fov_y_deg = float(maybe_fov)
            if metadata.get("image_convention") == "opengl":
                display_image = np.flipud(image)
            else:
                display_image = image
        else:
            display_image = image
        stream = getattr(frame, "stream", "camera")
        frame_id = stream if isinstance(stream, str) else "camera"
        if image_dump_dir is not None:
            _write_rgb_jpeg(image_dump_dir / f"{image_dump_label}_{frame_id}_raw.jpg", image)
            _write_rgb_jpeg(
                image_dump_dir / f"{image_dump_label}_{frame_id}_display.jpg",
                display_image,
            )
        if publisher is not None:
            publisher.publish_rgb(display_image, fov_y_deg=fov_y_deg, frame_id=frame_id)
            published += 1
    return published


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
        "--rerun",
        action="store_true",
        help="Publish Robosuite camera payloads to Rerun through DimOS streams.",
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
            "When --rerun is enabled, write every Nth fetched Robosuite camera payload "
            "as raw/display JPEGs under artifacts/.../images. Use 1 for every tick, <=0 to disable."
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
) -> Path:
    return _run_demo(
        config,
        rerun=rerun,
        rerun_memory_limit=rerun_memory_limit,
        rerun_grpc_port=rerun_grpc_port,
        rerun_lcm_port=rerun_lcm_port,
        rerun_max_hz=rerun_max_hz,
        camera_jpeg_dump_every=camera_jpeg_dump_every,
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
) -> Path:
    artifact_dir = (REPO_ROOT / config.artifact_dir).resolve()
    client_image_dump_dir = artifact_dir / "images" / "client"
    server_image_dump_dir = artifact_dir / "images" / "server"
    sidecar = _start_robosuite_sidecar(
        config,
        server_image_dump_dir=server_image_dump_dir if camera_jpeg_dump_every > 0 else None,
        image_dump_every=camera_jpeg_dump_every,
    )
    client = RuntimeSidecarClient(f"http://{config.runtime_host}:{config.runtime_port}")
    owner: MotorShmOwner | None = None
    coordinator: ControlCoordinator | None = None
    rerun_publisher: RerunStreamPublisher | None = None
    sidecar_output = ""
    cleanup_status: dict[str, object] = {
        "coordinator_stopped": False,
        "shm_unlinked": False,
        "sidecar_stopped": False,
    }
    selected_rerun_grpc_port: int | None = None
    selected_rerun_lcm_port: int | None = None
    try:
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
            EpisodeResetRequest(
                episode_id=plan.episode_id,
                task_id=plan.task_id,
                seed=config.seed,
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
            rerun_publisher = RerunStreamPublisher(
                grpc_port=selected_rerun_grpc_port,
                lcm_port=selected_rerun_lcm_port,
                memory_limit=rerun_memory_limit,
                max_hz=rerun_max_hz,
            )
            rerun_publisher.start()
            if camera_jpeg_dump_every > 0:
                _publish_rerun_observations(
                    client,
                    reset.observations,
                    None,
                    image_dump_dir=client_image_dump_dir,
                    image_dump_label="reset",
                )

        trace: list[dict[str, object]] = []
        published_rerun_frames = 0
        for tick in range(plan.ticks):
            target = _target_for_tick(
                plan.target_position, len(plan.motor_names), tick, config.visualize
            )
            accepted = coordinator.task_invoke(
                task_name, "set_target", {"positions": target, "t_now": None}
            )
            if accepted is not True:
                raise RuntimeError(f"servo task rejected target at tick {tick}")
            time.sleep(1.0 / plan.control_step_hz)
            command_sequence, action = _command_frame(owner, plan.robot_id)
            response = client.step(
                StepRequest(episode_id=plan.episode_id, tick_id=tick, action=action)
            )
            should_dump_jpeg = (
                rerun and camera_jpeg_dump_every > 0 and tick % camera_jpeg_dump_every == 0
            )
            published_rerun_frames += _publish_rerun_observations(
                client,
                response.observations,
                rerun_publisher,
                image_dump_dir=client_image_dump_dir if should_dump_jpeg else None,
                image_dump_label=f"tick_{tick:06d}",
            )
            owner.write_state(
                [
                    MotorState(
                        q=response.motor_state.q[i],
                        dq=response.motor_state.dq[i],
                        tau=response.motor_state.tau[i],
                    )
                    for i in range(len(plan.motor_names))
                ],
                sequence=response.motor_state.sequence,
            )
            trace.append(
                {
                    "tick": tick,
                    "command_sequence": command_sequence,
                    "state_sequence": response.motor_state.sequence,
                    "command_q": action.q,
                    "state_q": response.motor_state.q,
                    "observation_streams": [frame.stream for frame in response.observations],
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

        score = client.score()
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
                "server_jpeg_dump_dir": str(server_image_dump_dir)
                if camera_jpeg_dump_every > 0
                else None,
                "jpeg_dump_every": camera_jpeg_dump_every,
            },
        )
        write_json(artifact_dir / "score.json", score)
        write_json(artifact_dir / "health.json", health)
        return artifact_dir
    finally:
        if rerun_publisher is not None:
            try:
                rerun_publisher.stop()
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
        sidecar.terminate()
        try:
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            sidecar.kill()
            sidecar_output, _ = sidecar.communicate(timeout=2.0)
        cleanup_status["sidecar_returncode"] = sidecar.returncode
        cleanup_status["sidecar_stopped"] = sidecar.returncode is not None
        sidecar_log = artifact_dir / "robosuite_sidecar.log"
        sidecar_log.parent.mkdir(parents=True, exist_ok=True)
        sidecar_log.write_text(sidecar_output)
        write_json(sidecar_log.parent / "cleanup_status.json", cleanup_status)


if __name__ == "__main__":
    main()
