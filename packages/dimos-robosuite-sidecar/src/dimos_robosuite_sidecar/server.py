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

"""HTTP sidecar for a narrow Robosuite Panda Lift runtime profile."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
import json
from pathlib import Path
import time
from types import ModuleType
from typing import ClassVar, Mapping, Sequence, cast
from urllib.parse import unquote, urlparse

from dimos_runtime_protocol import (
    CommandMode,
    EpisodeResetRequest,
    EpisodeResetResponse,
    HealthResponse,
    MotorDescription,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    ProtocolVersion,
    RobotMotorSurface,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)


def require_robosuite() -> ModuleType:
    """Import Robosuite only inside the sidecar runtime path."""

    try:
        import robosuite
    except ImportError as exc:
        raise RuntimeError(
            "Robosuite is required for dimos-robosuite-sidecar. "
            "Install the sidecar in a Robosuite-compatible environment."
        ) from exc
    return robosuite


@dataclass(frozen=True)
class RobosuiteRuntimeConfig:
    host: str
    port: int
    env_name: str
    robot_id: str
    robot_model: str
    controller: str
    control_freq: int
    horizon: int
    camera_name: str
    seed: int | None
    visualize: bool = False
    image_dump_dir: str | None = None
    image_dump_every: int = 0


class RobosuiteRuntimeState:
    """Owns one Robosuite env and maps it to the DimOS runtime protocol."""

    def __init__(self, config: RobosuiteRuntimeConfig) -> None:
        self.config = config
        self._robosuite = require_robosuite()
        self._env = self._make_env()
        self._episode_id = ""
        self._sequence = 0
        self._last_obs: Mapping[str, object] = {}
        self._last_reward = 0.0
        self._last_done = False
        self._last_success: bool | None = None
        self._payloads: dict[str, bytes] = {}
        self._action_low, self._action_high = self._action_bounds()
        self.motor_names = self._motor_names()
        if len(self._action_low) != len(self.motor_names):
            raise RuntimeError(
                "Panda Lift v1 profile expects action dimension to match motor surface: "
                f"action_dim={len(self._action_low)} motors={len(self.motor_names)}"
            )

    def _make_env(self) -> object:
        robosuite = self._robosuite
        controllers = getattr(robosuite, "controllers")
        controller_config = self._controller_config(controllers)
        make_env = getattr(robosuite, "make")
        return make_env(
            env_name=self.config.env_name,
            robots=self.config.robot_model,
            controller_configs=controller_config,
            has_renderer=self.config.visualize,
            has_offscreen_renderer=True,
            use_camera_obs=True,
            camera_names=[self.config.camera_name],
            # Use free-camera mode for the on-screen viewer so the user can
            # rotate / pan / zoom interactively. The named camera remains enabled
            # for observation frames (`agentview` by default).
            render_camera=None if self.config.visualize else self.config.camera_name,
            camera_depths=False,
            control_freq=self.config.control_freq,
            horizon=self.config.horizon,
            seed=self.config.seed,
        )

    def _controller_config(self, controllers: object) -> object:
        load_composite = getattr(controllers, "load_composite_controller_config")
        controller_config = load_composite(controller="BASIC")
        if self.config.controller in {"JOINT_POSITION", "PANDA_JOINT_POSITION"}:
            load_part = getattr(controllers, "load_part_controller_config")
            right_config = load_part(default_controller="JOINT_POSITION")
            right_config["gripper"] = {"type": "GRIP"}
            controller_config["body_parts"]["right"] = right_config
            return controller_config
        if self.config.controller == "BASIC":
            return controller_config
        raise RuntimeError(f"unsupported Robosuite controller profile: {self.config.controller}")

    def _action_bounds(self) -> tuple[list[float], list[float]]:
        low, high = self._env.action_spec  # type: ignore[attr-defined]
        return _float_list(low), _float_list(high)

    def _motor_names(self) -> list[str]:
        # Narrow v1 profile: Panda arm joints plus scalar gripper command.
        if self.config.robot_model != "Panda":
            raise RuntimeError(f"unsupported robot model for v1 Robosuite sidecar: {self.config.robot_model}")
        return [f"{self.config.robot_id}/joint{i + 1}" for i in range(7)] + [
            f"{self.config.robot_id}/gripper"
        ]

    def describe(self) -> RuntimeDescription:
        return RuntimeDescription(
            runtime_id="robosuite-panda-lift",
            backend="robosuite",
            protocol=ProtocolVersion(),
            capabilities=["sync-http", "whole-body-motor-position", "robosuite-panda-lift"],
            robot_surfaces=[
                RobotMotorSurface(
                    robot_id=self.config.robot_id,
                    motors=[
                        MotorDescription(name=name, index=index)
                        for index, name in enumerate(self.motor_names)
                    ],
                    supported_command_modes=[CommandMode.POSITION],
                )
            ],
            control_step_hz=self.config.control_freq,
            observation_streams=[self.config.camera_name, "robot_state"],
            metadata={
                "env_name": self.config.env_name,
                "robot_model": self.config.robot_model,
                "controller": self.config.controller,
                "horizon": self.config.horizon,
                "visualize": self.config.visualize,
                "action_low": self._action_low,
                "action_high": self._action_high,
            },
        )

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        self._episode_id = request.episode_id
        self._sequence = 0
        self._last_obs = cast(Mapping[str, object], self._env.reset())  # type: ignore[attr-defined]
        self._last_reward = 0.0
        self._last_done = False
        self._last_success = None
        # Store protocol payloads before updating the interactive viewer. In
        # visual mode Robosuite/MuJoCo viewer updates can touch render buffers
        # that observation arrays may reference, causing alternating/stale
        # colors in the exported camera payloads.
        observations = self._observations(0)
        self._render_if_enabled()
        return EpisodeResetResponse(
            episode_id=request.episode_id,
            runtime_description=self.describe(),
            observations=observations,
        )

    def step(self, request: StepRequest) -> StepResponse:
        if request.action.robot_id != self.config.robot_id:
            raise ValueError(f"unexpected robot id {request.action.robot_id!r}")
        if request.action.names != self.motor_names:
            raise ValueError("action motor names do not match runtime surface")
        if request.action.mode != CommandMode.POSITION:
            raise ValueError(f"unsupported command mode {request.action.mode}")
        action = self._action_from_request(request)
        obs, reward, done, info = self._env.step(action)  # type: ignore[attr-defined]
        self._sequence += 1
        self._last_obs = cast(Mapping[str, object], obs)
        self._last_reward = float(reward)
        self._last_done = bool(done)
        self._last_success = _success_from_info(info)
        motor_state = self._motor_state(request.tick_id)
        # Store protocol payloads before updating the interactive viewer; see
        # reset() for the render-buffer ordering rationale.
        observations = self._observations(request.tick_id)
        self._render_if_enabled()
        return StepResponse(
            episode_id=request.episode_id,
            tick_id=request.tick_id,
            motor_state=motor_state,
            observations=observations,
            reward=self._last_reward,
            done=self._last_done,
            success=self._last_success,
            info={"robosuite_info_keys": sorted(str(key) for key in info.keys())},
        )

    def _render_if_enabled(self) -> None:
        if not self.config.visualize:
            return
        render = getattr(self._env, "render")
        render()

    def score(self) -> ScoreOutput:
        success = bool(self._last_success) if self._last_success is not None else False
        return ScoreOutput(
            episode_id=self._episode_id or "uninitialized",
            success=success,
            score=1.0 if success else 0.0,
            reason="robosuite success flag" if success else "success not observed",
            metrics={"reward": self._last_reward, "done": self._last_done, "sequence": self._sequence},
        )

    def _action_from_request(self, request: StepRequest) -> object:
        import numpy as np

        if len(request.action.q) != len(self.motor_names):
            raise ValueError(f"expected {len(self.motor_names)} q targets, got {len(request.action.q)}")
        values = np.array(request.action.q, dtype=np.float64)
        low = np.array(self._action_low, dtype=np.float64)
        high = np.array(self._action_high, dtype=np.float64)
        return np.clip(values, low, high)

    def _motor_state(self, tick_id: int) -> MotorStateFrame:
        joint_q = _float_list(self._last_obs.get("robot0_joint_pos", []))
        joint_dq = _float_list(self._last_obs.get("robot0_joint_vel", []))
        gripper_q = _float_list(self._last_obs.get("robot0_gripper_qpos", []))
        gripper_dq = _float_list(self._last_obs.get("robot0_gripper_qvel", []))
        q = (joint_q[:7] + [_mean_or_zero(gripper_q)])[: len(self.motor_names)]
        dq = (joint_dq[:7] + [_mean_or_zero(gripper_dq)])[: len(self.motor_names)]
        if len(q) != len(self.motor_names):
            q = q + [0.0] * (len(self.motor_names) - len(q))
        if len(dq) != len(self.motor_names):
            dq = dq + [0.0] * (len(self.motor_names) - len(dq))
        return MotorStateFrame(
            robot_id=self.config.robot_id,
            names=self.motor_names,
            q=q,
            dq=dq,
            tau=[0.0] * len(self.motor_names),
            sequence=self._sequence,
            timestamp_s=time.time(),
        )

    def _observations(self, tick_id: int) -> list[ObservationFrame]:
        frames = [
            ObservationFrame(
                stream="robot_state",
                kind=ObservationKind.STATE,
                inline_text=f"tick={tick_id} reward={self._last_reward}",
                metadata={"sequence": self._sequence},
            )
        ]
        image = self._camera_image()
        if image is not None:
            payload_id, payload_bytes = self._store_payload(self.config.camera_name, tick_id, image)
            frames.append(
                ObservationFrame(
                    stream=self.config.camera_name,
                    kind=ObservationKind.IMAGE,
                    encoding="npy",
                    shape=_shape_list(image),
                    dtype=str(getattr(image, "dtype", "")),
                    data_ref=f"/payloads/{payload_id}",
                    metadata={
                        "sequence": self._sequence,
                        "camera_name": self.config.camera_name,
                        "camera_mount": _camera_mount(self.config.camera_name),
                        "camera_source": "robosuite_observation",
                        "image_convention": self._image_convention(),
                        "fov_y_deg": self._camera_fov_deg(),
                        **_array_metadata(image, payload_bytes),
                    },
                )
            )
        return frames

    def _camera_image(self) -> object | None:
        """Return a copied Robosuite camera observation.

        Robosuite's interactive viewer and direct `sim.render(...)` paths can use
        different render contexts. For the protocol payload, use the observation
        produced by `env.reset()` / `env.step()` and copy it immediately so later
        viewer updates cannot mutate the exported array.
        """

        cached_image = self._last_obs.get(f"{self.config.camera_name}_image")
        if cached_image is None:
            return None
        return _array_copy(cached_image)

    def _store_payload(self, stream: str, tick_id: int, value: object) -> tuple[str, bytes]:
        payload_id = f"{stream}-{tick_id:06d}-{self._sequence:06d}.npy"
        payload_bytes = _npy_bytes(value)
        self._payloads[payload_id] = payload_bytes
        self._dump_source_image_if_enabled(payload_id.removesuffix(".npy"), value)
        if len(self._payloads) > 32:
            for key in list(self._payloads)[: len(self._payloads) - 32]:
                del self._payloads[key]
        return payload_id, payload_bytes

    def _dump_source_image_if_enabled(self, stem: str, value: object) -> None:
        if self.config.image_dump_dir is None or self.config.image_dump_every <= 0:
            return
        if self._sequence % self.config.image_dump_every != 0:
            return
        output_dir = Path(self.config.image_dump_dir)
        _write_rgb_jpeg(output_dir / f"{stem}_server_raw.jpg", value)
        if self._image_convention() == "opengl":
            import numpy as np

            _write_rgb_jpeg(output_dir / f"{stem}_server_display.jpg", np.flipud(np.asarray(value)))
        else:
            _write_rgb_jpeg(output_dir / f"{stem}_server_display.jpg", value)

    def payload_bytes(self, payload_id: str) -> bytes:
        try:
            return self._payloads[payload_id]
        except KeyError as exc:
            raise FileNotFoundError(f"unknown payload id {payload_id!r}") from exc

    def _camera_fov_deg(self) -> float:
        try:
            sim = getattr(self._env, "sim")
            model = getattr(sim, "model")
            camera_id = model.camera_name2id(self.config.camera_name)
            return float(model.cam_fovy[camera_id])
        except Exception:
            return 45.0

    def _image_convention(self) -> str:
        macros = getattr(self._robosuite, "macros", None)
        return str(getattr(macros, "IMAGE_CONVENTION", "opengl"))


class RobosuiteRuntimeHandler(BaseHTTPRequestHandler):
    state: ClassVar[RobosuiteRuntimeState]

    def do_GET(self) -> None:
        if self.path == "/health":
            self._write_model(
                HealthResponse(ok=True, runtime_id="robosuite-panda-lift", protocol=ProtocolVersion())
            )
        elif self.path == "/describe":
            self._write_model(self.state.describe())
        elif self.path == "/score":
            self._write_model(self.state.score())
        elif self.path.startswith("/payloads/"):
            payload_id = unquote(urlparse(self.path).path.removeprefix("/payloads/"))
            try:
                self._write_bytes(self.state.payload_bytes(payload_id), content_type="application/x-npy")
            except FileNotFoundError:
                self.send_error(HTTPStatus.NOT_FOUND)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            body = self.rfile.read(int(self.headers.get("content-length", "0"))).decode("utf-8")
            payload = json.loads(body) if body else {}
            if self.path == "/reset":
                self._write_model(self.state.reset(EpisodeResetRequest.model_validate(payload)))
            elif self.path == "/step":
                self._write_model(self.state.step(StepRequest.model_validate(payload)))
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.send_header("content-type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"code": "bad_request", "message": str(exc)}).encode())

    def log_message(self, format: str, *args: object) -> None:
        return

    def _write_model(self, model: object) -> None:
        model_dump = getattr(model, "model_dump", None)
        payload = model_dump(mode="json") if callable(model_dump) else model
        self.send_response(HTTPStatus.OK)
        self.send_header("content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode("utf-8"))

    def _write_bytes(self, payload: bytes, *, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def make_server(config: RobosuiteRuntimeConfig) -> HTTPServer:
    RobosuiteRuntimeHandler.state = RobosuiteRuntimeState(config)
    # Robosuite/MuJoCo render contexts are thread-sensitive. Keep all env reset,
    # step, offscreen camera, and interactive viewer calls on the server thread
    # instead of using ThreadingHTTPServer worker threads.
    return HTTPServer((config.host, config.port), RobosuiteRuntimeHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--env-name", default="Lift")
    parser.add_argument("--robot-id", default="panda")
    parser.add_argument("--robot-model", default="Panda")
    parser.add_argument("--controller", default="JOINT_POSITION")
    parser.add_argument("--control-freq", type=int, default=100)
    parser.add_argument("--horizon", type=int, default=200)
    parser.add_argument("--camera-name", default="agentview")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--image-dump-dir", default=None)
    parser.add_argument("--image-dump-every", type=int, default=0)
    args = parser.parse_args()
    config = RobosuiteRuntimeConfig(
        host=args.host,
        port=args.port,
        env_name=args.env_name,
        robot_id=args.robot_id,
        robot_model=args.robot_model,
        controller=args.controller,
        control_freq=args.control_freq,
        horizon=args.horizon,
        camera_name=args.camera_name,
        seed=args.seed,
        visualize=args.visualize,
        image_dump_dir=args.image_dump_dir,
        image_dump_every=args.image_dump_every,
    )
    server = make_server(config)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def _float_list(value: object) -> list[float]:
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    if isinstance(value, Sequence) and not isinstance(value, str | bytes):
        return [float(item) for item in value]
    return []


def _mean_or_zero(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def _shape_list(value: object) -> list[int]:
    shape = getattr(value, "shape", None)
    if isinstance(shape, Sequence):
        return [int(item) for item in shape]
    return []


def _npy_bytes(value: object) -> bytes:
    import numpy as np

    buffer = BytesIO()
    np.save(buffer, np.asarray(value), allow_pickle=False)
    return buffer.getvalue()


def _array_copy(value: object) -> object:
    import numpy as np

    return np.ascontiguousarray(np.asarray(value)).copy()


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


def _array_metadata(value: object, payload: bytes) -> dict[str, object]:
    import numpy as np

    array = np.ascontiguousarray(np.asarray(value))
    flat = array.reshape(-1, array.shape[-1]) if array.ndim == 3 else array.reshape(-1, 1)
    return {
        "array_sha256": hashlib.sha256(array.tobytes()).hexdigest(),
        "payload_sha256": hashlib.sha256(payload).hexdigest(),
        "min": float(array.min()) if array.size else 0.0,
        "max": float(array.max()) if array.size else 0.0,
        "mean": float(array.mean()) if array.size else 0.0,
        "top_left": [int(item) for item in flat[0].tolist()] if flat.size else [],
        "center": [int(item) for item in array[array.shape[0] // 2, array.shape[1] // 2].tolist()]
        if array.ndim == 3 and array.size
        else [],
        "bottom_left": [int(item) for item in array[-1, 0].tolist()]
        if array.ndim == 3 and array.size
        else [],
    }


def _success_from_info(info: Mapping[object, object]) -> bool | None:
    for key in ("success", "is_success", "task_success"):
        value = info.get(key)
        if isinstance(value, bool):
            return value
        if isinstance(value, int | float):
            return bool(value)
    return None


def _camera_mount(camera_name: str) -> str:
    if "eye_in_hand" in camera_name:
        return "wrist"
    if "robotview" in camera_name:
        return "robot_external"
    return "scene"


if __name__ == "__main__":
    main()
