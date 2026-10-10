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

"""Trusted engine, action admission and private evidence for one Robosuite lift episode."""

from __future__ import annotations

import argparse
import base64
from contextlib import ExitStack
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, cast

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
import zenoh

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import lifted
from dimos.evals.types import Outcome, Trajectory
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.mujoco_eval import (
    MAX_COMMAND_BYTES,
    Command,
    Sensor,
    State,
    key,
    session_config,
)
from dimos.simulation.engines.mujoco_engine import CameraConfig, MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import _RX180, _pose_matrix, _transform_from_matrix


class EpisodeConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    run: str
    episode: str
    endpoint: str
    scene: Path
    output: Path
    duration_s: float = Field(gt=0, le=1200)
    seed: int = 0
    case: str = "robosuite_lift_cube"
    camera: str = "wrist_camera"
    home: list[float] = Field(min_length=7, max_length=7)


class ActionBoundary:
    """Admission and application share a lock; no reset operation exists on this link."""

    def __init__(
        self,
        run: str,
        episode: str,
        lower: list[float],
        upper: list[float],
        velocity_max: list[float],
        events: Path,
    ) -> None:
        self.run, self.episode = run, episode
        self.lower, self.upper, self.velocity_max = lower, upper, velocity_max
        self.enabled = False
        self.sequence = -1
        self.attempts = 0
        self.pending: Command | None = None
        self.lock = threading.Lock()
        self.events = events.open("x")
        self.closed = False
        self.last_action = time.monotonic()

    def receive(self, payload: bytes) -> bool:
        with self.lock:
            if self.closed:
                return False
            self.attempts += 1
            reason = "accepted"
            packet: Command | None = None
            try:
                if len(payload) > MAX_COMMAND_BYTES:
                    raise ValueError("size")
                packet = Command.model_validate_json(payload)
                if (packet.run, packet.episode) != (self.run, self.episode):
                    raise ValueError("episode")
                if abs(time.time() - packet.sent_at) > 0.5:
                    raise ValueError("expired")
                if packet.sequence <= self.sequence:
                    raise ValueError("replay")
                if packet.kind == "position":
                    if not self.enabled:
                        raise ValueError("disabled")
                    if len(packet.values) != len(self.lower):
                        raise ValueError("dimensions")
                    for i, value in enumerate(packet.values):
                        lo, hi = self.lower[i], self.upper[i]
                        if not lo <= value <= hi:
                            raise ValueError("bounds")
                elif packet.values:
                    raise ValueError("unexpected_values")
            except ValueError as exc:
                reason = str(exc) if type(exc) is ValueError else "schema"
            accepted = reason == "accepted"
            if accepted and packet is not None:
                self.sequence = packet.sequence
                self.last_action = time.monotonic()
                if packet.kind == "enable":
                    self.enabled = True
                elif packet.kind == "disable":
                    self.enabled = False
                self.pending = packet
            self.events.write(
                json.dumps(
                    {
                        "run": self.run,
                        "episode": self.episode,
                        "attempt": self.attempts,
                        "accepted": accepted,
                        "reason": reason,
                        "sequence": packet.sequence if packet else None,
                    }
                )
                + "\n"
            )
            self.events.flush()
            return accepted

    def close(self) -> None:
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.pending = None
            self.enabled = False
            self.events.close()


class TrustedRuntime:
    def __init__(self, config: EpisodeConfig) -> None:
        if config.case != "robosuite_lift_cube":
            raise ValueError("This path supports the existing Robosuite lift case")
        self.config = config
        config.output.mkdir(parents=True, exist_ok=False)
        (config.output / "episode.json").write_text(config.model_dump_json(indent=2) + "\n")
        self._resources = ExitStack()
        try:
            self._initialize(config)
        except Exception as exc:
            self._resources.close()
            result = self._result()
            result["error"] = f"Infrastructure startup error: {exc}"
            (config.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            raise

    def _initialize(self, config: EpisodeConfig) -> None:
        self.recording = config.output / "memory.db"
        self.store = SqliteStore(path=str(self.recording))
        self._resources.callback(self.store.stop)
        self.engine = MujocoEngine(
            config_path=config.scene,
            headless=True,
            cameras=[
                CameraConfig(
                    name=config.camera, width=320, height=240, fps=10, base_body_name="link7"
                )
            ],
            reset_joint_positions=config.home,
        )
        if len(self.engine.joint_names) != 8:
            raise ValueError("Expected seven arm joints and one gripper")
        ranges = [self.engine.get_joint_range(i) for i in range(8)]
        if any(r is None for r in ranges):
            raise ValueError("Missing joint bounds")
        self.lower = [r[0] for r in ranges if r is not None]
        self.upper = [r[1] for r in ranges if r is not None]
        for i in range(7):
            actuator_range = self.engine.get_actuator_ctrl_range(i)
            if actuator_range is None:
                raise ValueError("Missing arm actuator bounds")
            self.lower[i] = max(self.lower[i], actuator_range[0])
            self.upper[i] = min(self.upper[i], actuator_range[1])
        if any(not math.isfinite(v) for v in (*self.lower, *self.upper)) or any(
            lo >= hi for lo, hi in zip(self.lower, self.upper, strict=True)
        ):
            raise ValueError("Invalid actuator bounds")
        gripper_range = self.engine.get_actuator_ctrl_range(7)
        if gripper_range is None or self.upper[7] <= self.lower[7]:
            raise ValueError("Missing gripper actuator bounds")
        self.gripper_range = gripper_range
        self.boundary = ActionBoundary(
            config.run,
            config.episode,
            self.lower,
            self.upper,
            [math.pi] * 7 + [0.0],
            config.output / "boundary.jsonl",
        )
        self._resources.callback(self.boundary.close)
        self.session = zenoh.open(
            session_config(config.endpoint, trusted=True, run=config.run, episode=config.episode)
        )
        self._resources.callback(self.session.close)
        self._resources.callback(self.engine.disconnect)
        self.subscriber = self.session.declare_subscriber(
            key(config.run, config.episode, "command"),
            lambda sample: self.boundary.receive(sample.payload.to_bytes()),
        )
        self._resources.callback(self.subscriber.undeclare)
        self._recording_lock = threading.Lock()
        self.engine.set_step_hooks(before=self._before_step, after=self._after_step)
        self._last_record = 0.0
        self._last_frame = 0.0
        self._state_sequence = 0
        self._sensor_sequence = 0
        self._samples = 0
        self._error = ""

    def _before_step(self, engine: MujocoEngine) -> None:
        if self._samples == 0:
            self._after_step(engine)
        with self.boundary.lock:
            packet = self.boundary.pending
            self.boundary.pending = None
            if packet is None:
                if time.monotonic() - self.boundary.last_action > 0.5:
                    self._hold(engine)
                return
            if abs(time.time() - packet.sent_at) > 0.5:
                self._hold(engine)
                self.boundary.events.write(
                    json.dumps(
                        {
                            "run": self.config.run,
                            "episode": self.config.episode,
                            "sequence": packet.sequence,
                            "accepted": False,
                            "reason": "expired_at_application",
                        }
                    )
                    + "\n"
                )
                self.boundary.events.flush()
                return
            if packet.kind == "position":
                self._apply_positions(engine, packet.values)
            else:
                self._hold(engine)

    def _apply_positions(self, engine: MujocoEngine, positions: list[float]) -> None:
        engine.write_joint_command(JointState(position=positions[:7]))
        lo, hi = self.gripper_range
        fraction = (positions[7] - self.lower[7]) / (self.upper[7] - self.lower[7])
        engine.set_position_target(7, hi - fraction * (hi - lo))

    def _hold(self, engine: MujocoEngine) -> None:
        positions = [
            max(lo, min(hi, q))
            for lo, hi, q in zip(self.lower, self.upper, engine.joint_positions, strict=True)
        ]
        self._apply_positions(engine, positions)

    def _after_step(self, engine: MujocoEngine) -> None:
        now = time.time()
        if now - self._last_record < 0.05:
            return
        with self._recording_lock:
            try:
                pose = engine.get_body_pose("cube_main")
                if pose is None:
                    raise LookupError("Missing cube_main evidence")
                position, rotation = pose
                if not all(math.isfinite(v) for v in (*position, *rotation)):
                    raise RuntimeError("Non-finite object evidence")
                self.store.stream("tf", TFMessage).append(
                    TFMessage(
                        Transform(
                            translation=Vector3(*position),
                            rotation=Quaternion(*rotation),
                            frame_id="world",
                            child_frame_id="cube_main",
                            ts=now,
                        )
                    ),
                    ts=now,
                )
                joints = JointState(
                    position=engine.joint_positions,
                    velocity=engine.joint_velocities,
                    effort=engine.joint_efforts,
                    ts=now,
                )
                self.store.stream("coordinator_joint_state", JointState).append(joints, ts=now)
                self._state_sequence += 1
                with self.boundary.lock:
                    state = State(
                        run=self.config.run,
                        episode=self.config.episode,
                        sequence=self._state_sequence,
                        position=joints.position,
                        velocity=joints.velocity,
                        effort=joints.effort,
                        lower=self.lower,
                        upper=self.upper,
                        velocity_max=self.boundary.velocity_max,
                        enabled=self.boundary.enabled,
                        accepted_sequence=self.boundary.sequence,
                    )
                self.session.put(
                    key(self.config.run, self.config.episode, "state"), state.model_dump_json()
                )
                self._last_record = now
                self._samples += 1
            except Exception as exc:
                self._error = str(exc)

    def _publish_sensor(self, kind: str, message: Any) -> None:
        self._sensor_sequence += 1
        packet = Sensor.model_validate(
            dict(
                run=self.config.run,
                episode=self.config.episode,
                sequence=self._sensor_sequence,
                kind=kind,
                data=base64.b64encode(message.lcm_encode()).decode("ascii"),
            )
        )
        self.session.put(
            key(self.config.run, self.config.episode, "sensor"), packet.model_dump_json()
        )

    def publish_camera(self) -> None:
        frame = self.engine.read_camera(self.config.camera)
        if frame is None or frame.timestamp <= self._last_frame:
            return
        self._last_frame = frame.timestamp
        optical = f"{self.config.camera}_color_optical_frame"
        image = Image(data=frame.rgb, format=ImageFormat.RGB, frame_id=optical, ts=frame.timestamp)
        self._publish_sensor("color_image", image)
        self._publish_sensor(
            "depth_image",
            Image(data=frame.depth, format=ImageFormat.DEPTH, frame_id=optical, ts=frame.timestamp),
        )
        fovy = self.engine.get_camera_fovy(self.config.camera)
        if fovy is None or frame.base_pos is None or frame.base_mat is None:
            raise RuntimeError("Missing robot-relative camera calibration")
        focal = 240 / (2 * math.tan(math.radians(fovy) / 2))
        self._publish_sensor(
            "camera_info",
            CameraInfo.from_intrinsics(
                fx=focal,
                fy=focal,
                cx=160,
                cy=120,
                width=320,
                height=240,
                frame_id=optical,
            ).with_ts(frame.timestamp),
        )
        inverse = np.linalg.inv(_pose_matrix(frame.base_pos, frame.base_mat))
        camera = _pose_matrix(frame.cam_pos, frame.cam_mat.reshape(3, 3) @ _RX180.as_matrix())
        self._publish_sensor(
            "tf",
            TFMessage(
                _transform_from_matrix(
                    inverse @ camera, frame_id="link7", child_frame_id=optical, ts=frame.timestamp
                )
            ),
        )
        with self._recording_lock:
            self.store.stream("color_image", Image).append(image, ts=frame.timestamp)

    def _result(self) -> dict[str, Any]:
        return {
            "run": self.config.run,
            "episode": self.config.episode,
            "case": self.config.case,
            "seed": self.config.seed,
            "score": None,
            "error": "",
            "attempts": 0,
        }

    def run(self) -> dict[str, Any]:
        result = self._result()
        try:
            if not self.engine.connect():
                raise RuntimeError("Engine failed to connect")
            deadline = time.monotonic() + self.config.duration_s
            while time.monotonic() < deadline:
                self.publish_camera()
                threading.Event().wait(0.01)
            if (
                self._error
                or self._samples < 2
                or self._last_frame == 0
                or time.time() - self._last_frame > 1
                or time.time() - self._last_record > 1
            ):
                raise RuntimeError(self._error or "Stale or incomplete evidence")
        except Exception as exc:
            result["error"] = str(exc)
        finally:
            self._resources.close()
        result["attempts"] = self.boundary.attempts
        if not result["error"]:
            try:
                with SqliteStore(path=str(self.recording), must_exist=True) as store:
                    MujocoEnvironment(blueprint=[], tracked_bodies=("cube_main",)).wait_ready(
                        store, deadline=time.monotonic() + 1.0
                    )
                    first_body_transform(store, "cube_main")
                    last_body_transform(store, "cube_main")
                result["score"] = lifted("cube_main", by_m=0.05)(
                    Outcome(
                        trajectory=cast("Trajectory", None), artifacts={"recording": self.recording}
                    )
                )
            except Exception as exc:
                result["error"] = f"Infrastructure evidence error: {exc}"
        (self.config.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    config = EpisodeConfig.model_validate_json(args.manifest.read_bytes())
    result = TrustedRuntime(config).run()
    print(json.dumps(result))
    if result["error"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
