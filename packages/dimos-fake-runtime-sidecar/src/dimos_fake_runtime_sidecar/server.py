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

"""HTTP fake sidecar that speaks the DimOS runtime protocol."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import time

from dimos_runtime_protocol.models import (
    CommandMode,
    EpisodeResetRequest,
    EpisodeResetResponse,
    HealthResponse,
    MotorDescription,
    MotorStateFrame,
    RobotMotorSurface,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)


def _default_names(robot_id: str, dof: int) -> list[str]:
    return [f"{robot_id}/joint{i + 1}" for i in range(dof)]


class FakeRuntimeState:
    """Deterministic state machine for the fake runtime sidecar."""

    def __init__(self, *, robot_id: str = "fakebot", dof: int = 3, step_hz: int = 100) -> None:
        self.robot_id = robot_id
        self.names = _default_names(robot_id, dof)
        self.step_hz = step_hz
        self.episode_id = "unreset"
        self.q = [0.0] * dof
        self.dq = [0.0] * dof
        self.tau = [0.0] * dof
        self.sequence = 0

    def describe(self) -> RuntimeDescription:
        motors = [MotorDescription(name=name, index=i) for i, name in enumerate(self.names)]
        surface = RobotMotorSurface(
            robot_id=self.robot_id,
            motors=motors,
            supported_command_modes=[CommandMode.POSITION],
        )
        return RuntimeDescription(
            runtime_id="fake-runtime",
            backend="fake",
            capabilities=["motor.position", "score.simple"],
            robot_surfaces=[surface],
            control_step_hz=self.step_hz,
            observation_streams=["fake_state"],
            metadata={"dof": len(self.names)},
        )

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        self.episode_id = request.episode_id
        self.q = [0.0] * len(self.names)
        self.dq = [0.0] * len(self.names)
        self.tau = [0.0] * len(self.names)
        self.sequence = 0
        return EpisodeResetResponse(
            episode_id=request.episode_id,
            runtime_description=self.describe(),
            observations=[],
        )

    def step(self, request: StepRequest) -> StepResponse:
        previous = list(self.q)
        targets = request.action.q
        if request.action.names != self.names:
            raise ValueError(f"motor names mismatch: expected {self.names}, got {request.action.names}")
        if len(targets) != len(self.names):
            raise ValueError(f"target length mismatch: expected {len(self.names)}, got {len(targets)}")
        alpha = 0.35
        self.q = [old + alpha * (target - old) for old, target in zip(self.q, targets, strict=True)]
        self.dq = [(new - old) * self.step_hz for old, new in zip(previous, self.q, strict=True)]
        self.sequence += 1
        motor_state = MotorStateFrame(
            robot_id=self.robot_id,
            names=self.names,
            q=self.q,
            dq=self.dq,
            tau=self.tau,
            sequence=self.sequence,
            timestamp_s=time.time(),
        )
        return StepResponse(
            episode_id=request.episode_id,
            tick_id=request.tick_id,
            motor_state=motor_state,
            observations=[],
            reward=float(sum(abs(v) for v in self.q)),
            done=False,
            success=False,
            info={"backend_sequence": self.sequence},
        )

    def score(self) -> ScoreOutput:
        moved = any(abs(v) > 1e-6 for v in self.q)
        return ScoreOutput(
            episode_id=self.episode_id,
            success=moved,
            score=1.0 if moved else 0.0,
            reason="fake runtime observed motor movement" if moved else "no movement observed",
            metrics={"sequence": self.sequence},
        )


class FakeRuntimeHandler(BaseHTTPRequestHandler):
    """Request handler bound to one FakeRuntimeState instance."""

    state: FakeRuntimeState

    def _write_json(self, status: int, payload: dict[str, object]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        data = self.rfile.read(length)
        value = json.loads(data.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("expected JSON object")
        return value

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._write_json(200, HealthResponse(ok=True, runtime_id="fake-runtime").model_dump())
        elif self.path == "/describe":
            self._write_json(200, self.state.describe().model_dump())
        elif self.path == "/score":
            self._write_json(200, self.state.score().model_dump())
        else:
            self._write_json(404, {"error": f"unknown path {self.path}"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            body = self._read_json()
            if self.path == "/reset":
                request = EpisodeResetRequest.model_validate(body)
                self._write_json(200, self.state.reset(request).model_dump())
            elif self.path == "/step":
                request = StepRequest.model_validate(body)
                self._write_json(200, self.state.step(request).model_dump())
            else:
                self._write_json(404, {"error": f"unknown path {self.path}"})
        except Exception as exc:
            self._write_json(400, {"error": str(exc)})

    def log_message(self, format: str, *args: object) -> None:
        return


def make_server(host: str, port: int, *, robot_id: str = "fakebot", dof: int = 3) -> ThreadingHTTPServer:
    state = FakeRuntimeState(robot_id=robot_id, dof=dof)

    class BoundHandler(FakeRuntimeHandler):
        pass

    BoundHandler.state = state
    return ThreadingHTTPServer((host, port), BoundHandler)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run fake DimOS runtime sidecar")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--robot-id", default="fakebot")
    parser.add_argument("--dof", type=int, default=3)
    args = parser.parse_args()
    server = make_server(args.host, args.port, robot_id=args.robot_id, dof=args.dof)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
