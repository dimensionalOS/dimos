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

"""DimOS module wrapper for the fake benchmark runtime."""

from __future__ import annotations

from dimos_runtime_protocol.models import (
    CommandMode,
    EpisodeResetRequest,
    EpisodeResetResponse,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)
import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.simulation.runtime_module import (
    SimulatorOwnerThread,
    module_runtime_description,
    publish_output,
)
from dimos_fake_runtime_sidecar.server import FakeRuntimeState


class FakeRuntimeModuleConfig(ModuleConfig):
    robot_id: str = "fakebot"
    dof: int = 3
    step_hz: int = 100


class FakeRuntimeModule(Module):
    """First-class DimOS module for the fake runtime protocol backend."""

    config: FakeRuntimeModuleConfig

    motor_state: Out[MotorStateFrame]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    runtime_event: Out[ObservationFrame]

    def __init__(
        self,
        robot_id: str = "fakebot",
        dof: int = 3,
        step_hz: int = 100,
        **kwargs: object,
    ) -> None:
        super().__init__(robot_id=robot_id, dof=dof, step_hz=step_hz, **kwargs)
        self._robot_id = robot_id
        self._owner = SimulatorOwnerThread(name="fake-runtime-owner")
        self._state = self._owner.call(
            lambda: FakeRuntimeState(robot_id=robot_id, dof=dof, step_hz=step_hz)
        )
        self._runtime_stopped = False

    @rpc
    def stop(self) -> None:
        if self._runtime_stopped:
            return
        self._owner.stop()
        self._runtime_stopped = True
        super().stop()

    @rpc
    def describe(self) -> RuntimeDescription:
        """Return the fake runtime motor surface and stream metadata."""

        return self._owner.call(lambda: module_runtime_description(self._state.describe()))

    @rpc
    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        """Reset the fake benchmark episode synchronously."""

        def _reset() -> EpisodeResetResponse:
            response = self._state.reset(request)
            self._publish_runtime_outputs(sequence=0, event="reset")
            return response.model_copy(
                update={
                    "runtime_description": module_runtime_description(response.runtime_description),
                    "observations": [],
                }
            )

        return self._owner.call(_reset)

    @rpc
    def step(self, request: StepRequest) -> StepResponse:
        """Advance the fake runtime by one synchronized benchmark tick."""

        def _step() -> StepResponse:
            if request.action.robot_id != self._robot_id:
                raise ValueError(
                    f"unexpected robot_id {request.action.robot_id!r}; expected {self._robot_id!r}"
                )
            if request.action.mode != CommandMode.POSITION:
                raise ValueError(f"unsupported command mode {request.action.mode!s}")
            response = self._state.step(request)
            publish_output(self.motor_state, response.motor_state)
            self._publish_runtime_outputs(sequence=response.motor_state.sequence, event="step")
            return response.model_copy(update={"observations": []})

        return self._owner.call(_step)

    @rpc
    def score(self) -> ScoreOutput:
        """Return the fake runtime score for the current episode."""

        return self._owner.call(self._state.score)

    def _publish_runtime_outputs(self, sequence: int, event: str) -> None:
        frame_id = "fake_camera"
        pixel_value = np.uint8(sequence % 255)
        image_data = np.full((2, 2, 3), pixel_value, dtype=np.uint8)
        publish_output(
            self.color_image,
            Image(data=image_data, format=ImageFormat.RGB, frame_id=frame_id),
        )
        publish_output(
            self.camera_info,
            CameraInfo.from_fov(fov_deg=60.0, width=2, height=2, frame_id=frame_id),
        )
        publish_output(
            self.runtime_event,
            ObservationFrame(
                stream="runtime_event",
                kind=ObservationKind.TEXT,
                inline_text=event,
                metadata={"sequence": sequence},
            ),
        )
