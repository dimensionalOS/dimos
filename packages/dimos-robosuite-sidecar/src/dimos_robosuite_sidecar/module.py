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

"""DimOS module wrapper for the Robosuite benchmark runtime."""

from __future__ import annotations

from io import BytesIO
from pathlib import Path

from dimos_runtime_protocol.models import (
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
    InlineSimulatorExecutor,
    SimulatorExecutor,
    SimulatorOwnerThread,
    module_runtime_description,
    publish_output,
)
from dimos_robosuite_sidecar.server import RobosuiteRuntimeConfig, RobosuiteRuntimeState


class RobosuiteRuntimeModuleConfig(ModuleConfig):
    env_name: str = "Lift"
    robot_id: str = "panda"
    robot_model: str = "Panda"
    controller: str = "JOINT_POSITION"
    control_freq: int = 20
    horizon: int = 200
    camera_name: str = "agentview"
    seed: int | None = None
    visualize: bool = False
    image_dump_dir: str | Path | None = None
    image_dump_every: int = 0


class RobosuiteRuntimeModule(Module):
    """First-class DimOS module for Robosuite benchmark runtimes."""

    config: RobosuiteRuntimeModuleConfig

    motor_state: Out[MotorStateFrame]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    runtime_event: Out[ObservationFrame]

    def __init__(
        self,
        env_name: str = "Lift",
        robot_id: str = "panda",
        robot_model: str = "Panda",
        controller: str = "JOINT_POSITION",
        control_freq: int = 20,
        horizon: int = 200,
        camera_name: str = "agentview",
        seed: int | None = None,
        visualize: bool = False,
        image_dump_dir: str | Path | None = None,
        image_dump_every: int = 0,
        **kwargs: object,
    ) -> None:
        super().__init__(
            env_name=env_name,
            robot_id=robot_id,
            robot_model=robot_model,
            controller=controller,
            control_freq=control_freq,
            horizon=horizon,
            camera_name=camera_name,
            seed=seed,
            visualize=visualize,
            image_dump_dir=image_dump_dir,
            image_dump_every=image_dump_every,
            **kwargs,
        )
        self._owner: SimulatorExecutor = (
            InlineSimulatorExecutor()
            if visualize
            else SimulatorOwnerThread(name="robosuite-runtime-owner")
        )
        self._runtime_config = RobosuiteRuntimeConfig(
            host="127.0.0.1",
            port=0,
            env_name=env_name,
            robot_id=robot_id,
            robot_model=robot_model,
            controller=controller,
            control_freq=control_freq,
            horizon=horizon,
            camera_name=camera_name,
            seed=seed,
            visualize=visualize,
            image_dump_dir=str(image_dump_dir) if image_dump_dir is not None else None,
            image_dump_every=image_dump_every,
        )
        self._state: RobosuiteRuntimeState | None = None
        self._runtime_stopped = False

    @rpc
    def stop(self) -> None:
        if self._runtime_stopped:
            return
        self._owner.call(self._close_state)
        self._owner.stop()
        self._runtime_stopped = True
        super().stop()

    @rpc
    def describe(self) -> RuntimeDescription:
        """Return Robosuite runtime metadata without HTTP capability markers."""

        def _describe() -> RuntimeDescription:
            state = self._state_or_create()
            return module_runtime_description(
                state.describe(), camera_streams=[state.config.camera_name]
            )

        return self._owner.call(_describe)

    @rpc
    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        """Reset the Robosuite episode on the simulator owner thread."""

        def _reset() -> EpisodeResetResponse:
            state = self._state_or_create()
            response = state.reset(request)
            self._publish_observation_outputs(response.observations, event="reset")
            runtime_description = module_runtime_description(
                response.runtime_description, camera_streams=[state.config.camera_name]
            )
            return response.model_copy(
                update={"runtime_description": runtime_description, "observations": []}
            )

        return self._owner.call(_reset)

    @rpc
    def step(self, request: StepRequest) -> StepResponse:
        """Advance Robosuite by one synchronized benchmark tick."""

        def _step() -> StepResponse:
            state = self._state_or_create()
            response = state.step(request)
            publish_output(self.motor_state, response.motor_state)
            self._publish_observation_outputs(response.observations, event="step")
            return response.model_copy(update={"observations": []})

        return self._owner.call(_step)

    @rpc
    def score(self) -> ScoreOutput:
        """Return the current Robosuite benchmark score."""

        return self._owner.call(lambda: self._state_or_create().score())

    def _state_or_create(self) -> RobosuiteRuntimeState:
        if self._state is None:
            self._state = RobosuiteRuntimeState(self._runtime_config)
        return self._state

    def _close_state(self) -> None:
        if self._state is None:
            return
        env = getattr(self._state, "_env", None)
        close = getattr(env, "close", None)
        if callable(close):
            close()
        self._state = None

    def _publish_observation_outputs(
        self, observations: list[ObservationFrame], event: str
    ) -> None:
        for observation in observations:
            if observation.kind == ObservationKind.STATE:
                publish_output(self.runtime_event, observation)
        state = self._state_or_create()
        image_frame = next(
            (
                observation
                for observation in observations
                if observation.kind == ObservationKind.IMAGE and observation.data_ref is not None
            ),
            None,
        )
        if image_frame is not None:
            payload_id = image_frame.data_ref.removeprefix("/payloads/")
            array = np.load(BytesIO(state.payload_bytes(payload_id)), allow_pickle=False)
            metadata = image_frame.metadata or {}
            frame_id = str(metadata.get("camera_name", state.config.camera_name))
            fov_y_deg = float(metadata.get("fov_y_deg", state._camera_fov_deg()))
            if metadata.get("image_convention") == "opengl":
                array = np.ascontiguousarray(np.flipud(array))
        else:
            image = state._camera_image()
            if image is None:
                return
            array = np.asarray(image)
            frame_id = state.config.camera_name
            fov_y_deg = state._camera_fov_deg()
        if array.size == 0:
            return
        publish_output(
            self.color_image,
            Image(data=array, format=ImageFormat.RGB, frame_id=frame_id),
        )
        height, width = array.shape[:2]
        publish_output(
            self.camera_info,
            CameraInfo.from_fov(
                fov_deg=fov_y_deg,
                width=width,
                height=height,
                frame_id=frame_id,
            ),
        )
        publish_output(
            self.runtime_event,
            ObservationFrame(
                stream="runtime_event",
                kind=ObservationKind.TEXT,
                inline_text=event,
                metadata={"sequence": state._sequence, "camera": frame_id},
            ),
        )
