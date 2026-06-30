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

"""DimOS module wrapper for LIBERO-PRO benchmark runtimes."""

from __future__ import annotations

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
    SimulatorOwnerThread,
    module_runtime_description,
    publish_output,
)
from dimos_libero_pro_sidecar.server import LiberoProRuntimeConfig, LiberoProRuntimeState


class LiberoProRuntimeModuleConfig(ModuleConfig):
    bddl_root: str | Path
    init_states_root: str | Path
    benchmark_name: str = "libero_90"
    robot_id: str = "panda"
    task_order_index: int = 0
    task_index: int = 0
    init_state_index: int = 0
    controller: str = "JOINT_POSITION"
    camera_names: tuple[str, ...] = ("agentview",)
    control_freq: int = 20
    horizon: int = 1000
    seed: int | None = None
    allow_asset_bootstrap: bool = False
    visualize: bool = False


class LiberoProRuntimeModule(Module):
    """First-class DimOS module for selected LIBERO-PRO benchmark tasks."""

    config: LiberoProRuntimeModuleConfig

    motor_state: Out[MotorStateFrame]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    runtime_event: Out[ObservationFrame]

    def __init__(
        self,
        bddl_root: str | Path,
        init_states_root: str | Path,
        benchmark_name: str = "libero_90",
        robot_id: str = "panda",
        task_order_index: int = 0,
        task_index: int = 0,
        init_state_index: int = 0,
        controller: str = "JOINT_POSITION",
        camera_names: tuple[str, ...] = ("agentview",),
        control_freq: int = 20,
        horizon: int = 1000,
        seed: int | None = None,
        allow_asset_bootstrap: bool = False,
        visualize: bool = False,
        **kwargs: object,
    ) -> None:
        super().__init__(
            bddl_root=bddl_root,
            init_states_root=init_states_root,
            benchmark_name=benchmark_name,
            robot_id=robot_id,
            task_order_index=task_order_index,
            task_index=task_index,
            init_state_index=init_state_index,
            controller=controller,
            camera_names=camera_names,
            control_freq=control_freq,
            horizon=horizon,
            seed=seed,
            allow_asset_bootstrap=allow_asset_bootstrap,
            visualize=visualize,
            **kwargs,
        )
        self._owner = SimulatorOwnerThread(name="libero-pro-runtime-owner")
        if visualize:
            self._owner.stop()
            raise ValueError(
                "visualize=True is not supported by LiberoProRuntimeModule until "
                "main-thread simulator workers are available"
            )
        self._runtime_config = LiberoProRuntimeConfig(
            host="127.0.0.1",
            port=0,
            benchmark_name=benchmark_name,
            bddl_root=Path(bddl_root),
            init_states_root=Path(init_states_root),
            robot_id=robot_id,
            task_order_index=task_order_index,
            task_index=task_index,
            init_state_index=init_state_index,
            controller=controller,
            camera_names=tuple(camera_names),
            control_freq=control_freq,
            horizon=horizon,
            seed=seed,
            allow_asset_bootstrap=allow_asset_bootstrap,
            visualize=visualize,
        )
        self._state: LiberoProRuntimeState | None = None
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
        """Return LIBERO-PRO runtime metadata without HTTP capability markers."""

        def _describe() -> RuntimeDescription:
            state = self._state_or_create()
            return module_runtime_description(
                state.describe(), camera_streams=list(state.config.camera_names)
            )

        return self._owner.call(_describe)

    @rpc
    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        """Reset the selected LIBERO-PRO task on the simulator owner thread."""

        def _reset() -> EpisodeResetResponse:
            state = self._state_or_create()
            response = state.reset(request)
            self._publish_observation_outputs(response.observations, event="reset")
            runtime_description = module_runtime_description(
                response.runtime_description, camera_streams=list(state.config.camera_names)
            )
            return response.model_copy(
                update={"runtime_description": runtime_description, "observations": []}
            )

        return self._owner.call(_reset)

    @rpc
    def step(self, request: StepRequest) -> StepResponse:
        """Advance LIBERO-PRO by one synchronized benchmark tick."""

        def _step() -> StepResponse:
            state = self._state_or_create()
            response = state.step(request)
            publish_output(self.motor_state, response.motor_state)
            self._publish_observation_outputs(response.observations, event="step")
            return response.model_copy(update={"observations": []})

        return self._owner.call(_step)

    @rpc
    def score(self) -> ScoreOutput:
        """Return the current LIBERO-PRO benchmark score."""

        return self._owner.call(lambda: self._state_or_create().score())

    def _state_or_create(self) -> LiberoProRuntimeState:
        if self._state is None:
            self._state = LiberoProRuntimeState(self._runtime_config)
        return self._state

    def _publish_observation_outputs(
        self, observations: list[ObservationFrame], event: str
    ) -> None:
        for observation in observations:
            if observation.kind == ObservationKind.STATE:
                publish_output(self.runtime_event, observation)
        state = self._state_or_create()
        for camera_name in state.config.camera_names:
            image = state._last_obs.get(f"{camera_name}_image")
            if image is None:
                continue
            array = np.asarray(image)
            publish_output(
                self.color_image,
                Image(data=array, format=ImageFormat.RGB, frame_id=camera_name),
            )
            height, width = array.shape[:2]
            publish_output(
                self.camera_info,
                CameraInfo.from_fov(
                    fov_deg=45.0,
                    width=width,
                    height=height,
                    frame_id=camera_name,
                ),
            )
        publish_output(
            self.runtime_event,
            ObservationFrame(
                stream="runtime_event",
                kind=ObservationKind.TEXT,
                inline_text=event,
                metadata={"sequence": state._sequence},
            ),
        )
