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

"""Configurable grasp proposals with optional learned inference."""

from typing import Annotated, Any, Literal

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXBackend, GraspGenXConfig
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspBackend
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.spec import BaseConfig


class HeuristicGraspConfig(BaseConfig):
    backend: Literal["heuristic"] = "heuristic"


GraspGeneratorConfig = Annotated[
    HeuristicGraspConfig | GraspGenXConfig, Field(discriminator="backend")
]


class GraspProposalConfig(ModuleConfig):
    generator: GraspGeneratorConfig = Field(default_factory=HeuristicGraspConfig)


class GraspProposalModule(Module, GraspGenSpec):
    """Propose ranked TCP poses using the backend selected at startup."""

    dedicated_worker = True
    config: GraspProposalConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: HeuristicGraspBackend | GraspGenXBackend | None = None

    @rpc
    def start(self) -> None:
        if self._backend is not None:
            return
        super().start()
        if isinstance(self.config.generator, GraspGenXConfig):
            backend = GraspGenXBackend(self.config.generator)
            backend.start()
            self._backend = backend
        else:
            self._backend = HeuristicGraspBackend()

    @rpc
    def stop(self) -> None:
        if isinstance(self._backend, GraspGenXBackend):
            self._backend.stop()
        self._backend = None
        super().stop()

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        """Return ordered proposals in the input cloud's coordinate frame."""
        if self._backend is None:
            raise RuntimeError("Grasp proposal module has not been started")
        return self._backend.propose_grasps(object_pointcloud)
