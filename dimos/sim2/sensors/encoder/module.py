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

"""Encoder acquisition from selected joint truth; no actuator feedback shortcut."""

from dataclasses import dataclass
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.sensors.sampling import Acquisition, measurement_header, same_acquisition


@dataclass(frozen=True)
class EncoderConfig:
    joints: tuple[str, ...]
    source_id: str = "encoders"
    hz: float = 500
    offset: float = 0
    resolution: float = 0
    noise_std: float = 0
    seed: int = 0


class EncoderModuleConfig(ModuleConfig):
    sensor: EncoderConfig


class SimJointEncoderModule(Module):
    config: EncoderModuleConfig
    observations: In[WorldState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        c = self.config.sensor
        self._sampling = Acquisition(c.hz)
        self._last: JointState | None = None
        if (
            not c.joints
            or len(set(c.joints)) != len(c.joints)
            or not np.isfinite((c.offset, c.resolution, c.noise_std)).all()
            or min(c.resolution, c.noise_std, c.seed) < 0
        ):
            raise ValueError("invalid encoder joint selection/calibration")

    def measure(self, truth: WorldState) -> JointState | None:
        c = self.config.sensor
        if self._last is not None and same_acquisition(truth.header, self._last.sample_header):
            return self._last
        if not self._sampling.due(truth.header):
            return None
        joints = truth.joint_truth
        indices = [joints.name.index(n) for n in c.joints]
        position = np.asarray(joints.position)[indices] + c.offset
        rng = np.random.default_rng([c.seed, truth.header.epoch, truth.header.time_ns])
        position += rng.normal(0, c.noise_std, len(indices))
        if c.resolution:
            position = np.round(position / c.resolution) * c.resolution
        # Velocity is an explicitly ideal encoder channel; effort is NOT measured.
        sample = JointState(
            name=list(c.joints),
            position=position.tolist(),
            velocity=np.asarray(joints.velocity)[indices].tolist(),
            effort=[],
            ts=truth.header.time_ns / 1e9,
        )
        self._last = sample.with_sample_header(measurement_header(truth.header, c.source_id, ""))
        return self._last

    async def handle_observations(self, truth: WorldState) -> None:
        sample = self.measure(truth)
        if sample is not None:
            self.joint_state.publish(sample)
