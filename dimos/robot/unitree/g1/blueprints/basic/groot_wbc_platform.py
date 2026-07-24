# Copyright 2025-2026 Dimensional Inc.
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

"""Platform-owned control and localization inputs for the G1 GR00T stack.

Hardware uses PointLIO for ``lidar`` and ``odometry``. A simulation provider
publishes those streams itself. Mapping and navigation remain outside this
boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.simulation.providers import SimulationRequest, load_simulation_provider
from dimos.utils.data import LfsPath

_ROBOT_ONLY_MJCF_PATH = Path(__file__).resolve().parents[2] / "assets" / "g1_29dof.xml"
_ROBOT_MESHDIR = LfsPath("g1_urdf/meshes")


@dataclass(frozen=True)
class G1GrootPlatform:
    backend: Blueprint
    localization_source: Blueprint
    simulation: bool
    adapter_type: str
    adapter_address: str | Path
    tick_rate: float
    policy_decimation: int
    auto_arm: bool
    auto_dry_run: bool
    ramp_seconds: float
    n_workers: int


def resolve_g1_groot_platform() -> G1GrootPlatform:
    if not global_config.simulation:
        from dimos.hardware.sensors.lidar.pointlio.module import PointLio
        from dimos.robot.unitree.g1.blueprints.basic.pointlio_zenoh_relay import (
            PointLioZenohRelay,
        )
        from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

        pointlio_transport = (
            PointLioZenohRelay.blueprint() if global_config.transport == "zenoh" else autoconnect()
        )
        return G1GrootPlatform(
            backend=G1WholeBodyConnection.blueprint(release_sport_mode=True),
            localization_source=autoconnect(pointlio_transport, PointLio.blueprint()),
            simulation=False,
            adapter_type="transport_lcm",
            adapter_address="",
            tick_rate=100.0,
            policy_decimation=2,
            auto_arm=False,
            auto_dry_run=True,
            ramp_seconds=10.0,
            n_workers=10,
        )

    if global_config.simulation != "mujoco":
        raise ValueError("unitree-g1-groot-wbc only supports --simulation mujoco")
    if not global_config.simulation_provider:
        raise ValueError("unitree-g1-groot-wbc simulation requires --simulation-provider pimsim")

    provider = load_simulation_provider(global_config.simulation_provider)
    binding = provider.build(
        SimulationRequest(
            robot_model="unitree_g1",
            model_path=_ROBOT_ONLY_MJCF_PATH,
            mesh_dir=_ROBOT_MESHDIR,
            scene_package=global_config.scene_package,
        )
    )
    return G1GrootPlatform(
        backend=binding.backend,
        localization_source=autoconnect(),
        simulation=True,
        adapter_type=binding.adapter_type,
        adapter_address=binding.adapter_address,
        tick_rate=50.0,
        policy_decimation=1,
        auto_arm=True,
        auto_dry_run=False,
        ramp_seconds=0.0,
        n_workers=10,
    )
