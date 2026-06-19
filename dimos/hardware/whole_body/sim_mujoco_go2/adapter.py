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

"""Registry entry for the MuJoCo Go2 whole-body sim adapter.

The actual implementation lives at
[dimos/simulation/mujoco/go2_whole_body.py]. This thin module exposes it to
the `whole_body_adapter_registry` so blueprints can construct it via
`adapter_type="sim_mujoco_go2"`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry
    from dimos.hardware.whole_body.spec import WholeBodyAdapter


def _factory(
    dof: int = 12,
    hardware_id: str = "go2",
    address: str | None = None,  # accepted-and-ignored
    domain_id: int | None = None,  # accepted-and-ignored
    mjcf_path: str | None = None,
    render: bool = True,
    step_period: float = 0.005,
    keyframe_name: str = "lie",
    **_: Any,
) -> WholeBodyAdapter:
    """Construct `MujocoGo2WholeBody` from coordinator-shaped kwargs."""
    from pathlib import Path

    from dimos.simulation.mujoco.go2_whole_body import (
        MujocoGo2Config,
        MujocoGo2WholeBody,
    )

    cfg_kwargs: dict[str, Any] = {
        "step_period": step_period,
        "keyframe_name": keyframe_name,
        "render": render,
    }
    if mjcf_path is not None:
        cfg_kwargs["mjcf_path"] = Path(mjcf_path)
    return MujocoGo2WholeBody(MujocoGo2Config(**cfg_kwargs))


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Auto-discovered by `whole_body_adapter_registry.discover()`."""
    registry.register("sim_mujoco_go2", _factory)
