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

"""Ties the PimSim spec protocols to the concrete implementations.

The explicit ``class PimSimClient(SceneControl)`` / ``class DimSimClient(
SceneControl)`` declarations make the *type checker* verify conformance at the
class definition; these tests make *CI* verify it even where mypy is not run,
and pin the one protocol that is not yet satisfied so the gap is tracked rather
than buried in prose.
"""

from __future__ import annotations

import pytest

from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.spec.protocols import SceneControl


def test_scene_control_clients_conform() -> None:
    """Both sim-control clients implement the backend-agnostic ``SceneControl``
    surface the parametrized e2e tests swap over (start / stop /
    set_agent_position / add_wall / publish_goal). ``SceneControl`` is
    method-only, so ``issubclass`` is a valid runtime conformance check."""
    assert issubclass(PimSimClient, SceneControl)
    assert issubclass(DimSimClient, SceneControl)


# ``PhysicsAuthority`` is the backend (authority) contract. The Babylon viewer
# implements ``entity_state_batch`` + ``spawn_entity`` but not the full surface:
#   * ``odom`` is declared ``In[]`` (server-side FK consumes it) where the spec
#     wants ``Out[]`` (the authority publishes it),
#   * there is no ``cmd_vel`` stream attribute (the browser consumes ``/cmd_vel``
#     itself, off the module's stream graph),
#   * ``authority_mode`` / ``capabilities`` are unimplemented.
# This xfail records that gap; ``strict=True`` turns it red the moment the
# backend gains every member, forcing whoever closes it to declare conformance
# properly (``class BabylonSceneViewerModule(Module, PhysicsAuthority)``) and
# delete this test.
@pytest.mark.xfail(
    strict=True,
    reason=(
        "BabylonSceneViewerModule does not yet satisfy PhysicsAuthority: odom is "
        "In (spec wants Out), no cmd_vel stream attr, authority_mode/capabilities "
        "unimplemented."
    ),
)
def test_babylon_backend_satisfies_physics_authority() -> None:
    from dimos.experimental.pimsim.babylon.module import BabylonSceneViewerModule

    required = {
        "entity_state_batch",
        "odom",
        "cmd_vel",
        "authority_mode",
        "capabilities",
        "spawn_entity",
    }
    annotated = set(getattr(BabylonSceneViewerModule, "__annotations__", {}))
    present = {name for name in required if hasattr(BabylonSceneViewerModule, name)}
    present |= required & annotated
    missing = required - present
    assert not missing, f"missing PhysicsAuthority members: {sorted(missing)}"
