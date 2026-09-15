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

"""Observation smoke using existing dimos modules, not a navigation benchmark.

    dimos evals run dimos.evals.suites.habitat_smoke --agent dimos.evals.agents.pi

Use build_suite(scene_dataset_config=..., scene_id=...) in
another suite module to select a downloaded scene. Defaults match habitat-nav.
"""

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.types import EvalCase, Outcome, Suite, recording


def sensor_score(outcome: Outcome) -> float:
    """Require readable RGB, derived point cloud, and achieved pose samples."""
    with recording(outcome) as store:
        required = ("color_image", "habitat_scan", "odometry")
        if any(name not in store.streams for name in required):
            return 0.0
        try:
            for name in required:
                getattr(store.streams, name).last().data  # noqa: B018 - force lazy decoding
        except LookupError:
            return 0.0
        return 1.0


def build_suite(*, scene_dataset_config: str | None = None, scene_id: str | None = None) -> Suite:
    return [
        EvalCase(
            id="habitat_observe",
            inputs=(
                "Use the available observation tool to inspect the scene. "
                "Briefly describe what you see."
            ),
            environment=HabitatEnvironment(
                scene_dataset_config=scene_dataset_config,
                scene_id=scene_id,
                blueprint=["habitat-nav", "mcp-server", "observe-skill"],
            ),
            grade=sensor_score,
            timeout_s=300.0,
            tags=frozenset({"habitat", "smoke", "observation"}),
        )
    ]


SUITE = build_suite()
