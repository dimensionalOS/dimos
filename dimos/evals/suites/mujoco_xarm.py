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

"""xArm7 at a table with an apple, an orange and a cup — MuJoCo manipulation cases.

Ground truth comes from the simulator: ``MujocoEnvironment(tracked_bodies=...)``
has MujocoSimModule publish ``world -> <body>`` on ``tf``, and graders read the
first and last recorded pose of each object. The scene is ``data/xarm7/scene.xml``.

    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.mcp_client_adapter
"""

from __future__ import annotations

from collections.abc import Callable
import math

from dimos.evals.environments.mujoco_sim import (
    MujocoEnvironment,
    first_body_transform,
    last_body_transform,
)
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording

TRACKED = ("apple", "orange", "cup")


# The blueprint pins Moondream (a 3.85 GB Hugging Face download on first use) and
# EdgeTAM (the ``dimos[perception]`` extra). OWLv2 takes text prompts natively at a
# ~600 MB download, and YOLOE box segmentation ships in LFS data. Pre-fetch once:
#     hf download google/owlv2-base-patch16-ensemble
LOCAL_PERCEPTION = {
    "OBJECTSCENEREGISTRATIONMODULE__DETECTOR_BACKEND": "owlv2",
    "OBJECTSCENEREGISTRATIONMODULE__SEGMENTATION_BACKEND": "yolo",
}


# Everything above the arm: detector, segmenter, grasp ranking and the pick pipeline.
# Without them the agent has the wrist camera (``observe``) and the planner skills.
PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)


def arm_only_environment(*, headless: bool = True, rerun: bool = False) -> MujocoEnvironment:
    """The xArm7 table scene with no perception: planner skills plus wrist-camera frames.

    ``rerun=True`` keeps the Rerun bridge, the way to watch on macOS where MuJoCo's
    own viewer needs mjpython and the sim therefore runs headless.
    """
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=(*PERCEPTION_MODULES, *(() if rerun else ("rerun-bridge-module",))),
        tracked_bodies=TRACKED,
        headless=headless,
    )


def environment(
    *, headless: bool = True, module_env: dict[str, str] | None = None
) -> MujocoEnvironment:
    """The xArm7 table scene; ``headless=False`` opens the MuJoCo viewer window."""
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server"],
        disable=("rerun-bridge-module",),
        tracked_bodies=TRACKED,
        headless=headless,
        module_env=LOCAL_PERCEPTION if module_env is None else module_env,
    )


def sensor_score(outcome: Outcome) -> float:
    """Require readable RGB, joint state and a pose for every tracked body."""
    with recording(outcome) as store:
        try:
            store.streams.color_image.last().data  # noqa: B018 - force lazy decoding
            store.streams.coordinator_joint_state.last().data  # noqa: B018
            for body in TRACKED:
                last_body_transform(store, body)
        except (LookupError, AttributeError):
            return 0.0
    return 1.0


def lifted(body: str, *, by_m: float) -> Callable[[Outcome], float]:
    """How far the body ended above where it started, full credit at ``by_m``."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation.z
                end = last_body_transform(store, body).translation.z
            except LookupError:
                return 0.0
        return min(max((end - start) / by_m, 0.0), 1.0)

    return grade


def _ended_near(body: str, x: float, y: float, *, band_m: float) -> Callable[[Outcome], float]:
    """Where the body ended up on the table plane: 1.0 at the target, 0.0 at ``band_m`` away."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return ramp(math.hypot(t.x - x, t.y - y), band=band_m)

    return grade


SUITE: Suite = [
    EvalCase(
        id="xarm_observe",
        inputs=(
            "Use the available observation tool to look at the table in front of you. "
            "Briefly list what you see."
        ),
        environment=environment(),
        grade=sensor_score,
        timeout_s=300.0,
        tags=frozenset({"mujoco", "manipulation", "smoke"}),
    ),
    EvalCase(
        id="xarm_lift_apple",
        inputs="Pick up the apple and hold it in the air above the table.",
        environment=environment(),
        grade=lifted("apple", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    ),
    EvalCase(
        id="xarm_move_orange",
        inputs=(
            "Move the orange to the other side of the table and release it there, "
            "around x=0.45 m, y=0.12 m in the world frame."
        ),
        environment=environment(),
        grade=_ended_near("orange", 0.45, 0.12, band_m=0.10),
        timeout_s=600.0,
        threshold=0.5,  # within 5 cm
        tags=frozenset({"mujoco", "manipulation", "pick", "place"}),
    ),
]
