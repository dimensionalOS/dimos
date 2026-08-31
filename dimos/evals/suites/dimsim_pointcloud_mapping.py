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

"""Multi-step point-cloud eval: three Pi conditions on one live DimSim case.

The same ``Pi`` agent maps an initially unmapped apartment and counts its rooms under
three configurations that differ only in ``system_prompt`` / ``skills`` —
identical case, stack, model settings, and grader::

    # 1. agent_encode — spatial evidence only from PointCloud2.agent_encode()
    #    (needs the point-cloud observability branch, PR #3415)
    DIMOS_N_WORKERS=10 dimos evals run dimos.evals.suites.dimsim_pointcloud_mapping \
        --agent dimos.evals.agents.pi --set max_steps=30 --set system_prompt="$(python -c \
        'import dimos.evals.suites.dimsim_pointcloud_mapping as m; print(m.AGENT_ENCODE_PROMPT)')"

    # 2. numpy control — raw point arrays, agent_encode forbidden
    DIMOS_N_WORKERS=10 dimos evals run dimos.evals.suites.dimsim_pointcloud_mapping \
        --agent dimos.evals.agents.pi --set max_steps=30 --set system_prompt="$(python -c \
        'import dimos.evals.suites.dimsim_pointcloud_mapping as m; print(m.NUMPY_PROMPT)')"

    # 3. spatial_skill — base prompt plus one explicit purpose-built harness skill
    DIMOS_N_WORKERS=10 dimos evals run dimos.evals.suites.dimsim_pointcloud_mapping \
        --agent dimos.evals.agents.pi --set max_steps=30 \
        --set skills='["/absolute/path/to/spatial-skill/SKILL.md"]'

``DIMOS_N_WORKERS=10`` matches the composed ``unitree-go2`` blueprint that the
stack below decomposes. The access contracts are prompt-enforced, not
sandboxed: audit each run's ``trajectory.json`` and raw requests before
comparing, and exclude (do not score) a violating run. Smoke-launch the stack
once before any paid run.
"""

from __future__ import annotations

import math

from dimos.evals.agents.pi import PI_SYSTEM_PROMPT
from dimos.evals.environments.sim import Sim
from dimos.evals.scorers import exact, first_number
from dimos.evals.types import EvalCase, Outcome, Suite, recording

# unitree-go2 decomposed into its atoms
# (dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py) minus the
# autonomous WavefrontFrontierExplorer and PatrollingModule: deliberate
# move_to navigation keeps working, the exploration shortcut is removed at
# the source.
STACK = (
    "unitree-go2-basic voxel-grid-mapper cost-mapper replanning-a-star-planner "
    "movement-manager mcp-server unitree-skill-container"
)

# Ground truth measured once from the apartment scene geometry
# (misc/DimSim/scenes/apartment/structure.glb wall meshes and
# objects/manifest.json; world frame: x east, y north — the frame odometry
# and the lidar readouts use). Interior x in (-6, 6), y in (-5, 5). A main
# wall along y=0 (doorway at x=0) splits the apartment; a kitchen wall at
# x=-2 (door at y~2.5) closes off the southwest; a bathroom wall at x=1
# (door at y~-2.5) closes off the northeast. The walled front yard (y > 5,
# reached through the entrance and sliding doors) is outdoors and not a room.
# Rooms are wall-separated interior spaces, open plan counted once — the
# living/dining space is one room.
ROOMS: dict[str, tuple[float, float]] = {
    "living_dining": (2.0, 2.5),  # sectional, TV, dining table
    "kitchen": (-4.0, 2.5),  # fridge, gas range, sink
    "bedroom": (-3.0, -2.5),  # queen bed, desk
    "bathroom": (3.0, -2.5),  # bathtub, toilet, vanity
}
N_ROOMS = len(ROOMS)
# Every representative point sits ~2 m past its room's doorway, so coming
# within this radius requires actually entering the room.
VISIT_RADIUS_M = 1.5

INSTRUCTION = (
    "Map the entire reachable apartment and determine how many distinct rooms "
    "it contains. Count wall-separated interior rooms: an open-plan area is "
    "one room, and outdoor space is not a room. Move only with deliberate "
    "move_to calls using world x/y coordinates; do not use exploration, "
    "patrol, follow, or other autonomous navigation tools. Decide what lidar "
    "history you need. When you are confident, answer with one integer and "
    "nothing else."
)

# Shared selection guidance: names the existing selectors so both controlled
# conditions spend their steps on the task, without choosing a frame, window,
# or fusion for the agent.
SELECTION_GUIDANCE = (
    "Choosing lidar history is your decision: a stream's .last() is the "
    "newest frame, .range_time(t0, t1) and .from_timestamp(ts) select "
    "windows, and a window fuses into one cloud with "
    ".transform(VoxelMapTransformer(voxel_size=0.05, emit_every=0)).last() "
    "(from dimos.mapping.voxels.module import VoxelMapTransformer)."
)

AGENT_ENCODE_CONTRACT = (
    "Point-cloud access contract: You may use Python in bash to open the "
    "provided recording, select any lidar frame or time window, and fuse "
    "frames with the existing stream transforms. For spatial evidence, "
    "inspect only PointCloud2.AGENT_ENCODE_LEGEND and values returned by "
    "PointCloud2.agent_encode(). Do not access raw point arrays, "
    "points_f32(), points(), NumPy/Open3D geometry, SQLite blobs, or the "
    "implementation/source of PointCloud2. You may read odometry and "
    "ordinary stream metadata for navigation. The implementation inside "
    "agent_encode() and VoxelMapTransformer is allowed; the restriction is "
    "on evidence your agent-authored analysis reads."
)

NUMPY_CONTRACT = (
    "Point-cloud access contract: You may use Python in bash to open the "
    "provided recording, select any lidar frame or time window, fuse frames "
    "with the existing stream transforms, and analyze raw point arrays with "
    "NumPy or other installed Python libraries. Do not call agent_encode(), "
    "read AGENT_ENCODE_LEGEND, or inspect their implementation/source. You "
    "may read odometry and ordinary stream metadata for navigation."
)

AGENT_ENCODE_PROMPT = f"{PI_SYSTEM_PROMPT}\n\n{SELECTION_GUIDANCE}\n\n{AGENT_ENCODE_CONTRACT}"
NUMPY_PROMPT = f"{PI_SYSTEM_PROMPT}\n\n{SELECTION_GUIDANCE}\n\n{NUMPY_CONTRACT}"


def grade_rooms(o: Outcome) -> float:
    """0.5 for the exact room count in the reply, plus 0.5 times the fraction
    of rooms whose representative point the recorded odometry came within
    ``VISIT_RADIUS_M`` of. Passing at the default threshold of 1.0 means the
    count was right and every room was physically entered."""
    try:
        count = exact(float(N_ROOMS), first_number(o.trajectory.final_answer))
    except ValueError:  # no number in the reply — coverage can still score
        count = 0.0
    store = recording(o)
    try:
        path = [(p.data.position.x, p.data.position.y) for p in store.streams.odom]
    finally:
        store.stop()
    visited = sum(
        any(math.hypot(x - rx, y - ry) <= VISIT_RADIUS_M for x, y in path)
        for rx, ry in ROOMS.values()
    )
    return 0.5 * count + 0.5 * visited / N_ROOMS


count_rooms = EvalCase(
    id="dimsim_count_rooms",
    inputs=INSTRUCTION,
    environment=Sim(blueprint=STACK, simulator="dimsim", scene="apartment"),
    grade=grade_rooms,
    timeout_s=1200.0,  # room for several blocking move_to calls (each up to ~100 s)
    tags=frozenset({"nav", "pointcloud", "system"}),
)

SUITE: Suite = [count_rooms]
