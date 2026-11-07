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

"""The autoresearch lab on one Metal in MuJoCo: an agent's programs run on the
bench, judged from the simulator's true object poses, and the successful
episodes are recorded as a Quest-profile dataset."""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.autoresearch.module import AutoresearchModule
from dimos.autoresearch.truth_scene import TruthObject, TruthSceneModule
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import collect
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection import (
    METAL_QUEST_PROFILE,
)
from dimos.robot.manipulators.makermods_metal.blueprints.sim import metal_sim

#: The bench objects' boxes (bench.xml): the cube and cylinder bodies sit at
#: their centres, the bowl body at the middle of its base.
METAL_BENCH_TRUTH = {
    "cube": TruthObject(size=(0.04, 0.04, 0.04)),
    "cylinder": TruthObject(size=(0.05, 0.05, 0.08)),
    "bowl": TruthObject(size=(0.10, 0.10, 0.04), centre=(0.0, 0.0, 0.02)),
}

# The recorder comes first so it stops after the producers it records, and the
# lab after the arm so its stop can still cancel motion. No ManipulationSkills
# and no PickAndPlaceModule: McpServer lists every @skill, and the actor must
# have no movement tool of its own. The recorder's wrist image is the sim camera.
autoresearch_metal_sim = autoconnect(
    collect(profile=METAL_QUEST_PROFILE),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    metal_sim,
    TruthSceneModule.blueprint(objects=METAL_BENCH_TRUTH),
    AutoresearchModule.blueprint(),
    McpServer.blueprint(),
).remappings([("recorder", "wrist_image", "color_image")])
