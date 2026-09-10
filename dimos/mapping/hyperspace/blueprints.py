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

"""Hyperspace on a replayed RGB-D recording, with the answers in Rerun.

    dimos run demo-hyperspace --hyperspacereplay.db-path ~/datasets/alfred/sf_office_drive1.db

The replay feeds colour, depth, camera infos and tf; ``HyperspacePatches``
keeps keyframes and writes their patch embeddings into ``hyperspace.db``;
``Hyperspace`` answers ``demo_queries`` against that db after
``demo_after_s`` and every ``demo_every_s`` after, publishing ``scene_map``
(grey) and ``query_result`` (scored) for Rerun. Change the questions with
``--hyperspace.demo-queries='["a chair","a door"]'``.
"""

from __future__ import annotations

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.hyperspace.module import Hyperspace, HyperspacePatches
from dimos.mapping.hyperspace.replay import HyperspaceReplay
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule

HYPERSPACE_DB = str(RECORDINGS_DIR / "hyperspace.db")

demo_hyperspace = autoconnect(
    HyperspaceReplay.blueprint(),
    HyperspacePatches.blueprint(db_path=HYPERSPACE_DB),
    Hyperspace.blueprint(
        db_path=HYPERSPACE_DB,
        demo_after_s=60.0,
        demo_every_s=30.0,
        demo_queries=["a traffic cone", "a chair"],
    ),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open=global_config.rerun_open,
        rerun_web=global_config.rerun_web,
    ),
)
