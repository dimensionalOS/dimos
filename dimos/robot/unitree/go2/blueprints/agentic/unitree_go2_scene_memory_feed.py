#!/usr/bin/env python3
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

"""Scene + spatial memory catalog with NO robot connection -- for external
sensor feeders instead of a live/simulated/replay-dataset Go2.

Unlike `unitree-go2-scene-memory(-agentic)`, this blueprint does not include
`GO2Connection` (or anything from `unitree_go2`) at all: no live/mujoco/
DimSim/`--replay`-dataset backend. `ObjectSceneRegistrationModule` and
`SpatialMemory` still declare the same `/color_image`, `/lidar`,
`/camera_info`, `/tf` inputs -- LCM doesn't care who publishes them, so an
external script (not a module in this graph) can feed sensor data over the
same topics a real GO2Connection would use, e.g. to drive the catalog from
an arbitrary recording format GO2Connection's own backends don't read (a
`.rrd` file, a different dataset's SQLite schema, ...). See
`dimos/e2e_tests/rrd_feed.py` for such a feeder and
`dimos/e2e_tests/master_vqa_test.py::test_master_vqa_china_office_full_rrd`
for the matching e2e test.

Run with a feeder already publishing (or about to start) on those topics:

    uv run dimos run unitree-go2-scene-memory-feed
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.spatial_perception import SpatialMemory
from dimos.skills.mapping.floorplan_skill import FloorplanSkillContainer
from dimos.skills.mapping.lidar_signal_skills import LidarSignalSkills

unitree_go2_scene_memory_feed = autoconnect(
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        localization="lidar",
        camera_optical_frame="camera_optical",
        # See unitree_go2_scene.py / unitree_go2_scene_memory.py for the
        # dedup-radius rationale (touring robot, sparse mid360-style returns).
        distance_threshold=0.35,
        # The rrd feeder stamps each /lidar window at its camera frame's
        # exact ts, so pairs match at any tolerance; keep a little slack for
        # other feeders whose clouds aren't ts-locked to the camera.
        lidar_match_tolerance=0.75,
        # The feeder's /lidar clouds are short accumulations of single mid360
        # scans -- far sparser per object than a live robot's rolling world
        # cloud. The default dense-cloud filter stack rejects most real
        # clusters at that density (measured 12/31 vs 22/31 localized).
        lidar_filter_preset="sparse",
        # The recording's camera runs at ~0.5Hz: a walked-past object is in
        # view for only ~2 frames; requiring 3 corroborations (live default
        # 6, touring blueprints 3) leaves nearly everything stuck pending.
        min_detections_for_permanent=2,
        # Hand-carried tour: some frames are near-black (turning into a dark
        # stairwell) or motion-smeared (walking pace). Skip those before the
        # detector -- dark frames especially yield junk detections and embed
        # semantically close to everything. Conservative floors: only the
        # genuinely unusable frames are dropped.
        min_frame_brightness=0.06,
        min_frame_sharpness=0.12,
    ),
    SpatialMemory.blueprint(),
    FloorplanSkillContainer.blueprint(),
    LidarSignalSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
).global_config(n_workers=6)
