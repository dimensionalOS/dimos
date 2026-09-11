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

"""Agentic recorded-memory world blueprints."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.memory.recording_player import RecordingPlayer
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.constants import BASE_LINK_HEIGHT, ROBOT_HEIGHT
from dimos.teleop.memory_world.module import MemoryWorldModule

MEMORY_WORLD_SYSTEM_PROMPT = """You answer questions about a recorded robot memory and plan routes through it.

Pick the tool by the question:
- "Where did you see X", "where did you last see X", "when did you see X": call
  find_in_memory with a short description of X. It reports each place X was seen and
  when it was last seen. Answer with that place and time.
- "Show me the top N images of X": call show_frames_in_memory with the description and N.
- "Navigate to X", "go to where you saw X", "plan a route to X": call navigate_with_text
  with the description. It looks X up in the memory, sets it as the navigation goal, and
  the viewer draws the planned route from the robot's last recorded pose. Report whether
  a goal was set.
- Anything else about the recording: call analyze_memory. Write complete Python that
  inspects the available mem2 streams and assigns a dictionary to `result`.

Only use these documented APIs in analyze_memory:

```python
names = store.list_streams()
summary = store.summary()
stream = store.streams["odom"]
for observation in stream:
    pose = observation.pose_tuple  # (x, y, z, qx, qy, qz, qw) or None
    payload = observation.data
    observation_id = observation.id
path = sample_pose_path("odom", max_points=200)
```

`store.read_stream` does not exist. Use `sample_pose_path` for trajectory questions.
Do not silently catch stream-access errors; let them surface so the tool reports failure.
The dictionary requires `answer` and may include:

- `focus_point`: one world-frame [x, y, z] answer location
- `regions`: objects with `points`, `label`, `color`, and `opacity`
- `evidence_paths`: objects with `points`, `label`, and `color`
- `points`: objects with `position`, `label`, and `color`
- `observation_ids`: color_image observation IDs supporting the answer

All geometry uses world-frame meters and every point has exactly three coordinates.
Prefer robust aggregates such as medians over single extreme observations. Include
the observations or geometry that support the answer. Keep result geometry concise.
Do not provide `route`; the viewer draws the planner's route itself.
Do not claim that a visualization was shown unless the tool succeeds.
"""

# Go2 recordings root their tf tree at odom.
WORLD_FRAME = "odom"
# A replay may run faster than the mapper. The mapper queues every scan and keeps
# the transforms for all of them, so lagging behind delays the map instead of holing it.
REPLAY_QUEUE_DEPTH = 20_000
REPLAY_TF_WINDOW_S = 7_200.0
VOXEL_SIZE = 0.08

memory_world_agent = autoconnect(
    # The recording stands in for the robot: PointLIO's sensor-frame scans and
    # the tf tree feed the same mapper and planner the Go2 runs.
    RecordingPlayer.blueprint(stream="pointlio_lidar", speed=1.0),
    # Replaying faster than real time only helps while the mapper keeps up.
    # Once it falls behind the tf window, every scan is dropped for want of
    # a pose, so the ray budget matches a live robot's.
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE,
        world_frame=WORLD_FRAME,
        max_range=20.0,
        ray_subsample=10,
        worker_threads=8,
        input_queues={"lidar": REPLAY_QUEUE_DEPTH},
        tf_window_s=REPLAY_TF_WINDOW_S,
        emit_every=5,
        global_emit_every=20,
        min_health=-1,
        max_health=5,
        support_min=4,
    ),
    # Stairs come out sparse from a single walk, and the defaults leave the
    # landing the robot ends on as its own surface island. A wider closing
    # radius, no wall clearance and a taller step keep the flights connected.
    MLSPlannerNative.blueprint(
        world_frame=WORLD_FRAME,
        base_frame="base_link",
        voxel_size=VOXEL_SIZE,
        robot_height=ROBOT_HEIGHT,
        start_z_offset_m=BASE_LINK_HEIGHT,
        surface_closing_radius=1.0,
        wall_clearance_m=0.0,
        step_threshold_m=0.4,
        viz_publish_hz=0.0,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    MovementManager.blueprint(),
    # The memory world's own search decides what is a match, so the container
    # accepts whatever it returns.
    NavigationSkillContainer.blueprint(similarity_threshold=0.0),
    # Outdoor walks with stairs span far more height than the indoor defaults.
    MemoryWorldModule.blueprint(
        voxel_size=VOXEL_SIZE,
        world_frame=WORLD_FRAME,
        lidar_stream_name="pointlio_lidar",
        map_z_min=-1.0,
        map_z_max=5.0,
        height_ramp_span_m=5.0,
        max_points=2_000_000,
    ),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MEMORY_WORLD_SYSTEM_PROMPT),
).global_config(
    n_workers=8,
    # zenoh 1.10 peers that start together can poison each other's dial and never
    # link. Without gossip each native dials its peers once and links every time.
    zenoh_gossip=False,
)
