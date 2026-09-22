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
- "Where did you see X", "where is X", "how many X", "how tall/big is X": call
  find_in_memory with a short description of X. When its result says `located: true`, a
  detector drew a box around each object. A place with an `extent` (x, y, z meters) was
  measured on the object: its position is the object's and you may give its size. The
  viewer draws a box around every measured object, so "show the box" needs no extra
  call unless a subset or another color is wanted. Count only these as objects. A
  place with `extent: null` is a sighting the detector could not place: its position
  is where the camera stood, and one object seen from several spots makes several
  sightings, so report them as "also seen but not placed", never added to the count.
  When the result says `located: false`, nothing was boxed and every place is where X
  was seen FROM; say so and count distinct places, not objects.
- "Show me the top N images of X": call show_frames_in_memory with the description and N.
- "Navigate to (x, y)", "go to that position", or any goal you already know the world
  coordinates of: call navigate_to_position with x, y and the ground z there. Never put
  coordinates into navigate_with_text; it only matches descriptions of what was seen.
- "Navigate to X", "go to where you saw X", "plan a route to X": call navigate_with_text
  with the description. It looks X up in the memory, sets a goal about a metre in front
  of X (or where the robot stood when it saw X, if X was never measured), and the
  viewer draws the planned route from the
  robot's last recorded pose. The planner needs a few seconds; if `route` is still None
  in analyze_memory right after, wait and call it again. Report whether a goal was set. Its replies call the memory the "semantic map"; to the user it is the
  recording, so say "the recording" or "where I saw X".
- Anything else about the recording: call analyze_memory. Write complete Python that
  inspects the available mem2 streams and assigns a dictionary to `result`. The streams
  are sensor data (camera frames, lidar, poses, tf).
- Questions that mix the two: chain the tools. find_in_memory returns each place's world
  position, time and frame id in its metadata; analyze_memory can then measure the lidar
  map around those positions, along the trajectory, or along `route`, and list the frame
  ids in `observation_ids` so the viewer shows the images beside the geometry.

Only use these documented APIs in analyze_memory:

```python
names = store.list_streams()
summary = store.summary()
stream = store.streams[pose_stream]  # the robot's poses, one per observation
for observation in stream:
    pose = observation.pose_tuple  # (x, y, z, qx, qy, qz, qw) or None
    ts = observation.ts  # seconds since the epoch
    payload = observation.data
    observation_id = observation.id
start, end = stream.get_time_range()
window = stream.time_range(start + 290.0, start + 310.0)  # observations around 5 minutes in
frames = store.streams["color_image"].at(start + 300.0, tolerance=2.0)
path = sample_pose_path(max_points=200)  # the trajectory from pose_stream
cloud = store.streams["voxel_keyframe"].last().data.points_f32()  # final lidar map, (N, 3)
```

`sample_pose_path` takes at most 2000 points; 200 is plenty for drawing and 2000 for
measuring distance. Poses jitter by a few centimeters and the robot often turns in
place, so never divide a height change by a horizontal step: measure slope, grade or
steepness over windows of at least 2 m of horizontal travel. `store.read_stream` does not exist. The robot's trajectory is the pose of every
observation of the stream named by `pose_stream`. Poses stamped on other streams are
in another frame, so never use them. Use `sample_pose_path` for whole-trajectory questions.
"N minutes in" means the recording's start time plus N minutes. To show the frames
behind an answer, put `color_image` observation ids in `observation_ids`. The lidar map
is 8 cm voxel centers in world meters; select the points within a few meters of a place
to measure ground height, clearance, widths and heights there. The ground is about 0.3 m
below a trajectory pose. The person walking the robot is in the map: ignore voxels
within 0.8 m of the trajectory between 0.2 and 1.6 m above the ground when measuring
widths, clearance or ceilings. `route` is the planner's current route as world [x, y, z]
points, or None until navigate_with_text has planned one.
`objects` lists every object find_in_memory has located so far, each a dict with `label`,
`position` ([x, y, z] on the object, world frame), `extent` (full sizes along the object's
own axes, x the long horizontal one), `yaw` (heading of that axis, world radians), `height`
(meters), `confidence`, `views`, `seen_from` ([x, y, z] where the robot stood when it saw
the object, on its trajectory), `ts` and `best_frame_id` (a `color_image` observation id).
Measure and compare objects from it; pass `position`, `extent` and `yaw` straight into
`boxes`; put the `best_frame_id`s in `observation_ids` to show them. To navigate to an
object you have already picked from `objects`, call navigate_to_position with a point
about a metre in front of its `position` on the line from `seen_from`, at the
`seen_from` height, with a yaw facing the object; do not search for it again.
Do not silently catch stream-access errors; let them surface so the tool reports failure.
The dictionary requires `answer`, one plain string (put lists of numbers in the
geometry fields, not in the answer; anything past 8000 characters is cut), and may include:

- `focus_point`: one world-frame [x, y, z] answer location
- `boxes`: 3D bounding boxes, objects with `center`, `extent` ([x, y, z] full sizes in
  meters along the box's own axes), `yaw` (heading of its x axis, world radians) `label`,
  and `color`. This is the only way to draw a box; never build one
  from `regions`, which are flat floor polygons filled at one height.
- `regions`: flat floor polygons with `points`, `label`, `color`, and `opacity`
- `evidence_paths`: objects with `points`, `label`, and `color`
- `points`: objects with `position`, `label`, and `color`

A `color` is a hex string such as "#ff8800". Leave it out to get the default.
- `observation_ids`: color_image observation IDs supporting the answer

All geometry uses world-frame meters and every point has exactly three coordinates.
Prefer robust aggregates such as medians over single extreme observations. Include
the observations or geometry that support the answer. Keep result geometry concise.
Do not provide `route`; the viewer draws the planner's route itself.
Do not claim that a visualization was shown unless the tool succeeds.
"""

# Go2 recordings root their tf tree at odom.
WORLD_FRAME = "odom"
REPLAY_QUEUE_DEPTH = 20_000
REPLAY_TF_WINDOW_S = 7_200.0
VOXEL_SIZE = 0.08
# The Go2 front camera at 1280x720, from the camera_info the same rig recorded
# in the office. Recordings that carry camera_info use theirs instead.
GO2_CAMERA_INTRINSICS = (797.4756, 796.4872, 643.5352, 349.2784)
GO2_CAMERA_DISTORTION = (-0.0730943, -0.0234114, -0.0069306, 0.0092387)

# The planner and the memory world share the topics a robot's mapper would
# publish on: the memory world publishes the stored map and the robot's final
# pose itself, so routes plan from the first second.
memory_world_agent = autoconnect(
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
    ),
    MovementManager.blueprint(),
    # The memory world's own search decides what is a match, so the container
    # accepts whatever it returns.
    NavigationSkillContainer.blueprint(similarity_threshold=0.0),
    # Outdoor walks span far more height than the indoor defaults: stairs go
    # 1.6 m below the start and building faces carry the streets up to 8 m.
    MemoryWorldModule.blueprint(
        voxel_size=VOXEL_SIZE,
        world_frame=WORLD_FRAME,
        lidar_stream_name="pointlio_lidar",
        # A recorded RealSense depth stream is on the depth camera's grid, not the
        # color camera's, so objects are measured on the lidar map instead.
        depth_stream_name="",
        camera_intrinsics=GO2_CAMERA_INTRINSICS,
        camera_distortion=GO2_CAMERA_DISTORTION,
        # OWLv2 scores this wide-angle outdoor camera lower than a RealSense
        # indoors: cars top out near 0.47 and trees 0.44, but people standing
        # right in front of the robot score 0.17 to 0.22, so 0.3 refuses them.
        locate_threshold=0.2,
        map_z_min=-2.0,
        map_z_max=8.0,
        height_ramp_span_m=5.0,
        max_points=2_000_000,
    ).remappings(
        [
            (MemoryWorldModule, "map", "global_map"),
            (MemoryWorldModule, "global_map", "global_map_unused"),
        ]
    ),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MEMORY_WORLD_SYSTEM_PROMPT),
).global_config(
    n_workers=8,
    # zenoh 1.10 peers that start together can poison each other's dial and never
    # link. Without gossip each native dials its peers once and links every time.
    zenoh_gossip=False,
)

# Maps a recording once: the recording stands in for the robot, PointLIO's
# sensor-frame scans and the tf tree feed the same mapper the Go2 runs, and the
# memory world records the mapper's snapshots into the recording. Run it on a
# recording without a map, then the agent blueprint loads that map at once.
memory_world_map = autoconnect(
    RecordingPlayer.blueprint(stream="pointlio_lidar", speed=4.0),
    # Replaying faster than real time only helps while the mapper keeps up.
    # Its scan queue and tf window hold a whole recording, so lagging behind
    # delays the map instead of holing it.
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
    MemoryWorldModule.blueprint(
        voxel_size=VOXEL_SIZE,
        world_frame=WORLD_FRAME,
        lidar_stream_name="pointlio_lidar",
        # A recorded RealSense depth stream is on the depth camera's grid, not the
        # color camera's, so objects are measured on the lidar map instead.
        depth_stream_name="",
        camera_intrinsics=GO2_CAMERA_INTRINSICS,
        camera_distortion=GO2_CAMERA_DISTORTION,
        # OWLv2 scores this wide-angle outdoor camera lower than a RealSense
        # indoors: cars top out near 0.47 and trees 0.44, but people standing
        # right in front of the robot score 0.17 to 0.22, so 0.3 refuses them.
        locate_threshold=0.2,
        map_z_min=-2.0,
        map_z_max=8.0,
        height_ramp_span_m=5.0,
        max_points=2_000_000,
        build_image_index_on_start=False,
    ).remappings(
        [(MemoryWorldModule, "map", "map_unused"), (MemoryWorldModule, "tf", "tf_unused")]
    ),
).global_config(n_workers=8, zenoh_gossip=False)
