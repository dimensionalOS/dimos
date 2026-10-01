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
"""TypeSafe agent constants."""

# API
DEFAULT_MODEL = "jev-latest"
DEFAULT_BASE_URL = "https://api.typesafe.ai"
BASE_URL_ENV = "TYPESAFE_BASE_URL"
REQUEST_TIMEOUT_S = 10.0
RETRY_STATUSES = frozenset({429, 500, 502, 503, 504, 529})
REQUEST_ATTEMPTS = 3

# Perception / world state
MAX_OBJECTS = 20  # 2D: closest first; the model reads words, not a scene graph
MAX_OBSTACLES = 5  # 3D: the nearest floor-level obstacles listed after the target
MAX_LABEL = 24  # catalogue titles run to a sentence; the target keeps its full label
OBSTACLE_RANGE_M = 4.0
TARGET_AT_M = 0.5  # the goal's coordinates name the object whose centre is within this
BASE_ABOVE_FLOOR_M = 0.15  # pose z above the floor (habitat, Go2 base_link)
BODY_BAND_M = (0.1, 0.9)  # above the floor: lower is driven over, higher is driven under
MIN_FOOTPRINT_M = 0.2
WALL_WORD = "wall"
RECENT_S = 8.0  # how far back `robot.recent` looks
STILL_S = 1.5  # standing this long from the start already reads still
SAME_WAY_DEG = 60.0  # a way past that moved less than this between ticks is the same one
FORGET_SIDE_S = 3.0  # the line clear this long: the side gone around by is forgotten
ROBOT_RADIUS_M = 0.25  # one body radius for every robot
OPEN_M = 1.5  # a direction is open when nothing solid lies within this range
CORNER_JUMP_M = 1.0  # neighbouring rays this different in range: a corner with space behind it
ADJACENT_M = 0.3  # furniture this close to the target's box belongs to it
STOPPED_BY_M = 0.5  # such furniture this close on the line has stopped the robot ...
AT_TARGET_M = 0.6  # ... and is in the way when the target's edge is still further than this
AHEAD_DEG = 15.0
WALL_RUN_M = 1.0  # a wall box at least this long (and twice its thickness) is a run of wall
DOORWAY_MIN_M = 0.7  # a gap in a run of wall the robot fits through ...
DOORWAY_MAX_M = 3.0  # ... and wider than this it is open room, not an opening in a wall
JAMB_M = 0.45  # a wide opening is passed no closer than this to its jambs
THROUGH_M = 0.5  # the points in front of and beyond an opening that bearings are taken to
BLOCKED_M = 0.5  # nearer than this reads blocked
TRAIL_STEP_M = 0.5  # the robot's own trail is kept at this spacing ...
TRAIL_AGE_S = 10.0  # ... and counts as `been_there` once it is this old
BEEN_M = 0.75  # a way whose far point is this close to the trail has been driven already
PROBE_M = 2.0  # how far along an open direction its far point is taken
SCAN_CELL_M = 0.1  # what the scan saw is remembered on this grid ...
SCAN_KEEP_S = 30.0  # ... this long ...
SCAN_KEEP_M = 2.5  # ... from returns no further than this
STUCK_S = 3.0  # driving picks for this long ...
STUCK_M = 0.15  # ... with less than this moved: stuck
RAY_STEP_DEG = 5
DISTANCE_WORDS = ((0.5, "touching"), (1.5, "near"), (4.0, "mid"), (float("inf"), "far"))
SECTOR_NAMES = (
    "ahead",
    "ahead_left",
    "left",
    "behind_left",
    "behind",
    "behind_right",
    "right",
    "ahead_right",
)
BEARING_WORDS_2D = ("far_left", "left", "center", "right", "far_right")
SIZE_WORDS_2D = ((0.4, "filling_view"), (0.15, "large"), (0.03, "medium"), (-1.0, "small"))
IMAGE_SIZE = (1280, 720)  # for 2D detections
LIDAR_BAND = (-0.2, 0.8, 5.0)  # z_min, z_max, max_range (m) relative to the robot
STALE_S = 2.0  # inputs older than this are ignored

# Navigation
NAV_MAX_HZ = 2.0  # pose updates arrive faster than the model should be asked
NAV_TIMEOUT_S = 5.0  # a reply slower than this is skipped; the next tick asks again
LINEAR_SPEED = 0.5  # m/s
ANGULAR_SPEED = 0.8  # rad/s
LINEAR_ACCEL = 0.8  # m/s^2 ramp
ANGULAR_ACCEL = 1.6  # rad/s^2 ramp
PUBLISH_HZ = 10.0  # cmd_vel rate
STOP_THRESHOLD = 0.7  # noul
MIN_PROBABILITY = 0.0  # an axis pick below this probability reads as none; 0: every pick counts
REACHED_M = 0.5
GIVE_UP_S = 5.0  # goal clears after this long without motion
SLOW_WITHIN_M = 1.5  # speed scales down inside this distance to the goal
TURN_FULL_AT_DEG = 45.0  # turn rate scales down inside this bearing error
DEADMAN_MIN_S = (
    1.0  # cmd_vel zeroes when no decision arrived for max(this, DEADMAN_PERIODS / max_hz)
)
DEADMAN_PERIODS = 2.5
TASK = (
    "You are a mobile ground robot indoors. Everything in this JSON is relative to you. "
    "`goal`: what to do. `robot`: your last motion and picks; `recent`: what "
    "you did over the last 8 seconds (`pattern` stuck: driving without moving). `objects`: first the target named in `goal` "
    "(`target: true`), then the nearest floor-level obstacles, each with `bearing` (8-way word; "
    "ahead means within 15 degrees), `distance_m` to its nearest edge and `width_m`; the target "
    "also has `bearing_deg` (positive is left) and a `distance` word. `way_to_target`: whether "
    "the straight line to the target is free (`state` clear / blocked). Clear: `room` is the "
    "room along that line, `narrowed_on` the side from which something beside the line narrows "
    "it. Blocked: `blocked_by` names what stands on it (obstacle: only the depth scan sees it), "
    "`blocked_at_m` how far away, "
    "and `open_sides` lists, left and right of that line, the nearest way past: kind doorway "
    "is an opening in a wall with a free straight line to it (`range_m`, `width_m`, "
    "`target_beyond`: the target is on the other side of that wall); kind open / corner is a "
    "direction with free length `clear_m` (kind free: nothing is open, only the longest). Each has its own `bearing` and `detour_deg` away "
    "from the line, agrees with what the depth scan sees or saw lately, and has `been_there` true when you have "
    "already driven where it leads; `going_around` is the side your own picks began steering to. "
    "`free_space`: the nearest obstacle in each direction (`clear_m`, `state` clear / tight / "
    "blocked, `by` what it is). You cannot drive through what blocks the line: while blocked, "
    "steer by the `bearing` of one of `open_sides` instead of the target's. The task is "
    "finished when the target's `distance` is touching, or near with the robot stopped as "
    "close as it can get and no wall on the line to it: then report finished."
)
