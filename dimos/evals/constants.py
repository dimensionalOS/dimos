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

"""Tunables for the eval harness. Literals only; nothing here imports dimos."""

# Model providers: API key variable and default upstream base URL, by provider name.
PROVIDERS: dict[str, tuple[str, str]] = {
    "openai": ("OPENAI_API_KEY", "https://api.openai.com/v1"),
    "anthropic": ("ANTHROPIC_API_KEY", "https://api.anthropic.com"),
}

# Host variables a Pi process may inherit; the provider key is added by name.
PASSTHROUGH_ENV: tuple[str, ...] = (
    "PATH", "HOME", "LANG", "LC_ALL", "LC_CTYPE", "TERM", "TMPDIR",
    "XDG_CONFIG_HOME", "XDG_STATE_HOME", "XDG_CACHE_HOME", "SSL_CERT_FILE", "SSL_CERT_DIR",
)  # fmt: skip

# Upper bound on one bash tool call, seconds.
MAX_TOOL_SECONDS = 300.0

# Trace proxy: provider responses retried before Pi sees them.
RETRY_STATUSES = frozenset({429, 500, 502, 503, 529})
RETRIES = 3
RETRY_BACKOFF_S = 2.0

# Keyword guard: how a denied tool call reports itself, and the no_dimos defaults.
DENIED = "Tool call denied"
NO_DIMOS_KEYWORDS = ("dimos", "dimensionalos")
NO_DIMOS_GUIDANCE = (
    "There is no robotics framework installed. You may use any public tool, library, SDK or "
    "web resource, and write whatever code you need under the current directory. Do not "
    "install, clone, import or run dimOS or anything from the dimensionalOS organisation; "
    "tool calls that mention it are denied."
)

# Raw robot bridge: default endpoint (an attached dimos runs its bridge here) and limits.
RAW_ENDPOINT = "tcp/127.0.0.1:7448"
RAW_TOPIC_PREFIX = "robot"
RAW_MAX_CMD_S = 2.0
RAW_MAX_LINEAR_MPS = 1.0
RAW_MAX_ANGULAR_RPS = 1.5
RAW_DRIVE_HZ = 10.0
RAW_JPEG_QUALITY = 90
RAW_WORLD_STATE_HZ = 2.0

RAW_TOPICS: tuple[str, ...] = (
    "camera",
    "lidar",
    "odom",
    "camera_info",
    "world_state",
    "cmd_vel",
    "finished",
)
RAW_TOPIC_DOCS: dict[str, str] = {
    "camera": 'robot/camera/jpeg        JPEG bytes per frame; attachment is JSON {{"t": unix_seconds}}',
    "lidar": 'robot/lidar/xyz_f32      float32 little-endian (N,3) x y z in metres, lidar frame; attachment {{"t": ...}}',
    "odom": 'robot/odom/json          {{"t","x","y","z","qx","qy","qz","qw"}}: base_link pose in the odom frame',
    "camera_info": 'robot/camera_info/json   {{"width","height","K"}}: intrinsics, republished periodically',
    "world_state": (
        'robot/world_state/json   JSON a few times per second: "robot" (position, yaw_deg, heading), '
        '"objects" (label, position, size,\n'
        '                         distance_m, bearing, bearing_deg; closest first) and "room.sectors" '
        "(nearest obstacle per 45-degree sector\n"
        "                         in the robot frame, state blocked/tight/clear). Text is the only sensor."
    ),
    "cmd_vel": (
        'robot/cmd_vel/json       publish {{"vx": m/s, "vy": m/s, "wz": rad/s, "t": seconds}}; the robot holds\n'
        "                         that velocity for t seconds (max {max_cmd_s:g}), then stops. Republish to keep moving.\n"
        "                         Speeds are clamped to {max_linear:g} m/s and {max_angular:g} rad/s; non-finite values are ignored."
    ),
    "finished": 'robot/finished/json      publish {{"done": true}} once, when you consider the task complete',
}

RAW_README = """\
Robot interface: a Zenoh peer at {endpoint}. Connect to it directly (multicast scouting is off);
any Zenoh client works, e.g. `pip install eclipse-zenoh`.

{topics}

There is no other interface to this robot.
"""

# Navigation benchmark thresholds and the grade weights.
NAV_SUCCESS_RADIUS_M = 1.0  # around the case's end point
NAV_FACING_TOL_DEG = 45.0  # toward the target's centre
NAV_MIN_CMD_MPS = 0.1  # a linear command below this is not an attempt to move
NAV_BUMP_RATIO = 0.2  # achieved / commanded speed below this is contact
NAV_BUMP_MIN_S = 0.5
NAV_CMD_HOLD_S = 2.0  # how long a recorded command is assumed to act
NAV_JITTER_M = 0.005
NAV_TURN_HYSTERESIS_DEG = 5.0
NAV_BUMPS_FOR_ZERO_CREDIT = 5
NAV_WEIGHTS: dict[str, float] = {"reached": 0.5, "facing": 0.2, "straightness": 0.15, "bumps": 0.15}
