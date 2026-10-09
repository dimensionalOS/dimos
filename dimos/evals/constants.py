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
RAW_MAX_EE_LINEAR_MPS = 0.1
RAW_MAX_EE_ANGULAR_RPS = 0.5
RAW_DRIVE_HZ = 10.0
RAW_JPEG_QUALITY = 90

RAW_README = """\
Robot interface: a Zenoh peer at {endpoint}. Connect to it directly (multicast scouting is off);
any Zenoh client works, e.g. `pip install eclipse-zenoh`.

  robot/camera/jpeg        JPEG bytes per frame; attachment is JSON {{"t": unix_seconds}}
  robot/lidar/xyz_f32      float32 little-endian (N,3) x y z in metres, lidar frame; attachment {{"t": ...}}
  robot/odom/json          {{"t","x","y","z","qx","qy","qz","qw"}}: base_link pose in the odom frame
  robot/camera_info/json   {{"width","height","K"}}: intrinsics, republished periodically
  robot/cmd_vel/json       publish {{"vx": m/s, "vy": m/s, "wz": rad/s, "t": seconds}}; the robot holds
                           that velocity for t seconds (max {max_cmd_s:g}), then stops. Republish to keep moving.
                           Speeds are clamped to {max_linear:g} m/s and {max_angular:g} rad/s; non-finite values are ignored.

There is no other interface to this robot.
"""

# The xArm7 gripper's geometry, stated the same way to every agent that drives it.
XARM7_GRIPPER_NOTES = (
    "Gripper: the TCP is 0.172 m along the gripper axis from its root; the finger pads sit "
    "11-48 mm behind the TCP. The jaw gap runs from about 1.6 mm (closed) to 88.9 mm (open)."
)

RAW_ARM_README = """\
Robot interface: a robot arm with a parallel gripper and a wrist RGB-D camera, as a Zenoh peer
at {endpoint}. Connect to it directly in peer mode with multicast and gossip scouting off,
e.g. `uv venv .clientenv && uv pip install --python .clientenv/bin/python eclipse-zenoh numpy pillow`.
Close the session before your script exits. Subscribe before commanding.

Binary payloads carry an attachment {{"t": unix_seconds}}.
  robot/arm/state/json     {{"t", "joint_names", "positions" (rad), "velocities" (rad/s),
                           "ee_pose": {{"frame", "xyz", "quaternion_xyzw"}} or null,
                           "gripper_opening" (0 closed .. 1 open)}}, 20 Hz.
                           ee_pose is the measured tool centre point (TCP) in world.
  robot/camera/jpeg        wrist RGB, 15 Hz
  robot/camera/depth_f32   wrist depth aligned to the RGB: float32 little-endian
                           (height, width), optical-axis Z in metres
  robot/camera/depth_info/json  {{"t", "width", "height", "dtype", "unit", "frame_id"}}
  robot/camera_info/json   {{"width", "height", "K"}}
  robot/camera_pose/json   {{"t", "frame", "xyz", "quaternion_xyzw"}}: wrist optical pose
                           in world; optical +Z forward, +X image right, +Y image down
  robot/arm/twist/json     publish {{"vx", "vy", "vz" (m/s), "wx", "wy", "wz" (rad/s), "t" (s)}}:
                           TCP velocity about fixed world axes, held for t seconds
                           (max {max_cmd_s:g}) and then stopped. A new twist replaces the previous
                           one; a zero twist stops. Components clamp to {max_ee_linear:g} m/s and
                           {max_ee_angular:g} rad/s; omitted fields are zero.
  robot/arm/gripper/json   publish {{"opening": 0..1}}, 0 closed and 1 open; persists until changed.
Commands get no acknowledgement. Twists are clamped to the limits above; a gripper opening
outside 0..1 or a malformed packet is dropped.

Motion is local IK tracking without collision checking. Distance is velocity x time and
only approximate, so check ee_pose and the cameras after every move. Depth pixel (u, v)
back-projects as z = depth[v, u], x = (u - cx) z / fx, y = (v - cy) z / fy, then camera_pose
takes it to world. A gripper blocked by an object holds its target; judge a grasp from
object motion.

There is no other interface to this robot.
"""
