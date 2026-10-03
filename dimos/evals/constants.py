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

RAW_MANIPULATION_README = """\
Robot interface: a Zenoh peer at {endpoint}. Use a plain Zenoh client, for example
`pip install eclipse-zenoh`. Disable multicast and gossip scouting, and connect
directly to this endpoint. Explicitly close the session before your script exits.

Python connection setup (use the same Python environment for installing and running):
If pip is unavailable, create a client environment with:
  uv venv .clientenv
  uv pip install --python .clientenv/bin/python eclipse-zenoh numpy pillow
Then run your scripts with .clientenv/bin/python.

  import json, zenoh
  config = zenoh.Config()
  config.insert_json5("mode", '"peer"')
  config.insert_json5("connect/endpoints", json.dumps(["{endpoint}"]))
  config.insert_json5("scouting/multicast/enabled", "false")
  config.insert_json5("scouting/gossip/enabled", "false")
  session = zenoh.open(config)

Subscribe before commanding. Topics:
  robot/camera/jpeg        JPEG wrist-camera frames; attachment {{"t": unix_seconds}}
  robot/camera/depth_f32   Float32 little-endian (height,width), row-major optical Z
                           depth in metres; attachment {{"t": unix_seconds}}
  robot/camera/depth_info/json  {{"t", "width", "height", "dtype":"<f4", "unit":"metres", "frame_id"}}
  robot/camera_info/json   {{"width", "height", "K"}}: camera intrinsics
  robot/camera_pose/json   {{"t", "frame":"world", "xyz", "quaternion_xyzw"}}:
                           camera optical pose; +Z forward, +X right, +Y down
  robot/overview/jpeg      RGB-only fixed env_camera overview; attachment {{"t": unix_seconds}}
  robot/overview/camera_info/json  {{"width", "height", "K"}}: overview's own intrinsics
  robot/overview/camera_pose/json  {{"t", "frame":"world", "xyz", "quaternion_xyzw"}}:
                           overview optical pose in the world frame
  robot/arm/info/json      static info at 1 Hz: commands, twist_frame, max_linear_mps,
                           max_angular_rps, max_cmd_s, gripper units
  robot/arm/state/json     {{"t", "joint_names", "positions", "velocities", "gripper_opening"}}
                           measured arm joints (radians, rad/s) and gripper opening 0..1

There is no end-effector pose topic: infer progress from joints and the cameras.
If robot context files are listed, read robot/README.md and robot/robot_info.json
once for URDFs, joint/frame conventions and gripper geometry.
Keep the latest sensor message per topic rather than printing every frame.
Use compact state snapshots to decide whether the robot reached your intended target.
Save JPEG/depth bytes to files instead of printing binary payloads or entire arrays.
Use a depth neighborhood or compact numeric summary for the region being inspected.

The wrist camera provides aligned RGB and depth: use camera_info K and camera_pose
for that pair. The fixed overview provides RGB only, with DIFFERENT intrinsics and
pose on its own overview topics. Use it to see the whole workspace and check the
object after a grasp/lift, especially when the wrist view is blocked by the gripper.
Never index wrist depth using an overview image pixel. Match image/depth attachments
and pose by timestamp within each camera; topics arrive separately and the two
cameras may run at different frame rates (wrist 15 Hz, overview 5 Hz by default).
Decode wrist depth with NumPy:
  depth = np.frombuffer(payload, dtype="<f4").reshape(height, width)
Depth is optical-axis Z, not distance along the pixel ray. For pixel (u,v), let
z=depth[v,u], x=(u-cx)*z/fx, y=(v-cy)*z/fy, using K's fx,fy,cx,cy. Transform
[x,y,z] into the robot base frame with camera_pose's quaternion and translation.
Ignore non-finite/nonpositive depths; distant background can have very large
depths. Depth contains the visible robot/gripper as well as scene surfaces.
Save the numeric array as .npy if needed; JPEG or a colorized preview loses depth.

Publish JSON to robot/arm/command/json. Commands are fire-and-forget:
  {{"kind":"twist", "linear":[0,0,0.05], "angular":[0,0,0], "t":1.0}}
  {{"kind":"gripper", "opening":1.0}}

twist moves the end effector (tool centre point) at the given velocity: linear in m/s,
angular in rad/s about fixed world X/Y/Z axes, both expressed in the world frame
(the robot base is unrotated in this scene). The robot holds that velocity for t seconds
(max {max_cmd_s:g}), then stops. Republish to keep moving; a new twist replaces the
previous one, and a zero twist (or t=0) stops the arm. Components are clamped to
{max_ee_linear:g} m/s and {max_ee_angular:g} rad/s. Omitted linear/angular mean zero.
Motion is local IK tracking, not obstacle-aware planning: move in small steps and check.

gripper opening is normalized: 0.0 closed, 1.0 fully open. It is independent of the arm
and persists until changed. A gripper blocked by an object holds its target; inspect
measured gripper_opening and object motion rather than treating closure as grasp success.

There are no command IDs, acknowledgements or execution-status messages. Publishing
does not mean the motion finished. Observe measured robot state and camera images.
Invalid or non-finite commands are dropped.

Only these robot observations and commands are available. The full internal TF
tree, object ground-truth poses, and higher-level manipulation tools are not exposed.
"""
