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

from dataclasses import asdict, replace
import json
from typing import Literal

from dimos.perception.localize.localize_params import XARM_PARAMS

# Model providers: API key variable and default upstream base URL, by provider name.
PROVIDERS: dict[str, tuple[str, str]] = {
    "openai": ("OPENAI_API_KEY", "https://api.openai.com/v1"),
    "anthropic": ("ANTHROPIC_API_KEY", "https://api.anthropic.com"),
    "google": ("GEMINI_API_KEY", "https://generativelanguage.googleapis.com/v1beta"),
}

# Pi's --thinking level as a Gemini thinkingLevel ("off" leaves the model's default).
GOOGLE_THINKING_LEVELS: dict[str, Literal["LOW", "MEDIUM", "HIGH"] | None] = {
    "off": None,
    "minimal": "LOW",
    "low": "LOW",
    "medium": "MEDIUM",
    "high": "HIGH",
    "xhigh": "HIGH",
    "max": "HIGH",
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

# Localize in MuJoCo: the wrist camera starts parked, so confirm objects from one view, and
# robosuite's flat-shaded objects score about 0.3 with OWLv2, under the xarm preset's 0.40.
SIM_LOCALIZE_PARAMS = replace(XARM_PARAMS, min_views=1, candidate_floor=0.2, accept_score=0.25)
SIM_LOCALIZE_ENV = {"PERCEPTIONLOCALIZE__PARAMS": json.dumps(asdict(SIM_LOCALIZE_PARAMS))}

DIMOS_PYTHON_README = """\
Robot interface: a live dimOS runtime driving the arm, reached from Python. Write scripts and run
them with bash (`python script.py`); each script connects, acts and exits.
Don't name scripts after standard-library modules (inspect.py, test.py, types.py, ...): Python
imports the script's folder first, so they shadow the real module and break `import dimos`.

    from dimos import Dimos
    from dimos.manipulation.sdk import Arm, MotionError

    app = Dimos.connect()
    arm = Arm.from_app(app)
    try:
        ...
    finally:
        app.stop()  # closes this client's connection; the robot keeps running

Arm (help(arm.<method>) shows the full docstring):
  arm.pose()                     measured tool centre point (TCP) in world: .position.x/.y/.z,
                                 .orientation (x, y, z, w)
  arm.joints()                   joint positions, radians
  arm.state().gripper_position   measured gripper opening, 0 closed .. 1 open; fingers that stop
                                 short of closed are holding something
  arm.move_pose([x, y, z], orientation=[qx, qy, qz, qw])
                                 plan and move the TCP to a world pose; omit orientation to keep
                                 the current one. Endpoint only, the path may curve.
  arm.move_linear(dx=0, dy=0, dz=0, check_collision=False)
                                 straight world-frame translation of the TCP, in metres
  arm.move_joints(q)             plan and move to joint positions
  arm.set_gripper_position(p)    0 closed .. 1 open; arm.open_gripper(), arm.close_gripper()
Moves block until the motion ends, so no sleep is needed before looking. They raise MotionError on
failure, including when the arm stops short of the planned end (e.g. blocked by contact); the
message says how far off, error.result says why.
Every arm move accepts speed_scale and timeout. Gripper calls accept timeout only (default 2 s):
they wait until the fingers stop, at the target or on an object. arm.reset() stops any motion and clears a failed
state; a move that fails with "call reset()" needs it before the next one.

Camera (the wrist camera, moving with the gripper; at the start pose it looks straight down:
image right = world -Y, image up = world +X):
  app.peek_stream("color_image", 2.0).save("view.jpg")
                                 the latest RGB frame, saved for the read tool
  app.peek_stream("depth_image", 2.0).as_numpy()
                                 the latest depth frame: float32 metres per pixel, the same
                                 pixels as color_image
  app.peek_stream("camera_info", 2.0).K
                                 the 3x3 intrinsics of both, row-major (fx, 0, cx, 0, fy, cy, 0, 0, 1)
  app.skills.camera_pose()       JSON {"t", "frame", "xyz", "quaternion_xyzw"}: the camera's optical
                                 pose in world at its newest frame; optical +Z forward, +X image
                                 right, +Y image down. Depth pixel (u, v) back-projects as
                                 z = depth[v, u], x = (u - cx) z / fx, y = (v - cy) z / fy, then
                                 this pose takes it to world.

Perception:
  app.skills.localize(objects="red cube, blue cup")
                                 the named objects in the camera's recent frames, as text. Name what
                                 the camera sees, colour + shape from above ("yellow rectangular
                                 object", "blue circle"), not what the object is ("bin", "cap"):
                                 names often score under 0.15. It cannot read text or letters.
                                 Each found object reports:
                                 world position; footprint: length x width (m) of the smallest
                                 rectangle around it seen from above, its centre and the long
                                 side's yaw; the z range of the surfaces the camera saw (the top
                                 is reliable, the bottom only as low as was in view); score,
                                 views and an id that stays the same across calls.
  app.skills.localize(objects="...", params='{"accept_score": 0.2}')
                                 per-call thresholds, a JSON object of LocalizeParams fields:
                                 candidate_floor (detector score to form a box), accept_score
                                 (score to report an object), min_views (camera positions to
                                 confirm it), max_object_extent_m, min_points. A miss reports
                                 its best candidate's score and views; lowering accept_score or
                                 min_views to it accepts that candidate, and lower values also
                                 admit wrong matches. help(LocalizeParams) from
                                 dimos.perception.localize.types lists every field.
  app.skills.state()             whether localize is ready to answer

Grasps:
  app.skills.grasps(object_id="31e1f624")
                                 grasp poses for an object localize reported, by its id: the
                                 best ones IK reaches from the current joints, best first. Each
                                 gives the TCP position and orientation for arm.move_pose, the
                                 approach direction, a pre-grasp pose back along it, the
                                 object's width between the jaws and a score. They point
                                 straight down at the centre of the object's footprint, jaws
                                 across its narrow side, and read only the surfaces the camera
                                 saw; suited to small box-like objects. Collisions are not
                                 checked, and the proposals do not know the task; choose the one
                                 that suits it. localize merges every view of an object into its
                                 cloud; when the surfaces seen span only a few millimetres, the
                                 camera saw just the top: look from another angle and localize
                                 again before asking for grasps.

Other streams: app.list_streams() lists every module's streams, and app.peek_stream(name) reads
any of them except the simulator's ground truth (exact object poses, the scene camera), which
reads as absent. help(<call>) shows any call's full docstring.

Print only what you need: what a script prints is what you see.
"""

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
