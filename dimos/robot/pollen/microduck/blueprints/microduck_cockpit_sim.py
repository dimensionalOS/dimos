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

"""Microduck cockpit: a four-room MuJoCo apartment driven from the browser.

The duck walks a compact 4 x 4 m flat with four rooms (kitchen = "space A",
living = "space B", bedroom = "space C", office = "space D") around a
central hub. The web cockpit shows the chase and head cameras, the costmap
with named places, WASDQE teleop, one button per RL policy (walk, kicks,
roulade, sit/stand, ...) and - in agent mode - the chat with the LLM that
drives the duck through the places memory and the nav stack:

    browser --tele_cmd_vel/goal_request/ui_command/human_input--> relay bridge
        -> DuckControlModule (teleop | agent arbitration, nav <-> policies)
        -> MicroduckSimModule (policies in the loop) / ReplanningAStarPlanner
        -> McpClient (LLM) -> McpServer -> MicroduckSkillContainer skills

Usage:
    dimos run microduck-cockpit-sim --local-relay      # needs OPENAI_API_KEY
    dimos run microduck-cockpit-sim-ollama --local-relay
    # then open the cockpit URL the relay prints (http://127.0.0.1:7780)

``MICRODUCK_VARIANT=rollers`` selects the wheeled-feet robot and its policies.
"""

from __future__ import annotations

import os

from langchain_core.messages.base import BaseMessage

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.ollama_agent import ollama_installed
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner

# Imported for the registration side effect: the cockpit blueprint resolves
# these encoders by id when it compiles MICRODUCK_COCKPIT_CHANNELS.
from dimos.robot.pollen.microduck import web_codecs  # noqa: F401
from dimos.robot.pollen.microduck.config import MICRODUCK
from dimos.robot.pollen.microduck.control_module import DuckControlModule
from dimos.robot.pollen.microduck.places import (
    FOUR_ROOM_XML,
    MICRODUCK_OBJECTS,
    MICRODUCK_ROOMS,
)
from dimos.robot.pollen.microduck.sim_module import (
    LIDAR_CAMERA_SPECS,
    POV_CAMERA_NAME,
    MicroduckSimModule,
)
from dimos.robot.pollen.microduck.skills import MicroduckSkillContainer
from dimos.web.cockpit import (
    Channel,
    Chat,
    Col,
    Control,
    NavMap,
    Row,
    Teleop,
    Video,
    cockpit,
)

# Robot model + policy set; see policies.py for the two variants.
_VARIANT = os.environ.get("MICRODUCK_VARIANT", "default")

# The gait covers ~0.06 m/s, so the planner's default stuck detector (< 0.4 m
# in 8 s) fires on every leg and the replan limiter gives up mid-room. Over
# 10 s a walking duck moves ~0.6 m (>= 0.3 m from the window's centroid);
# a real stall stays under 0.15 m.
_DUCK_STUCK_TIME_WINDOW = 10.0
_DUCK_STUCK_THRESHOLD = 0.15

# The walking policy is trained for |vx| <= 0.3 m/s (after the sim module's
# command gain); faster teleop requests only saturate.
_TELEOP_MAX_LINEAR = 0.15
_TELEOP_MAX_ANGULAR = 0.6

# Camera rates: what the sim renders at, and the cockpit's cap on top.
#
# The cap must sit ABOVE the render rate, never on it. The bridge's rate gate
# drops any frame arriving less than 1/max_hz after the last, so a cap equal
# to the source aliases against publisher jitter and silently loses ~30% of
# frames (measured: 11.5 Hz in, 8.4 fps out under a 12 Hz cap).
#
# Two budgets bound these numbers, and they respond to different knobs.
#
# RENDER (sim thread). Cameras render inline in the sim loop, so every
# millisecond spent rendering is one physics does not advance. Cost here is
# GEOMETRY, not pixels: the duck's own 215k-vertex body dominates, so a
# 64x48 render measures ~27 ms against ~29 ms for 640x360. Only the frame
# RATE moves this. At these rates the real-time factor stays 1.00 and the
# gait, policies and nav all pass end to end.
#
# DELIVERY (relay -> browser). This is the one that produces "stale", and it
# is BYTES, not stream count. Measured against the live cockpit while the
# duck walked, watching the relay's own counters: the sim never hitches (bus
# gaps p50 56 ms, worst 79 ms) and the robot->relay leg never hitches, but
# relay->browser freezes for seconds at a time, ALL CHANNELS AT ONCE - the
# signature of connection-level flow control rather than a per-stream
# problem. Halving JPEG quality, which leaves the stream count untouched,
# cut the freezes from 2.1-4.1 s to 0.8-1.3 s; shrinking the inset (~7% of
# the bytes) barely moved them. So quality and rate on the CHASE camera are
# the levers that matter - it is ~90% of the byte budget - and 40 is the
# highest quality measured to keep freezes under the 2 s that trips the
# panel badge.
#
# This is a workaround, not a fix. The freeze is the relay blocking in
# createUnidirectionalStream (web/relay/session.ts) with waitUntilAvailable
# and no timeout, which stops every channel rather than dropping a frame on
# a latest-wins video feed. Fixing that upstream would let all of these
# numbers go back up.
_CHASE_CAM_SIZE = (640, 360)
_CHASE_CAM_FPS = 30.0
_CHASE_CAM_MAX_HZ = 40.0
_CHASE_CAM_QUALITY = 40
# The head camera only fills a ~260 px inset, so it ships at a quarter of the
# pixels. That costs nothing in render time (geometry-bound, see above) but a
# lot in BYTES, which is the constraint that actually bites - see the
# flow-control note below. It stays the frame `observe` reads; 320x180 is
# ample for describing a room.
_HEAD_CAM_SIZE = (320, 180)
_HEAD_CAM_FPS = 6
_HEAD_CAM_MAX_HZ = 12.0
_HEAD_CAM_QUALITY = 40

MICRODUCK_COCKPIT_SYSTEM_PROMPT = """\
You are the brain of Microduck, a tiny (25 cm tall) two-legged duck robot
living in a small simulated flat. You walk slowly (about 0.1 m/s), so
crossing the flat takes a minute or two - that is normal, and navigation
tools block until the duck arrives or gives up.

The flat has four rooms around a central hub; each room also goes by a
"space" letter:
- kitchen  = space A  (red_box, orange_crate)
- living   = space B  (blue_box)
- bedroom  = space C  (green_cylinder)
- office   = space D  (yellow_pillar)

Tools:
- list_places / where_am_i to orient yourself (rooms, objects, remembered spots)
- go_to_room(name) for a room or space letter, go_to_object(name) for a landmark,
  go_to_place(name) for anything by name (rooms, objects, remembered spots)
- move_to(x, y) for raw coordinates, stop_moving to halt, wait(seconds)
- remember_place(name) to save where you stand for later
- list_policies / perform(name) for tricks (kicks, roulade, ...), sit / stand_up
- observe to look through the head camera

Call movement tools strictly ONE AT A TIME and wait for each result before
the next; never combine go_to_*, move_to or perform in the same step. When
the human names a room, space letter or object, prefer go_to_room /
go_to_object over raw coordinates. Sit only when asked; the duck cannot walk
while seated, so stand_up first.

Keep answers short and playful - you are a duck. Report what you did once
actions finish.
"""

# The control strip sizes to its content; the row below takes the rest.
MICRODUCK_COCKPIT_LAYOUT = Col(
    Control(),
    Row(
        # The chase camera is the main view - you need to see the duck to
        # drive it - with the duck's own view inset in the corner, so both
        # are visible at once instead of trading places. Rate/quality are
        # pinned here and mirrored by the chase_image Channel below (a panel
        # and a channel for one stream must agree on everything but max_hz);
        # color_image is a built-in bridge port, so it needs no Channel.
        Video(
            "chase_image",
            title="Chase cam (duck view inset)",
            max_hz=_CHASE_CAM_MAX_HZ,
            quality=_CHASE_CAM_QUALITY,
            inset="color_image",
            inset_max_hz=_HEAD_CAM_MAX_HZ,
            inset_quality=_HEAD_CAM_QUALITY,
        ),
        Col(
            NavMap(),
            Teleop(
                max_linear=_TELEOP_MAX_LINEAR,
                max_angular=_TELEOP_MAX_ANGULAR,
                # DuckControl drops teleop twists in agent mode; naming
                # the stream lets the pad say so instead of arming.
                mode="mode",
                title="Teleop (WASDQE)",
            ),
            shares=[3, 2],
        ),
        Chat(),
        # The transcript wraps tool calls and their results, so the agent
        # column earns more width than the map/teleop stack beside it.
        shares=[5, 3, 4],
    ),
)

# The streams the panels above need that no robot bridge has a built-in port
# for. Declaring them here (rather than in the bridge) is what keeps duck
# vocabulary out of dimos.web: cockpit() generates a typed port per entry and
# resolves the encoder from web_codecs by id.
#
# All of them resend on subscribe - a cockpit opened mid-run must show the
# duck's current mode, places and nav state instead of waiting for the next
# publish - and the event-shaped ones skip the rate gate, because a missed
# sample there is lost data, not a dropped frame.
MICRODUCK_COCKPIT_CHANNELS = (
    Channel(
        "chase_image",
        Image,
        encoding="jpeg.v1",
        delivery="latest",
        max_hz=_CHASE_CAM_MAX_HZ,
        params={"quality": _CHASE_CAM_QUALITY},
        resend_on_subscribe=True,
    ),
    Channel(
        "agent",
        BaseMessage,
        encoding="chat.json.v1",
        max_hz=30.0,
        resend_on_subscribe=True,
        rate_gate=False,
        # A reload must not lose the conversation so far.
        replay_depth=200,
    ),
    Channel(
        "agent_idle",
        bool,
        encoding="flag.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "path",
        NavPath,
        encoding="path.json.v1",
        delivery="latest",
        max_hz=5.0,
        resend_on_subscribe=True,
    ),
    Channel(
        "nav_state",
        str,
        encoding="navstate.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "mode",
        str,
        encoding="mode.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "places",
        str,
        encoding="places.json.v1",
        max_hz=2.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "policy_state",
        str,
        encoding="policy.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
)


def _stack(mcp_client_kwargs: dict[str, object]):  # type: ignore[no-untyped-def]
    return autoconnect(
        MicroduckSimModule.blueprint(
            scene_xml=FOUR_ROOM_XML,
            headless=True,
            spawn_xy=(0.0, 0.0),
            variant=_VARIANT,
            # First-person camera: the cockpit's primary view and the frame
            # the observe skill reads. NOT the MJCF's stock `head_camera`,
            # which is mounted backwards and renders the duck's own jaw (see
            # POV_CAMERA_NAME in sim_module.py).
            camera_name=POV_CAMERA_NAME,
            width=_HEAD_CAM_SIZE[0],
            height=_HEAD_CAM_SIZE[1],
            fps=_HEAD_CAM_FPS,
            enable_color=True,
            enable_depth=False,
            # Raycast lidar -> world-frame pointcloud for mapping. World
            # geometry is group 0; the robot and the ball are not.
            enable_pointcloud=True,
            pointcloud_fps=3.0,
            enable_mujoco_lidar=True,
            mujoco_lidar_camera_names=[name for name, _ in LIDAR_CAMERA_SPECS],
            mujoco_lidar_geom_groups=[0],
            mujoco_lidar_raycast_width=96,
            mujoco_lidar_raycast_height=48,
            mujoco_lidar_min_range=0.05,
            mujoco_lidar_max_range=6.0,
            mujoco_lidar_max_height=0.6,
            mujoco_lidar_voxel_size=0.03,
            mujoco_lidar_robot_exclusion_radius=0.2,
            chase_cam=True,
            chase_cam_fps=_CHASE_CAM_FPS,
            chase_cam_size=_CHASE_CAM_SIZE,
        ),
        # CPU voxel grid: the flat is tiny, and this also runs on macOS.
        VoxelGridMapper.blueprint(emit_every=1, voxel_size=0.03, device="CPU:0"),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=0.03,
                can_pass_under=MICRODUCK.height_clearance + 0.05,
                can_climb=0.03,
            ),
            initial_safe_radius_meters=MICRODUCK.width_clearance + 0.15,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=MICRODUCK.width_clearance,
            robot_rotation_diameter=MICRODUCK.rotation_diameter,
            stuck_time_window=_DUCK_STUCK_TIME_WINDOW,
            stuck_threshold=_DUCK_STUCK_THRESHOLD,
        ),
        # Teleop/agent arbitration; replaces MovementManager (nav_cmd_vel
        # only reaches cmd_vel in agent mode).
        DuckControlModule.blueprint(),
        MicroduckSkillContainer.blueprint(rooms=MICRODUCK_ROOMS, objects=MICRODUCK_OBJECTS),
        ObserveSkill.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(system_prompt=MICRODUCK_COCKPIT_SYSTEM_PROMPT, **mcp_client_kwargs),
        cockpit(layout=MICRODUCK_COCKPIT_LAYOUT, channels=MICRODUCK_COCKPIT_CHANNELS),
    ).remappings([(VoxelGridMapper, "lidar", "pointcloud")])


# The last call in each assignment below has to be a builder method: the
# blueprint registry is generated by a static AST scan
# (test_all_blueprints_generation) that only recognises `autoconnect(...)` by
# name or an expression ending in one of these builders. A bare `_stack({})`
# is invisible to it, and `dimos run microduck-cockpit-sim` then fails with
# "Unknown blueprint" while the ollama variant - which ends in
# .requirements() - resolves fine.
#
# nerf_speed halves the planner's 0.55 m/s default; the sim module's command
# gain maps the rest into the gait's trained velocity range.
microduck_cockpit_sim = _stack({}).global_config(robot_model="microduck", nerf_speed=0.5)

microduck_cockpit_sim_ollama = (
    _stack({"model": "ollama:qwen3:8b"})
    .global_config(robot_model="microduck", nerf_speed=0.5)
    .requirements(ollama_installed)
)
