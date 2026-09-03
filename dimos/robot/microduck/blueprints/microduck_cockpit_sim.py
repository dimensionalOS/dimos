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
from dimos.robot.microduck import web_codecs  # noqa: F401
from dimos.robot.microduck.control_module import DuckControlModule
from dimos.robot.microduck.places import (
    FOUR_ROOM_XML,
    MICRODUCK_OBJECTS,
    MICRODUCK_ROOMS,
)
from dimos.robot.microduck.sim_module import LIDAR_CAMERA_SPECS, MicroduckSimModule
from dimos.robot.microduck.skills import MicroduckSkillContainer
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

# The duck is ~25 cm tall and ~12 cm wide; nav scales follow (as in
# microduck_sim.py, which stays untouched for the exploration demo).
_DUCK_WIDTH = 0.2
_DUCK_HEIGHT = 0.28
_DUCK_ROTATION_DIAMETER = 0.3

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
# Two separate budgets bound these numbers, and the second one bites first.
#
# Render: each 640 x 360 frame costs ~11 ms on the SIM THREAD (see
# MicroduckSimModuleConfig.cast_shadows). At 30 fps the chase cam alone
# spends ~0.33 s of every second rendering. That is affordable here - the
# sim's real-time factor stays 1.00 and the gait, policies and nav all pass
# end to end - but render cost scales with PIXELS, so shrink the frame before
# raising the rate.
#
# Delivery: every `latest` frame costs one relay->viewer QUIC stream, and
# that stream is held until the reaper frees it (web/README.md bug 12). A
# real viewer sustains only ~30 latest-frames per second IN TOTAL, shared
# across every latest channel - measured, and independent of frame size: a
# 500 byte state frame costs the same as a 15 kB JPEG. That is why the state
# channels above are `reliable` (one persistent stream, no per-frame cost)
# and why the head cam is deliberately slower than the chase cam: the big
# panel should get the budget. Raising these past the ceiling does not add
# frames, it just splits the same ~30 fps more ways.
#
# Frame SIZE is the weaker lever of the two and it is not free: the cockpit's
# video canvas scales down to its panel but never up (VideoPanel.module.css
# `max-width: 100%`), so a smaller frame just leaves dead space around a
# smaller picture. 480 x 270 measured only ~7% more delivered fps than
# 640 x 360 and looked worse, so the chase cam keeps its size.
_CHASE_CAM_SIZE = (640, 360)
_CHASE_CAM_FPS = 30.0
_CHASE_CAM_MAX_HZ = 40.0
_CHASE_CAM_QUALITY = 70
_HEAD_CAM_FPS = 6
_HEAD_CAM_MAX_HZ = 12.0

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
        # Rate/quality are pinned here and mirrored by the chase_image
        # Channel below (a panel and a channel for one stream must agree on
        # everything but max_hz).
        Video(
            "chase_image",
            title="Chase cam",
            max_hz=_CHASE_CAM_MAX_HZ,
            quality=_CHASE_CAM_QUALITY,
        ),
        Col(
            NavMap(),
            Row(
                Video("color_image", title="Head cam", max_hz=_HEAD_CAM_MAX_HZ),
                Teleop(
                    max_linear=_TELEOP_MAX_LINEAR,
                    max_angular=_TELEOP_MAX_ANGULAR,
                    # DuckControl drops teleop twists in agent mode; naming
                    # the stream lets the pad say so instead of arming.
                    mode="mode",
                    title="Teleop (WASDQE)",
                ),
                shares=[1, 1],
            ),
            shares=[3, 2],
        ),
        Chat(),
        shares=[5, 4, 3],
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
            # Head camera for the cockpit and the observe skill.
            width=320,
            height=240,
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
                can_pass_under=_DUCK_HEIGHT + 0.05,
                can_climb=0.03,
            ),
            initial_safe_radius_meters=_DUCK_WIDTH + 0.15,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=_DUCK_WIDTH,
            robot_rotation_diameter=_DUCK_ROTATION_DIAMETER,
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
