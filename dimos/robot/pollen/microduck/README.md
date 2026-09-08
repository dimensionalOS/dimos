# Microduck simulation

[Microduck](https://github.com/pollen-robotics/microduck) — pollen-robotics's
~25 cm, ~800 g open-source biped — walking in a small MuJoCo room with the
standard dimOS navigation stack and agent on top of its pretrained RL gait.

```
humancli -> /human_input -> McpClient (LLM) -> skills
    begin_exploration / go_to_object / move_to / ...
        -> WavefrontFrontierExplorer / ReplanningAStarPlanner
            -> MovementManager -> cmd_vel
                -> alpha_walking.onnx (50 Hz) -> MuJoCo (200 Hz)
```

## Quick start

```bash
# Simulation + nav only (drive it from `dimos shell` / planner RPCs):
dimos --viewer none run microduck-sim

# With the agent, using a local LLM via ollama (pulls qwen3:8b on first use):
dimos --viewer none run microduck-agentic-sim-ollama
# ...or with OPENAI_API_KEY set:
dimos --viewer none run microduck-agentic-sim

# In a second terminal:
humancli
> explore the room for 30 seconds, then walk to the red ball
```

On first start the robot model/meshes and walking policy (~26 MB) are
downloaded from the public pollen-robotics GitHub repos into
`~/.cache/dimos/microduck` (see `assets_fetch.py`; `DIMOS_MICRODUCK_ASSETS`
overrides the location, and the fetch is pinned to upstream commits).

If module startup hangs with `RPC call ... timed out` on macOS with
Tailscale (or other VPNs that own the multicast route), zenoh's
loopback-only peer discovery is broken on your machine; run everything with
`--zenoh-scouting` (and `ZENOH_SCOUTING=true humancli`), or put
`zenoh_scouting=true` in your `.env`.

## Web cockpit (`microduck-cockpit-sim`)

A four-room flat (kitchen = "space A", living = "space B", bedroom =
"space C", office = "space D", around an open hub) driven from the browser:
WASDQE teleop, one button per RL policy, a teleop/agent switch, the agent's
humancli transcript, and click-to-goal on the costmap through the standard
`ReplanningAStarPlanner`.

```bash
# OPENAI_API_KEY must be in the environment (the ollama variant needs no key):
dimos --viewer none run microduck-cockpit-sim --local-relay
dimos --viewer none run microduck-cockpit-sim-ollama --local-relay
# then open http://127.0.0.1:7780 (localhost only; the URL is also logged)

MICRODUCK_VARIANT=rollers dimos --viewer none run microduck-cockpit-sim --local-relay
```

Stop any other microduck blueprint first - they share topic names on the
zenoh router. On macOS with Tailscale, add `--zenoh-scouting` as above.

Panels:

- **Control strip** - `Teleop` / `Agent` mode, one button per policy
  (`walk`, `stand`, `roller*`, `sitstand`, `kick_left/right`, `roulade`,
  `ground_pick`; greyed with a reason when the variant or the asset lacks
  it), the nav state chip and its cancel (✕).
- **Nav map** - costmap, rooms, landmark objects, remembered places, the
  planner's path and goal. Click anywhere to send a goal, click a room label
  to go to that room's target, `Esc` (map focused) or ✕ to cancel.
- **Teleop (WASDQE)** - W/S forward-back, A/D strafe, Q/E yaw; only acts in
  teleop mode, and only while the tab is in the foreground (browsers
  throttle background tabs; the deadman then zeroes the twist).
- **Chat** - the agent transcript (tool calls and results included) with an
  input box, live in agent mode. Try `go to space A`, `go to the kitchen`,
  `walk to the blue box`, `remember this place as fridge`, `what do you
  see?`, `do a roulade`.
- **Chase cam / Head cam** - decoded only while the tab is visible.

In teleop mode the planner's `nav_cmd_vel` is muted (a click still plans
and draws the path); switching to agent mode hands `cmd_vel` to the planner
and enables the chat. Navigation is locked (and any active goal cancelled)
while a one-shot policy runs, while the duck is seated, fallen or standing
back up; the chip shows the reason.

## What's in the box

- `sim_module.py` — `MicroduckSimModule(MujocoSimModule)`: composes the room
  scene + robot MJCF (adding three trunk-mounted raycast-lidar cameras),
  runs the ONNX walking policy in the engine's step hook at 50 Hz, and maps
  `cmd_vel` twists into the policy's command space. No ControlCoordinator:
  the whole robot is one module. Odom/tf/IMU/pointcloud publishing is
  inherited from `MujocoSimModule`.
- `gait.py` — the 61-dim observation contract of the alpha policies
  (documented in microduck's `duck-control/src/obs.rs`), with joint order,
  home pose and action scale read from the ONNX metadata.
- `skills.py` — `MicroduckSkillContainer`: `go_to_object` / `list_objects` /
  `move_to` / `where_am_i` / `stop_moving` / `wait`. Object positions are the
  scene's ground truth (configured in the blueprint), not perception — this
  is the deliberately-basic demo.
- `assets/room_scene.xml` — a 4 x 3 m walled room with four colored objects.
  World geometry is geom group 0; the lidar only raycasts group 0, so the
  robot (groups 2/3) never sees itself.
- `web_codecs.py` — the cockpit's own `@web_encoder`s (the transcript, the
  planner path, the JSON-string state streams). The blueprint declares the
  matching streams as `Channel(...)` and `cockpit()` generates a relay-bridge
  subclass with a typed port for each, so none of this robot's vocabulary -
  or its langchain dependency - lands in `dimos.web`.
- `blueprints/` — `microduck-sim` (sim + nav + explorer),
  `microduck-agentic-sim[-ollama]` (adds McpServer/McpClient + skills) and
  `microduck-cockpit-sim[-ollama]` (the web cockpit above).

## Quirks worth knowing

- **Command shaping** (`sim_module.py`): the policy tracks its velocity
  command with a ~2.5x undershoot, so requested twists are multiplied up
  (`cmd_gain_linear/angular`), and it has a yaw deadband — pure-turn
  commands below the top of its range barely rotate it (~3-9 deg/s at
  1.0 rad/s vs 25-31 deg/s at 1.5) — so turn commands are bumped to
  `min_effective_wz` (1.5, the range maximum).
- **Falls**: only the plain walking policy is published (no fall recovery),
  and the walk-optimized model has no trunk collision geoms. When the trunk
  stays tilted > ~55 degrees for 2 s the module stands the duck back up,
  nudged toward the room origin so it doesn't re-spawn wedged inside
  whatever it tripped over.
- **Rendered cameras are Linux-only**: `mujoco.Renderer` needs a GL context
  that macOS only allows on the main thread; creating one from the engine's
  sim thread deadlocks the worker. The raycast lidar is pure `mj_ray` and
  works everywhere; the blueprint enables `color_image` only off-macOS.
- The planner floors commands at 0.2 m/s; the duck actually walks ~0.1 m/s,
  so room crossings take a minute or two. That's the robot, not a bug.

## Licensing

Code in the upstream microduck repos is Apache-2.0; the 3D model files are
CC BY-NC-SA — they are downloaded to a local cache, not redistributed here.
