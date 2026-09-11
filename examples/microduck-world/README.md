# Microduck World

Source snapshot of the hosted 3v3 football demo, imported from application commit
`5ae1f01`. This folder is an independently installed application inside the DimOS
repository. Run its commands from this folder, not the repository root.

It retains the production-tested framework revision in `dimos-revision.txt`
and the compatibility patch in `patches/`. Its nested `vendor/dimos` checkout
is a dependency, not a second copy committed here. This snapshot does not claim
runtime compatibility with every change on the enclosing PR branch.

A persistent, headless Microduck world on Omarchy. Open
**https://sim.tule.world** and sign in with GitHub to play or watch. The private
operator gateway remains at **https://omarchy.tailca0707.ts.net:8443** on the tailnet.

## Enter the world

All six Microducks are visible together: Red 1 to Red 3 and Blue 1 to Blue 3.
Click an available duck to open its connection dialog.
Both teams spawn along opposite midfield sidelines, just outside the pitch and facing it.
Enter an optional player name in that dialog, then confirm Join. It appears above your duck for other
players and viewers. Use the Names checkbox to hide labels in your own view,
or choose a player from the follow menu. **Follow duck** locks a close camera
behind the duck and turns with its heading. Click and drag to unlock the camera;
click **Follow duck** again to lock it back on. **Watch the match** joins as a
spectator without taking a duck. Availability is live and every duck has one
exclusive controller. Hover or focus a duck card to preview its walking gait;
touch screens have a preview button. This animation runs in the browser.

Every duck has the same full DimOS blueprint: agent conversation, MCP tools,
camera, map, navigation, exploration, policies and WASD controls. All six start
idle knowing the pitch and both locker-room locations. Their maps, observations,
named discoveries and conversations remain private. The four benchmark rooms
are preserved in a side wing reached through the narrow corridor behind the lockers.

**Leave duck** releases the slot. A disconnected owner has 60 seconds to reconnect
to the same session. An available duck is absent from physics until claimed; a new
occupant receives fresh private knowledge. All six follow the same rules. The
legacy `?host` shortcut opens the Red 1 connection dialog.

Public players and spectators authenticate with GitHub. Cloudflare serves the
website and relays its DimOS streams; the shared world and native football
detection camera run on Omarchy. See the
[implementation, validation and launch procedure](docs/public-launch.md).

## Operate the world

```bash
cd ~/projects/microduck-world
./service status
./service restart world
./service logs world
./world status
./world --mcp-port 9990 mcp list-tools
```

Choose the MCP port for the occupied duck: Red 1 to Red 3 use 9990 to 9992;
Blue 1 to Blue 3 use 9993 to 9995. For example,
`./world --mcp-port 9994 mcp list-tools` targets Blue 2.

`./service` accepts install/start/stop/restart/status/logs and an optional
world/gateway target. Both services are enabled for boot and restart after failure.
User lingering is enabled, so an SSH login or monitor is not required.
`./world stop` and `./world restart` also use the supervisor when it owns the world.
Do not launch a second world stack. The single world process owns shared physics;
the supervisor launches each robot blueprint in a separate process tree.

In each duck cockpit, the Control strip exposes the policies available for the selected robot variant.
Select walk for movement; stand deliberately holds position. In Teleop mode,
focus the Teleop panel, then hold WASD after the server acknowledges Teleop mode. Click the map or a room label to
navigate; its cancel button or Escape cancels navigation. Keyboard driving interrupts it.
The main world panel renders locally with Three.js. Drag to orbit, scroll to zoom,
and right-drag to pan. Overview frames the connected world; Follow duck tracks the robot.
Each browser has its own camera. Camera gestures do not command the duck.
The Duck camera also renders locally with Three.js. Both panels show FPS and offer
a Three.js / MuJoCo JPEG selector. The agent still receives its native MuJoCo camera.
The YOLO panel displays that native sensor image with YOLO ball boxes
computed on Omarchy by a shared DimOS perception module. Spectators can select
one occupied duck's public football feed. The image and boxes share a timestamp;
old frames and previous occupants' results are cleared.
The optional world JPEG view uses a shared follow camera capped at 12 FPS; its
server renderer runs only while someone selects it.
Panels can be moved, resized, minimized and maximized. Dark and light themes are available.
Click humancli to request Agent mode; the transcript stays visible in Teleop mode.
See [cockpit behavior and validation](docs/cockpit-ui.md).

The pitch is 2.6 by 1.6 metres. Duck respawn returns to the midfield sideline.
Individual ball buttons drop each ball from two metres above midfield.
The wall scorer board records authenticated last-touch goals in SQLite across restarts;
own goals do not earn personal credit.

The Agent is enabled with the authorized OpenAI credential. Select Agent in the
Control strip, then type in the Agent panel. Try "what can you see?" or
"go to the kitchen". Camera observation, room navigation, posture actions and
cancellation have been verified through browser chat.

The private shell file `config/agent.env` is sourced by `./world`; keep it mode 600
and restart the world after changing it. It is ignored by Git. Without
OPENAI_API_KEY the simulation still runs and the composer is read-only.

Physics and live occupancy maps reset on world restart. Named-place memory uses
`state/robots/football-club-v3/<duck>/<session>/`. The scene version prevents old
apartment coordinates from becoming the new prior. Reload retains a session;
leaving and rejoining starts fresh.
Boot registration and process recovery have been tested; a whole-PC reboot has not.

## Project ownership

- `app/microduck_world/`: blueprint, cockpit composition, prompt, scene validation,
  private gateway, and focused tests.
- `web/`: project Three.js panel and build configuration; imports the pinned cockpit UI.
- `assets/scenes/apartment/`: scene XML, ScenePackage metadata, named places and viewer appearance.
- `assets/scenes/football/`: the connected pitch, goals, nets, scoreboard and turf texture.
- `assets/scenes/benchmark/`: preserved four-room source and translated side-wing scene.
- `assets/microduck/`: downloaded robot models and policies, with upstream licenses.
- `ops/systemd/`: service source; systemd installs registration symlinks in the account.
- `patches/`: the small shared DimOS fixes applied by setup.
- `vendor/dimos/`: independent source checkout pinned by `dimos-revision.txt`.
- `config/`: private runtime settings and TLS files.
- `state/`, `logs/`: persistent data and operational/test evidence.
- `backups/`: private local restore material for the pinned DimOS base.
- `cache/`, `tmp/`, `tools/`, `.venv/`: disposable caches and the local toolchain.

## Sharing and reproducing this demo

This is the application repository, built on the pinned DimOS framework. It is a
working hosted demo, not yet a turnkey installer for a fresh machine. The setup
script assumes the Python toolchain, vendor checkout, robot assets and policies
have been provisioned. See `dimos-revision.txt`, `env.sh`, and the setup notes below.

Private credentials, TLS files, login databases, player state, logs, model downloads
and installed dependencies are excluded from Git. A new deployment needs its own
GitHub OAuth application, Cloudflare resources, runtime configuration and agent key.
The checked-in Cloudflare configuration identifies the existing demo deployment;
replace those resource IDs and domains before deploying your own copy.

## Setup and checks

The project toolchain and pinned vendor checkout are already provisioned on Omarchy.
Setup uses the vendor lockfile, installs the application requirements, and checks
dependency compatibility. The optional test extra avoids installing test tools in
a runtime-only setup.

```bash
./service stop
./setup --test
./service start

source env.sh
python -m pytest -c pyproject.toml --asyncio-mode=auto app/microduck_world -q
(cd web && deno task check && deno task test && deno task build)
```

Setup verifies the vendor revision and applies the project patch only when needed.
It fails on conflicting vendor edits instead of overwriting them. See
[implementation notes](docs/implementation-notes.md) for source restoration.

The gateway's `/healthz` reports whether a robot is connected to the relay; it is
a readiness check, not proof that every sensor or AI provider works. A browser
opening the world during startup loads the project frontend and reconnects automatically.

See [tailnet operations](docs/tailnet.md), [public launch](docs/public-launch.md),
the [checkpoint report](docs/overnight-report.md), and [client 3D rendering](docs/client-rendering.md). The connected football room is described in
[football](docs/football.md). See
[football club iteration](docs/football-club.md) for the current scene and
[multiplayer architecture and checks](docs/multiplayer.md) for the earlier checkpoint.

## Overnight browser checks

The bounded read-only check uses installed headless Chromium and writes
docs/stability-report.md, JSON checkpoints and hourly screenshots under logs/.
It does not drive the robot or call the agent.

Run: source env.sh, then python ops/demo_browser_soak.py --hours 6 --interval 300.

See docs/overnight-report.md for completed work and remaining limitations.

## Simulation fidelity

The robot uses the upstream 14-joint MuJoCo model and learned policies. Body mass,
inertia, actuator limits, gravity and contact forces participate in the dynamics;
these parameters have not been calibrated against a physical Microduck here.

The RGB-D/agent camera and browser Duck camera use the exported `head_camera` mount
on the moving head. The project's `camera.py` converts that CAD site's
forward/left/up axes into MuJoCo camera axes. Camera intrinsics remain the model's
values and are not claimed to be a measured calibration of a physical unit.

Automatic upright recovery is disabled. The amber **Respawn** button is an explicit
simulation reset: it cancels navigation, returns to Teleop, and places only the
selected duck upright at a clear midfield entrance slot assigned to its team. If all its team
bays are blocked, it waits for a clear bay. Its session, map frame, discoveries
and conversation are preserved. This is not a learned stand-up action or an agent
skill. The learned sit-to-stand behavior remains a separate policy action.

Sensors remain idealized: exact odometry, rendered depth and additional virtual
range sensors. Landmarks are fixed geometry. Kicks no longer move the ball to the
foot: the duck must approach, align and make physical contact. The football room
adds three free balls, collision goals and direction-aware scoring. See
[football physics and checks](docs/football.md) for dimensions and approximations.
Jaw articulation is not implemented. These are explicit limits on
real-world transfer, not evidence of physical-robot validation.
