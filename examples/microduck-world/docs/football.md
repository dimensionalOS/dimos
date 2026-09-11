# Football room

The six ducks spawn in two spaced columns at the central entrance just outside the midfield sideline. The
four benchmark rooms remain in the side wing, reached through a narrow corridor.
The playable pitch is 5.2 × 3.2 metres, inside a 6.7 × 4.4 metre enclosure.
Every floor meets at z=0 without a step. Two goals have a 1 metre clear
opening, 48 cm clearance below the crossbar, and 40 cm deep nets.

## Try it

Open https://omarchy.tailca0707.ts.net:8443/ and choose an available duck or watch.
Click **Football field** in the world panel to frame the pitch. That button
moves only your viewing camera. Drive straight onto the pitch from the midfield entrance,
or use the existing navigation tools. All six ducks remember the pitch and locker
locations, and retain their private sensors, policies, teleop and human CLI.

Three balls start on the pitch. Approach and line up a ball in front of the
appropriate foot, stop, and select **kick left** or **kick right**. The original
apartment ball can also be brought onto the pitch. Selecting a kick no longer
teleports any ball to the foot. The learned policy and physical contact must move it.

The scoreboard is mounted on the long wall, with matching blue/coral digits and a
compact browser score display. Blue scores into the coral goal; coral scores into
the blue goal. A goal requires the entire ball to cross the goal line from the
field, between the posts and below the crossbar. Crossing backward, entering from
the side, hitting a post, or bouncing in the net does not award another goal.
Each ball can score again after fully returning to the field.

Goals do not reset or reposition balls. Scores and ball positions reset when the
world service restarts. Three digits and an overflow indicator display large
scores; the underlying count continues beyond 999.

## Physical and visual model

MuJoCo owns all motion, contact, ball trajectories and scoring. Physics runs at
200 Hz and the unchanged learned policies at 50 Hz. Three.js receives compiled
model geometry and body transforms; it never applies its own ball physics.

All four balls, including the original benchmark ball, use Pollen's 5 cm radius,
30 g mass and solid-sphere inertia of 0.00003 kg m² on each axis. Ball friction is
[0.4, 0.01, 0.003], solref [0.03, 0.4], and condim 6. Existing flat floor boxes
keep friction [1, 0.005, 0.0001], solref [0.02, 1], and condim 3. Both surfaces
use default solimp [0.9, 0.95, 0.001, 0.5, 2], priority 0, solmix 1, margin 0
and gap 0. MuJoCo therefore produces contact friction [1, 1, 0.01, 0.003, 0.003]
and solref [0.025, 0.7], with condim 6.

Ball centres start and reset at z=0.051 m, leaving 1 mm of initial floor clearance.
Painted panels scale with the radius and remain massless and noncolliding.
Three.js receives the enlarged compiled geometry, and goal detection reads each
compiled ball radius to require the entire 10 cm ball to cross. See the
[physics comparison](ball-physics.md) for upstream sources and measured results.

Posts, crossbars, benches and net cords have collision geometry. Nets are anchored
rigid capsule grids with compliant MuJoCo contacts, not flexible fabric. There is
no hidden wall behind a net. Flat turf uses the same generated PNG in both
MuJoCo and Three.js; the painted lines add no physical bumps or overlapping floor.

The wall scoreboard is updated from the physics thread. Scoring examines the
swept ball volume at the goal plane, including angled and fast crossings, rather
than counting frames where a ball happens to be behind a goal. A 3 mm ground
tolerance accommodates MuJoCo's soft contact penetration. Score updates change
only lamp colors in the independent render models. Agents learn the score by
looking through their own cameras, not through an added global scoreboard tool.

The full scene's size no longer inflates the head camera's near clipping plane:
it remains 5 mm, with a 30 metre far plane. The physical head mount and existing
explicit manual respawn remain.

## Ownership and reproducibility

- `assets/scenes/football/field.xml` and `surface.png` contain the generated room.
- `ops/demo_build_football.py` deterministically regenerates those assets.
- `assets/scenes/apartment/` owns the connection, named room and camera view.
- `app/microduck_world/football.py` owns ball composition, scoring and lamp states.
- `app/microduck_world/ball_physics.py` owns the app's ball and floor parameters.
- `web/src/` owns camera framing, texture loading, display and static mesh batching.

No football scene or game logic was added to the shared DimOS demos or kernel.

```bash
cd ~/projects/microduck-world
source env.sh
python ops/demo_build_football.py
python -m pytest -c pyproject.toml --asyncio-mode=auto app/microduck_world -q
(cd web && deno task check && deno task test && deno task build)
MUJOCO_GL=egl python ops/demo_football_checkpoint.py
python ops/demo_ball_physics_checkpoint.py
python ops/demo_football_browser.py
```

The physics checkpoint builds an isolated world and writes real-policy kick GIFs,
measurements and a native render under `logs/football-checkpoint/`. Its initial
ball placement is test setup, never an action in the running world. The browser
checkpoint uses an available Duck 1 slot, drives briefly, tests manual respawn,
checks camera modes and spectator permissions, captures desktop/mobile screens,
and releases its slot. It does not send agent messages.

## Verification, 2026-09-06

The final hosted suite passed 96 Python tests, 9 frontend tests and 12 lobby
admission tests. TypeScript checking, the production build, focused Python type
checks and linting of the new football modules and scripts also passed.

The checkpoint was rerun with the live default `robot_allcollisions.xml` model
(the earlier script selected `robot_walk.xml`). Both real ONNX kick policies
made foot contact and scored in isolated MuJoCo; the measured results were unchanged:
left kick moved the ball 0.862 m with a 0.941 m/s peak speed; right kick moved it
0.858 m with a 0.803 m/s peak speed. The duck stayed upright in both tests.
Tests also cover both goal directions, full-ball clearance, post/crossbar misses,
reverse and side entry, repeat prevention, net retention, floor continuity,
ball mass/inertia, native scoreboard lamps and camera transforms.

The browser checkpoint verified the Three.js room and POV, scoreboard display,
walking, manual respawn, the 640 × 360 MuJoCo camera, observer restrictions and
a 390 px mobile lobby without horizontal overflow. Static batching reduced the
captured field view from 370 to 160 draw calls.

The server's headless Chromium test uses SwiftShader, not its NVIDIA GPU. With two
WebGL panels it produced about 2 FPS, and nominal 50 ms timers stalled up to
2.2 seconds. Walking was therefore checked with the JPEG renderers selected;
the existing safety timeouts were preserved. This is a software-renderer limit,
not a measurement of a hardware-accelerated client browser. No physical Microduck
or measured turf/ball calibration was available for hardware validation.
