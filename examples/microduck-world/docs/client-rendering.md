# Client rendering checkpoint — 2026-09-05

Live on https://omarchy.tailca0707.ts.net:8443. Chrome/Chromium on the tailnet is
required by the existing WebTransport cockpit. This is a first visual foundation;
public access, visitor-owned ducks and the scene/UI redesign remain separate work.

## Responsibilities

MuJoCo on Omarchy remains authoritative for physics, policies, collisions, navigation
and the robot camera. The chase camera is disabled. The head camera remains 640×360
at up to 6 fps, including the image used by the agent. It works without any visitor
providing a rendered frame.

WorldSimModule extends the existing Microduck simulation hooks in the project package.
At startup it exports the compiled visible geoms and meshes, then publishes body poses
from the physics thread at a maximum of 30 Hz. No physics state is mutated by the viewer.
The existing DimOS stream, codec registry, relay and browser SDK carry world_state.
The initial model is identified by a content hash, so a restarted/changed model cannot
be paired accidentally with old geometry. Geometry is transferred once and cached.

The project web entry registers world and POV variants of world3d, then mounts the
pinned stock cockpit with the SDK session exposed through a project context.
Controls, navigation, policies, chat and transport stay in the existing implementation.
Three.js renders the model on the visitor's GPU, interpolating poses and maintaining a
camera local to that browser. Orbit, zoom, pan, Overview and Follow duck affect only
that camera. A frozen pose stream displays a waiting indicator. Rendering stops while
the tab is hidden, and resources are disposed when the panel is removed.

Source ownership:
- app/microduck_world/visual_scene.py: compiled model export and immutable assets.
- app/microduck_world/world_sim.py: existing sim hooks and the pose stream.
- app/microduck_world/cockpit.py: panel declaration and layout.
- web/src/: panel, renderer, protocol validation and styling.
- assets/scenes/apartment/viewer.json: scene-specific browser appearance.
- state/viewer/: ignored generated JSON/gzip model files; no secrets or policies.

No new vendor or demo edits were required. The existing DimOS patch checksum is
unchanged: a4dd8f9074cb72ce7c9cc98ddec3f871b63c362308aeae2c043521ae7ee9155d.

## Build and operate

Source env.sh before commands. ./setup now installs frontend dependencies from
web/deno.lock, type-checks and builds the project UI. For a frontend-only change:

    (cd web && deno install --frozen && deno task check && deno task test && deno task build)

Reload the browser after a frontend build. For scene/appearance or Python changes,
restart the world with ./service restart world. Gateway private configuration:

    "frontend_dir": "/home/tule/projects/microduck-world/web/dist",
    "world_assets_dir": "/home/tule/projects/microduck-world/state/viewer"

Only these generated/static directories are served, after gateway access checks.
The patch and all project code/assets remain within the project folder.

## Validation

- 34 project Python tests pass. New tests compare exported positions AND rotations
  against real MuJoCo geom transforms after free-body movement and a joint rotation;
  compare binary mesh buffers; check immutable/gzip model export; verify frontend and
  model route access restrictions, including a symlink escape attempt.
- Strict mypy passes on the four new/changed runtime modules; Ruff check/format pass.
- Three frontend protocol tests, TypeScript check and production build pass.
- uv pip check: all 265 installed packages compatible. Existing vendor patch still
  applies to the clean pinned checkout and matches the actual vendor diff exactly.
- Two independent Chromium sessions on the Mac received the same moving duck while
  retaining separate cameras. Orbiting one changed its camera from (4.2,-5.2,4.5)
  to approximately (-3.978,-3.141,6.322); the other stayed at its original camera.
- Mouse orbit, wheel zoom, right-drag pan, Follow duck, Overview, maximize and Escape
  restore were exercised. Maximized canvas filled 1438×846 CSS pixels.
- A bounded 2-second W drive moved the rendered trunk about 24 cm; follow camera
  moved with it and release returned all command velocities to zero. The test uses
  DOM key events with code=KeyW because agent-browser 0.27.1 keydown omits code.
- Clicking the kitchen room reached approximately (1.082,0.891). Sit/stand changed
  rendered trunk height from about 0.116 m to 0.059 m and back, without a fall.
- Agent chat called observe, received an image, and described the yellow/green floor,
  gray column, white ball and yellow cylinder visible in its MuJoCo camera.
- Reload reused the model from cache (0 transferred bytes). Restarting the simulation
  recovered live 3D and camera feeds in already-open browsers.
- Final state: Teleop, walk, no active goal, drive disarmed. No browser JS errors.

Measured model: 89 geoms, 38 unique meshes, 17 body transforms. Initial asset is
10,383,726 bytes decoded / 4,279,166 bytes gzip. Live poses were observed around
26–28 Hz (30 Hz cap); head camera around 5.5–6 fps. The client draws frames locally
between pose updates. These are desktop smoke measurements, not a load benchmark.

Evidence: logs/client-3d-overview.png, logs/client-3d-follow.png,
logs/client-3d-maximized.png. The earlier overnight soak predates this renderer.

## Limits and next extensions

The visitor pays the initial geometry download and local GPU cost. This exporter
covers the primitives/meshes and visible geom groups used by this scene, using colors
and computed mesh normals; it does not promise full MuJoCo texture/material parity,
skins, flex bodies or heightfields. Unsupported visible geom types fail explicitly.
Project lighting/colors already differ intentionally from the agent's camera.
Decorative additions must stay consistent with physical geometry where interaction
matters. Camera-wall occlusion is ordinary orbit-camera behavior; camera collision
avoidance is not implemented. Small-screen cockpit layout still needs the planned UI pass.

Server logs still contain the previously observed shared-memory fallback and skipped
camera-render warnings; live streams and controls passed the checks above. No claim
of eliminating all server bottlenecks, reboot validation or overnight 3D soak is made.
The current access boundary is still the tailnet, not public visitor authorization.


## Camera comparison checkpoint — 2026-09-05

Both World and Duck camera now default to Three.js and display completed client
draws per second. This differs from simulation pose updates (maximum 30 Hz): poses
are interpolated between updates. The POV uses the actual MuJoCo camera's body,
local transform, field of view and clipping planes, with a fitted 16:9 viewport.

Each panel has an independent Three.js / MuJoCo JPEG selector. The native duck feed
is still the agent's 640×360, maximum 6 FPS camera. The optional world JPEG uses a
fixed shared follow camera, 640×360, intentionally capped at 12 FPS to limit cost.
This cap is a configuration choice, not a claim about MuJoCo's maximum performance.

Project comparison.py owns a separate render thread and private model/data. It
polls the existing relay's subscriptions once per second. With no world JPEG viewers
it releases its renderer and does not render or copy physics snapshots. With viewers
it renders one shared feed regardless of viewer count. The project panel explicitly
subscribes only while JPEG is selected and visible; Three.js drawing pauses for that
panel. No new vendor/demo changes or agent image calls were needed.

Added ownership: comparison.py handles native comparison rendering; frameRate.ts
measures draws; viewerSession.ts exposes the existing SDK session for subscriptions.
The two browser panels share a single cached model download.

Validation for this checkpoint:
- 42 Python tests, 6 frontend tests, strict TypeScript/build and runtime mypy pass.
  Real MuJoCo tests verify camera world position/orientation, clipping planes and
  the native follow camera position; subscription tests verify idle demand handling.
- Mac Chromium observed approximately 60 FPS for both Three.js panels, with poses
  arriving around 26–28 Hz. Separate viewers retained independent backend choices.
- Native world production delivered 96 frames in 8.015 seconds; simulation time
  advanced 8.015 seconds as well. Browser delivery was about 11.5 FPS world / 6 FPS
  duck in the desktop sample. Indicative render time was about 6 ms per world frame.
- Switching all viewers back to Three.js made the native worker inactive; its frame
  counter stayed unchanged and the additional GPU allocation was released.
- The bounded browser checkpoint captured both backends, verified hidden canvas
  behavior and idle shutdown, then drove the duck about 24 cm while the new POV
  followed it. Evidence is logs/camera-checkpoint.json, camera-three.png and
  camera-jpeg.png. Run with: python ops/demo_camera_checkpoint.py.

The bounded check runs headless Chromium with software WebGL on Omarchy. Its low
world FPS is not a client GPU benchmark. Mac automation later stalled on screenshot
capture, so final visual captures and the full flow used this independent browser.
The prior agent observation and overnight soak are earlier checkpoints, not reruns
of those checks after this change.
