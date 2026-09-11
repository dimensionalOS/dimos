# Football club iteration

Six equivalent player slots share one MuJoCo world: Red 1 to Red 3 (`duck1` to
`duck3`) and Blue 1 to Blue 3 (`duck4` to `duck6`). The roster, team colors,
spawn bays, facing direction and private MCP ports are defined in
`assets/scenes/apartment/multiplayer.json` and consumed by Python and the lobby.

## Scene and play

The pitch retains its current dimensions, three field balls, goal geometry, nets,
physical kicking and direction-aware scoreboard. The score protocol retains its
existing `blue` and `coral` keys; the human UI calls the second team Red. There are
no kickoff, timer, automatic ball placement or new score reset rules. Existing
restart/reset behavior remains unchanged.

Each team starts in its own locker room just south of the pitch. Each room is
2.55 by 1.625 metres, half its previous floor area. Its three spawn bays keep their
original lateral spacing and move forward to y=1.375. Both rooms open
into the central player tunnel. The narrow corridor behind the lockers leads east
to the original four-room benchmark scene, translated by (8.3, -0.825) metres.
The internal room layout and objects are retained. Its original ball moves with it.

The roster holds three spawn bays per team. A spawn or explicit respawn uses the
requested bay if clear, then another clear bay in the same locker room. It never
uses the opposing team's room. A blocked request waits. Respawn cancels previous
motion while preserving that occupant's knowledge and conversation.

## Knowledge and controls

All six use an independent complete robot blueprint, including the existing agent,
MCP tools, teleop, learned policies, camera, mapper, planner and knowledge modules.
They share the world coordinate convention so the supplied pitch and locker-room
positions work for every duck. This does not share map contents or observations.
Only the pitch and two locker rooms are supplied semantic prior knowledge.

Each occupant gets a UUID namespace and private database directory under
`state/robots/football-club-v3/<duck>/<generation>/`. Commands, sensor packets,
subscriptions and tool events are scoped to that generation. Duck 1 now has the
same lifecycle as every other slot. A new scene version avoids reusing coordinates
from older layouts. The human overview remains global and is not an agent sensor.

## Lobby walking preview

`ops/demo_record_walk.py` runs the existing Pollen ONNX walking policy in an
isolated MuJoCo instance and records a near-periodic 0.82-second gait. The resulting
`web/public/duck-preview.json` is about 90 KB before compression. At runtime the
lobby reuses geometry from its existing world preview and one Three.js renderer.
Static portraits are cached; only the active card animates, capped at 30 FPS.
Hover, keyboard focus and the touch preview button activate it. It fades to a
standing pose on exit and stops when hidden or offscreen. Deliberate hover, focus
and button requests still show the gait with reduced motion enabled; that
preference disables the card lift and pose transitions.
No live simulation or policy inference is started for a lobby animation.

## Regenerate and verify

Run from the app root with `source env.sh`:

```bash
python ops/demo_build_football.py
python ops/demo_build_club.py
python ops/demo_record_walk.py
python ops/demo_club_checkpoint.py
python ops/demo_football_checkpoint.py
python -m pytest -c pyproject.toml --asyncio-mode=auto app/microduck_world -q
deno test -A --no-check --config vendor/dimos/web/relay/deno.json relay/lobby_test.ts
(cd web && deno task check && deno task test && deno task build)
```

The club checkpoint runs six unchanged gait policies, checks upright movement and
physical clearance through both locker doors, the pitch tunnel and side corridor,
and renders native scene screenshots. The football checkpoint checks foot contact
and ball travel without policy-triggered ball placement. Reports live under `logs/`.

The application remains separate from the DimOS framework. This iteration does not
change the pinned vendor source, robot dynamics or learned policy weights.

## Completed acceptance checks (2026-09-08)

- Python: 98 passing tests. Lobby: 12 passing tests. Frontend: 9 passing tests,
  TypeScript check and production build. Ruff passed for the application and new scripts.
- Six independent browser players connected with a seventh viewer. Occupied slots
  returned HTTP 409. Reload retained the same duck; a viewer took a released slot.
- Under six-player load, simulation time advanced 10.005 seconds in 10.000 seconds
  of wall time. This is a bounded load check, not a long-running performance guarantee.
- Driving Red 2 moved it about 27 cm; the other five changed position by less than
  a millimetre during that check.
- Both native kick policies made foot contact and moved their test ball about 86 cm.
- The Blue 2 agent identified its team and all three prior locations, then navigated
  from the blue locker room to the pitch and stopped upright. Its private camera
  returned a current 640 by 360 observation, and human CLI calls reached its MCP server.
- Explicit respawn returned Blue 2 from the pitch to its own locker bay, restored
  Teleop mode, and retained the conversation.
- The lobby animation advanced only for the active card and stopped after pointer
  exit. Reduced motion retained static portraits. The 390-pixel mobile viewport
  had no horizontal overflow.

Repeat the browser acceptance check with all six player slots available using
`python ops/demo_football_lobby_checkpoint.py`. It releases its own test slots on exit.

## Player identity and browser appearance

All six duck cards are shown together. Clicking a duck opens a native modal
dialog for an optional player name; confirming it joins the selected slot.
Cancel and Escape close the prompt without claiming a duck. Spectators enter
directly without a name prompt. Blank names use the team
number. Names are normalized, limited to 24 characters, and unique among current
occupants (including reconnect reservations). Invalid or conflicting names leave
the current assignment untouched. Names persist across reload and respawn, clear
on departure or expiry, and belong to the occupant's UUID generation.

The lobby shares display names with human browsers. Physics assignments remain
robot-to-generation pairs. Names are not added to sensors, agent prompts or MCP
tools. The browser checks the current generation before attaching a name to a duck.
It projects a small HTML label through the Three.js camera, using ordinary text
nodes. Labels add no WebGL draw calls. The Names checkbox is a local preference,
saved in the browser; the follow menu can select a player by display name.

The lobby, Three.js world and Three.js duck camera share the same white-shell
palette, with team color on the face surround, thigh detail plates and ankle
details. Named mesh metadata selects the parts. These are browser material
overrides; native MuJoCo cameras and physical properties are unchanged.

Automated checks for this iteration: 99 Python tests, 15 lobby tests and 11
frontend tests, plus TypeScript checking and a production build. The native
checkpoint confirms all six gaits stay upright and the smaller rooms retain clear
passages to the pitch and benchmark wing.

Browser acceptance for the compact rooms and player names also passed:

- Red and blue lobby portraits show white shells with team accents. A 390-pixel
  viewport has no horizontal overflow.
- A player named Ada appears above its duck in both the player and spectator
  Three.js views. A case-insensitive duplicate is rejected before claiming a slot.
- Selecting Ada in the follow menu tracks its duck. The follow camera keeps the duck visible in its locker room; its current
  close framing and wall handling are described below.
- Hiding names affects only that browser and persists through reload. The other
  viewer keeps its labels. Respawn retains the name; departure hides it.
- Both test participants released their slots. All six slots were free afterward.

Evidence is in `logs/name-check/` on the deployment host. The local macOS
headless browser stopped delivering animation frames even for a plain browser
callback, so visual acceptance used isolated Chromium contexts on the server.
Software-rendered headless frame rates are not a client performance measurement.


## Six-duck picker and hover follow-up

The picker shows all six ducks together with separate spectator access. The name
prompt opens only after choosing a duck, focuses the input, and confirms the slot
on submit. Occupancy remains checked when connecting. The legacy host shortcut
uses the same name prompt for Red 1.

The walking preview is attached inside the active card, so scrolling and card
transforms keep it aligned. React updates its active selection directly. Only one
Three.js canvas animates; all other portraits remain cached images. Reduced motion
no longer silently blocks a deliberately requested preview.

Live Chromium checks passed for actual pointer movement over all six cards with
normal and reduced-motion settings, differing rendered walking poses, one active
canvas, and stopping after pointer exit. The connection prompt passed focus,
Escape/cancel without claiming a slot, a named join, and spectator entry without
a prompt. Desktop and 390-pixel mobile screenshots were inspected. Evidence is
in `logs/picker-check/` on the deployment host. Repeat with
`python ops/demo_picker_checkpoint.py` when at least one duck is free. The frontend's 11 tests, TypeScript
check and production build also passed. This update required no simulation restart.


## Close, fixed follow camera

Follow duck and the player dropdown now select a fixed shoulder view about
0.8 metres from the aim point. The camera follows the duck's position and heading,
without copying its walking body's roll or pitch. A ray against scene geometry
shortens the offset when a wall or furniture would block the view. This is a
browser camera adjustment with no changes to physics or robot commands.

A mouse press, click, tiny pointer movement or scroll keeps the camera locked.
Dragging at least four pixels switches to free orbit/pan within the same gesture.
The free camera then stays in place as the duck moves. Follow duck locks it again.
The toolbar shows the drag-to-unlock hint while following.

Validation: 14 frontend tests passed, including translation/heading tracking,
roll/pitch isolation and wall clearance, plus TypeScript checking and the build.
The live Chromium check confirmed close framing, click and wheel staying locked,
a moving duck remaining centered, drag unlocking, the free camera remaining fixed
through another robot movement, and relocking. The test used two short navigation
goals for its own duck and released that duck afterward. Evidence is saved in
`logs/follow-check/` on the deployment host. No simulation restart was needed.

## Pitch entrance floor repair, 2026-09-09

The club floor ended at y=2.05 while the pitch began at y=2.10. This left a
5 cm unsupported strip across the entrance. Real walking policies reproduced
falls at that boundary from left, centre and right approaches. Extending only
the club floor's north edge to y=2.10 removed the falls. Its south edge remains
y=0.275 and its top remains z=0; room dimensions, walls and contact settings
are unchanged. The club generator now produces the repaired floor.

The earlier checkpoint checked horizontal doorway clearance at z=0.2 and only
walked briefly inside the lockers. It missed the missing floor at the threshold.
The new regression checks 255 downward rays across the entrance, then walks
each of the six ducks through it in both directions using the real policy and
checks they remain upright for a second after stopping. All 145 Python tests
passed, including ball contact and scoring regressions. The floor-support test
was also run before the fix and failed on the 5 cm gap. Before/after walking
measurements are in `logs/entrance-checkpoint/comparison.json` in the worktree.
