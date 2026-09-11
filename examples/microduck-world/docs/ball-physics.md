# Pollen ball and flat-floor parity

Verified 2026-09-08 against Hugging Face revision
`e81974b932c7ca1819843b7bb3dcd42e2993e98e`:

- [game.js](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/e81974b932c7ca1819843b7bb3dcd42e2993e98e/app/src/game/game.js), lines 186-225: timestep, unqualified flat plane, and ball contact attributes.
- [constants.js](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/e81974b932c7ca1819843b7bb3dcd42e2993e98e/app/src/game/constants.js), lines 43-44 and 67-68: timestep, radius and parking height.
- [robot_allcollisions.xml](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/e81974b932c7ca1819843b7bb3dcd42e2993e98e/app/public/robot/mjlab/robot_allcollisions.xml): no root geom contact or option overrides. Robot-specific named defaults do not apply to the added ball or floor. The roller MJCF was also checked and has no root contact overrides.
- [package-lock.json](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/blob/e81974b932c7ca1819843b7bb3dcd42e2993e98e/app/package-lock.json): upstream WASM is locked to MuJoCo 3.11.0.

## Applied configuration

| Parameter | All four balls | All five physical floor boxes |
| --- | --- | --- |
| Radius | 0.05 m | Existing shape and dimensions |
| Mass | 0.03 kg | Static |
| Sliding, torsional, rolling friction | 0.4, 0.01, 0.003 | 1, 0.005, 0.0001 |
| solref | 0.03, 0.4 | 0.02, 1 |
| condim | 6 | 3 |
| solimp | 0.9, 0.95, 0.001, 0.5, 2 | Same |
| priority / solmix | 0 / 1 | Same |
| margin / gap | 0 / 0 | Same |

The compiled sphere inertia is 0.00003 kg m² on each axis. The benchmark ball
and three pitch balls share app-owned settings in `ball_physics.py`; no shared
framework constant or dependency changed. Floors already inherited these values;
the app now sets them explicitly during composition without altering geometry.
The floor names are `floor`, `football_floor`, `club_floor`, `tunnel_floor` and
`corridor_floor`. Decorative overlays remain noncolliding.

All 20 ball-floor combinations were checked in the compiled six-robot scene.
Each contact has dimension 6, friction [1, 1, 0.01, 0.003, 0.003], solref
[0.025, 0.7] and the default solimp. These are the result of MuJoCo's maximum
friction and equal-weight contact mixing, not the ball coefficients alone.
See [MuJoCo contact mixing](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters).

Spawn centres and reset qpos0 are z=0.051 m, providing 1 mm initial clearance.
The benchmark ball retains x/y [9.8, 0.075]; pitch balls retain their existing
x/y sites. Panel positions and sizes scale with radius; each panel has mass 0
and both collision masks 0. Three.js receives the compiled 5 cm spheres. Scoring
already reads each compiled radius and now requires the full larger sphere to
pass the opening; the tests cover the extra 1.5 cm before a goal is awarded.

## Validation

The independent fixture in `ops/fixtures/pollen_flat_ball.xml` reproduces Pollen's
ball and flat plane without importing application constants. Its only placement
change is translating the horizontal parking position to the origin. Each
candidate is extracted from the compiled world ball and the existing pitch box,
with that box translated so its top is z=0. Ball inertia and simulation options
are checked before comparison. Both sides run on the same MuJoCo 3.10.0 process,
at a 0.005 s timestep with gravity [0, 0, -9.81]. No robot, policies, walls or
terrain affect these isolated comparisons.

| Experiment | Reference and each of the four candidates |
| --- | --- |
| Rolling, initial 0.5 m/s and matching 10 rad/s spin | Below 0.01 m/s after 0.744627 m and 4.815 s |
| Maximum rolling state difference over 10 s | 6.22e-15 |
| Drop, centre initially at 0.5 m | First contact at 0.310 s |
| First rebound, centre / bottom height | 0.057960 m / 0.007960 m |
| Resting centre height | 0.049869 m |
| Maximum drop state difference over 10 s | 3.89e-16 |

The drop produces up to 24.10 mm of transient soft-contact penetration on both
configurations, settling to 0.131 mm penetration. This is the verified upstream
soft-contact response, separate from the initial placement, which clears the
floor. The approximate 0.75 m / 5 s rolling sanity check is satisfied; it is not
used as an exact assertion across engine versions.

Both existing ONNX kick policies made physical contact with the appropriate ankle,
moved the ball, scored once, and left the duck upright:

| Policy | Net ball travel after 2 s | Peak speed | Score |
| --- | --- | --- | --- |
| Left kick | 0.810 m | 1.403 m/s | 1-0 |
| Right kick | 0.836 m | 1.260 m/s | 1-0 |

All four actual scene balls also scored in both directions in isolated physics
tests. Scoring was verified not to mutate qpos or qvel. All six walking policies
ran for four simulation seconds from their locker spawns, and all five passages
between lockers, tunnel, pitch, corridor and benchmark wing remained clear.
The Python suite passed 132 tests, including whole-ball scoring, post/crossbar
misses, reset clearance, massless markings and the geometry exported to Three.js.
The 14 frontend tests, 15 lobby tests, TypeScript check, app physics type checks
and Ruff checks also passed. Regenerating the pitch preserved all 269 geom
attribute values; two XML number formats were normalized without changing values.

Physics commit `a464222` was deployed to the existing world service. Its live
model is `scene-7b19186ca353a26ef125.json`: all four spheres report radius 0.05 m
and rest at z=0.049869 m at their original x/y positions. Six actors remain in
the streamed model. The live browser checkpoint passed all six walking previews,
connection-time naming, cancellation, mobile layout and the spectator path.
Temporary sessions started all six private robot runtimes; each exposed its 20
human CLI/agent tools. The checkpoint sent no movement commands and released all
six slots. Live evidence is in `logs/ball-physics-checkpoint/live.json` and
`cli.json`, with browser results in `logs/picker-check/results.json`.

Run the checks from the app repository after sourcing its environment:

```bash
python -m pytest -c pyproject.toml --asyncio-mode=auto app/microduck_world -q
python ops/demo_ball_physics_checkpoint.py
MUJOCO_GL=egl python ops/demo_football_checkpoint.py
MUJOCO_GL=egl python ops/demo_club_checkpoint.py
```

Worktrees must set `PYTHONPATH` to their own `app` directory when reusing the
production virtual environment. Comparison reports and NumPy trajectories are
written to `logs/ball-physics-checkpoint/`; kick GIFs and measurements are in
`logs/football-checkpoint/`.

## Runtime decision and remaining differences

Keep the server's MuJoCo 3.10.0. Upstream uses WASM 3.11.0, but the required
contacts already compile correctly and both configurations match on the current
engine. The [3.11 release notes](https://mujoco.readthedocs.io/en/stable/changelog.html#version-3-11-0-july-27-2026)
include new contact features, an implicitfast integration change, a different
sleep tolerance and API/ABI changes. This scene uses Euler integration, default
Newton/pyramidal solving and no added adhesion or surface velocities. No upgrade
is needed for the requested ball-floor settings. This comparison does not claim
bit-identical native 3.10 and browser WASM 3.11 trajectories.

Our finite box floors, football goals, rigid nets, walls and six-robot interactions
remain different from Pollen's single-robot arena. Tests establish parity for
isolated flat-floor contacts, not full-scene trajectory identity. Spawn centres
have 1 mm clearance instead of upstream's resting park height. The existing
server-authoritative physics, Three.js viewer, names, camera behavior, teams,
locker spawns, benchmark rooms, teleop, policies and human CLI remain in place.
There are no terrain bumps, artificial kick impulses, new kickoff/reset rules
or automatic ball respawns.
