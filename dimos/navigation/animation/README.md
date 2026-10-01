# Animation

A camera flythrough over a relocalization replay. Pick waypoints on a premap, then render
a Rerun recording seen from a camera gliding along them, always facing the lidar.

Run everything from the repo root. Waypoints live in `waypoints.json` next to this README.

## Picker

```sh skip
uv run dimos run animation-waypoints
```

Opens the premap in Rerun. Each click appends a numbered waypoint (magenta) and the
cyan line is the path the camera will fly, at the height it will fly. Edits to
`waypoints.json` show up within a second. Another map: `--waypointpicker.map-file=<premap>`.

## Editor

With the picker running:

```sh skip
uv run python -m dimos.navigation.animation.edit list       # waypoints, and what the next click does
uv run python -m dimos.navigation.animation.edit place 3    # next click moves waypoint 3
uv run python -m dimos.navigation.animation.edit insert 3   # next click becomes a new waypoint 3
uv run python -m dimos.navigation.animation.edit delete 3   # remove waypoint 3 now
uv run python -m dimos.navigation.animation.edit cancel     # forget a pending place/insert
```

## Renderer

```sh skip
uv run python -m dimos.navigation.animation.flythrough --out flythrough.rrd
uv run rerun flythrough.rrd
```

Replays `mid360_raycast_door` (LFS, a Go2 Mid-360 walk through the SF office door) through the relocalizer against the corrected SF premap and
flies the camera over it; Rerun shows the camera view. The camera climbs from 0.6 m
to 15 m (`HEIGHT` in `waypoints.py`) and ignores clicked heights.

Speed: the camera covers `--slow-m` (15 m) in the first `--slow-s` (30 s), with `--ease`
(1.5) shaping that start, then cruises at whatever speed lands it on the last waypoint at
`--duration` (60 s). Slower overall: raise `--duration`. Slower start: lower `--slow-m` or
raise `--slow-s`.

Other recording: pass it as the first argument, plus `--lidar <stream>` and `--premap <map>`.

`--scan` overlays each raw lidar scan; the map carves out moving people, the scan keeps them.

`--near-m` (8 m) shows the premap only that close to where the lidar was at the fix
until `--reveal-s` (25 s) into the flight, then all of it; `0` shows everything from the start.
