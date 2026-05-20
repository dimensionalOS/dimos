---
name: measure_distance
description: Use when asked to MEASURE A DISTANCE — between two things the robot saw, between two map coordinates, from a viewpoint to an object, how far the robot walked in total or between two moments, or how close the robot came to a given point. Pick the appropriate sub-procedure below; do every arithmetic step through `calc` so the answer is deterministic.
---

# Measuring distances in the recording

Several different things people call "distance"; each has its own
procedure. Read the question first, pick the matching flavour, then
follow the steps. All arithmetic goes through `calc` (Monty REPL):
do NOT compute square roots, sums, or magnitudes in your head.

## When to use

- "How far apart are X and Y?"
- "How far is the chair from the door?"
- "How far did you walk?"
- "How close did you get to the meeting table?"
- "What's the distance from where I'm pointing on the map to that object?"

If the question asks about *the height of a wall*, *the width of an
object*, *the depth of a corridor*, or *how far away the wall is from
this single frame* — STOP. The recording does not contain a depth-per-
frame channel and we have no tool that estimates single-image depth.
Tell the user that's not available rather than guessing.

## Tools allowed

- `calc`                     (all arithmetic, list-walking, sums)
- `near`                     (to pull odom samples by spatial filter)
- `recent` / `summary`       (orientation on odom if needed)
- `frames_facing`            (camera-to-point distance, already metric)
- `search_semantic` + `show_image` (only when you need to localise an
  observed object first — i.e. when you don't have its (x, y) yet;
  consider loading the `position_of_thing` skill for that subtask)
- (When you're done, end with a plain text reply — no tool call.)

Do NOT use `walkthrough`, `show_map`, or `recall_view` for distance
work — they don't give you metric output you can compute on.

## Flavour A — point-to-point in known map coordinates

Both endpoints are already known as `(x, y)` (either from the user,
from earlier tool output, or from `position_of_thing`). One `calc`
finishes the job:

```python
calc('''
ax, ay = -2.20, 8.70
bx, by = 2.65, 8.35
d = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
d
''')
```

`** 0.5` is the supported way to take a square root — there's no
`math.sqrt` in this sandbox.

## Flavour B — camera to an observed point

You already have a query coordinate `(x, y)` and want to know how
far it was from a specific camera pose:

```python
frames_facing(x=<x>, y=<y>, k=1, max_range_m=20, check_occlusion=False)
```

The returned candidate's caption includes `dist=<m> m` directly —
that's the camera-to-query distance in metres. No further math.

## Flavour C — thing to thing (neither localised yet)

Two objects you remember seeing but haven't pinned to map
coordinates. Do this in three stages:

1. Load the `position_of_thing` skill (`load_skill("position_of_thing")`)
   and follow its hypothesise/verify loop to get `(x_A, y_A)` for the
   first object.
2. Repeat for the second object → `(x_B, y_B)`.
3. Compute via Flavour A. Report both individual coordinates AND the
   distance, with the same caveats `position_of_thing` carries (these
   are floor-projection estimates, not labelled positions).

## Flavour D — total path length (or path length between two ts)

The robot's full trajectory is in the `odom` stream. To get a numeric
distance you fetch the poses, sort by ts, and sum step lengths via
`calc`.

Stage 1: pull odom samples in a big radius (this returns observations
with `pose` attached). For the *whole walk*, use a centre near the
trajectory and a generous radius:

```python
near(stream="odom", x=0, y=10, radius=50, k=2000)
```

Read the result — each line gives `id`, `ts`, `pose=(x, y, z)`. Pick
the timestamps + positions you need (all of them for full path; the
ones in `[t_start, t_end]` for a slice).

Stage 2: feed the points into `calc` as a list and sum step lengths:

```python
calc('''
# pts is a chronologically-sorted list of (ts, x, y)
pts = [
    (1778055838.69, -0.48, 4.54),
    (1778055845.29, -0.64, 7.75),
    ...
]
pts.sort(key=lambda p: p[0])
total = 0.0
for i in range(1, len(pts)):
    dx = pts[i][1] - pts[i-1][1]
    dy = pts[i][2] - pts[i-1][2]
    total += (dx * dx + dy * dy) ** 0.5
total
''')
```

Two pragmatic notes:
- Pasting 1122 points into one `calc` is fine — Monty handles it.
  But it's wasteful in tokens. If the answer doesn't need precision,
  use every Nth point (e.g. every 5th odom hit).
- The path length on dense odom (every ~50 ms) overestimates real
  walked distance slightly because micro-jitter sums up. Subsampling
  by 0.5 – 1 s gives a cleaner number.

## Flavour E — closest approach to a point

How near did the robot ever get to `(qx, qy)`?

```python
near(stream="odom", x=<qx>, y=<qy>, radius=20, k=2000)
```

Then in `calc`, scan the returned poses and find the minimum:

```python
calc('''
qx, qy = <qx>, <qy>
poses = [
    (<x1>, <y1>),
    (<x2>, <y2>),
    ...
]
dists = [((p[0]-qx)**2 + (p[1]-qy)**2) ** 0.5 for p in poses]
min_d = min(dists)
i_best = dists.index(min_d)
(min_d, poses[i_best])
''')
```

Report the closest distance AND the pose at which it occurred.

## What this skill does NOT cover

- **Single-frame depth** (camera → wall, "how far is that object in
  the image"): there is no depth channel and no single-image depth
  estimator wired in. Tell the user this isn't available.
- **Real object dimensions** (height / width of a chair, the man, a
  door): same reason. We have 2D image data plus 3D lidar points;
  combining them into per-object bounding boxes would require a
  perception pass we don't have.
- **Lidar-only ray casts** (camera-to-nearest-surface in a direction):
  geometry is there but no tool slices it that way. Out of scope.

For anything in this list: respond honestly that the recording
doesn't support the measurement, rather than producing a number.

## Submit

When you have the number, end with a plain text reply containing:
- the figure in metres,
- which flavour you used,
- the inputs (coordinates, ts range, etc.) that produced it,
- any caveat from the skill (especially "lower bound from
  walked-coverage" or "floor-projection estimate").
