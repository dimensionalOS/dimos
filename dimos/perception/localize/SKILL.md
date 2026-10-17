---
name: perception-localize
description: Locate objects with a running PerceptionLocalize stack over MCP (`state`, `localize`) - probe the machine, build the time window, watch labels on a schedule, narrow an object's position, change params per call, and tell where a failure comes from. Use whenever a blueprint with PerceptionLocalize and McpServer runs, such as go2-localize or xarm-localize-live.
---

# Localize over MCP

`localize` answers from memory: the colour frames the module has embedded,
lifted to 3D through depth or lidar. It does not look at the current frame.
Every answer is a verified 3D object seen inside the asked window, with a score,
a view count and when it was first and last seen; an empty answer is an answer.

## Before the first call

    nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader
    uv run dimos mcp call state

- Probe the GPU first. How many labels a call can carry depends on it, see
  "Labels and GPU".
- `state()` returns `ready: N frames embedded, localize will answer` or
  `not ready: <stage>`. Until it reads ready, `localize` returns its stage
  instead of an answer.
- `Error: no running MCP server (is DimOS running?)` means the stack is down.
  Check `uv run dimos status` and report it. Do not stop or restart the stack.

## Calling localize

    uv run dimos mcp call localize -j '{"objects": "chair,door", "start": -30, "duration": 30}' --timeout 300

- `localize(objects, start=-10, duration=10, params="")` returns once the answer
  is built. Send the next call after it returns.
- `objects` is one label, or a few separated by commas. The labels of one call
  share one detector pass per frame.
- The client waits 30 s. A window with many unread frames takes longer, so pass
  `--timeout 300` or set `MCP_TIMEOUT=300`.

## Building the window

`start` and `duration` are seconds over the embedded frames the module holds.

| arguments | window |
|---|---|
| `start=-10, duration=10` (default) | the last 10 s |
| `start=-60, duration=60` | the last minute |
| `start=-60, duration=20` | from 60 s ago to 40 s ago |
| `start=30, duration=20` | 20 s, from 30 s after the oldest frame held |

- A negative `start` counts back from the newest embedded frame, a positive one
  forward from the oldest frame still held. The module holds `horizon_s` of
  frames, 600 s by default, so once memory is full the oldest frame moves.
- Frames are embedded at 1 per second, 3 on a walking robot (`mobile=True`),
  each the sharpest of its slot.
- The first answer line is the window as resolved:
  `window A s to B s of C s of feed, N frames`. With `N = 0` nothing was read
  and the answer is memory only.
- An answer holds only what was seen inside its window. Earlier calls never add
  objects to it, and nothing seen outside the window changes its positions,
  ages, scores or views.
- A span of time is read once per label and params; the fields that only build
  the answer, `accept_score`, `min_views`, `cluster_radius_m` and
  `fuse_voxel_m`, aside. A window never read for a label costs detector passes.
  The same window asked again is answered from memory with no detector pass,
  and a window overlapping earlier ones reads only its new part.

## Reading the answer

    window 0.0s to 30.0s of 28.3s of feed, 76 frames
    'chair' at (-0.97, 7.31, 0.40) in world score=0.49 views=6 id=ed6e5cde seen 24s ago, first 28s ago
    no verified detection of 'bag'

- One line per verified object, newest sighting first. The position is the
  centroid of the object's newest sighting in the window, in the frame the
  line names.
- `seen Ns ago, first Ms ago` are the ages of the object's newest and oldest
  sighting inside the window.
- An object that moved inside the window is one line per place, each with its
  own ages: the trace of where it was. Two lines of one label that were seen
  over the same seconds are two objects.
- `id` names an object in this window. The same window asked again gives the
  same `id`; another window can name the same object differently.
- `score` is the detector's best score for the object. Compare scores within one
  label only; the scale changes with the wording.
- `views` counts the distinct camera positions, 1 cm apart, that saw the object.
  An object needs 2, and a camera that does not move adds none. A camera that
  never moves, such as a fixed top view, stays silent until `min_views` is 1:
  pass `'{"min_views": 1}'`, which reads no frame again.
- `no verified detection` means nothing passed the gates at these params. It is
  not proof that the object is absent.
- Labels are not compared: one object can be answered under two labels.

## Watching labels on a schedule

Four labels on a rolling 5 s window, one call after another:

    uv run dimos mcp call localize -j '{"objects": "chair,door,bag,plant", "start": -5, "duration": 5}' --timeout 60

- A laptop RTX 5070 with 8 GB sustained this, calling continuously.
- A call reads only the frames not yet read for its labels, so each call costs
  the new seconds, not its whole window.
- The window must span the time between the starts of two calls: the call's own
  run time plus any wait. If calls take longer than the window, widen it. The
  overlap is free, because read frames are skipped; a gap loses frames.
- Keep the label strings and `params` the same on every call. Memory is per
  exact label string, and a window read under other params is read again,
  unless only the fields that build the answer changed.
- Each answer is what was seen in that call's window. An object that was taken
  away drops out once the window no longer reaches its last sighting; one that
  moved shows at its new place only.

## Narrowing a position

The position is the newest sighting's, so a closer sighting refines it.

1. Bring the camera close to the object and move it a little: every new camera
   position adds a view.
2. Ask a short recent window, `{"objects": "red mug", "start": -5, "duration": 5}`.
   The answer holds only that window's sightings, so its position is the
   newest one.
3. Check that `seen` is near 0 s.

At a table, a walking robot's room-scale params are coarse for small objects:

| field | room (`go2`) | desk (`xarm`) | desk (`dual_yam`) |
|---|---|---|---|
| `cluster_radius_m`: sightings this close are one object | 0.30 | 0.08 | 0.08 |
| `fuse_voxel_m`: object cloud resolution | 0.03 | 0.01 | 0.01 |
| `min_camera_range_m`: closer sightings are dropped | 0.5 | 0.28 | 0.07 |
| `surface_patch_max_rise_m`: a sighting this flat on the support is surface | 0.08 | 0.003 | 0.003 |

Pass `"params": "xarm"` on every call for that label while working at the
table. A window first read at room scale is read again under desk params.

## Changing params per call

The presets are defined in `localize_params.py`, and its `PRESETS` maps their
names:

| `params` | constant | rig |
|---|---|---|
| `"xarm"` | `XARM_PARAMS` | desk: a wrist camera over a tabletop |
| `"dual_yam"` | `DUAL_YAM_PARAMS` | desk: dual OpenYAM arms whose D405 wrist cameras see objects from 7 cm |
| `"go2"` | `GO2_PARAMS` | room: a walking robot, furniture-sized objects, a 5 cm lidar map |

`params` changes the module's params for that call only. The next call without
it is back on the module's, the ones its blueprint set.

| `params` value | meaning |
|---|---|
| `""` | the module's params |
| a preset name | that preset |
| `'{"accept_score": 0.45}'` | these fields on the module's params |
| `'{"base": "xarm", "accept_score": 0.45}'` | these fields on a preset |

    uv run dimos mcp call localize -j '{"objects": "apple", "params": "xarm"}' --timeout 300
    uv run dimos mcp call localize -j '{"objects": "apple", "params": "{\"min_views\": 3}"}' --timeout 300

| field | acts on | what it gates |
|---|---|---|
| `accept_score` | the answer | best score an object needs to be answered |
| `min_views` | the answer | camera positions an object needs |
| `cluster_radius_m` | the answer | sightings this close are one object |
| `fuse_voxel_m` | the answer | resolution of the object's cloud |
| `candidate_floor` | what is read | detector score that forms a box; lower means more boxes and more GPU memory |
| `max_object_extent_m`, `min_points`, `min_camera_range_m` | what is read | size, points and camera distance a sighting needs |
| `verify_radius_m`, `peak_prominence` | what is read | how many frames retrieval hands to the detector |

- The fields that act on the answer are applied when it is built: changing
  them reads nothing again, and every window answered from memory uses them.
  Try them first.
- A field that acts on what is read starts a separate memory: a window asked
  under it is read again.
- A misspelled field fails the call with `unexpected keyword argument`, an
  unknown preset with `unknown preset`.
- Every field is defined on `LocalizeParams` in `types.py`, and each preset's
  values are in `localize_params.py`.

## Labels and GPU

- Do not pass many labels at once. Every detector box is segmented on the GPU,
  so each label adds memory and time.
- Reference: a laptop RTX 5070 with 8 GB sustained 4 labels per call on a
  rolling 5 s window, calling continuously. With less free GPU memory, or with
  Rerun or other models on the same GPU, start with fewer.
- Use concrete nouns with one attribute that sets the object apart: `red mug`,
  `wooden crate`. Broad or overlapping labels make many boxes.
- On `Remote torch.OutOfMemoryError`, halve the labels per call.

## Where a failure comes from

Embedding, detection and segmentation work from the image and find almost any
object. Lifting a detection to 3D needs depth at the object, and that depends on
the rig. Most failures to get a position come from the lift or from the rig,
not from the image models.

### Room: walking robots

- **No depth at the object.** The stack lifts through the rig's depth camera or
  lidar, so a detected object gets no 3D answer until depth reaches it. Go2's
  L1 lidar is sparse and covers a small radius: when the robot starts moving,
  localize can lift only what is close. Bring the robot nearer and ask a recent
  window again.
- **Odometry drift.** A static object seen at two times with drifted poses lands
  in two places, as two `id`s. The one seen most recently matches the current
  odometry.

### Desk: arms

- **Depth** is rarely the problem.
- **Look-alike objects.** A group of objects of the same colour and shape, with
  few unique features, can be missed by the detector from some angles.

### Any rig

- **Calibration.** A poorly calibrated rig shows every object twice, the copies
  shifted by the same offset for every object. That points at the rig, not at
  the params.
- **Light and environment.** Every rig depends heavily on them. Tune params on
  the fly with the call's `params`, the score fields `candidate_floor` and
  `accept_score` first.
- **Clouds for grasping.** `fuse_voxel_m` is 0.01 in the desk presets and 0.03
  in `go2`. For GraspGenX, go as low as 0.003, because it can reject clouds
  fused at larger voxels. The voxel applies every time a sighting joins an
  object, so pass it on every call for that label, for example
  `'{"base": "xarm", "fuse_voxel_m": 0.003}'`. A cloud already fused coarser
  does not regain detail.
- **Simulation.** Sim images and MuJoCo workflows need their own params; the
  presets were fit on images from real rigs.

Tuning params offline on a recording, before deploying them, is in `README.md`.
