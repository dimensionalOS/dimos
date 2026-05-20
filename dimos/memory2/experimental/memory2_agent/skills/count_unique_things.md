---
name: count_unique_things
description: Use when asked HOW MANY of a particular kind of thing the robot saw or passed by — "how many people", "how many white robots", "how many vending machines", "how many chairs". The recording is a moving camera, so the same object is typically visible from multiple angles across multiple frames; naively counting sightings double-counts. This skill localises each candidate to a verified world (x, y) using `frames_facing` as a visual feedback loop, then merges sightings whose refined positions coincide. The final count is the number of distinct refined world positions.
---

# Counting unique objects by world-position convergence

Rule: call `frames_facing` at least once per candidate sighting before
submitting a count. A candidate that hasn't been through `frames_facing`
doesn't count.

A moving camera sees the same physical object from many angles. The trick
to counting *uniquely* is not to compare sightings against each other —
that's slow and error-prone — but to **pin each sighting to a world (x,
y)** using a visual feedback loop, then **merge any sightings whose
pinned positions land on top of each other**.

The loop, per candidate sighting:

1. Identify a target in one frame.
2. Propose an initial world (x, y) for it (using the camera's pose +
   roughly where the target sits in the frame).
3. Call `frames_facing(x=…, y=…)`. The tool projects that world point as
   a red cross into every frame whose cone could contain it.
4. Look at the projected frames: does the red cross land **on the
   target's body** in the frames where the target is visible?
   - **Yes** → (x, y) is correct for this object. Lock it in.
   - **No, the cross is offset** → adjust (x, y) and call
     `frames_facing` again. Repeat until convergence (usually 1–3
     iterations).
   - **No frames show the target where the cross lands** → either the
     sighting was a hallucination, or the position is far enough off
     that no covering frame can see it. Drop the sighting or re-estimate
     the bearing.

Once every sighting has converged to a verified world (x, y), the count
is just **the number of distinct (x, y)s, modulo a small radius**. Two
sightings whose verified positions land within ~1 m of each other came
from the same physical object — they're the same robot/person/whatever,
seen from different camera angles.

## When to use

- "How many people did you see?"
- "How many white robots did you pass by?"
- "How many chairs/tables/vending machines/doors are there?"
- "Did you see more than one X?"

## Tools allowed

- `walkthrough_timestamps`  (per-chunk sampling schedule)
- `show_image`              (one per index; also revisit specific frames
                              after refinement)
- `frames_facing`           (the position feedback loop — see step 3 in
                              the procedure)
- `show_map`                (optional, to sanity-check final positions
                              against lidar walls)
- `calc`                    (bookkeeping, distance math)
- `search_semantic`         (optional shortcut to find candidate sightings
                              when the target has a clear visual signature)
- (When you're done, end with a plain text reply — no tool call.)

## Procedure

The procedure is **per candidate**: as soon as you identify a candidate
sighting, immediately do steps 2 and 3 for it before you move on to the
next candidate. Do NOT enumerate all candidates first and then "go back
and verify" — you will skip the verification.

1. **Find ONE candidate sighting at a time.** Two options:
   - **Dense:** call `walkthrough_timestamps(t_start="start",
     t_end="<X>", step_seconds=1.0)` and then `show_image` on each frame
     in order. When a frame contains the target → STOP iterating, drop
     into steps 2–3 for this candidate, then resume.
   - **Sparse / faster:** call `search_semantic(stream=
     "color_image_embedded", query="<short description>", k=10)` to get
     candidate frames. Open the top one with `show_image`. When you see
     the target → drop into steps 2–3 immediately.

   For each candidate, record:
   ```
   sighting = {
       "frame_ts": <ts>,
       "cam_x": <cam.x from caption>,
       "cam_y": <cam.y from caption>,
       "cam_yaw_deg": <cam.yaw from caption>,
       "bearing_in_frame": "left" | "centre" | "right",
       "range_guess_m": <2 / 3 / 5 / 8 …>,
       "initial_xy": (<x>, <y>)  # see step 2
   }
   ```

2. **Propose an initial world (x, y) for THIS sighting.** Project from
   the camera using bearing + range guess:
   ```python
   calc('''
   import math
   theta = math.radians(cam_yaw_deg + bearing_offset_deg)
   wx = cam_x + range_m * math.cos(theta)
   wy = cam_y + range_m * math.sin(theta)
   (wx, wy)
   ''')
   ```
   `bearing_offset_deg`: roughly −20° for "left", 0° for "centre", +20°
   for "right" (head camera FOV is ~76°, so the edge is ~±38°).
   `range_m`: conservative mid-band (3 m) unless you can clearly see the
   target is close or far.

3. **Verify THIS sighting with `frames_facing` BEFORE you look at the
   next candidate.** This step is mandatory; you may not skip it. The
   `frames_facing` call is the only thing that confirms a sighting is
   real and locates it. The feedback loop:

   a. Call `frames_facing(x=wx, y=wy, k=4, max_range_m=8,
      check_occlusion=False)`. The tool returns up to 4 frames whose
      cones could contain (wx, wy), with a red cross drawn where (wx, wy)
      projects in each frame.

   b. Examine the returned frames:
      - **Cross is on the target's body** (within the silhouette, not
        just nearby) in ≥1 frame → (wx, wy) is GOOD. Mark this sighting
        VERIFIED and move on.
      - **Cross is consistently off** in the same direction (e.g. always
        too far left, or always too close to the camera) → adjust (wx,
        wy) in the opposite direction and call `frames_facing` again.
        Cap iterations at 3 per sighting; if it doesn't converge in 3,
        drop the sighting (probably a hallucinated detection or
        ambiguous geometry).
      - **No covering frames returned at all** → the proposed (wx, wy)
        is outside the explored area; treat the sighting as
        un-localisable and drop it.

   c. Keep the iteration in `calc`:
      ```python
      calc('verified.append({"sighting_idx": i, "xy": (wx, wy), "desc": "..."})')
      ```

4. **Merge: close-and-never-co-visible → same object.** For every
   pair (A, B) of verified positions with `dist(A, B) ≤ 2 m`, presume
   same object unless a single frame shows both A and B as two
   distinct targets simultaneously.

   To check: look at the `frames_facing` images you already have for
   A and B from step 3. Any frame that appears in both result sets
   with two distinct targets visible (one at the A cross, one at the
   B cross) → keep A and B separate. Otherwise → merge.

   ```python
   calc('''
   from math import hypot
   merge_radius = 2.0  # m
   # ``co_visible[(i, j)] = True`` if a shared frame showed two
   # distinct targets at sighting_idx i and j. Defaults to False → merge.
   co_visible = {}
   merged = []
   for v in verified:
       vx, vy = v["xy"]
       hit = next(
           (m for m in merged
            if hypot(m["xy"][0] - vx, m["xy"][1] - vy) <= merge_radius
            and not any(
                co_visible.get(tuple(sorted((mi, v["sighting_idx"]))), False)
                for mi in m["members"]
            )),
           None,
       )
       if hit is None:
           merged.append({"xy": v["xy"], "members": [v["sighting_idx"]]})
       else:
           hit["members"].append(v["sighting_idx"])
           n = len(hit["members"])
           hit["xy"] = (
               (hit["xy"][0] * (n - 1) + vx) / n,
               (hit["xy"][1] * (n - 1) + vy) / n,
           )
   merged
   ''')
   ```

   Set `co_visible[tuple(sorted((i, j)))] = True` (via `calc`) for any
   pair you confirm is simultaneously visible. Skip the marking step
   if you can't find shared frames — the default (presume merge) is
   correct.

5. **(Optional) Map sanity-check.** Call `show_map(when="now")` and look
   at where each merged position falls. Drop any position floating
   outside lidar-known free space — it's a localisation error. (If the
   map shows N distinct, sensibly-placed positions and your merged list
   has N, you're done.)

6. **Sanity check before replying.** The number you submit must equal
   `len(merged)`. If your `frames_facing` call count is below the
   number of candidates you identified, go back to step 3 for the
   un-verified ones first.

7. **Plain-text reply.**
   - The integer count = `len(merged)`.
   - Honour the user's format directive (e.g. "reply with only the
     number, nothing else" → just the integer).
   - If descriptions were asked for, one line per merged position.

## Discipline notes

- **The refinement loop replaces visual comparison.** Don't open frame A
  and frame B and ask the model "is this the same robot?" — that's
  unreliable and the model over-counts. Trust the geometry: if two
  refined positions land within 1 m, they're the same object regardless
  of how the views happen to look.
- **Position refinement uses bearing more than range.** "Left / centre /
  right" of the camera is much more reliable than "2 m vs 5 m away".
  Iterate the (x, y) primarily along the bearing direction.
- **`frames_facing` is the only verification step you need.** A sighting
  whose proposed position projects onto the target's body in a
  third-party frame *is* a unique object. You don't need to compare
  sightings against each other — the geometry will collapse duplicates
  automatically in step 4.
- **One sighting → one (x, y).** Don't multiply sightings just because
  one frame is blurry. Either the position refines to within the
  target's body or it doesn't.
- **Camera yaw / heading changes are not new objects.**
- If the question is just "did you see more than one X?", you can stop
  as soon as you have 2 verified positions ≥1 m apart.
