---
name: room_extents
description: Use for any room-level question — counting ("how many rooms did I walk through", "did I change rooms", "how many distinct spaces"), sizing ("how big is each room", "where is the biggest room", "what are the dimensions / bounding box of room X"), or location ("where on the map is room Y"). One procedure handles all of them: classify each sampled frame into a room by pairwise visual comparison, collect the camera (x, y) per frame, then verify against the lidar occupancy map. The final count is len(rooms) after verification; the sizes and extents come from the per-room polygons.
---

# Room analysis from per-frame poses + lidar verification

There is no automatic room segmentation. To answer any room-level
question — count, size, position — we classify sampled camera frames
into rooms pairwise, AND keep the camera (x, y) per frame, AND verify
the resulting partition against the lidar occupancy map. The visual
pass produces candidate rooms; the map pass refines them so visually
distinct views of the same enclosed space don't get double-counted.

## When to use

- Count: "How many rooms did I walk through?", "Did I change rooms?"
- Size: "Where is the biggest room?", "How big are the rooms?"
- Bounds: "What's the bounding box of room X?", "Where on the map is
  room Y?"

The verification step is mandatory for all of these — the visual
classifier alone over-segments (calls every distinct *scene* a new
room), and the lidar map is what disproves that.

## Tools allowed

- `show_map`               (look at the map to draw room polygons)
- `walkthrough_timestamps` (the sampling schedule for pairwise classification)
- `show_image`             (per-frame visual classification)
- `calc`                   (bookkeeping + area math)
- `frames_facing`          (verify a proposed polygon CORNER by
                            projecting that (x, y) back into the
                            camera frames — see step 5b)
- `verify_room_partition`  (overlay your polygons on the map and
                            flag missed odom, lidar-free cells, and
                            overclaim into UNKNOWN territory)
- (When you're done, end with a plain text reply — no tool call.)

Use `calc` for all arithmetic.

## Procedure

1. **Sample the walk — one chunk at a time.**
   Call `walkthrough_timestamps(t_start="start", t_end="<X>",
   step_seconds=2.0)` over a sub-range no wider than 60s. The tool
   refuses anything that would require >32 samples.

   **CRITICAL:** Do NOT pre-fetch every chunk's schedule up front.
   Fetch ONE chunk, then fully complete step 2 + step 3 on that
   chunk's frames (open each frame with `show_image`, decide
   continuation/revisit/NEW, append to `rooms`) before you call
   `walkthrough_timestamps` again for the next chunk. Interleave
   schedule → per-frame analysis → next schedule, so context stays
   focused on the frames you're actively classifying. The running
   `rooms` dict in `calc` carries state across chunks.

2. **Open frame 0** with `show_image(stream="color_image", ts=<ts at
   index 0>)`. Read the camera's `(x, y)` from the caption.

   Initialise the room bookkeeping in `calc`:
   ```python
   calc("""
   rooms = {
       1: {"desc": "<short description from frame 0>", "frames": [(0, <ts_0>, <x_0>, <y_0>)]}
   }
   """)
   ```

3. **Loop over i = 1, 2, …, N-1, in order. For each i:**

   a. Call `show_image(stream="color_image", ts=<ts at index i>)`.
      Read the camera's `(x_i, y_i)` from the caption.

   b. Compare frame i-1 (still in your context) and frame i. Decide
      exactly one:
      - **continuation** — same room as i-1
      - **revisit** — same as an earlier room in `rooms`
      - **NEW** — new room

   c. Append the frame to the chosen room via `calc`. If continuation
      or revisit:
      ```python
      calc('rooms[<room_id>]["frames"].append((<i>, <ts_i>, <x_i>, <y_i>))')
      ```
      If NEW:
      ```python
      calc('''
      rooms[<new_id>] = {"desc": "<short desc>", "frames": [(<i>, <ts_i>, <x_i>, <y_i>)]}
      ''')
      ```

   d. State your decision out loud (in your assistant message)
      summarising the frames compared, the room assigned, and the
      `(x, y)` you recorded. Example:
      *"frames 2 → 3 (pos (-1.91,+10.80) → (-1.44,+15.44)): transition
      into NEW room #2 (store aisles). Frame 3 added."*

4. **After all N-1 pairwise decisions**, compute each room's bounds
   in a single `calc` call. Example:
   ```python
   calc('''
   summary = []
   for rid, room in rooms.items():
       frames = room["frames"]
       xs = [f[2] for f in frames]
       ys = [f[3] for f in frames]
       n = len(frames)
       if n == 1:
           summary.append({
               "id": rid, "desc": room["desc"], "n": 1,
               "centroid": (xs[0], ys[0]),
               "bbox": None, "area_m2": None,
           })
       else:
           x_min, x_max = min(xs), max(xs)
           y_min, y_max = min(ys), max(ys)
           cx = sum(xs) / n
           cy = sum(ys) / n
           summary.append({
               "id": rid, "desc": room["desc"], "n": n,
               "centroid": (cx, cy),
               "bbox": (x_min, y_min, x_max, y_max),
               "area_m2": (x_max - x_min) * (y_max - y_min),
           })
   summary.sort(key=lambda s: -(s["area_m2"] or -1))
   summary
   ''')
   ```
   This returns the per-room summary as a Python list, sorted by
   coverage area descending. A room with `n == 1` has `area_m2 =
   None` — flag those as "extent unknown from this sample" in your
   final answer rather than calling them zero-sized.

   The bbox from sampled poses is a *lower bound* — the robot may
   have entered the room from one side and only walked part of it.
   Refine it visually in the next step.

5. **Look at the map and propose final room POLYGONS.**
   Call `show_map(when="now")` and look at the lidar-derived
   occupancy. Use your `summary` from step 4 as a STARTING POINT and
   then **trace each room's walls**, recording the (x, y) of every
   corner where the wall changes direction.

   Four hard rules:

   - **The world frame is NOT aligned to the room walls.** The SLAM
     coordinate system started wherever the robot booted; the
     building's walls run at whatever angle they happen to. Do not
     assume rooms are axis-aligned. Even a simple quad room is
     usually a rotated parallelogram in world coords — its corners
     will have DIFFERENT y-values on the same wall and DIFFERENT
     x-values on the perpendicular wall.

   - **Don't submit an axis-aligned bounding box.** If your
     polygon is `[[x_min, y_min], [x_max, y_min], [x_max, y_max],
     [x_min, y_max]]`, you've drawn a rectangle around the room's
     bounding box, not the room. STOP and re-look at the map — the
     walls do NOT go straight up/down/left/right in world coords.

   - **Trace the walls, corner by corner.** Pick one corner of the
     room on the map. Read its (x, y) by eye from the show_map
     grid. Walk along the wall (visually) to the NEXT corner — note
     that this corner's x and y are almost certainly BOTH different
     from the previous one, because the wall is at an angle.
     Continue around the room. 4 corners for a true quadrilateral
     (rotated rectangle = parallelogram); 5–8 for L-shapes or
     rooms with alcoves.

   - **Rooms must NOT overlap.** Where two rooms meet (a doorway,
     an archway), pick a single line on the map and let each
     polygon end at that line. Adjacent polygons share an edge,
     never an interior.

   Express each room as:
   `polygon = [[x1, y1], [x2, y2], [x3, y3], ...]`
   (3+ corners, world coords, clockwise or counter-clockwise order).

5b. **(Optional) Verify any corner you're unsure about.** If you're
    not confident about a corner's exact (x, y) — typically when the
    wall in the map image is fuzzy or you can't tell exactly where a
    corner sits — call `frames_facing(x=<corner_x>, y=<corner_y>)`.
    The tool projects that world point into every camera frame whose
    cone could contain it and draws a red cross at the projection in
    each. Look at the crosses:
    - If the cross consistently lands on a visible wall corner, base
      of a column, or actual wall edge in the recorded frames → the
      corner is well-placed.
    - If the cross lands away from any wall in the camera views →
      move the corner to where the wall actually is.
    Use this sparingly — one or two ambiguous corners per polygon,
    not every corner.

6. **Verify with `verify_room_partition`.**
   ```
   verify_room_partition(rooms=[
       {"id": 1, "desc": "<short>", "polygon": [[x, y], [x, y], [x, y], [x, y]]},
       {"id": 2, "desc": "<short>", "polygon": [[x, y], [x, y], ...]},
       ...
   ])
   ```
   The returned image overlays your rectangles on the occupancy map
   semi-transparently and marks:
     - GREEN dots = odom samples inside some room (✓)
     - RED dots   = odom samples OUTSIDE every room (you missed
                    somewhere the robot walked)
     - ORANGE markers = the most SALIENT blobs of unpartitioned
                        free space the lidar saw. The marker size is
                        proportional to area; each is labelled in
                        m². Only meaningful blobs appear — tiny
                        single-cell stragglers along walls are
                        filtered out.

   Look at the image AND the per-room text. Four things to check:
     1. **No overlap warning.** If the text shows `⚠ overlap with →`
        for any room, two of your polygons interior-overlap. Fix
        it by shrinking one polygon so they share an edge instead
        of crossing.
     2. **No red wash inside any polygon.** Red on the map means
        the polygon covers UNKNOWN cells (cells lidar never saw —
        space past walls). High `overclaim` percentage in the text
        means the same thing. Shrink the polygon's corners inward
        to follow the walls and remove the red.
     3. **Few RED dots.** Many red dots in a cluster mean part of
        the robot's path isn't covered → extend the nearest
        room's polygon outward (but stay inside the lidar-known
        free space — don't trade off (2) for (3)).
     4. **No large orange marker inside the explored area
        without a covering polygon.** A large orange marker far
        from your polygons but in the lidar-mapped free space
        means you missed a sub-room → extend an existing polygon,
        OR (only if it's clearly separated by walls/doorway) add a
        new room.
     - Small orange markers at the edge of the explored area
       (beyond an external wall) are lidar bleed-through — ignore.

   Iterate: adjust the rectangles, call `verify_room_partition`
   again, repeat until you're happy. Cap iterations at ~3 — diminishing
   returns after that.

7. **Compute the final area from the verified polygons.**
   `verify_room_partition` already returns per-polygon `area_m2` in
   its text output. Copy those numbers into the answer; you don't
   need a separate `calc` for the area when you used polygons.

   If you DO want to recompute or centroid-summarise in `calc`, use
   the shoelace formula:
   ```python
   calc('''
   def poly_area(pts):
       n = len(pts); s = 0.0
       for i in range(n):
           x1, y1 = pts[i]
           x2, y2 = pts[(i+1) % n]
           s += x1*y2 - x2*y1
       return abs(s) / 2
   def poly_centroid(pts):
       n = len(pts)
       return (sum(p[0] for p in pts)/n, sum(p[1] for p in pts)/n)

   verified = [
       {"id": 1, "desc": "...", "polygon": [[x, y], ...]},
       {"id": 2, "desc": "...", "polygon": [[x, y], ...]},
   ]
   for r in verified:
       r["area_m2"]  = poly_area(r["polygon"])
       r["centroid"] = poly_centroid(r["polygon"])
   verified.sort(key=lambda r: -r["area_m2"])
   verified
   ''')
   ```

8. **Reply.** End your turn with a plain-text answer. Match the
   answer shape to the question — the procedure above produces both
   the count AND the per-room geometry, so use only what was asked:

   - **Count question** ("how many rooms", "did I change rooms"):
     reply with just the integer (or "yes"/"no"). If the user asked
     for "only the number", give only that. Do NOT dump the per-room
     description list unless they asked for one.
   - **Biggest-room question** ("where is the biggest room", "how
     big is the biggest"): lead with that room and its area /
     polygon. "Biggest" = largest verified-polygon area.
   - **Sizing / bounds question** ("how big are the rooms", "what
     are the dimensions of room X"): list every room with id, short
     description, polygon corners, centroid (x̄, ȳ), area (m²) from
     the verified polygon, and number of sample frames classified
     into it.

   If the user added a format directive (e.g. "reply with only the
   number, nothing else"), honour it strictly — the room polygons
   and frame lists you computed are intermediate work, not the
   final answer.

## Discipline notes

- The walked-coverage bbox from step 4 is a *lower bound*. The
  verified rectangle in step 6 is what you actually report — extend
  it on the map (step 5) to cover lidar-visible floor the robot
  didn't step on but is clearly inside the room's walls.
- Don't extend a rectangle through walls or doorways into the next
  room. The orange markers from `verify_room_partition` show
  unpartitioned floor; some of those markers will be in adjacent
  rooms or outside the building. Use your visual judgement on the
  map to decide which markers should be absorbed by extending an
  existing room vs which indicate genuinely separate rooms.
- **Prefer extending an existing room's wall to inventing a new
  room.** A new room only makes sense if the unpartitioned area is
  clearly separated from existing rooms by walls or a doorway.
- A "continuation" still adds the new frame's `(x, y)` to the
  room's frame list — that's how the initial bbox grows.
- Don't conjure a number when you only have one sample in a room.
  Say "extent unknown (only one sample)".
- Camera yaw / facing changes are not room changes.
- This procedure is geometry on top of the same pairwise visual
  reasoning that `count_rooms` uses. If the visual classification is
  wrong, the rectangles will be wrong too.
