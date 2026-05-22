---
name: unexplored_spaces
description: Use when asked where to explore next, what areas the robot hasn't visited, what's beyond the explored region, or which directions to prioritise for further mapping — "where should we explore next?", "what areas did you miss?", "what's in the unmapped part of this place?". Composes `thinking_about_rooms`: after partitioning the explored area into rooms, the ORANGE blobs flagged by `verify_room_partition` are unpartitioned free space; the subset of those blobs that are NOT fully surrounded by occupied walls (magenta cells) point into unknown territory and are the exploration frontiers.
---

# Finding exploration frontiers from the verified room partition

A "frontier" is a piece of known free space that touches unknown space.
That's a classic robotics target for next-step exploration: you can drive
there, and once there, the lidar will see further. To find them in this
recording's map, we re-use the segmentation pipeline:

1. `thinking_about_rooms` partitions the explored area into rooms.
2. `verify_room_partition` highlights ORANGE blobs of free space that
   weren't covered by any room polygon.
3. Each orange blob is either a *missed room* (fully enclosed by walls)
   or a *frontier* (open to unknown space). Only the second kind is an
   exploration target.

## When to use

- "Where should we explore next?"
- "What parts of the space did we miss?"
- "Are there areas you didn't walk through?"
- "Which direction should we send the robot now?"

## Tools allowed

- Everything `thinking_about_rooms` uses (`walkthrough_timestamps`, `show_image`,
  `show_map`, `verify_room_partition`, `calc`, `frames_facing`).
- (When you're done, end with a plain text reply — no tool call.)

## Procedure

1. **Run `thinking_about_rooms`.** Follow its full procedure end-to-end. By the
   end you'll have, in `calc`, the `rooms` dict and a final
   `verify_room_partition` call that returned the annotated map +
   per-room stats. The orange markers on that map are the
   unpartitioned-free-space blobs — these are the candidates.

2. **Read off each orange blob's position and area** from the
   `verify_room_partition` text output. The tool labels each orange
   marker with its area in m². Record them in `calc`:
   ```python
   calc('''
   blobs = [
       {"xy": (x_1, y_1), "area_m2": a_1},
       {"xy": (x_2, y_2), "area_m2": a_2},
       ...
   ]
   ''')
   ```

3. **Classify each blob.** Call `show_map(when="now")` (or look at the
   existing `verify_room_partition` image) and for each blob, look at
   what surrounds it on the occupancy map:

   - **MAGENTA on every side** (walls / occupied cells fully enclosing
     it) → this is a *missed room or alcove* the robot didn't enter.
     It's interior space, not a frontier. Mark `kind = "enclosed"`. It
     can be explored but only by entering a doorway from an adjacent
     room first; don't recommend it as a top-level frontier.

   - **BLUE/KNOWN free space on most sides, with some BLACK/UNKNOWN
     edges** → this is a *frontier*. Lidar saw partway in but the
     region opens out into unmapped territory. Mark `kind = "frontier"`.

   - **Tiny, hugging an external wall** → probably lidar bleed-through
     past an exterior wall. Mark `kind = "noise"` and drop.

   Record each classification in `calc`:
   ```python
   calc('blobs[i]["kind"] = "frontier" | "enclosed" | "noise"')
   ```

4. **Rank frontiers by usefulness.** A frontier is more valuable if it
   is:
   - Larger (bigger `area_m2` → more potential reward).
   - Adjacent to a recently-visited part of the trajectory (cheap to
     reach).
   - Aligned with the robot's last heading (no need to back out).

   The first criterion is dominant; the others break ties. Use `calc`
   to sort:
   ```python
   calc('''
   frontiers = [b for b in blobs if b["kind"] == "frontier"]
   frontiers.sort(key=lambda b: -b["area_m2"])
   frontiers
   ''')
   ```

5. **(Optional) Project the top frontier into a recorded view.** If
   you want to describe what the robot was *facing toward* the
   frontier, call `frames_facing(x=fx, y=fy, max_range_m=10)` at the
   top frontier's (x, y). Frames whose viewing cone includes that
   point are the ones that already glimpsed the frontier — useful for
   describing what's at the edge.

6. **Plain-text reply.**
   - Lead with the top-ranked frontier: its approximate world (x, y),
     its area, and which direction it sits relative to the robot's
     final pose ("east / north / behind you").
   - Mention 1–2 more frontiers if they're meaningfully different.
   - If the question implies a single answer ("where should we
     explore next?"), return *one* recommendation, not a list.
   - If there are no frontiers (all orange blobs were enclosed or
     noise), say so explicitly — "the robot covered the reachable
     extent of the explored area; there's nothing visibly unmapped to
     go to."

## Discipline notes

- **The classification is visual, not numerical.** The skill works
  because `verify_room_partition` already filtered out small noise
  and labelled real free-space gaps. Trust the orange markers; don't
  invent frontiers somewhere else on the map.
- **Don't recommend frontiers inside unknown black space.** A spot
  surrounded entirely by black has no lidar evidence that it's even
  reachable — there could be a wall there. Frontiers must have some
  blue (known free) cells touching them.
- **An enclosed orange blob ≠ a frontier**, even if large. If walls
  surround it on all sides, the robot would need to find a doorway
  first. That's a different question (`describe_room` on a missed
  room) than "explore next".
- **Camera content doesn't decide.** Use the lidar map for frontier
  detection; the camera frames just describe *what the robot saw*,
  not *what's beyond what it saw*.
