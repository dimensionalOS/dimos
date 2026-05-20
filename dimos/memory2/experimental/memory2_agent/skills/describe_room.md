---
name: describe_room
description: Use when asked what's IN a particular room, or what the robot would see if it walked into another room — "what's in the other room", "if you walked through the door, what would you see", "what's in the room at (x, y)", "describe the room with the vending machines". Composes `room_extents`: first identify the rooms and which frames belong to which room, then look at the frames belonging to the target room. The answer comes from those frames, not from semantic search — that avoids biasing the result toward whatever the question hints at.
---

# Describing a room's contents from its classified frames

The trap: if you `search_semantic("pantry with shelves and drinks")` to find
what's in a room, CLIP returns the frames that best match that query — so the
answer confirms the hypothesis you put in. To avoid this, identify the rooms
*first* (geometrically, via `room_extents`), then describe the contents of
the target room by opening the frames that were classified into it.

## When to use

- "What's in the other room?"
- "If you walked straight through the door, what would you find?"
- "Describe the room at the south end of the building."
- "What's in the room with the vending machines?"

## Tools allowed

- Everything `room_extents` uses (`walkthrough_timestamps`, `show_image`,
  `show_map`, `verify_room_partition`, `calc`, `frames_facing`).
- (When you're done, end with a plain text reply — no tool call.)

Do NOT call `search_semantic` while running this procedure. The point of the
skill is to avoid the confirmation-bias path that `search_semantic` opens up
when the question hints at content.

## Procedure

1. **Run `room_extents`.** Follow its procedure end-to-end (visual pairwise
   classification → map verification → per-room polygons and frame lists).
   When you finish, you should have, in `calc`:
   ```python
   rooms = {
       1: {"desc": "...", "polygon": [[x, y], ...],
           "frames": [(idx, ts, x, y), ...]},
       2: {...},
       ...
   }
   ```

2. **Identify the target room.** Re-read the user's question and decide
   which room they're asking about. Common shapes:
   - **"The other room" / "the next room" / "through the door":** the
     question presupposes the robot is somewhere now and is asking about
     a different room. Determine the robot's *current* room (the room
     whose polygon contains the robot's last odom pose, via `show_map`
     or directly from `rooms`), then "the other room" is whichever
     remaining room the door points into. If there are >1 other rooms,
     pick the one the robot was facing or moving toward at the end of
     the recording (use the camera yaw at the last frame).
   - **"The room at (x, y)" / "the room with the vending machines":**
     match the position or description against `rooms[i]["desc"]` or
     polygon centroid.

3. **Pick 3–5 representative frames from that room's `frames` list.**
   Choose frames spread across the room (different (x, y) inside the
   polygon, different camera yaws), not three near-identical adjacent
   frames. For each pick:
   ```python
   calc('''
   import random
   frames = rooms[<target_id>]["frames"]
   # spread by index — first, last, middle, plus 2 more
   n = len(frames)
   picks = [frames[0], frames[n // 4], frames[n // 2], frames[3 * n // 4], frames[-1]]
   picks
   ''')
   ```

4. **Open each pick with `show_image(stream="color_image", ts=<ts>)`.**
   Look at what's visible. List concrete things you can see: objects,
   furniture, lighting, people, signage. Do NOT speculate about things
   that aren't in the frame ("there's probably a kitchen behind the
   wall") — only describe what the recording captured.

5. **Synthesise.** Write a one- or two-sentence description of the
   target room based on what *appeared across multiple of the frames*.
   If something is only visible in one frame, mention it but flag it
   as such ("a person was visible at one point near the door").

## Discipline notes

- **No `search_semantic`.** It biases results toward whatever you wrote
  in the query. Use only the room-classification frames as evidence.
- **Stick to what's actually visible.** "The other room contains shelves"
  is fine if you see shelves. "The other room is probably a kitchen"
  is not — that's a guess from environmental context, not from a frame.
- **If the robot never actually entered the target room** (e.g., asked
  "what's through the door" but the recording ends at the threshold),
  use the *last* few frames in which the doorway is visible and
  describe what's visible *through* it. State explicitly that the
  robot didn't enter, so the description is what was seen from outside.
- **If `room_extents` only finds one room**, the question presupposing
  "another room" can't be answered from this recording — say so.
- Camera yaw / lighting differences are not new rooms; trust the
  `room_extents` partition over your own visual second-guessing.
