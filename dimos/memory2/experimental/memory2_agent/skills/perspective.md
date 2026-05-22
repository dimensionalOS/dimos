---
name: perspective
description: Use when a question asks about something from another entity's point of view — "what is to the right of <X>", "what does <X> see", "what is behind <X>" — where <X> is a person/robot/object you observed (not yourself). Requires inferring that entity's body orientation, which is NOT in the recording's metadata.
---

# Analyzing the perspective of another entity

"Right of the man" means the man's right, not the camera's. The robot's
`odom` only tells us *the camera's* pose; nothing in the recording
records the orientation of people, objects, or other robots the camera
saw. You have to infer it from the image, then translate into a
world-frame query.

## When to use this skill

The question contains a phrase that puts another entity at the center
of the spatial reference frame:

- "what is to the right/left/forward/behind <entity>"
- "what does <entity> see"
- "what is in front of <entity>"
- "from <entity>'s point of view, what is …"

If the question is about *your own* (the Go2's) viewpoint
("what was on my right at t=30?"), use `recall_view` directly instead —
do not load this skill.

## Procedure

Follow these steps in order. Do not skip step 2 — it's the only step
that lets you recover the entity's body frame.

1. **Find a frame with the entity.**
   Use `search_semantic("color_image_embedded", "<entity description>",
   k=3)`. Note the timestamps and the Go2's pose for each hit.
   Sanity check: if the top similarity is below ~0.30, the entity may
   not be in the recording — flag this honestly.

2. **VIEW the image.** This step is mandatory.
   Call `show_image(stream="color_image", ts=<ts of top hit>)`.
   You will see the frame inline. Look at the entity in the picture.

3. **Estimate the entity's world-frame body yaw.**
   Look at the image you fetched in step 2 and use the Go2's camera yaw
   (from the search hit's pose) to translate what you see into a
   world-frame heading for the entity:

   | What you see of the entity | Entity's body yaw (world) |
   |---|---|
   | Facing you (camera sees their front) | camera_yaw + 180° |
   | Back to you (you see their back) | camera_yaw |
   | Right side toward you (you see their right profile) | camera_yaw + 90° |
   | Left side toward you (you see their left profile) | camera_yaw − 90° |
   | Diagonal / partial — pick the closer cardinal | round to nearest 45° |

   State your estimate explicitly so the answer is auditable.

4. **Translate the requested direction into a world-frame yaw.**
   Given the entity's body yaw from step 3:

   - entity_forward = entity_yaw
   - entity_right   = entity_yaw − 90°
   - entity_left    = entity_yaw + 90°
   - entity_back    = entity_yaw + 180°

5. **Query in the entity's frame.**
   Call `recall_view(at_ts="<ts of the frame>", yaw_deg=<the yaw from step 4>)`.
   You may also widen `max_yaw_deg` if no hits return. Optionally call
   `show_map(when=<ts>)` for a top-down view of the area.

6. **If `recall_view` returns no frames** — that's an honest empty
   answer: the Go2 never looked that way from that spot. Do not
   fabricate. Say so explicitly.

7. **Answer.** In your final reply, state:
   - The frame you saw the entity in (ts, pose)
   - Your inferred body yaw for the entity, and why (what you saw in the image)
   - The world-frame direction you queried
   - What was visible there (or "no recorded view available")

## Worked example

Question: *"What is to the right of the man in black?"*

- Step 1: `search_semantic("color_image_embedded", "man in black", k=3)` →
  top hit at ts=1778055894.71, Go2 pose=(-2.07, 4.31), Go2 yaw≈+82°.
- Step 2: `show_image(stream="color_image", ts=1778055894.71)` → I see
  the man seated at a desk, his left side toward the camera.
- Step 3: left side toward camera → entity_yaw = camera_yaw − 90° ≈ −8°
  (he is facing roughly east).
- Step 4: man's right = entity_yaw − 90° = −98° (roughly south).
- Step 5: `recall_view(at_ts="1778055894.71", yaw_deg=-98)` → returns
  whatever the Go2 has recorded looking south from near (-2.07, 4.31).
- Step 6/7: answer based on what was actually found, or report empty.
