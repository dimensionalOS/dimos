# Manipulation

Classical manipulation moves configured robot joints and uses perceived objects
to perform grasping and placement.

## Language

**Planning group**:
A named subset of a robot model's controllable joints that can be targeted together.
_Avoid_: Robot ID, hardware ID

**Generated plan**:
A proposed motion containing selected planning groups, a geometric path, and a
timed trajectory. A plan does not establish that motion has occurred.

**Detected object**:
An object identified in the latest scene scan, with a name and an ID that selects
it for picking. A name can describe several objects; an ID identifies one.

**Held object**:
An object the pick/place workflow considers grasped and not yet released.
A later motion failure can leave an object held.

**Placement destination**:
The intended end-effector position for releasing the held object, expressed in
the configured planning frame. It is not the object's center or a semantic label.
