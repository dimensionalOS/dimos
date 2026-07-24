# DimOS

DimOS composes robot capabilities from modules that communicate through typed streams.

## Manipulation Planning

**Atomic scene operation**:
A single planning-world query or mutation that observes either the complete scene before an obstacle update or the complete scene after it. A multi-step planning run may contain atomic scene operations from different scene revisions.
_Avoid_: Atomic plan, planning snapshot

**Opaque scene operation**:
A backend call whose internal planning-world queries cannot be synchronized individually. The complete call is one atomic scene operation and excludes obstacle replacement for its duration.
_Avoid_: Interruptible native plan

**Obstacle replacement**:
A complete obstacle value supplied by a caller to supersede the existing obstacle identified by its name. Omitted or stale information is the caller's responsibility; the planning world does not merge partial updates.
_Avoid_: Obstacle patch, partial update

**Obstacle pose update**:
An atomic change to an existing obstacle's pose that preserves every non-pose property. It is a distinct operation from complete obstacle replacement.
_Avoid_: Obstacle patch, partial replacement

**Obstacle snapshot**:
The planning world's private copy of a complete obstacle value. Caller-owned inputs and returned obstacle values do not provide mutable access to this snapshot.
_Avoid_: Shared obstacle object, live obstacle reference

**Obstacle identity**:
The non-empty, immutable `Obstacle.name` shared by the planning world, monitors, and visualization. Changing identity is removal of the old obstacle followed by addition of a new obstacle.
_Avoid_: Obstacle ID argument, obstacle rename

**Invalid planning world**:
A planning world whose native scene may no longer represent its accepted obstacle state after an invariant violation. It cannot serve scene operations and must be reconstructed.
_Avoid_: Degraded world, partially valid world

**Planning world**:
The authoritative robot-and-obstacle scene used for planning and collision safety. Visualization is a best-effort projection of this world and does not participate in its commits.
_Avoid_: Visualization scene, rendered world

**Finalized world**:
A planning world whose robot topology and native scene initialization are complete. Planning-world obstacle operations are valid only in this lifecycle state.
_Avoid_: Partially initialized scene, mutable topology
