# Reusable ACT manipulation primitives

The requested behavior is `pick(object, arm)` → held object → independent
`place(region, arm)`. A language agent composes these actions only when the user
asks for them. "Pick" ends with a grasped, lifted object, without release or an
inferred destination. A destination-bearing instruction can compose pick and place.

## Current implementation

`r1pro-objects-sim-agent` exposes `pick_object`, `place_object`, measured inventory,
stop, recovery and reset. The first command stops after current two-pad contact,
a 10 cm lift and a verified hold after cancelling ACT. Placement is a new rollout
from measured held state, recomputes free tray space, and verifies supported release.
A full tray does not prevent picking; it can refuse placement while preserving the hold.

These commands reuse the existing full-sequence checkpoint. Pick still supplies a
legacy tray-goal context; this is a transitional execution adapter, not proof that
the model learned an independent or destination-invariant pick primitive. Current
capabilities remain right-hand table picks and tray placement. Tray unloading,
left-hand picking, arbitrary supports and carrying a held random object through
the house remain unimplemented here. The original bottle/navigation blueprint is
separate and does not automatically confer those capabilities on this blueprint.

## Target architecture

1. Resolve an object and an arm from measured scene state. An explicitly requested
   arm must be preserved. Automatic arm selection scores feasible candidates.
2. Select a stable, collision-free object pose within the requested placement
   region. Account for the object footprint, obstacles, support and available space.
   The agent names a region; it does not invent low-level joint targets.
3. Ask the locomomanipulation layer for a base/torso pose that puts the source or
   destination within that arm's validated policy workspace. Check physical IK
   reachability, collision clearance and demonstrated coverage separately.
4. Execute the selected ACT pick or place policy through ControlCoordinator.
   Policy output must own only the intended arm and agreed torso resources;
   a policy must not open or reset another hand already holding an object.
5. Check current contacts, lift, support, release and non-target motion. Maintain
   held-object state per arm. Failures stop or recover explicitly and never become
   success simply because a trajectory finished.

Use a small family of policies conditioned on object/goal geometry and the chosen
arm, rather than a policy for every object, order or named room. Whether arms share
one canonical policy or use two adapted checkpoints depends on measured transfer;
the earlier naive mirroring test failed and is not a deployable left-arm skill.
Changing the room should be handled by navigation and local-frame goals, provided
the resulting grasp/place is within the policy's trained geometry and workspace.
This is not a guarantee of generalization to new shapes or support heights.

## Reusing the existing demonstrations

`demo_segment_objects` creates pick/place manifest views of the original RGB files:

```bash
python -m dimos.robot.galaxea.r1pro.demo_segment_objects \
  --source recordings/r1pro-act-task/jobs/random-objects-act-v1/collection \
  --output recordings/r1pro-act-task/jobs/my-primitive-data
```

Pick includes above/approach/grasp/lift and the recorded lift hold. Place starts at
clear_sources from the already-held state and includes supported release/retreat.
The converter uses the same frame slice for images, actions, state and statistics.
Episode boundaries prevent action chunks from crossing from pick into place.
Original images, demonstrations and weights are not rewritten. Keep paired views
from a layout on the same train/validation side; they are not independent evidence.

The prepared views are `jobs/random-objects-primitive-data-v1`: 115 pick segments
(17,480 frames) and 115 place segments (53,490 frames). They preserve the old
right-arm observation/action contract as preparation inputs. Before training a
new arm/goal contract, explicitly migrate the feature mapping and normalization,
then warm-start compatible weights. ACT task text alone is not a substitute for
an implemented conditioning input.

Add targeted demonstrations for held-state variation, tray-source grasps, varied
supported destinations and left-arm use. Retain old data as rehearsal. Fine-tune
new artifacts and evaluate independent pick-hold, place-from-hold, pause/resume,
wrong-arm rejection, full tray, and mixed requested sequences on fresh seeds.
Do not schedule another long run on unchanged data just to repeat the same loss.

## Navigation tuning

The user permits higher yaw speed for future navigation tests. The current planar
servo and holonomic profile cap yaw at 0.12 rad/s. Tune both the task profile and
servo/slew limits together, verify loaded-object stability and swept collisions,
and keep the final approach precise. No navigation speed changed in this primitive
interface update; locomomanipulation integration remains a separate workstream.
