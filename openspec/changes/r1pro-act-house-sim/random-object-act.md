# ACT for randomly generated objects

Status: the first random-object ACT pilot trained and evaluated (6/12 single picks; 0/8 complete scenes). A detached 30,000-update refinement and automatic evaluation are running. Native integration and acceptance are pending. See handoffs-random-objects.md for evidence and job locations.

The user explicitly requires ACT to perform the grasps. The next target is a scene with four or five randomly generated rigid objects, selectable individually and movable into available tray space. Classical bottle unloading and tray transport remain separate skills. Do not substitute classical grasps and report them as ACT successes.

## Why the current checkpoint is insufficient

The existing PackingTask already conditions on the selected source, destination, radius and half-height. It is not trained as a single five-bottle sequence: a pick is one episode. However, the scene fixes five source locations, restricts reset jitter to one centimetre, and repeats one collision geometry and dimensions. The teacher also uses fixed heights and routes, some selected by bottle index. Repeating more object orders can diagnose gaps in this narrow distribution, but does not establish general grasping.

The desired abstraction is one selected-object pick-and-place skill. An agent resolves the requested object from the current scene; a geometric planner chooses an available placement footprint; ACT receives the selected target, scene observations and robot state, and executes the grasp, lift, transfer and release through the existing control task. Completion and recovery use measured physics. An object's ordinal position in a sequence is not a skill identity.

## First supported object distribution

Start with upright, rigid, graspable primitives and compound bottle-like shapes: cylinders, small boxes and bottles with variable dimensions, colour, mass and pose. Sample four or five objects with separation, workspace and support checks. Reject impossible gripper widths and unreachable source placements before collection. Keep the house assets unchanged by generating a local scene overlay.

This is a bounded distribution, not support for arbitrary meshes, handled mugs, bags or deformable objects. Simulator geometry may provide target identity, pose, dimensions and evaluation contacts initially; label that ground-truth dependency explicitly. Hundreds of procedurally generated instances are distinct from hundreds of unseen shape categories.

## Policy and data changes

- Replace fixed bottle geometry and index-dependent motion assumptions with per-object geometry shared by scene generation, goal planning, demonstrations and evaluation.
- Keep joint-position actions through the current ControlCoordinator initially. Evaluate gripper-relative target/destination features, object orientation and dimensions, selected-object visual information and surrounding clutter as observations. Changing coordinate systems alone does not prove generalization.
- Collect independent successful single-object picks from varied layouts and partial tray occupancies. Include different requested targets in the same initial scene so the target signal is necessary. Do not require collecting every ordering of a complete scene.
- Vary source position, destination position, dimensions, appearance and robot starting posture independently where physically feasible. Record rejected teacher attempts and their coverage; never hide a difficult family by silently excluding every failed demonstration.
- Use SDK planning for offline demonstrations and explicit recovery. During an ACT rollout, do not provide teacher trajectories, phase clocks or IK targets as policy observations. Record interventions separately from learned success.
- Keep a disjoint evaluation set of shapes/dimensions/layouts and target orders. Hold out some geometry families to measure extrapolation separately from interpolation within supported families. Freeze a final test set after development to avoid tuning on it repeatedly.

## Delivery gates

First verify generated scenes settle physically and the teacher can collect examples across the intended distribution. Then run a small training pilot and compare held-out single-pick results against the current checkpoint; use that evidence to choose further data and training budget. Do not spend another long run merely filling fixed-bottle permutations.

The final native blueprint must demonstrate correct requested-object selection, physical grasp and lift, supported placement, no disturbance to other objects, and repeated requests with four or five random objects. Test stop-when-full, infeasible requests and explicit recovery. Report both individual-pick success and complete-scene success, including seeds, failure categories and interventions. Training loss or evaluator exit code alone is not acceptance.

## Existing work retained

The deployed policy remains policy-packing-augmented. Flexible checkpoint/order/horizon experiments did not establish reliable arbitrary-order operation. Their supervisor is paused and no further fitting is scheduled. Existing navigation, tray delivery, grounded selectors, reset and recovery work remains useful; random-object support is not implemented by this document.
