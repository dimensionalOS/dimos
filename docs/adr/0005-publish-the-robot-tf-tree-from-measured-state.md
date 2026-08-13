# Publish the robot TF tree from measured state

Each measured joint state publishes the complete robot kinematic TF tree using the joint state's timestamp. Parent-to-child joint edges represent the base, articulated links, gripper, and modeled sensor mounts; the periodic 10 Hz wall-clock publisher for the end effector and manually selected `tf_extra_links` is removed. The measured joint-state rate becomes the dynamic TF publication rate.

## Considered Options

- Event-driven kinematic tree: chosen because every edge shares the measured robot-state time, interpolation respects joint-edge motion, and required links come from the robot model.
- Periodic direct world-to-link transforms: rejected because it samples latest state at publication time, interpolates links independently, and requires a manually maintained link list.
- Publish both paths: rejected because multiple authorities for the same frames make graph lookup ambiguous and preserve an obsolete timing contract.

## Consequences

TF publication is driven by joint-state updates rather than a separate rate. Capture-time consumers use the nearest dynamic sample within a strict tolerance, while fixed model edges remain constant. Missing or stale required state cannot be hidden by a more recent wall-clock publication.
