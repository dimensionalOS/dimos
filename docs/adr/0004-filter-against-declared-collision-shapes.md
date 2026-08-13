# Filter against declared collision shapes

Robot self-exclusion preserves the collision shapes declared by the robot model. Primitive shapes use analytic containment and padded-surface checks; mesh shapes use a padded bounding box to select candidate points followed by distance-to-surface queries against the collision mesh. This avoids removing real obstacles from empty space inside coarse link-level bounding boxes.

## Considered Options

- Declared collision shapes: chosen because they preserve the authoritative model's spatial detail and the project already provides mesh-loading and proximity-query dependencies.
- Generated boxes, capsules, or spheres: rejected as the default because their conservative empty volume can erase nearby environment observations, especially around wrists and grippers.
- Mesh inside/outside tests alone: rejected because RGB-D observations lie on surfaces and robot collision meshes are not guaranteed to be watertight.

## Consequences

Mesh queries use a cheap padded-AABB broad phase before triangle-distance evaluation. Runtime performance must be verified against the sensor update budget, and the exclusion padding remains an explicit sensor/calibration policy rather than being baked into generated geometry.
