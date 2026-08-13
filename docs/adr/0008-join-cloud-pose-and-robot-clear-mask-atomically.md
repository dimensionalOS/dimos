# Join cloud, pose, and robot clear mask atomically

The voxel mapper updates its state only after joining a self-filtered cloud, its capture-time camera pose, and a Robot Self Geometry clear mask with the same Capture Time. It clears the mask's voxel keys and then integrates the environmental cloud as one ordered update; an incomplete tuple expires without mutating the map.

## Considered Options

- Timestamped clear-mask stream joined in the mapper: chosen because it keeps robot-model policy outside ray tracing while making clear-before-integrate ordering explicit across processes.
- Clear RPC followed by cloud publication: rejected because the operations are not atomic and may interleave with another observation.
- Robot model and TF lookup inside the mapper: rejected because it couples sensor synchronization and robot geometry processing to the native ray tracer.

## Consequences

The mapper's existing timestamp join expands from cloud plus odometry to cloud plus odometry plus clear mask. All three inputs share the cloud's Capture Time, and timeout or capacity eviction discards the whole pending update.
