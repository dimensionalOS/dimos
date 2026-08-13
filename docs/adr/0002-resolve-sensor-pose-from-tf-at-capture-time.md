# Resolve sensor pose from TF at capture time

Point clouds retain their Capture Time, and mapping and robot self-exclusion resolve every required transform from TF near that time. This change uses TF's existing nearest-sample behavior with a strict maximum timestamp error and raises dynamic TF publication to the measured state rate; it does not add interpolation. Cloud and TF remain separate streams because TF is already the shared temporal frame graph, while a compound cloud-and-pose message would duplicate that responsibility and would not solve capture-time robot-link lookup.

## Considered Options

- Capture-time TF lookup: chosen because it establishes one temporal contract for camera placement and whole-robot self-exclusion while preserving the existing typed streams.
- Compound point-cloud and camera-pose message: rejected because self-exclusion still needs capture-time poses for every robot link and the message would duplicate TF state.
- Nearest current transform: rejected because independently sampled poses cause map drift and move the self-exclusion mask during robot motion.
- Transform interpolation: deferred because correctly stamped, high-rate state is expected to make bounded nearest-sample error sufficient for this feature.

## Consequences

Publishers must stamp transforms with measurement time rather than publication time. Consumers must always supply a strict time tolerance; the TF buffer's broad default is not a safety boundary. Clouds without a sufficiently close capture-time transform are dropped instead of being mapped with stale geometry.
