# Derive self-exclusion from the robot model

Point-cloud self-exclusion uses Robot Self Geometry derived from the robot's URDF/Xacro collision model and places it using capture-time transforms. The filter covers the base, articulated links, gripper, sensor mounts, and registered attachments, keeping the motion-planning collision model authoritative instead of maintaining separate exclusion regions.

## Considered Options

- Robot-model geometry: chosen because the same definition applies to simulation and physical RGB-D cameras and evolves with the robot configuration.
- MuJoCo renderer masking: rejected as the production path because it hides the problem in simulation and cannot serve physical sensors; it may remain a test oracle.
- Hand-authored per-link regions: rejected because they duplicate the robot description and become incomplete or stale as links and tools change.

## Consequences

Every required link pose must be resolvable at the cloud's Capture Time. Missing required geometry or transforms makes the observation invalid rather than partially self-filtered.
