# Clear robot volumes during map integration

Every valid capture clears the previous and current Robot Self Geometry volumes from the voxel map before integrating the capture's self-filtered environmental returns. Point filtering prevents new robot returns from becoming occupancy, while explicit clearing removes false robot voxels left by earlier timing, calibration, or filtering errors.

## Considered Options

- Clear previous and current robot volumes: chosen because self-occupied space is not environmental occupancy, and newly exposed real obstacles can be reinserted by the same capture.
- Filter incoming points only: rejected because an earlier false robot hit can persist when the robot continues to occlude the voxel and no later free-space ray traverses it.
- Clear only the current robot volume: rejected because voxels left at the preceding robot pose may survive after the robot moves away.

## Consequences

Robot-volume clearing and filtered-cloud integration form one ordered map update. A capture with incomplete self geometry or capture-time transforms performs neither operation. The clearing representation must cover occupied volume, not only the modeled surface.
