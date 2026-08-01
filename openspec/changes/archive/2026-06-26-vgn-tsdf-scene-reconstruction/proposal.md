## Why

DimOS has an intended grasp-generation seam, but no working native learned grasp backend. VGN is now installable behind the `grasp` extra, and integrating it cleanly requires a depth-based scene reconstruction path because VGN consumes TSDF grids reconstructed from depth observations, not raw pointcloud-only inputs.

## What Changes

- Add a generic scene reconstruction capability that consumes depth images, depth camera calibration, and TF to maintain a reconstructed scene.
- Add a `TSDFGrid` message type aligned with Open3D/VGN grid semantics.
- Publish reconstructed scene products at a fixed reconstruction output rate, including `PointCloud2`, `TSDFGrid`, and reconstruction status.
- Add TSDF-native grasp generation contracts and a VGN-backed module that consumes latest TSDF data.
- Add grasp candidate message types carrying pose, jaw width, and score, with PoseArray compatibility for existing consumers.
- Output VGN grasp candidates in `world` frame for simple downstream use.
- Keep high-rate sensor data on streams; use RPC for reconstruction lifecycle/control only.

## Capabilities

### New Capabilities
- `scene-reconstruction`: Generic depth-based scene reconstruction that produces pointcloud and TSDF scene products from depth observations.
- `tsdf-grasp-generation`: TSDF-native grasp generation that converts reconstructed TSDF grids into scored world-frame grasp candidates.

### Modified Capabilities
- None.

## Impact

- Affected code areas: `dimos/msgs`, `dimos/perception` or reconstruction modules, `dimos/manipulation/grasping`, and xArm/manipulation blueprints that opt into the new modules.
- New runtime path depends on the existing `grasp` optional extra for VGN and existing depth camera streams.
- Existing `GraspGenSpec` and pointcloud-based grasp paths remain available; this change adds a TSDF-native path rather than replacing them.
- Sensor payloads remain stream-based to avoid large pickled RPC requests.
