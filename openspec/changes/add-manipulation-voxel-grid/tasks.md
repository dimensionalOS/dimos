# Tasks

- [x] Add capture-triggered TF-to-Odometry conversion and capture timestamp propagation.
- [x] Add generic bounded cloud/Odometry synchronization with explicit map-frame validation.
- [x] Keep navigation compatible when no voxel-clear mask is connected.
- [x] Add independent metric voxel-clear masks and native partial clearing.
- [x] Add complete measured-state robot TF publication as a reusable module.
- [x] Filter exact padded URDF collision geometry for base, arm, and gripper links.
- [x] Replace shape-specific manipulation obstacle RPCs with complete typed `Obstacle` values.
- [x] Add a latest-wins global-map-to-stable-OCTREE obstacle bridge.
- [x] Remove snapshot staging, periodic TF polling, static-TF storage, atomic triple joins, and demo-only attachment boxes.
- [x] Rewire the xArm MuJoCo/Viser blueprint through the mounted camera frame chain.
- [x] Render the backend-accepted OCTREE through the ordinary obstacle lifecycle.
- [x] Add focused Python/Rust tests and update the generated blueprint registry.
- [x] Replace superseded ADRs with this single current design and update the manual guide.
- [ ] Manually verify desk registration remains fixed while moving the wrist.
- [ ] Manually verify base, arm, wrist, fingers, and knuckles leave no persistent occupied trail.
- [ ] Manually verify collision-aware plan and execution completion from the Viser gizmo.
