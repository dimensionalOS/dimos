# Habitat Simulation

Run mapping and navigation against photorealistic scans of real buildings, with no robot. [Habitat-sim](https://github.com/facebookresearch/habitat-sim) renders an [HM3D](https://aihabitat.org/datasets/hm3d/) house; dimos sees a robot that takes `Twist` and publishes RGB-D, pose and a scan.

It is a renderer over a static mesh with a navmesh, not a physics engine: no legs, no contact, no dynamics. The robot is a pose and a camera rig that slides along walls. Good for perception, mapping, planning and following; useless for locomotion.

## Try It

```bash
dimos run habitat-nav
```

The first run builds the simulator environment, which takes 5 to 10 minutes and about 3.5 GB of disk: a python 3.9 conda env with habitat-sim, plus one annotated HM3D house with 908 labelled objects that needs no Matterport credentials. Later runs start in seconds.

Drive with the viewer's keyboard controls. Click the planner's surface in the 3D view to set a goal.

## Blueprints

Layered so a failure can be bisected by dropping a level:

| Blueprint           | Adds                                                                                                                                         |
|---------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| `habitat-teleop`    | The sim and its streams. Drive it, nothing else.                                                                                             |
| `habitat-raycaster` | `RayTracingVoxelMap` on a sensor-frame scan.                                                                                                 |
| `habitat-nav`       | `MLSPlannerNative` and `BasicPathFollower`. Goal by clicking.                                                                                |
| `habitat-voxel`     | `VoxelGridMapper` on a pre-registered scan. An alternative to the raycaster, not a layer: the two mappers want the scan in different frames. |

## Requirements

- Linux x86_64. habitat-sim is published as conda packages for `linux-64` only.
- nix with flakes, which the other native modules already need.
- An NVIDIA driver providing EGL for headless rendering. This comes from the host, not nix.
- Network access for the first build: nixpkgs, conda-forge and the `aihabitat` channel, PyPI, and Meta's dataset host.

## How It Is Built

habitat-sim only ships python 3.9 builds, so it cannot share the dimos interpreter. `HabitatConnection` is a [native module](/docs/usage/native_modules.md): the simulator runs as a subprocess and speaks dimos over zenoh through `dimos_lcm`, the standalone message package. Nothing on that side imports dimos, and a test enforces it.

`dimos/simulation/habitat/nix/install.sh` is the module's `build_command`. Run through the module's flake, it creates the conda env, installs the message and transport packages, downloads the example scene and writes the `habitat-native` wrapper whose existence `NativeModule` treats as the build sentinel. Everything it produces lands in `target/habitat`, beside the cargo natives' output; delete `target/habitat/env` to force a rebuild.

## Frames

Habitat is y-up with -z forward; dimos is z-up with x forward. `dimos/simulation/habitat/frames.py` owns the conversion and is tested against basis vectors and a quaternion recorded from the live simulator.

The tf tree is `world -> base_link -> camera -> camera_optical`. Images hang off `camera_optical`: a pinhole points down its own +z, which in the body frame is straight up. `base_link` sits on the navmesh, so the planner's `start_z_offset_m` is 0, unlike a real robot whose base link rides above the ground.

## Licensed Scenes

The bundled house is the only one that needs no credentials. The full HM3D splits require a [research access grant](https://matterport.com/habitat-matterport-3d-research-dataset) from Matterport, after which:

```bash
cd dimos/simulation/habitat/nix
./env/bin/python -m habitat_sim.utils.datasets_download \
    --username <token-id> --password <token-secret> \
    --uids hm3d_minival_v0.2 --data-path ./data
```

Then point `HabitatConnection`'s `scene_dataset_config` and `scene_id` at the scene you want.
